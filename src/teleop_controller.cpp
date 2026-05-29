#include "teleop_controller.hpp"
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>

TeleopController::TeleopController(std::unique_ptr<IHapticDevice> device, std::unique_ptr<IArmChannel> arm, const YAML::Node& cfg, const std::string& log_dir)
    : device_(std::move(device))
    , arm_(std::move(arm))
    , log_dir_(log_dir)
    , F_prev_(Eigen::Matrix<double,6,1>::Zero())
    , logger_(log_dir + "/" + std::string(arm_->name()) + "_control.csv", controlLogHeader, controlLogRow)
{
    auto rows = cfg["frame_rotation"].as<std::vector<std::vector<double>>>();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R_device_to_world_(r, c) = rows[r][c];

    motion_scaling_ = cfg["motion_scaling"].as<double>(4.0);

    auto k_vec = cfg["control"]["stiffness"].as<std::vector<double>>();
    auto d_vec = cfg["control"]["damping"].as<std::vector<double>>();
    for (int i = 0; i < 6; ++i) {
        stiffness_(i) = k_vec[i];
        damping_(i)   = d_vec[i];
    }

    max_force_       = cfg["safety"]["max_force"].as<double>(12.0);
    max_torque_      = cfg["safety"]["max_torque"].as<double>(0.5);
    max_force_rate_  = cfg["safety"]["max_force_rate"].as<double>(500.0);
    max_torque_rate_ = cfg["safety"]["max_torque_rate"].as<double>(20.0);

    double max_b = cfg["passivity"]["max_damping_gain"].as<double>(50.0);
    passivity_.setMaxDampingGain(max_b);

    start_time_ = std::chrono::high_resolution_clock::now();
}

TeleopController::~TeleopController() { stop(); }

void TeleopController::start() {
    arm_->start();
    device_->open();
    device_->enableForce(true);
    device_->zero();
    logger_.start();
    running_ = true;
    thread_  = std::thread(&TeleopController::runControlLoop, this);
}

void TeleopController::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    device_->zero();
    device_->enableForce(false);
    device_->close();
    arm_->stop();
    logger_.stop();
}

void TeleopController::captureOrigin() {
    // Reset device reference point, then snapshot both poses.
    device_->zero();
    HapticState dev_now = device_->readState();
    ArmState    arm_now = arm_->getState();
    std::lock_guard<std::mutex> lock(origin_mtx_);
    origin_         = dev_now;
    arm_origin_pos_ = arm_now.is_valid ? arm_now.position    : Eigen::Vector3d::Zero();
    arm_origin_ori_ = arm_now.is_valid ? arm_now.orientation : Eigen::Quaterniond::Identity();
    has_origin_     = true;
    passivity_.reset();
    F_prev_.setZero();
    std::cout << "[TeleopController:" << arm_->name() << "] Origin captured.\n";
}

void TeleopController::setEngaged(bool engaged) {
    engaged_ = engaged;
    if (!engaged) {
        device_->zero();
        passivity_.reset();
        F_prev_.setZero();
    }
    logger_.enable(engaged);
}

void TeleopController::restartLogger(const std::string& path) {
    logger_.restart(path);
}

Eigen::Vector3d TeleopController::toWorld(const Eigen::Vector3d& v_device) const {
    return R_device_to_world_ * v_device;
}

ArmCommand TeleopController::computeCommand(const HapticState& state) const {
    ArmCommand cmd;
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(origin_mtx_));
    if (!has_origin_) {
        cmd.position       = Eigen::Vector3d::Zero();
        cmd.orientation    = Eigen::Quaterniond::Identity();
        cmd.gripper_closed = state.gripper_closed;
        return cmd;
    }

    cmd.position = toWorld(state.position - origin_.position) * motion_scaling_;

    Eigen::Quaterniond q_delta = origin_.orientation.inverse() * state.orientation;
    if (q_delta.w() < 0.0) q_delta.coeffs() *= -1.0;
    Eigen::Quaterniond R_dw(R_device_to_world_);
    // Send the world-frame delta only — avatar applies its own origin on top,
    // matching exactly how the VR interface works.
    cmd.orientation    = (R_dw * q_delta * R_dw.inverse()).normalized();
    cmd.gripper_closed = state.gripper_closed;
    return cmd;
}

Eigen::Matrix<double,6,1> TeleopController::computeHapticWrench(
    const HapticState& device_state,
    const ArmState&    arm_state,
    double dt)
{
    // Position error: (arm movement from origin) − (device movement from origin × scale)
    Eigen::Vector3d delta_arm    = arm_state.position - arm_origin_pos_;
    Eigen::Vector3d delta_dev    = toWorld(device_state.position - origin_.position) * motion_scaling_;
    Eigen::Vector3d pos_error    = delta_arm - delta_dev;

    // Orientation error: arm deviation from the orientation the device commanded
    Eigen::Quaterniond q_dev_delta = origin_.orientation.inverse() * device_state.orientation;
    if (q_dev_delta.w() < 0.0) q_dev_delta.coeffs() *= -1.0;
    Eigen::Quaterniond R_dw(R_device_to_world_);
    //Eigen::Quaterniond q_cmd_world = (arm_origin_ori_ * R_dw * q_dev_delta * R_dw.inverse()).normalized();
    Eigen::Quaterniond q_cmd = (R_dw * q_dev_delta * R_dw.inverse()).normalized();
    Eigen::Quaterniond q_cmd_world = (q_cmd * arm_origin_ori_).normalized();
    if (q_cmd_world.dot(arm_state.orientation) < 0.0) q_cmd_world.coeffs() *= -1.0;
    Eigen::Quaterniond q_err = q_cmd_world.inverse() * arm_state.orientation;
    if (q_err.w() < 0.0) q_err.coeffs() *= -1.0;
    Eigen::Vector3d ori_error(q_err.x(), q_err.y(), q_err.z());

    Eigen::Matrix<double,6,1> error;
    error.head<3>() = pos_error;
    error.tail<3>() = ori_error;

    // Device velocity in world frame for damping
    Eigen::Matrix<double,6,1> twist_world;
    twist_world.head<3>() = toWorld(device_state.twist.head<3>());
    twist_world.tail<3>() = toWorld(device_state.twist.tail<3>());

    Eigen::Matrix<double,6,1> F_imp_world =
        stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist_world);

    // Rotate impedance force back to device frame for rendering and passivity
    Eigen::Matrix<double,6,1> F_device;
    F_device.head<3>() = R_device_to_world_.transpose() * F_imp_world.head<3>();
    F_device.tail<3>() = R_device_to_world_.transpose() * F_imp_world.tail<3>();

    Eigen::Matrix<double,6,1> twist_device = device_state.twist;
    Eigen::Matrix<double,6,1> F_pc = passivity_.update(F_device, twist_device, dt);

    Eigen::Matrix<double,6,1> F_total = F_device + F_pc;

    // Rate limit
    Eigen::Matrix<double,6,1> dF = F_total - F_prev_;
    double dt_safe = std::max(dt, 1e-4);
    for (int i = 0; i < 3; ++i)
        dF(i) = std::clamp(dF(i), -max_force_rate_ * dt_safe, max_force_rate_ * dt_safe);
    for (int i = 3; i < 6; ++i)
        dF(i) = std::clamp(dF(i), -max_torque_rate_ * dt_safe, max_torque_rate_ * dt_safe);
    F_total = F_prev_ + dF;

    // Magnitude clamp
    double f_norm = F_total.head<3>().norm();
    if (f_norm > max_force_)  F_total.head<3>() *= max_force_  / f_norm;
    double t_norm = F_total.tail<3>().norm();
    if (t_norm > max_torque_) F_total.tail<3>() *= max_torque_ / t_norm;

    F_prev_          = F_total;
    F_impedance_log_ = F_device;   // store for logging
    F_pc_log_        = F_pc;
    return F_total;
}

void TeleopController::runControlLoop() {
    constexpr int RATE_HZ = 1000;
    const auto period = std::chrono::microseconds(1000000 / RATE_HZ);
    auto next = std::chrono::high_resolution_clock::now();
    auto last = next;

    while (running_) {
        auto now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        HapticState dev = device_->readState();

        if (engaged_ && has_origin_) {
            arm_->sendCommand(computeCommand(dev));

            ArmState arm = arm_->getState();
            Eigen::Matrix<double,6,1> F = Eigen::Matrix<double,6,1>::Zero();
            if (arm.is_valid)
                F = computeHapticWrench(dev, arm, dt);

            device_->setForce(F.head<3>(), F.tail<3>());

            double t = std::chrono::duration<double>(now - start_time_).count();
            ControlLogEntry entry{};
            entry.time          = t;
            entry.dev_px        = static_cast<float>(dev.position.x());
            entry.dev_py        = static_cast<float>(dev.position.y());
            entry.dev_pz        = static_cast<float>(dev.position.z());
            entry.dev_vx        = static_cast<float>(dev.twist(0));
            entry.dev_vy        = static_cast<float>(dev.twist(1));
            entry.dev_vz        = static_cast<float>(dev.twist(2));
            entry.arm_px        = static_cast<float>(arm.position.x());
            entry.arm_py        = static_cast<float>(arm.position.y());
            entry.arm_pz        = static_cast<float>(arm.position.z());
            entry.f_impedance_x = static_cast<float>(F_impedance_log_(0));
            entry.f_impedance_y = static_cast<float>(F_impedance_log_(1));
            entry.f_impedance_z = static_cast<float>(F_impedance_log_(2));
            entry.f_passivity_x = static_cast<float>(F_pc_log_(0));
            entry.f_passivity_y = static_cast<float>(F_pc_log_(1));
            entry.f_passivity_z = static_cast<float>(F_pc_log_(2));
            entry.f_total_x     = static_cast<float>(F(0));
            entry.f_total_y     = static_cast<float>(F(1));
            entry.f_total_z     = static_cast<float>(F(2));
            entry.e_obs         = static_cast<float>(passivity_.energyObservation());
            entry.b_linear      = static_cast<float>(passivity_.injectedDampingLinear());
            entry.sys_state     = 4u;
            logger_.write(entry);
        } else {
            device_->zero();
        }

        next += period;
        std::this_thread::sleep_until(next);
    }
}
