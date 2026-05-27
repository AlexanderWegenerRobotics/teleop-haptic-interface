#include "teleop_controller.hpp"
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>

// ─── Construction / destruction ───────────────────────────────────────────────

TeleopController::TeleopController(IHapticDevice* device,
                                   IArmChannel*   arm,
                                   const YAML::Node& cfg,
                                   const std::string& log_dir)
    : device_(device)
    , arm_(arm)
    , log_dir_(log_dir)
    , F_prev_(Eigen::Matrix<double,6,1>::Zero())
    , logger_(log_dir + "/" + std::string(arm->name()) + "_control.csv",
              controlLogHeader, controlLogRow)
{
    // ── Frame rotation ────────────────────────────────────────────────────────
    auto rows = cfg["frame_rotation"].as<std::vector<std::vector<double>>>();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R_device_to_world_(r, c) = rows[r][c];

    motion_scaling_ = cfg["motion_scaling"].as<double>(4.0);

    // ── Control gains ─────────────────────────────────────────────────────────
    auto k_vec = cfg["control"]["stiffness"].as<std::vector<double>>();
    auto d_vec = cfg["control"]["damping"].as<std::vector<double>>();
    for (int i = 0; i < 6; ++i) {
        stiffness_(i) = k_vec[i];
        damping_(i)   = d_vec[i];
    }

    // ── Safety ────────────────────────────────────────────────────────────────
    max_force_        = cfg["safety"]["max_force"].as<double>(12.0);
    max_torque_       = cfg["safety"]["max_torque"].as<double>(0.5);
    max_force_rate_   = cfg["safety"]["max_force_rate"].as<double>(500.0);
    max_torque_rate_  = cfg["safety"]["max_torque_rate"].as<double>(20.0);

    // ── Passivity ─────────────────────────────────────────────────────────────
    double max_b = cfg["passivity"]["max_damping_gain"].as<double>(50.0);
    passivity_.setMaxDampingGain(max_b);

    start_time_ = std::chrono::high_resolution_clock::now();
}

TeleopController::~TeleopController() {
    stop();
}

// ─── Start / stop ─────────────────────────────────────────────────────────────

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
    HapticState current = device_->readState();
    std::lock_guard<std::mutex> lock(origin_mtx_);
    origin_     = current;
    has_origin_ = true;
    passivity_.reset();
    F_prev_.setZero();
    std::cout << "[TeleopController:" << arm_->name()
              << "] Origin captured.\n";
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

// ─── Coordinate helpers ───────────────────────────────────────────────────────

Eigen::Vector3d TeleopController::toWorld(const Eigen::Vector3d& v_device) const {
    return R_device_to_world_ * v_device;
}

// ─── Command computation ──────────────────────────────────────────────────────

ArmCommand TeleopController::computeCommand(const HapticState& state) const {
    ArmCommand cmd;

    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(origin_mtx_));
    if (!has_origin_) {
        cmd.position    = Eigen::Vector3d::Zero();
        cmd.orientation = Eigen::Quaterniond::Identity();
        cmd.gripper_closed = state.gripper_closed;
        return cmd;
    }

    // Position delta in world frame, scaled
    Eigen::Vector3d dp_device = state.position - origin_.position;
    cmd.position = toWorld(dp_device) * motion_scaling_;

    // Orientation delta from origin
    // R_delta = R_origin^-1 * R_current, then rotate to world frame
    Eigen::Quaterniond q_origin  = origin_.orientation;
    Eigen::Quaterniond q_current = state.orientation;
    Eigen::Quaterniond q_delta   = q_origin.inverse() * q_current;
    // Flip if needed for shortest path
    if (q_delta.w() < 0.0) q_delta.coeffs() *= -1.0;

    // Express in world frame: R_world_delta = R_device_to_world * R_delta * R_device_to_world^T
    // (similarity transform of the delta rotation into world frame)
    Eigen::Quaterniond R_dw(R_device_to_world_);
    cmd.orientation = (R_dw * q_delta * R_dw.inverse()).normalized();

    cmd.gripper_closed = state.gripper_closed;
    return cmd;
}

// ─── Haptic force computation ─────────────────────────────────────────────────

Eigen::Matrix<double,6,1> TeleopController::computeHapticWrench(
    const HapticState& device_state,
    const ArmState&    arm_state,
    double dt)
{
    // ── Position error in world frame ─────────────────────────────────────────
    // Desired EE position = arm_origin + R_base * command_position
    // We compare what the arm IS doing vs. what we commanded.
    // Simplified: use arm state position directly as the "setpoint receiver"
    // for the virtual coupling.  The arm tracks it; we track the arm.

    // Transform device position to world frame
    Eigen::Vector3d p_device_world = toWorld(device_state.position);
    Eigen::Vector3d p_arm_world    = arm_state.position;

    // Position error (world frame): arm behind device → positive error
    Eigen::Vector3d pos_error = p_arm_world - p_device_world * motion_scaling_;

    // Orientation error (simplified: body-frame angle-axis)
    Eigen::Quaterniond q_dev_world(R_device_to_world_ * device_state.orientation);
    Eigen::Quaterniond q_arm = arm_state.orientation;
    if (q_dev_world.dot(q_arm) < 0.0) q_dev_world.coeffs() *= -1.0;
    Eigen::Quaterniond q_err = q_dev_world.inverse() * q_arm;
    if (q_err.w() < 0.0) q_err.coeffs() *= -1.0;
    Eigen::Vector3d ori_error(q_err.x(), q_err.y(), q_err.z()); // ≈ half angle-axis

    Eigen::Matrix<double,6,1> error;
    error.head<3>() = pos_error;
    error.tail<3>() = ori_error;

    // Velocity (device frame → world frame)
    Eigen::Matrix<double,6,1> twist_world;
    twist_world.head<3>() = toWorld(device_state.twist.head<3>());
    twist_world.tail<3>() = toWorld(device_state.twist.tail<3>());

    // ── Impedance force ───────────────────────────────────────────────────────
    Eigen::Matrix<double,6,1> F_impedance =
        stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist_world);

    // Transform back to device frame for rendering
    Eigen::Matrix<double,6,1> F_device;
    F_device.head<3>() = R_device_to_world_.transpose() * F_impedance.head<3>();
    F_device.tail<3>() = R_device_to_world_.transpose() * F_impedance.tail<3>();

    // ── Passivity correction ──────────────────────────────────────────────────
    // Operate in device frame (velocity and force both in device frame)
    Eigen::Matrix<double,6,1> twist_device = device_state.twist;
    Eigen::Matrix<double,6,1> F_pc = passivity_.update(F_device, twist_device, dt);

    Eigen::Matrix<double,6,1> F_total = F_device + F_pc;

    // ── Rate limiting ─────────────────────────────────────────────────────────
    Eigen::Matrix<double,6,1> dF = F_total - F_prev_;
    double dt_safe = std::max(dt, 1e-4);
    for (int i = 0; i < 3; ++i)
        dF(i) = std::clamp(dF(i), -max_force_rate_ * dt_safe,
                                    max_force_rate_ * dt_safe);
    for (int i = 3; i < 6; ++i)
        dF(i) = std::clamp(dF(i), -max_torque_rate_ * dt_safe,
                                    max_torque_rate_ * dt_safe);
    F_total = F_prev_ + dF;

    // ── Magnitude clamp ───────────────────────────────────────────────────────
    double f_norm = F_total.head<3>().norm();
    if (f_norm > max_force_)
        F_total.head<3>() *= max_force_ / f_norm;

    double t_norm = F_total.tail<3>().norm();
    if (t_norm > max_torque_)
        F_total.tail<3>() *= max_torque_ / t_norm;

    F_prev_ = F_total;
    return F_total;
}

// ─── Control loop ─────────────────────────────────────────────────────────────

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
            // ── Send command to arm ───────────────────────────────────────────
            ArmCommand cmd = computeCommand(dev);
            arm_->sendCommand(cmd);

            // ── Compute and render haptic force ───────────────────────────────
            ArmState arm = arm_->getState();
            Eigen::Matrix<double,6,1> F = Eigen::Matrix<double,6,1>::Zero();

            if (arm.is_valid) {
                F = computeHapticWrench(dev, arm, dt);
            }

            device_->setForce(F.head<3>(), F.tail<3>());

            // ── Log ───────────────────────────────────────────────────────────
            double t = std::chrono::duration<double>(
                now - start_time_).count();
            ControlLogEntry entry{};
            entry.time     = t;
            entry.dev_px   = static_cast<float>(dev.position.x());
            entry.dev_py   = static_cast<float>(dev.position.y());
            entry.dev_pz   = static_cast<float>(dev.position.z());
            entry.dev_vx   = static_cast<float>(dev.twist(0));
            entry.dev_vy   = static_cast<float>(dev.twist(1));
            entry.dev_vz   = static_cast<float>(dev.twist(2));
            entry.arm_px   = static_cast<float>(arm.position.x());
            entry.arm_py   = static_cast<float>(arm.position.y());
            entry.arm_pz   = static_cast<float>(arm.position.z());
            entry.f_total_x = static_cast<float>(F(0));
            entry.f_total_y = static_cast<float>(F(1));
            entry.f_total_z = static_cast<float>(F(2));
            entry.e_obs     = static_cast<float>(passivity_.energyObservation());
            entry.b_linear  = static_cast<float>(passivity_.injectedDampingLinear());
            entry.sys_state = engaged_ ? 4u : 1u; // ENGAGED=4, IDLE=1
            logger_.write(entry);
        } else {
            device_->zero();
        }

        next += period;
        std::this_thread::sleep_until(next);
    }
}
