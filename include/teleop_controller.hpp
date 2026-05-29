#pragma once

#include "haptic_device.hpp"
#include "arm_channel.hpp"
#include "passivity_controller.hpp"
#include "data_logger.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <string>

struct ControlLogEntry {
    double  time;
    float   dev_px, dev_py, dev_pz;
    float   dev_vx, dev_vy, dev_vz;
    float   arm_px, arm_py, arm_pz;
    float   f_impedance_x, f_impedance_y, f_impedance_z;
    float   f_passivity_x, f_passivity_y, f_passivity_z;
    float   f_total_x,     f_total_y,     f_total_z;
    float   e_obs;
    float   b_linear;
    uint8_t sys_state;
};

static inline std::string controlLogHeader() {
    return "time,"
           "dev_px,dev_py,dev_pz,dev_vx,dev_vy,dev_vz,"
           "arm_px,arm_py,arm_pz,"
           "f_imp_x,f_imp_y,f_imp_z,"
           "f_pc_x,f_pc_y,f_pc_z,"
           "f_tot_x,f_tot_y,f_tot_z,"
           "e_obs,b_linear,state\n";
}

static inline std::string controlLogRow(const ControlLogEntry& e) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "%.6f,"
        "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,"
        "%.5f,%.5f,%.5f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.5f,%.4f,%u\n",
        e.time,
        e.dev_px, e.dev_py, e.dev_pz,
        e.dev_vx, e.dev_vy, e.dev_vz,
        e.arm_px, e.arm_py, e.arm_pz,
        e.f_impedance_x, e.f_impedance_y, e.f_impedance_z,
        e.f_passivity_x, e.f_passivity_y, e.f_passivity_z,
        e.f_total_x, e.f_total_y, e.f_total_z,
        e.e_obs, e.b_linear, e.sys_state);
    return buf;
}

class TeleopController {
public:
    TeleopController(std::unique_ptr<IHapticDevice> device,
                     std::unique_ptr<IArmChannel>   arm,
                     const YAML::Node& haptic_config,
                     const std::string& log_dir);
    ~TeleopController();

    void start();
    void stop();

    // Snapshot current device + arm pose as the zero-error reference.
    void captureOrigin();
    void setEngaged(bool engaged);
    bool isEngaged() const { return engaged_; }

    void restartLogger(const std::string& path);

private:
    void runControlLoop();

    Eigen::Vector3d toWorld(const Eigen::Vector3d& v_device) const;
    ArmCommand computeCommand(const HapticState& device_state) const;
    Eigen::Matrix<double,6,1> computeHapticWrench(
        const HapticState& device_state,
        const ArmState&    arm_state,
        double dt);

    std::unique_ptr<IHapticDevice> device_;
    std::unique_ptr<IArmChannel>   arm_;

    // Config
    Eigen::Matrix3d           R_device_to_world_;
    double                    motion_scaling_;
    Eigen::Matrix<double,6,1> stiffness_;
    Eigen::Matrix<double,6,1> damping_;
    double max_force_, max_torque_, max_force_rate_, max_torque_rate_;

    // Origin — haptic device pose and arm pose at capture time
    std::mutex         origin_mtx_;
    HapticState        origin_;
    Eigen::Vector3d    arm_origin_pos_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond arm_origin_ori_{Eigen::Quaterniond::Identity()};
    bool               has_origin_ = false;

    std::atomic<bool> running_{false};
    std::atomic<bool> engaged_{false};
    std::thread       thread_;

    PassivityController       passivity_;
    Eigen::Matrix<double,6,1> F_prev_;

    // Last breakdown for logging (set inside computeHapticWrench)
    Eigen::Matrix<double,6,1> F_impedance_log_{Eigen::Matrix<double,6,1>::Zero()};
    Eigen::Matrix<double,6,1> F_pc_log_{Eigen::Matrix<double,6,1>::Zero()};

    std::string log_dir_;
    std::chrono::high_resolution_clock::time_point start_time_;
    DataLogger<ControlLogEntry> logger_;
};
