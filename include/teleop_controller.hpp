#pragma once

// ─── teleop_controller.hpp ────────────────────────────────────────────────────
// Per-arm teleoperation controller.
// Owns one IHapticDevice + one IArmChannel and runs the haptic control loop.
// Implemented in Step 2 of the build plan.
// ─────────────────────────────────────────────────────────────────────────────

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

// Log entry written at haptic control rate
struct ControlLogEntry {
    double time;
    // Device state (device frame)
    float  dev_px, dev_py, dev_pz;
    float  dev_vx, dev_vy, dev_vz;
    // Arm state (world frame)
    float  arm_px, arm_py, arm_pz;
    // Forces (device frame)
    float  f_impedance_x, f_impedance_y, f_impedance_z;
    float  f_passivity_x, f_passivity_y, f_passivity_z;
    float  f_total_x,     f_total_y,     f_total_z;
    // Passivity diagnostics
    float  e_obs;
    float  b_linear;
    // State
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

// ─── TeleopController ─────────────────────────────────────────────────────────

class TeleopController {
public:
    TeleopController(IHapticDevice* device,
                     IArmChannel*   arm,
                     const YAML::Node& haptic_config,
                     const std::string& log_dir);

    ~TeleopController();

    // Start/stop the 1 kHz haptic control thread
    void start();
    void stop();

    // Called by TeleopSession to engage or disengage teleoperation.
    // captureOrigin() should be called once the device is at its neutral pose.
    void captureOrigin();
    void setEngaged(bool engaged);
    bool isEngaged() const { return engaged_; }

    // Pass a new log path for the next episode
    void restartLogger(const std::string& path);

private:
    void runControlLoop();

    // Coordinate transform: device frame → world frame (position + velocity)
    Eigen::Vector3d toWorld(const Eigen::Vector3d& v_device) const;
    // Delta pose computation: sigma pose relative to captured origin → command
    ArmCommand computeCommand(const HapticState& device_state) const;
    // Haptic force: position error + passivity correction
    Eigen::Matrix<double,6,1> computeHapticWrench(
        const HapticState& device_state,
        const ArmState&    arm_state,
        double dt);

    IHapticDevice* device_;
    IArmChannel*   arm_;

    // Config
    Eigen::Matrix3d R_device_to_world_;
    double          motion_scaling_;
    Eigen::Matrix<double,6,1> stiffness_;
    Eigen::Matrix<double,6,1> damping_;
    double max_force_;
    double max_torque_;
    double max_force_rate_;
    double max_torque_rate_;

    // State
    std::atomic<bool> running_{false};
    std::atomic<bool> engaged_{false};
    std::thread       thread_;
    std::mutex        origin_mtx_;
    HapticState       origin_;
    bool              has_origin_ = false;

    // Passivity
    PassivityController passivity_;

    // Previous force for rate limiting
    Eigen::Matrix<double,6,1> F_prev_;

    // Logging
    std::string log_dir_;
    std::chrono::high_resolution_clock::time_point start_time_;
    DataLogger<ControlLogEntry> logger_;
};
