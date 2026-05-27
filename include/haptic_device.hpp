#pragma once

// ─── haptic_device.hpp ────────────────────────────────────────────────────────
// Abstract interface for a haptic input/force-feedback device.
// SigmaDevice (WITH_SIGMA) provides the real DHD SDK implementation.
// MockHapticDevice provides a sim stub.
//
// The TeleopController always programs against IHapticDevice* so the
// control logic is hardware-independent.
// ─────────────────────────────────────────────────────────────────────────────

#include <Eigen/Core>
#include <Eigen/Geometry>

// ─── Shared data types ────────────────────────────────────────────────────────

// Full state snapshot read from the device in one call.
struct HapticState {
    Eigen::Vector3d    position;        // device-frame position [m]
    Eigen::Quaterniond orientation;     // device-frame orientation
    Eigen::Matrix<double, 6, 1> twist;  // [vx,vy,vz,wx,wy,wz] device frame [m/s, rad/s]
    double  gripper_angle   = 0.0;      // degrees; 0 = closed
    bool    gripper_closed  = false;    // debounced open/close state
    bool    is_valid        = false;    // false until first successful read
};

// ─── Interface ────────────────────────────────────────────────────────────────

class IHapticDevice {
public:
    virtual ~IHapticDevice() = default;

    // Open / initialise the device.  Returns false on failure.
    virtual bool open()  = 0;

    // Release the device.  Safe to call even if open() failed.
    virtual void close() = 0;

    // Read the current device state (position, velocity, gripper).
    // Should be called at the haptic control rate (~1 kHz for Sigma).
    virtual HapticState readState() = 0;

    // Apply force [N] and torque [Nm] in the DEVICE frame.
    // The device must have force enabled (enableForce(true)) first.
    virtual void setForce(const Eigen::Vector3d& force,
                          const Eigen::Vector3d& torque) = 0;

    // Enable or disable the force-feedback motors.
    // Always call enableForce(false) or zero() on shutdown.
    virtual void enableForce(bool enable) = 0;

    // Set force and torque to zero (brakes released).
    virtual void zero() = 0;

    // Returns true if the device is physically connected and responding.
    virtual bool isConnected() const = 0;

    // Human-readable name for logging (e.g. "sigma_left", "mock_left")
    virtual const char* name() const = 0;
};
