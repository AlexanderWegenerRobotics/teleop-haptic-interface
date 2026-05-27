#pragma once

// ─── arm_channel.hpp ──────────────────────────────────────────────────────────
// Abstract interface for communication with one robot arm.
// UdpArmChannel provides the real UDP implementation.
// MockArmChannel (future) provides a simulated surface for offline testing.
//
// Commands and states are expressed in the avatar world frame
// (X=forward, Y=left, Z=up) as DELTAS from the arm's origin pose.
// ─────────────────────────────────────────────────────────────────────────────

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>

// ─── Shared data types ────────────────────────────────────────────────────────

// Command sent TO the arm (ArmCommandMsg payload, world-frame deltas)
struct ArmCommand {
    Eigen::Vector3d    position;        // delta from arm origin [m], world frame
    Eigen::Quaterniond orientation;     // delta rotation from arm origin, world frame
    bool               gripper_closed = false;
};

// State received FROM the arm (ArmStateMsg payload)
struct ArmState {
    Eigen::Vector3d    position;        // EE position in world frame [m]
    Eigen::Quaterniond orientation;     // EE orientation in world frame
    bool               is_recovering   = false;
    bool               is_valid        = false; // false until first packet received
    std::chrono::steady_clock::time_point timestamp;
};

// ─── Interface ────────────────────────────────────────────────────────────────

class IArmChannel {
public:
    virtual ~IArmChannel() = default;

    // Start the background UDP thread.
    virtual void start() = 0;

    // Stop the background thread and close sockets.
    virtual void stop() = 0;

    // Send a command to the arm (non-blocking, queued for next send cycle).
    virtual void sendCommand(const ArmCommand& cmd) = 0;

    // Get the most recently received arm state.
    virtual ArmState getState() = 0;

    // True if a new state packet arrived since the last getState() call.
    virtual bool hasNewState() const = 0;

    // True if arm state packets are arriving within the keep-alive window.
    virtual bool isAlive() const = 0;

    // Human-readable name for logging (e.g. "arm_left", "mock_right")
    virtual const char* name() const = 0;
};
