#pragma once

// ─── common.hpp ───────────────────────────────────────────────────────────────
// Shared protocol types for the teleop system.
//
// !! IMPORTANT: This file MUST remain binary-compatible with the avatar
//    project's src/network/common.hpp.  Any change to enum values or struct
//    layout here must be mirrored there (and vice-versa).
//    All wire structs are packed with #pragma pack(push,1) — no hidden padding.
//    The UdpStream template verifies packets via sizeof(TMsg), so a layout
//    mismatch results in silently dropped packets.
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdint>
#include <chrono>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// ─── System state machine ─────────────────────────────────────────────────────
enum class SysState : uint8_t {
    OFFLINE    = 0,
    IDLE       = 1,
    HOMING     = 2,
    AWAITING   = 3,
    ENGAGED    = 4,
    PAUSED     = 5,
    FAULT      = 6,
    STOP       = 7,
    RECOVERING = 8,
    UNDEFINED  = 255
};

inline const char* sysStateStr(SysState s) {
    switch (s) {
        case SysState::OFFLINE:    return "OFFLINE";
        case SysState::IDLE:       return "IDLE";
        case SysState::HOMING:     return "HOMING";
        case SysState::AWAITING:   return "AWAITING";
        case SysState::ENGAGED:    return "ENGAGED";
        case SysState::PAUSED:     return "PAUSED";
        case SysState::FAULT:      return "FAULT";
        case SysState::STOP:       return "STOP";
        case SysState::RECOVERING: return "RECOVERING";
        default:                   return "UNKNOWN";
    }
}

// ─── Device / role enums ──────────────────────────────────────────────────────
enum class DeviceId : uint8_t {
    LEFT_ARM  = 1,
    RIGHT_ARM = 2,
    HEAD      = 3,
    AVATAR    = 4
};

enum class TransmissionRole : uint8_t {
    ARM    = 0,
    HEAD   = 1,
    AVATAR = 2
};

// ─── Fault codes ──────────────────────────────────────────────────────────────
enum class FaultCode : uint8_t {
    NONE                = 0,
    JOINT_LIMIT         = 1,
    JOINT_LOCKED        = 2,
    HIGH_EXTERNAL_FORCE = 3,
    VELOCITY_LIMIT      = 4,
    IMPLAUSIBLE_COMMAND = 5,
    COMM_LOSS           = 6,
    INTERNAL_ERROR      = 7,
    HMD_NOT_WORN        = 8,
    COLLISION_RISK      = 9,
    WORKSPACE_LIMIT     = 10
};

// ─── Timestamp helper ─────────────────────────────────────────────────────────
inline uint64_t timestamp_ns() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

// ─── Eigen type aliases ───────────────────────────────────────────────────────
using Matrix6x7 = Eigen::Matrix<double, 6, 7>;
using Matrix7   = Eigen::Matrix<double, 7, 7>;
using Matrix4   = Eigen::Matrix<double, 4, 4>;
using Vector7   = Eigen::Matrix<double, 7, 1>;
using Vector2   = Eigen::Matrix<double, 2, 1>;

// ─── Wire structs (packed — no padding bytes) ─────────────────────────────────
// All binary UDP messages use #pragma pack(push,1).
// MsgHeader: 4 + 8 + 1 + 1 + 1 = 15 bytes
// ArmCommandMsg: 15 + 12 + 16 + 4 = 47 bytes
// ArmStateMsg:   15 + 12 + 16 + 28 + 28 + 1 = 100 bytes

#pragma pack(push, 1)

struct MsgHeader {
    uint32_t  sequence     = 0;
    uint64_t  timestamp_ns = 0;
    SysState  state        = SysState::OFFLINE;
    FaultCode fault_code   = FaultCode::NONE;
    DeviceId  device_id    = DeviceId::AVATAR;
};
static_assert(sizeof(MsgHeader) == 15,
    "MsgHeader must be 15 bytes (packed)");

struct ArmCommandMsg {
    MsgHeader header;
    float     position[3];   // delta from origin, world frame [m]
    float     quaternion[4]; // delta rotation [w, x, y, z]
    float     gripper;       // 0.0 = open, 1.0 = closed
};
static_assert(sizeof(ArmCommandMsg) == 47,
    "ArmCommandMsg must be 47 bytes (packed)");

struct ArmStateMsg {
    MsgHeader header;
    float     position[3];        // EE position in world frame [m]
    float     quaternion[4];      // EE orientation [w, x, y, z]
    float     joint_positions[7]; // Franka joint angles [rad]
    float     tau_ext[7];         // External torques [Nm]
    uint8_t   recovering;         // 1 if arm is in recovery mode
};
static_assert(sizeof(ArmStateMsg) == 100,
    "ArmStateMsg must be 100 bytes (packed)");

struct HeadCommandMsg {
    MsgHeader header;
    float     pan;
    float     tilt;
};

struct HeadStateMsg {
    MsgHeader header;
    float     pan;
    float     tilt;
};

#pragma pack(pop)

// ─── YAML helper ─────────────────────────────────────────────────────────────
template<int N>
Eigen::Matrix<double, N, 1> yamlToVector(const YAML::Node& node) {
    auto vec = node.as<std::vector<double>>();
    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(vec.data());
}
