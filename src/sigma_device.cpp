#ifdef WITH_SIGMA

#include "sigma_device.hpp"
#include <dhdc.h>
#include <iostream>
#include <cstring>

SigmaDevice::SigmaDevice(int device_id)
    : device_id_(device_id)
    , prev_orientation_(Eigen::Quaterniond::Identity())
{
    name_ = "sigma_" + std::to_string(device_id);
}

bool SigmaDevice::open() {
    if (dhdOpenID(device_id_) < 0) {
        std::cerr << "[SigmaDevice:" << device_id_ << "] Failed to open: " << dhdErrorGetLastStr() << "\n";
        return false;
    }
    connected_ = true;

    // Release brakes; device ready to read but force disabled until enableForce()
    if(dhdSetBrakes(DHD_OFF, device_id_) < 0){
        std::cout << "[ERROR]: Failed to release breaks" << std::endl;
        return;
    }
    if(dhdEnableForce(DHD_OFF, device_id_) < 0){
        std::cout << "[ERROR]: Failed to enable force" << std::endl;
        return;
    }

    std::cout << "[SigmaDevice:" << device_id_ << "] Opened: " << dhdGetSystemName(device_id_) << "\n";
    return true;
}

void SigmaDevice::close() {
    if (!connected_) return;
    zero();
    dhdEnableForce(DHD_OFF, device_id_);
    dhdSetBrakes(DHD_ON, device_id_);
    dhdClose(device_id_);
    connected_     = false;
    force_enabled_ = false;
}

HapticState SigmaDevice::readState() {
    HapticState s{};

    double px, py, pz;
    double Rm[3][3];
    if (dhdGetPositionAndOrientationFrame(&px, &py, &pz, Rm, device_id_) < 0) {
        std::cout << "[WARNING]: " << name_ << " not valid anymore" << std::endl;
        s.is_valid = false;
        return s;
    }

    s.position = Eigen::Vector3d(px, py, pz);

    // Build rotation from DHD 3×3 column-major array
    Eigen::Matrix3d R;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            R(row, col) = Rm[row][col];
    Eigen::Quaterniond q(R);
    q.normalize();

    // Quaternion continuity — flip to maintain shortest path
    if (q.dot(prev_orientation_) < 0.0) q.coeffs() *= -1.0;
    prev_orientation_ = q;
    s.orientation = q;

    // Linear + angular velocity
    double vx, vy, vz, wx, wy, wz;
    dhdGetLinearVelocity(&vx, &vy, &vz, device_id_);
    dhdGetAngularVelocityRad(&wx, &wy, &wz, device_id_);
    s.twist << vx, vy, vz, wx, wy, wz;

    // Gripper
    double grip_angle_deg = 0.0;
    dhdGetGripperAngleDeg(&grip_angle_deg, device_id_);
    s.gripper_angle  = grip_angle_deg;
    // Threshold: < 15° = closed (tune to taste)
    s.gripper_closed = (grip_angle_deg < 15.0);

    s.is_valid = true;
    return s;
}

void SigmaDevice::setForce(const Eigen::Vector3d& force, const Eigen::Vector3d& torque) {
    if (!connected_ || !force_enabled_) return;
    dhdSetForceAndTorqueAndGripperForce(force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z(), 0.0, device_id_);
}

void SigmaDevice::enableForce(bool enable) {
    if (!connected_) return;
    if (enable) {
        // Always zero before enabling to avoid jumps
        dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, device_id_);
        dhdEnableForce(DHD_ON, device_id_);
    } else {
        dhdEnableForce(DHD_OFF, device_id_);
    }
    force_enabled_ = enable;
}

void SigmaDevice::zero() {
    if (!connected_) return;
    dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, device_id_);
}

#endif // WITH_SIGMA
