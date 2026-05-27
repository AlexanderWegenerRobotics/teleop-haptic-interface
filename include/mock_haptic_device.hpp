#pragma once

#include "haptic_device.hpp"
#include <chrono>
#include <cmath>
#include <string>

// Mock device — selectable motion profile via constructor.
// FREE_X:         slow sine on X (baseline pipeline / passivity test)
// CONTACT_Z:      slow sine on -Z + ±30° sweeping pitch (reach-and-tilt grasp approach)
// CONTACT_Z_DOWN: slow sine on -Z + constant -90° pitch (fixed EE-down grasp approach)
enum class MockProfile { FREE_X, CONTACT_Z, CONTACT_Z_DOWN };

class MockHapticDevice : public IHapticDevice {
public:
    explicit MockHapticDevice(int id, MockProfile profile = MockProfile::FREE_X)
        : name_("mock_" + std::to_string(id)), profile_(profile) {}

    bool open() override {
        start_ = std::chrono::high_resolution_clock::now();
        return true;
    }
    void close() override {}

    HapticState readState() override {
        double t = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - start_).count();

        constexpr double W = 0.3141592654;  // 2π × 0.05 Hz — one cycle per 20 s

        HapticState s{};
        s.orientation = Eigen::Quaterniond::Identity();
        s.is_valid    = true;

        if (profile_ == MockProfile::CONTACT_Z) {
            // ±80 mm at 0.05 Hz on -Z: with 4× scaling → ±320 mm arm travel
            // ±90° pitch about sigma Y in phase with Z — tilt peaks at max downward reach
            constexpr double AMP     = 0.080;
            constexpr double AMP_ROT = 1.5708;  // 90° in radians
            double z     = -AMP     * std::sin(W * t);
            double theta = -AMP_ROT * std::sin(W * t);  // negative: tilt down when arm descends
            s.position    = Eigen::Vector3d(0.0, 0.0, z);
            s.twist(2)    = -AMP     * W * std::cos(W * t);
            s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
            s.twist(4)    = -AMP_ROT * W * std::cos(W * t);
        } else if (profile_ == MockProfile::CONTACT_Z_DOWN) {
            // Same Z travel as CONTACT_Z but orientation ramps to -90° and holds.
            // Ramp is necessary: captureOrigin() calls zero() then readState() at t≈0,
            // so the origin is captured at Identity. The mock then ramps away from that,
            // giving a non-trivial q_delta that drives the arm to the target rotation.
            constexpr double AMP     = 0.080;
            constexpr double AMP_ROT = 1.5708;  // 90° in radians
            constexpr double RAMP_T  = 3.0;     // seconds to reach full rotation
            double theta = -AMP_ROT * std::min(t / RAMP_T, 1.0);
            s.position    = Eigen::Vector3d(0.0, 0.0, -AMP * std::sin(W * t));
            s.twist(2)    = -AMP * W * std::cos(W * t);
            s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
            s.twist(4)    = (t < RAMP_T) ? (-AMP_ROT / RAMP_T) : 0.0;
        } else {
            // ±20 mm at 0.05 Hz on X
            constexpr double AMP = 0.020;
            double x = AMP * std::sin(W * t);
            s.position = Eigen::Vector3d(x, 0.0, 0.0);
            s.twist(0) = AMP * W * std::cos(W * t);
        }

        return s;
    }

    void setForce(const Eigen::Vector3d&, const Eigen::Vector3d&) override {}
    void enableForce(bool) override {}
    void zero() override { start_ = std::chrono::high_resolution_clock::now(); }
    bool isConnected() const override { return true; }
    const char* name()    const override { return name_.c_str(); }

private:
    std::string  name_;
    MockProfile  profile_;
    std::chrono::high_resolution_clock::time_point start_;
};
