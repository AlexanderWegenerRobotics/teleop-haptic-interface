#pragma once

#include "haptic_device.hpp"
#include <chrono>
#include <cmath>
#include <string>

// Generates slow sinusoidal X motion for pipeline testing without real hardware.
class MockHapticDevice : public IHapticDevice {
public:
    explicit MockHapticDevice(int id) : name_("mock_" + std::to_string(id)) {}

    bool open() override {
        start_ = std::chrono::high_resolution_clock::now();
        return true;
    }
    void close() override {}

    HapticState readState() override {
        double t = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - start_).count();

        // ±20 mm at 0.1 Hz on X axis
        constexpr double AMP = 0.020;
        constexpr double W   = 0.6283185307;  // 2π × 0.1 Hz

        HapticState s{};
        s.position    = Eigen::Vector3d(AMP * std::sin(W * t), 0.0, 0.0);
        s.twist(0)    = AMP * W * std::cos(W * t);  // dx/dt
        s.orientation = Eigen::Quaterniond::Identity();
        s.is_valid    = true;
        return s;
    }

    void setForce(const Eigen::Vector3d&, const Eigen::Vector3d&) override {}
    void enableForce(bool) override {}
    void zero()            override {}
    bool isConnected() const override { return true; }
    const char* name()    const override { return name_.c_str(); }

private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
};
