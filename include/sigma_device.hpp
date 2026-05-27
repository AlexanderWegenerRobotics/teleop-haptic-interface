#pragma once

#ifdef WITH_SIGMA

// ─── sigma_device.hpp ─────────────────────────────────────────────────────────
// Concrete IHapticDevice implementation for Force Dimension Sigma 7.
// Only compiled when WITH_SIGMA is defined (requires DHD SDK).
// ─────────────────────────────────────────────────────────────────────────────

#include "haptic_device.hpp"
#include <Eigen/Geometry>
#include <string>

class SigmaDevice : public IHapticDevice {
public:
    explicit SigmaDevice(int device_id);

    bool        open()          override;
    void        close()         override;
    HapticState readState()     override;
    void        setForce(const Eigen::Vector3d& force,
                         const Eigen::Vector3d& torque) override;
    void        enableForce(bool enable) override;
    void        zero()          override;
    bool        isConnected() const override { return connected_; }
    const char* name()        const override { return name_.c_str(); }

private:
    int                  device_id_;
    bool                 connected_      = false;
    bool                 force_enabled_  = false;
    std::string          name_;
    Eigen::Quaterniond   prev_orientation_;
};

#endif // WITH_SIGMA
