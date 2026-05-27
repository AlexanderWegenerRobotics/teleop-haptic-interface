#pragma once

// ─── passivity_controller.hpp ─────────────────────────────────────────────────
// Energy-bounding passivity observer + controller for haptic teleoperation.
//
// Background
// ──────────
// When force feedback is rendered from delayed robot state the haptic
// controller can inject energy that has no physical origin — the classic
// instability source under network latency.  The standard remedy is the
// Passivity Observer / Passivity Controller (PO/PC) [Ryu 2004, Hannaford 2002]:
//
//   1. PO (observer): track the net energy flowing OUT of the haptic device
//      to the operator's hand:
//          E_obs(t) = ∫ F_haptic(s) · v_device(s) ds
//      A passive environment can only absorb energy, so E_obs must be ≥ 0.
//      When E_obs < 0 the controller is generating energy — a passivity
//      violation.
//
//   2. PC (controller): when E_obs < 0, inject just enough variable damping
//      to restore passivity:
//          b(t) = max(0, -E_obs / (||v||² · dt))
//          F_damping = -b(t) · v_device
//
// The variable damping adapts automatically to the current latency and
// contact stiffness — no manual tuning per scenario.
//
// Usage
// ─────
//   PassivityController pc(50.0);   // max_damping_gain [Ns/m]
//   // Inside the 1 kHz haptic loop:
//   Eigen::Matrix<double,6,1> F_pc = pc.update(F_haptic, twist_device, dt);
//   F_total = F_haptic + F_pc;
//
// Notes
// ─────
// - Force and velocity are in the DEVICE frame (before world-frame transform).
// - The controller is split into linear (3D) and rotational (3D) parts that
//   share one energy account.  Separating them can give finer tuning but is
//   rarely necessary in practice.
// - Call reset() when transitioning from IDLE/HOMING to ENGAGED so stale
//   energy estimates do not contaminate the first contact.
// ─────────────────────────────────────────────────────────────────────────────

#include <Eigen/Core>
#include <algorithm>

class PassivityController {
public:
    // max_damping_gain: upper bound on injected damping coefficient [Ns/m for
    // linear, Nms/rad for rotational].  Set from config passivity.max_damping_gain.
    explicit PassivityController(double max_damping_gain = 50.0);

    // Compute the passivity-correcting damping wrench.
    //
    //   F_haptic  : wrench the haptic controller WOULD apply [N, Nm], device frame
    //   twist     : current device velocity [m/s, rad/s], device frame
    //   dt        : control time step [s]
    //
    // Returns an additional wrench to add to F_haptic so that the total
    // output remains passive.  Typically negative (dissipative).
    Eigen::Matrix<double, 6, 1> update(
        const Eigen::Matrix<double, 6, 1>& F_haptic,
        const Eigen::Matrix<double, 6, 1>& twist,
        double dt);

    // Reset the energy accumulator.  Call this on every ENGAGED transition.
    void reset();

    // Diagnostics: current energy observation [J, Nm·rad] and injected damping
    double energyObservation()    const { return E_obs_; }
    double injectedDampingLinear() const { return b_linear_; }
    double injectedDampingAngular() const { return b_angular_; }

    void setMaxDampingGain(double gain) { max_damping_gain_ = gain; }

private:
    double max_damping_gain_ = 50.0; // Ns/m (and Nms/rad for rotational part)
    double E_obs_            = 0.0;  // observed energy [J]
    double b_linear_         = 0.0;  // last injected linear damping [Ns/m]
    double b_angular_        = 0.0;  // last injected angular damping [Nms/rad]
};
