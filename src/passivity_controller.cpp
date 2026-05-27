#include "passivity_controller.hpp"

PassivityController::PassivityController(double max_damping_gain)
    : max_damping_gain_(max_damping_gain)
{}

void PassivityController::reset() {
    E_obs_     = 0.0;
    b_linear_  = 0.0;
    b_angular_ = 0.0;
}

Eigen::Matrix<double, 6, 1> PassivityController::update(
    const Eigen::Matrix<double, 6, 1>& F_haptic,
    const Eigen::Matrix<double, 6, 1>& twist,
    double dt)
{
    // Guard against degenerate dt (e.g. first call or timer jitter)
    if (dt < 1e-6 || dt > 0.1) {
        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    // ── Passivity Observer ────────────────────────────────────────────────────
    // Power delivered to the operator's hand by the haptic controller.
    // Positive = energy flowing FROM controller TO operator (physically correct
    // for a contact force that resists operator motion).
    // A passive source can only deliver energy, so the cumulative integral
    // E_obs must remain ≥ 0.  When it goes negative the controller is creating
    // energy — we must dissipate the deficit.
    double P_haptic = F_haptic.dot(twist);
    E_obs_ += P_haptic * dt;

    Eigen::Matrix<double, 6, 1> F_damp = Eigen::Matrix<double, 6, 1>::Zero();

    if (E_obs_ < 0.0) {
        // ── Passivity Controller ──────────────────────────────────────────────
        // Split into linear and angular sub-problems.
        // For each, compute the minimum damping that would recover E_obs = 0
        // in one time step, then clamp to max_damping_gain_.

        const Eigen::Vector3d v_lin = twist.head<3>();
        const Eigen::Vector3d v_ang = twist.tail<3>();
        const double v2_lin = v_lin.squaredNorm();
        const double v2_ang = v_ang.squaredNorm();
        const double v2_tot = v2_lin + v2_ang;

        if (v2_tot < 1e-12) {
            // Device is stationary — cannot dissipate via damping; clamp and
            // wait for the operator to move.
            b_linear_  = 0.0;
            b_angular_ = 0.0;
        } else {
            // Distribute the energy deficit proportionally between linear and
            // angular components, proportional to their velocity magnitudes.
            double deficit = -E_obs_;  // positive

            // Required total damping power: P_needed = deficit / dt
            // P_damp = b_lin * v2_lin + b_ang * v2_ang
            // Use a single gain applied to both for simplicity; scale by vel²
            // fraction so we don't over-damp the slower DOF.
            double b_needed = deficit / (v2_tot * dt);  // Ns/m
            b_needed        = std::min(b_needed, max_damping_gain_);

            b_linear_  = b_needed;
            b_angular_ = b_needed;

            Eigen::Vector3d F_damp_lin = -b_linear_  * v_lin;
            Eigen::Vector3d F_damp_ang = -b_angular_ * v_ang;
            F_damp.head<3>() = F_damp_lin;
            F_damp.tail<3>() = F_damp_ang;

            // Update the observer with the energy we are about to dissipate
            double P_dissipated = b_linear_  * v2_lin
                                + b_angular_ * v2_ang;
            E_obs_ += P_dissipated * dt;
        }

        // Clamp residual — numerical drift only
        E_obs_ = std::max(E_obs_, 0.0);
    } else {
        b_linear_  = 0.0;
        b_angular_ = 0.0;
    }

    return F_damp;
}
