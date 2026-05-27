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

    // Passivity observer: track net energy delivered to the operator.
    // F·v > 0 means force aligned with motion — system adds energy (active, bad).
    // F·v < 0 means force opposes motion — operator does work on system (passive, fine).
    // E_obs > 0 means cumulative net energy has flowed TO the operator — violation.
    // Under latency the delayed force can drift in-phase with velocity, pushing
    // E_obs positive; we drain the excess with variable damping.
    E_obs_ += F_haptic.dot(twist) * dt;

    Eigen::Matrix<double, 6, 1> F_damp = Eigen::Matrix<double, 6, 1>::Zero();

    if (E_obs_ > 0.0) {
        const Eigen::Vector3d v_lin = twist.head<3>();
        const Eigen::Vector3d v_ang = twist.tail<3>();
        const double v2_lin = v_lin.squaredNorm();
        const double v2_ang = v_ang.squaredNorm();
        const double v2_tot = v2_lin + v2_ang;

        if (v2_tot < 1e-12) {
            b_linear_  = 0.0;
            b_angular_ = 0.0;
        } else {
            // Minimum damping to drain the surplus in one step, clamped to max.
            double b_needed = E_obs_ / (v2_tot * dt);
            b_needed        = std::min(b_needed, max_damping_gain_);

            b_linear_  = b_needed;
            b_angular_ = b_needed;

            F_damp.head<3>() = -b_linear_  * v_lin;
            F_damp.tail<3>() = -b_angular_ * v_ang;

            // Remove drained energy from the observation
            double P_dissipated = b_linear_ * v2_lin + b_angular_ * v2_ang;
            E_obs_ -= P_dissipated * dt;
            if (E_obs_ < 0.0) E_obs_ = 0.0;  // don't overshoot
        }
    } else {
        b_linear_  = 0.0;
        b_angular_ = 0.0;
    }

    return F_damp;
}
