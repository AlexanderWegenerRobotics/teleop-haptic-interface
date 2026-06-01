#pragma once

#include <Eigen/Core>
#include <algorithm>

class PassivityController {
public:

    explicit PassivityController(double max_damping_gain = 50.0);
    Eigen::Matrix<double, 6, 1> update(const Eigen::Matrix<double, 6, 1>& F_haptic, const Eigen::Matrix<double, 6, 1>& twist,double dt);

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
