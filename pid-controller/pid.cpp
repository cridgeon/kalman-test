#include "pid.hpp"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double setpoint)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , setpoint_(setpoint)
    , integral_(0.0)
    , last_error_(0.0)
    , use_limits_(false)
    , min_output_(-std::numeric_limits<double>::infinity())
    , max_output_(std::numeric_limits<double>::infinity())
    , last_p_term_(0.0)
    , last_i_term_(0.0)
    , last_d_term_(0.0)
{
}

double PIDController::update(double measured_value, double dt) {
    // Calculate error
    double error = setpoint_ - measured_value;

    // Proportional term
    last_p_term_ = kp_ * error;

    // Integral term (accumulate error over time)
    integral_ += error * dt;
    last_i_term_ = ki_ * integral_;

    // Derivative term (rate of change of error)
    double derivative = (error - last_error_) / dt;
    last_d_term_ = kd_ * derivative;

    // Calculate total output
    double output = last_p_term_ + last_i_term_ + last_d_term_;

    // Apply output limits if set
    if (use_limits_) {
        // Clamp output to limits
        double unclamped_output = output;
        output = std::max(min_output_, std::min(max_output_, output));

        // Anti-windup: if output is saturated, prevent integral windup
        if (output != unclamped_output) {
            // Back-calculate integral to prevent windup
            integral_ -= error * dt;
            last_i_term_ = ki_ * integral_;
        }
    }

    // Update state for next iteration
    last_error_ = error;

    return output;
}

void PIDController::reset() {
    integral_ = 0.0;
    last_error_ = 0.0;
    last_p_term_ = 0.0;
    last_i_term_ = 0.0;
    last_d_term_ = 0.0;
}

void PIDController::setSetpoint(double setpoint) {
    setpoint_ = setpoint;
}

double PIDController::getSetpoint() const {
    return setpoint_;
}

void PIDController::setTunings(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::getTunings(double& kp, double& ki, double& kd) const {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PIDController::setOutputLimits(double min_output, double max_output) {
    if (min_output > max_output) {
        return; // Invalid limits
    }
    
    use_limits_ = true;
    min_output_ = min_output;
    max_output_ = max_output;

    // Clamp current integral if needed
    if (use_limits_) {
        double i_term = ki_ * integral_;
        if (i_term < min_output_) {
            integral_ = min_output_ / ki_;
        } else if (i_term > max_output_) {
            integral_ = max_output_ / ki_;
        }
    }
}

void PIDController::clearOutputLimits() {
    use_limits_ = false;
    min_output_ = -std::numeric_limits<double>::infinity();
    max_output_ = std::numeric_limits<double>::infinity();
}

double PIDController::getError() const {
    return last_error_;
}

double PIDController::getIntegral() const {
    return integral_;
}

void PIDController::getComponents(double& p_term, double& i_term, double& d_term) const {
    p_term = last_p_term_;
    i_term = last_i_term_;
    d_term = last_d_term_;
}
