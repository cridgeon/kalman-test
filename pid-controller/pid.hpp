#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <limits>
#include <utility>

/**
 * @brief PID Controller for control systems
 * 
 * A standalone proportional-integral-derivative controller implementation
 * that can be used for various control applications.
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param setpoint Desired target value (default: 0.0)
     */
    PIDController(double kp, double ki, double kd, double setpoint = 0.0);

    /**
     * @brief Calculate PID output based on measured value
     * 
     * @param measured_value Current process variable
     * @param dt Time step (optional, use 1.0 for discrete updates)
     * @return double Control output value
     */
    double update(double measured_value, double dt = 1.0);

    /**
     * @brief Reset the controller state (integral and derivative terms)
     */
    void reset();

    /**
     * @brief Set a new target setpoint
     * 
     * @param setpoint New desired target value
     */
    void setSetpoint(double setpoint);

    /**
     * @brief Get the current setpoint
     * 
     * @return double Current setpoint value
     */
    double getSetpoint() const;

    /**
     * @brief Update PID tuning parameters
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setTunings(double kp, double ki, double kd);

    /**
     * @brief Get current tuning parameters
     * 
     * @param kp Output proportional gain
     * @param ki Output integral gain
     * @param kd Output derivative gain
     */
    void getTunings(double& kp, double& ki, double& kd) const;

    /**
     * @brief Set output limits (with anti-windup)
     * 
     * @param min_output Minimum output value
     * @param max_output Maximum output value
     */
    void setOutputLimits(double min_output, double max_output);

    /**
     * @brief Remove output limits
     */
    void clearOutputLimits();

    /**
     * @brief Get the last calculated error
     * 
     * @return double Last error value (setpoint - measured_value)
     */
    double getError() const;

    /**
     * @brief Get the current integral term value
     * 
     * @return double Accumulated integral
     */
    double getIntegral() const;

    /**
     * @brief Get individual PID component values for debugging
     * 
     * @param p_term Output proportional term
     * @param i_term Output integral term
     * @param d_term Output derivative term
     */
    void getComponents(double& p_term, double& i_term, double& d_term) const;

private:
    // PID gains
    double kp_;
    double ki_;
    double kd_;

    // Setpoint (target value)
    double setpoint_;

    // Internal state
    double integral_;
    double last_error_;

    // Output limits
    bool use_limits_;
    double min_output_;
    double max_output_;

    // Last calculated components (for debugging)
    double last_p_term_;
    double last_i_term_;
    double last_d_term_;
};

#endif // PID_CONTROLLER_HPP
