# PID Controller Module

A standalone C++ implementation of a PID (Proportional-Integral-Derivative) controller for control systems applications.

## Features

- **Proportional (P)**: Responds to current error
- **Integral (I)**: Eliminates steady-state error by accumulating past errors
- **Derivative (D)**: Predicts future error based on rate of change
- **Output Limiting**: Configurable min/max output bounds
- **Anti-windup**: Prevents integral windup when output is saturated
- **Tunable Parameters**: Easy to adjust Kp, Ki, Kd gains
- **State Management**: Reset and inspect internal state

## Building

```bash
# From the kalman-test directory
mkdir -p build
cd build
cmake ..
make pid-test
```

## Running the Test

```bash
./pid-test
```

The test program demonstrates the PID controller tracking a setpoint by controlling a simulated first-order system.

## Usage Example

```cpp
#include "pid.hpp"

// Create PID controller with gains and setpoint
PIDController pid(0.8, 0.3, 0.1, 100.0);

// Optional: Set output limits
pid.setOutputLimits(-50.0, 50.0);

// Control loop
double dt = 0.1;  // 100ms time step
double process_value = 0.0;

for (int i = 0; i < 100; ++i) {
    // Get control signal
    double control = pid.update(process_value, dt);
    
    // Apply control to your system
    // ... (system-specific code)
    
    // Update process_value based on system response
}

// Change setpoint during operation
pid.setSetpoint(50.0);

// Reset controller state
pid.reset();
```

## API Reference

### Constructor
```cpp
PIDController(double kp, double ki, double kd, double setpoint = 0.0)
```

### Main Update Method
```cpp
double update(double measured_value, double dt = 1.0)
```
Returns the control output based on the measured value and time step.

### Configuration Methods
- `void setSetpoint(double setpoint)` - Set target value
- `void setTunings(double kp, double ki, double kd)` - Update PID gains
- `void setOutputLimits(double min, double max)` - Set output bounds
- `void clearOutputLimits()` - Remove output bounds
- `void reset()` - Clear integral and derivative state

### Inspection Methods
- `double getSetpoint()` - Get current setpoint
- `double getError()` - Get last error value
- `double getIntegral()` - Get accumulated integral
- `void getComponents(double& p, double& i, double& d)` - Get P, I, D terms
- `void getTunings(double& kp, double& ki, double& kd)` - Get PID gains

## Tuning Guidelines

1. **Start with P only**: Set Ki=0, Kd=0, increase Kp until oscillation
2. **Add I**: Increase Ki to eliminate steady-state error
3. **Add D**: Increase Kd to reduce overshoot and oscillation

### Ziegler-Nichols Method
1. Set Ki=0, Kd=0
2. Increase Kp until sustained oscillation (critical gain Ku)
3. Measure oscillation period Tu
4. Set: Kp = 0.6*Ku, Ki = 1.2*Ku/Tu, Kd = 0.075*Ku*Tu

## License

Standalone module - no dependencies on other code in the repository.
