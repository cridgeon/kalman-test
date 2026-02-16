#include "pid.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

/**
 * @brief Simple test/demo program for the PID controller
 * 
 * Simulates a first-order system and uses PID to control it to a setpoint
 */
int main() {
    std::cout << "PID Controller Test\n";
    std::cout << "===================\n\n";

    // Create a PID controller
    // Tuning: Kp=0.8, Ki=0.3, Kd=0.1
    double setpoint = 100.0;
    PIDController pid(0.8, 0.3, 0.1, setpoint);
    
    // Set output limits to prevent excessive control effort
    pid.setOutputLimits(-50.0, 50.0);

    // Simulation parameters
    double dt = 0.1; // Time step (100ms)
    int steps = 200; // Number of simulation steps
    
    // System state (simple first-order system)
    double process_value = 0.0;  // Current value
    double system_tau = 2.0;     // Time constant
    
    std::cout << "Setpoint: " << setpoint << "\n";
    std::cout << "Initial value: " << process_value << "\n\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Step\tTime\tValue\tError\tControl\tP\tI\tD\n";
    std::cout << "------------------------------------------------------------\n";

    // Simulation loop
    for (int i = 0; i < steps; ++i) {
        double time = i * dt;
        
        // Get control signal from PID
        double control = pid.update(process_value, dt);
        
        // Simulate first-order system response: dy/dt = (u - y) / tau
        // where u is the control input and y is the process value
        double derivative = (control - process_value) / system_tau;
        process_value += derivative * dt;
        
        // Get PID components for display
        double p_term, i_term, d_term;
        pid.getComponents(p_term, i_term, d_term);
        double error = pid.getError();
        
        // Print every 10 steps
        if (i % 10 == 0) {
            std::cout << i << "\t"
                      << time << "\t"
                      << process_value << "\t"
                      << error << "\t"
                      << control << "\t"
                      << p_term << "\t"
                      << i_term << "\t"
                      << d_term << "\n";
        }
    }
    
    std::cout << "\n";
    
    // Test setpoint change
    std::cout << "Changing setpoint to 50.0...\n\n";
    pid.setSetpoint(50.0);
    
    std::cout << "Step\tTime\tValue\tError\tControl\n";
    std::cout << "------------------------------------\n";
    
    for (int i = 0; i < 100; ++i) {
        double time = (steps + i) * dt;
        
        double control = pid.update(process_value, dt);
        double derivative = (control - process_value) / system_tau;
        process_value += derivative * dt;
        
        if (i % 10 == 0) {
            std::cout << (steps + i) << "\t"
                      << time << "\t"
                      << process_value << "\t"
                      << pid.getError() << "\t"
                      << control << "\n";
        }
    }
    
    std::cout << "\nFinal value: " << process_value << "\n";
    std::cout << "Final error: " << pid.getError() << "\n";
    
    // Test reset
    std::cout << "\nResetting controller...\n";
    pid.reset();
    std::cout << "Integral after reset: " << pid.getIntegral() << "\n";
    std::cout << "Error after reset: " << pid.getError() << "\n";

    return 0;
}
