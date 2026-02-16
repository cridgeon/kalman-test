#ifndef TEST_TRACK_HPP
#define TEST_TRACK_HPP

#include <cmath>
#include <vector>

/**
 * TestTrack - Generates a realistic car path with acceleration and deceleration
 * 
 * The track is a figure-8 loop with straightaways and curves.
 * The car accelerates on straights and decelerates through curves realistically.
 */
class TestTrack {
public:
    struct Position {
        float x;
        float y;
        float vx;  // velocity x
        float vy;  // velocity y
    };

private:
    float time_;                    // Current time in seconds
    float max_speed_;               // Maximum speed in units/second
    float min_speed_;               // Minimum speed in units/second (for curves)
    float acceleration_;            // Acceleration rate
    float deceleration_;            // Deceleration rate (braking)
    float current_speed_;           // Current speed
    float scale_;                   // Scale factor for the track size
    float center_x_;                // Center X position
    float center_y_;                // Center Y position
    
    // Track parameters
    static constexpr float TRACK_PERIOD = 30.0f;  // Time to complete one loop (seconds)
    
    /**
     * Calculate the raw path position at time t (figure-8 pattern)
     */
    void getRawPath(float t, float& x, float& y) const {
        // Normalized time [0, 1] for one complete loop
        float t_norm = fmod(t, TRACK_PERIOD) / TRACK_PERIOD;
        float angle = t_norm * 2.0f * M_PI;
        
        // Figure-8 (lemniscate) path
        // x = scale * sin(angle)
        // y = scale * sin(angle) * cos(angle)
        x = center_x_ + scale_ * sin(angle);
        y = center_y_ + scale_ * sin(angle) * cos(angle);
    }
    
    /**
     * Calculate the curvature at the current position
     * Higher curvature = tighter turn = need to slow down more
     */
    float getCurvature(float t) const {
        float dt = 0.01f;
        float x1, y1, x2, y2, x3, y3;
        
        getRawPath(t - dt, x1, y1);
        getRawPath(t, x2, y2);
        getRawPath(t + dt, x3, y3);
        
        // Calculate curvature using finite differences
        float dx = (x3 - x1) / (2.0f * dt);
        float dy = (y3 - y1) / (2.0f * dt);
        float ddx = (x3 - 2.0f * x2 + x1) / (dt * dt);
        float ddy = (y3 - 2.0f * y2 + y1) / (dt * dt);
        
        float speed_sq = dx * dx + dy * dy;
        if (speed_sq < 1e-6f) return 0.0f;
        
        float curvature = fabs(dx * ddy - dy * ddx) / pow(speed_sq, 1.5f);
        return curvature;
    }
    
    /**
     * Calculate target speed based on curvature
     * Tighter curves require lower speeds
     */
    float getTargetSpeed(float t) const {
        float curvature = getCurvature(t);
        
        // Map curvature to speed (higher curvature = lower speed)
        // Use exponential decay for realistic feel
        float curvature_factor = exp(-curvature * 5.0f * scale_);
        float target = min_speed_ + (max_speed_ - min_speed_) * curvature_factor;
        
        return target;
    }

public:
    /**
     * Constructor
     * @param scale Size of the track (larger = bigger track)
     * @param center_x X coordinate of track center (normalized -1 to 1)
     * @param center_y Y coordinate of track center (normalized -1 to 1)
     * @param max_speed Maximum speed (units per second)
     * @param min_speed Minimum speed for curves (units per second)
     */
    TestTrack(float scale = 0.6f, float center_x = 0.0f, float center_y = 0.0f,
              float max_speed = 2.0f, float min_speed = 0.5f)
        : time_(0.0f)
        , max_speed_(max_speed)
        , min_speed_(min_speed)
        , acceleration_(1.5f)      // Units/second^2
        , deceleration_(2.5f)      // Units/second^2 (can brake harder than accelerate)
        , current_speed_(min_speed)
        , scale_(scale)
        , center_x_(center_x)
        , center_y_(center_y)
    {
    }
    
    /**
     * Update the car's position for the next frame
     * @param dt Time delta in seconds
     * @return Current position and velocity
     */
    Position update(float dt) {
        // Get target speed for current position
        float target_speed = getTargetSpeed(time_);
        
        // Adjust current speed towards target with realistic acceleration/deceleration
        if (current_speed_ < target_speed) {
            // Accelerate
            current_speed_ += acceleration_ * dt;
            if (current_speed_ > target_speed) {
                current_speed_ = target_speed;
            }
        } else if (current_speed_ > target_speed) {
            // Decelerate (brake)
            current_speed_ -= deceleration_ * dt;
            if (current_speed_ < target_speed) {
                current_speed_ = target_speed;
            }
        }
        
        // Ensure speed stays within bounds
        current_speed_ = std::max(min_speed_, std::min(max_speed_, current_speed_));
        
        // Calculate the distance to travel this frame based on current speed
        float distance = current_speed_ * dt;
        
        // Convert distance to time increment along the path
        // This ensures constant speed along the path
        float path_speed = 1.0f / TRACK_PERIOD;  // Base path speed
        float time_increment = distance * path_speed;
        
        // Update time (wraps around for continuous loop)
        time_ += time_increment;
        if (time_ > TRACK_PERIOD) {
            time_ = fmod(time_, TRACK_PERIOD);
        }
        
        // Get current position
        Position pos;
        getRawPath(time_, pos.x, pos.y);
        
        // Calculate velocity by looking ahead slightly
        float t_ahead = time_ + 0.016f;  // Look ahead ~1 frame
        float x_ahead, y_ahead;
        getRawPath(t_ahead, x_ahead, y_ahead);
        
        pos.vx = (x_ahead - pos.x) / 0.016f;
        pos.vy = (y_ahead - pos.y) / 0.016f;
        
        return pos;
    }
    
    /**
     * Reset the track to starting position
     */
    void reset() {
        time_ = 0.0f;
        current_speed_ = min_speed_;
    }
    
    /**
     * Get current speed
     */
    float getCurrentSpeed() const {
        return current_speed_;
    }
    
    /**
     * Get current time on track
     */
    float getTime() const {
        return time_;
    }
    
    /**
     * Set track scale
     */
    void setScale(float scale) {
        scale_ = scale;
    }
    
    /**
     * Set speed limits
     */
    void setSpeedLimits(float max_speed, float min_speed) {
        max_speed_ = max_speed;
        min_speed_ = min_speed;
    }
    
    /**
     * Set acceleration parameters
     */
    void setAcceleration(float accel, float decel) {
        acceleration_ = accel;
        deceleration_ = decel;
    }
};

#endif // TEST_TRACK_HPP
