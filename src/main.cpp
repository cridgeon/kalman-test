#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <queue>
#include <chrono>

#include "kalman.hpp"
#include <random>
#include "pid-controller/pid.hpp"
#include "test_track.hpp"
#include "rendering_system.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "shader/all.hpp"

using namespace std::chrono;
#define GET_SYS_MILLIS() duration_cast<milliseconds>(system_clock::now().time_since_epoch())

static const int WINDOW_WIDTH = 1280;
static const int WINDOW_HEIGHT = 720;
static const float UPDATE_FREQ = 150.0f;
static const float MEAS_NOISE_STDDEV = 0.05f;
static const milliseconds delay_time(500);

// Shared state for circle position
struct CircleState {
    std::mutex mutex;
    float x = 0.0;
    float y = 0.0;
    float radius = 10.0f;
    float velocity_x = 0.0f;
    float velocity_y = 0.0f;
};

CircleState g_real_state;
CircleState g_measured_state;
CircleState g_reported_state;
CircleState g_estimated_state;
CircleState g_smoothed_state_temp;
CircleState g_smoothed_state;
std::atomic<bool> g_running(true);
std::mutex g_state_mutex;

KalmanFilter filter;

// Test track for realistic car movement
TestTrack test_track(0.3f, 0.5f, 0.5f, 100.0f, 80.0f);

// PID Controllers for x and y axes with more responsive gains
PIDController pid_x(15.0, 1.5, 0.75, 0.0);  // Increased Kp and Kd for faster response
PIDController pid_y(15.0, 1.5, 0.75, 0.0);

// Error tracking for moving average
const size_t ERROR_HISTORY_SIZE = 100; // Track last 100 samples
float error_measured_wt_ave = 0.0f;
float error_estimated_wt_ave = 0.0f;
float error_smoothed_wt_ave = 0.0f;
std::mutex error_mutex;


std::queue<std::pair<milliseconds, std::pair<float, float>>> position_queue;

static bool setupImGui(GLFWwindow* window, const char* glsl_version) {
     // Setup Dear ImGui context
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    
    return true;
}

static void cleanupImGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// Thread function to update circle position
void update_circle_position() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> nd(0.0f, MEAS_NOISE_STDDEV);

    while (g_running) {
        // Update real state from test track
        TestTrack::Position track_pos = test_track.update(1.0f / UPDATE_FREQ);
        track_pos.x *= WINDOW_WIDTH;
        track_pos.y *= WINDOW_HEIGHT;
        
        g_state_mutex.lock();
        g_real_state.x = track_pos.x;
        g_real_state.y = track_pos.y;
        g_real_state.velocity_x = track_pos.vx;
        g_real_state.velocity_y = track_pos.vy;
        
        g_measured_state.x = g_real_state.x + nd(gen) * WINDOW_WIDTH; // Scale noise to be more realistic
        g_measured_state.y = g_real_state.y + nd(gen) * WINDOW_HEIGHT; // Scale noise to be more realistic
        position_queue.push({GET_SYS_MILLIS(), {g_measured_state.x, g_measured_state.y}});

        milliseconds now = GET_SYS_MILLIS();
        while(!position_queue.empty() && position_queue.front().first <= now - delay_time) {
            g_reported_state.x = position_queue.front().second.first;
            g_reported_state.y = position_queue.front().second.second;
            position_queue.pop();
        }

        filter.update(Eigen::Vector2d(g_reported_state.x, g_reported_state.y));
        Eigen::VectorXd estimated = filter.state();
        g_estimated_state.x = estimated(0);
        g_estimated_state.y = estimated(1);

        // Use PID controller to control smoothed state
        // The PID takes the estimated state as input and outputs control signal
        // Update PID setpoints to follow the estimated state
        Eigen::MatrixXd A(6, 6); // System dynamics matrix
        // Constant acceleration model for car dynamics
        // State: [x, y, vx, vy, ax, ay]
        // x_new = x + vx*dt + 0.5*ax*dt²
        // vx_new = vx + ax*dt
        // ax_new = ax (with process noise)
        float dt_ = float(delay_time.count() + 650.0) / 1000.0; // Time step in seconds
        float dt2 = 0.5 * dt_ * dt_; // 0.5 * dt²
        A << 
            1, 0, dt_, 0, dt2, 0,     // x position
            0, 1, 0, dt_, 0, dt2,     // y position
            0, 0, 1, 0, dt_, 0,       // x velocity
            0, 0, 0, 1, 0, dt_,       // y velocity
            0, 0, 0, 0, 1, 0,         // x acceleration
            0, 0, 0, 0, 0, 1;         // y acceleration

        Eigen::VectorXd future = filter.predict(A);
        
        float diff = 1.0/UPDATE_FREQ;
        g_smoothed_state_temp.x = (1.0 - diff) * g_smoothed_state_temp.x + diff * future(0);
        g_smoothed_state_temp.y = (1.0 - diff) * g_smoothed_state_temp.y + diff * future(1);
        
        // Calculate PID outputs based on current smoothed position
        pid_x.setSetpoint(g_smoothed_state_temp.x);
        pid_y.setSetpoint(g_smoothed_state_temp.y);
        double dt = 1.0 / UPDATE_FREQ;
        double control_x = pid_x.update(g_smoothed_state.x, dt);
        double control_y = pid_y.update(g_smoothed_state.y, dt);
        
        // Apply control outputs to smoothed state
        g_smoothed_state.x += control_x * dt;
        g_smoothed_state.y += control_y * dt;

        // g_smoothed_state.x = g_smoothed_state_temp.x;
        // g_smoothed_state.y = g_smoothed_state_temp.y;

        // Calculate and track error between real and measured/estimated/smoothed positions
        float error_measured = std::sqrt(
            std::pow(g_real_state.x - g_measured_state.x, 2) + 
            std::pow(g_real_state.y - g_measured_state.y, 2)
        );
        float error_estimated = std::sqrt(
            std::pow(g_real_state.x - g_estimated_state.x, 2) + 
            std::pow(g_real_state.y - g_estimated_state.y, 2)
        );
        float error_smoothed = std::sqrt(
            std::pow(g_real_state.x - g_smoothed_state.x, 2) + 
            std::pow(g_real_state.y - g_smoothed_state.y, 2)
        );
        
        error_mutex.lock();
        error_measured_wt_ave = (1.0 - diff) * error_measured_wt_ave + diff * error_measured;
        error_estimated_wt_ave = (1.0 - diff) * error_estimated_wt_ave + diff * error_estimated;
        error_smoothed_wt_ave = (1.0 - diff) * error_smoothed_wt_ave + diff * error_smoothed;
        error_mutex.unlock();

        g_state_mutex.unlock(); 
        
        // Update position
        // g_real_state.x += g_real_state.velocity_x;
        // g_real_state.y += g_real_state.velocity_y;

        
        // Bounce off walls
        // if (g_real_state.x - g_real_state.radius < 0 || 
        //     g_real_state.x + g_real_state.radius > WINDOW_WIDTH) {
        //     g_real_state.velocity_x = -g_real_state.velocity_x;
        //     g_real_state.x = std::max(g_real_state.radius, 
        //                                std::min(g_real_state.x, WINDOW_WIDTH - g_real_state.radius));
        // }
        
        // if (g_real_state.y - g_real_state.radius < 0 || 
        //     g_real_state.y + g_real_state.radius > WINDOW_HEIGHT) {
        //     g_real_state.velocity_y = -g_real_state.velocity_y;
        //     g_real_state.y = std::max(g_real_state.radius, 
        //                                std::min(g_real_state.y, WINDOW_HEIGHT - g_real_state.radius));
        // }
        
        // Sleep to control update rate (~60 updates per second)
        std::this_thread::sleep_for(std::chrono::microseconds(int(1000000 / UPDATE_FREQ)));
    }
}

int main() {
    std::cout << "Multi-threaded Circle Animation with ImGui" << std::endl;
    
    // Get singleton instance and initialize rendering system
    RenderingSystem& renderer = RenderingSystem::getInstance();
    if (!renderer.initialize(WINDOW_WIDTH, WINDOW_HEIGHT, "Kalman Test - ImGui Demo")) {
        std::cerr << "Failed to initialize rendering system" << std::endl;
        return 1;
    }
    if (!setupImGui(renderer.getWindow(), renderer.getGLSLVersion())) {
        std::cerr << "Failed to setup ImGui" << std::endl;
        return false;
    }

    // Our state
    bool show_demo_window = false;
    bool show_controls = true;
    ImVec4 real_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
    ImVec4 measured_color = ImVec4(0.8f, 0.2f, 0.2f, 1.0f);
    ImVec4 reported_color = ImVec4(0.8f, 0.8f, 0.2f, 1.0f);
    ImVec4 estimated_color = ImVec4(0.2f, 0.2f, 0.8f, 1.0f);
    ImVec4 smoothed_color = ImVec4(0.8f, 0.2f, 0.8f, 1.0f);
    
    // kalman filter initialization
    int n = 6; // Number of states
    int m = 2; // Number of measurements

    double dt = 1.0/UPDATE_FREQ; // Time step
    double dt2 = 0.5 * dt * dt;  // 0.5 * dt²

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Constant acceleration model for car dynamics
    // State: [x, y, vx, vy, ax, ay]
    // Position: x_new = x + vx*dt + 0.5*ax*dt²
    // Velocity: vx_new = vx + ax*dt
    // Acceleration: ax_new = ax (with process noise to model changing acceleration)
    A << 
        1, 0, dt, 0, dt2, 0,      // x position
        0, 1, 0, dt, 0, dt2,      // y position
        0, 0, 1, 0, dt, 0,        // x velocity
        0, 0, 0, 1, 0, dt,        // y velocity
        0, 0, 0, 0, 1, 0,         // x acceleration
        0, 0, 0, 0, 0, 1;         // y acceleration
    C << 
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
    
    // Reasonable covariance matrices
    // Process noise: allow for changes in acceleration (car can accelerate/brake/turn)
    // Higher noise on acceleration states since car actively changes acceleration
    Q << 
        .0, .0, .0, .0, .0, .0,           // x position (deterministic from velocity)
        .0, .0, .0, .0, .0, .0,           // y position (deterministic from velocity)
        .0, .0, .001, .0, .0, .0,         // x velocity (some uncertainty)
        .0, .0, .0, .001, .0, .0,         // y velocity (some uncertainty)
        .0, .0, .0, .0, .01, .0,          // x acceleration (higher - car can change accel)
        .0, .0, .0, .0, .0, .01;          // y acceleration (higher - car can turn)
    R << std::pow(MEAS_NOISE_STDDEV, 2), 0, 
         0, std::pow(MEAS_NOISE_STDDEV, 2);
    P << 1.0, .0, 0, 0, 0, 0,
         0, 1.0, 0, 0, 0, 0,
         0, 0, 1.0, 0, 0, 0,
         0, 0, 0, 1.0, 0, 0,
         0, 0, 0, 0, 100.0, 0,
         0, 0, 0, 0, 0, 100.0;

    // Construct the filter
    filter = KalmanFilter(dt,A, C, Q, R, P);

    // Initialize with 6D state vector: [x, y, vx, vy, ax, ay]
    Eigen::VectorXd initial_state(6);
    initial_state << g_real_state.x, g_real_state.y, 0.0, 0.0, 0.0, 0.0;
    filter.init(1.0/UPDATE_FREQ, initial_state);

    // Start the update thread  
    std::thread update_thread(update_circle_position);

    // Main loop
    while (renderer.shouldContinue()) {
        // Begin frame
        renderer.beginFrame();
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Real circle position is now updated by the test track in update_circle_position thread
        // No longer using mouse position
        
        // Get circle position for rendering (thread-safe)
        float real_x, real_y, real_r, 
            measured_x, measured_y, measured_r, 
            estimated_x, estimated_y, estimated_r,
            smoothed_x, smoothed_y, smoothed_r;
        g_state_mutex.lock();
        real_x = g_real_state.x;
        real_y = g_real_state.y;
        real_r = g_real_state.radius;
        measured_x = g_measured_state.x;  
        measured_y = g_measured_state.y;
        measured_r = g_measured_state.radius;
        estimated_x = g_estimated_state.x;
        estimated_y = g_estimated_state.y;
        estimated_r = g_estimated_state.radius;
        smoothed_x = g_smoothed_state.x;
        smoothed_y = g_smoothed_state.y;
        smoothed_r = g_smoothed_state.radius;
        g_state_mutex.unlock();

        // Create a fullscreen ImGui window for drawing
        // renderer.beginCanvas();
        
        // Convert positions from current coordinate system to normalized coordinates [-1, 1]
        // Current system: positions are in [-1, 1] range already
        Render::polygon(
            {
            WINDOW_WIDTH /4, WINDOW_HEIGHT /4,
            WINDOW_WIDTH /4 + 60, WINDOW_HEIGHT /4,
            WINDOW_WIDTH /4 + 80, WINDOW_HEIGHT /4 + 30,
            WINDOW_WIDTH /4 + 40, WINDOW_HEIGHT /4 + 20,
            WINDOW_WIDTH /4 + 60, WINDOW_HEIGHT /4 + 60,
            WINDOW_WIDTH /4 + 20, WINDOW_HEIGHT /4 + 80,
            WINDOW_WIDTH /4 - 20, WINDOW_HEIGHT /4 + 60,
            WINDOW_WIDTH /4, WINDOW_HEIGHT /4 + 40,
            WINDOW_WIDTH /4 - 30, WINDOW_HEIGHT /4 + 20
            },
            1.0, 1.0, 1.0, 0.2
        );

        Render::polygonFilled(
            {
                WINDOW_WIDTH /4 - 100, WINDOW_HEIGHT /4 - 100,
                WINDOW_WIDTH /4 + 40, WINDOW_HEIGHT /4 - 100,
                WINDOW_WIDTH /4 + 40, WINDOW_HEIGHT /4 + 40,
                WINDOW_WIDTH /4 - 100, WINDOW_HEIGHT /4 + 40
            },
            0.8, 1.0, 1.0, 0.1
        );

        Render::line(WINDOW_WIDTH / 4, 0, WINDOW_WIDTH / 4, WINDOW_HEIGHT, 0.3, 0.3, 0.3, 0.5);

        Render::lines(
            {
                WINDOW_WIDTH * 3 /4, 0, WINDOW_WIDTH * 3 /4, WINDOW_HEIGHT,
                0, WINDOW_HEIGHT * 3 /4, WINDOW_WIDTH, WINDOW_HEIGHT * 3 /4
            },
            1.0, 1.0, 1.0, 0.2
        );

        Render::circle(real_x, real_y, real_r, real_color.x, real_color.y, real_color.z, real_color.w);
        Render::circle(measured_x, measured_y, measured_r, measured_color.x, measured_color.y, measured_color.z, measured_color.w);
        Render::circle(g_reported_state.x, g_reported_state.y, g_reported_state.radius, reported_color.x, reported_color.y, reported_color.z, reported_color.w);
        Render::circle(estimated_x, estimated_y, estimated_r, estimated_color.x, estimated_color.y, estimated_color.z, estimated_color.w);
        Render::circle(smoothed_x, smoothed_y, smoothed_r, smoothed_color.x, smoothed_color.y, smoothed_color.z, smoothed_color.w);
        Render::circle(WINDOW_WIDTH / 4, WINDOW_HEIGHT / 4, 10, 0.3, 0.2, 0.3, 0.5);

        // renderer.endCanvas();

        // Show controls window
        if (show_controls) {
            ImGuiIO& io = ImGui::GetIO();
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_FirstUseEver);
            ImGui::Begin("Controls", &show_controls);
            
            ImGui::Text("Multi-threaded Circle Animation");
            ImGui::Separator();
            ImGui::Text("Test Track Mode (Figure-8 Loop)");
            ImGui::Text("Position: (%.3f, %.3f)", real_x, real_y);
            ImGui::Text("Velocity: (%.3f, %.3f)", g_real_state.velocity_x, g_real_state.velocity_y);
            ImGui::Text("Speed: %.3f", test_track.getCurrentSpeed());
            ImGui::Text("Track Time: %.2f / %.2f sec", test_track.getTime(), 30.0f);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                       1000.0f / io.Framerate, io.Framerate);
            
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Track Speed Settings");
            
            static float max_speed = 100.0f;
            static float min_speed = 80.0f;
            
            if (ImGui::SliderFloat("Max Speed", &max_speed, 0.5f, 100.0f, "%.2f")) {
                // Ensure max speed is always greater than min speed
                if (max_speed <= min_speed) {
                    max_speed = min_speed + 0.1f;
                }
                test_track.setSpeedLimits(max_speed, min_speed);
            }
            
            if (ImGui::SliderFloat("Min Speed", &min_speed, 0.1f, 100.0f, "%.2f")) {
                // Ensure min speed is always less than max speed
                if (min_speed >= max_speed) {
                    min_speed = max_speed - 0.1f;
                }
                test_track.setSpeedLimits(max_speed, min_speed);
            }
            
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Checkbox("Show ImGui Demo", &show_demo_window);
            
            if (ImGui::Button("Reset Track")) {
                test_track.reset();
            }
            
            ImGui::End();
        }

        // Show error tracking window with bar graph
        {
            ImGui::SetNextWindowPos(ImVec2(10, 150), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(450, 280), ImGuiCond_FirstUseEver);
            ImGui::Begin("Error Tracking");
            
            ImGui::Text("Moving Average Error (Last %zu samples)", ERROR_HISTORY_SIZE);
            ImGui::Separator();
            
            // Get error values in thread-safe manner
            float err_measured, err_estimated, err_smoothed;
            error_mutex.lock();
            err_measured = error_measured_wt_ave;
            err_estimated = error_estimated_wt_ave;
            err_smoothed = error_smoothed_wt_ave;
            error_mutex.unlock();
            
            // Display numerical values
            ImGui::Text("Measured to Real:  %.6f", err_measured);
            ImGui::Text("Estimated to Real: %.6f", err_estimated);
            ImGui::Text("Smoothed to Real:  %.6f", err_smoothed);
            
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Error Bar Graph:");
            
            // Create bar graph
            float max_error = std::max({err_measured, err_estimated, err_smoothed, 0.001f}); // Avoid division by zero
            float bar_height = 100.0f;
            
            // Draw bars using ImGui's plotting
            ImGui::BeginChild("BarGraph", ImVec2(0, bar_height + 30), true);
            ImDrawList* draw_list_bars = ImGui::GetWindowDrawList();
            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            float bar_width = 70.0f;
            float bar_spacing = 110.0f;
            
            // Draw Measured Error Bar (Red)
            float height_measured = (err_measured / max_error) * bar_height;
            draw_list_bars->AddRectFilled(
                ImVec2(canvas_pos.x + 20, canvas_pos.y + bar_height - height_measured),
                ImVec2(canvas_pos.x + 20 + bar_width, canvas_pos.y + bar_height),
                IM_COL32(200, 50, 50, 255)
            );
            draw_list_bars->AddText(ImVec2(canvas_pos.x + 25, canvas_pos.y + bar_height + 5), 
                                   IM_COL32(255, 255, 255, 255), "Measured");
            
            // Draw Estimated Error Bar (Blue)
            float height_estimated = (err_estimated / max_error) * bar_height;
            draw_list_bars->AddRectFilled(
                ImVec2(canvas_pos.x + 20 + bar_spacing, canvas_pos.y + bar_height - height_estimated),
                ImVec2(canvas_pos.x + 20 + bar_spacing + bar_width, canvas_pos.y + bar_height),
                IM_COL32(50, 50, 200, 255)
            );
            draw_list_bars->AddText(ImVec2(canvas_pos.x + 25 + bar_spacing, canvas_pos.y + bar_height + 5), 
                                   IM_COL32(255, 255, 255, 255), "Estimated");
            
            // Draw Smoothed Error Bar (Magenta)
            float height_smoothed = (err_smoothed / max_error) * bar_height;
            draw_list_bars->AddRectFilled(
                ImVec2(canvas_pos.x + 20 + 2*bar_spacing, canvas_pos.y + bar_height - height_smoothed),
                ImVec2(canvas_pos.x + 20 + 2*bar_spacing + bar_width, canvas_pos.y + bar_height),
                IM_COL32(200, 50, 200, 255)
            );
            draw_list_bars->AddText(ImVec2(canvas_pos.x + 25 + 2*bar_spacing, canvas_pos.y + bar_height + 5), 
                                   IM_COL32(255, 255, 255, 255), "Smoothed");
            
            ImGui::EndChild();
            
            ImGui::End();
        }

        // Show the big demo window
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);

        // Show shader effects window
        // {
        //     ImGui::SetNextWindowPos(ImVec2(470, 150), ImGuiCond_FirstUseEver);
        //     ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_FirstUseEver);
        //     ImGui::Begin("Post-Processing Effects");
            
        //     ImGui::Text("Available Shader Effects:");
        //     ImGui::Separator();
            
        //     PostProcessor& postProcessor = renderer.getPostProcessor();
        //     std::vector<std::string> effects = postProcessor.getEffectNames();
        //     for (const std::string& effect : effects) {
        //         bool enabled = postProcessor.isEffectEnabled(effect);
        //         if (ImGui::Checkbox(effect.c_str(), &enabled)) {
        //             postProcessor.setEffectEnabled(effect, enabled);
        //         }
        //     }
            
        //     ImGui::End();
        // }
        // End frame
        renderer.endFrame();
        // Rendering - render ImGui to the framebuffer first
        ImGui::Render();
        // Then render ImGui on top
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(renderer.getWindow());

    }

    // Signal the update thread to stop and wait for it
    Render::destroyAllShaders();
    g_running = false;
    update_thread.join();
    cleanupImGui();

    // Cleanup is handled automatically by RenderingSystem destructor
    return 0;
}