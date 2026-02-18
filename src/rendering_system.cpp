#include "rendering_system.hpp"
#include <iostream>

RenderingSystem::RenderingSystem(int window_width, int window_height, const std::string& window_title)
    : window_width_(window_width), window_height_(window_height), window_title_(window_title),
      window_(nullptr), clear_color_(0.1f, 0.1f, 0.15f, 1.0f), glsl_version_("#version 130"),
      initialized_(false) {
}

RenderingSystem::~RenderingSystem() {
    shutdown();
}

bool RenderingSystem::initialize() {
    if (initialized_) {
        return true;
    }
    
    if (!setupGLFW()) {
        std::cerr << "Failed to setup GLFW" << std::endl;
        return false;
    }
    
    if (!setupImGui()) {
        std::cerr << "Failed to setup ImGui" << std::endl;
        return false;
    }
    
    if (!setupPostProcessing()) {
        std::cerr << "Failed to setup post-processing" << std::endl;
        return false;
    }
    
    setupShaderEffects();
    
    initialized_ = true;
    return true;
}

void RenderingSystem::glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

bool RenderingSystem::setupGLFW() {
    // Setup GLFW
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // GL 3.0 + GLSL 130
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window with graphics context
    window_ = glfwCreateWindow(window_width_, window_height_, window_title_.c_str(), nullptr, nullptr);
    if (window_ == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable vsync

    // Load OpenGL extensions
    if (!GLExtensionLoader::loadExtensions()) {
        std::cerr << "Failed to load OpenGL extensions" << std::endl;
        glfwTerminate();
        return false;
    }
    
    return true;
}

bool RenderingSystem::setupImGui() {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl_version_);
    
    return true;
}

bool RenderingSystem::setupPostProcessing() {
    // Initialize post-processing system
    if (!post_processor_.initialize(window_width_, window_height_)) {
        std::cerr << "Failed to initialize post-processor" << std::endl;
        return false;
    }
    return true;
}

void RenderingSystem::setupShaderEffects() {
    // Load post-processing shaders
    post_processor_.addEffect("passthrough", R"(
        #version 130
        in vec2 TexCoords;
        out vec4 color;
        uniform sampler2D screenTexture;
        void main() {
            color = texture2D(screenTexture, TexCoords);
        }
    )");

    // Enable passthrough by default
    post_processor_.setEffectEnabled("passthrough", true);

    post_processor_.addEffect("grayscale", R"(
        #version 130
        in vec2 TexCoords;
        out vec4 color;
        uniform sampler2D screenTexture;
        void main() {
            vec3 texColor = texture2D(screenTexture, TexCoords).rgb;
            float gray = dot(texColor, vec3(0.299, 0.587, 0.114));
            color = vec4(vec3(gray), 1.0);
        }
    )");

    post_processor_.addEffect("invert", R"(
        #version 130
        in vec2 TexCoords;
        out vec4 color;
        uniform sampler2D screenTexture;
        void main() {
            vec3 texColor = texture2D(screenTexture, TexCoords).rgb;
            color = vec4(1.0 - texColor, 1.0);
        }
    )");

    post_processor_.addEffect("blur", R"(
        #version 130
        in vec2 TexCoords;
        out vec4 color;
        uniform sampler2D screenTexture;
        uniform vec2 resolution;
        void main() {
            vec2 texelSize = 1.0 / resolution;
            vec3 result = vec3(0.0);
            for(int x = -1; x <= 1; ++x) {
                for(int y = -1; y <= 1; ++y) {
                    vec2 offset = vec2(float(x), float(y)) * texelSize;
                    result += texture2D(screenTexture, TexCoords + offset).rgb;
                }
            }
            result /= 9.0;
            color = vec4(result, 1.0);
        }
    )");

    post_processor_.addEffect("edge_detect", R"(
        #version 130
        in vec2 TexCoords;
        out vec4 color;
        uniform sampler2D screenTexture;
        uniform vec2 resolution;
        void main() {
            vec2 texelSize = 1.0 / resolution;
            vec3 top = texture2D(screenTexture, TexCoords + vec2(0.0, texelSize.y)).rgb;
            vec3 bottom = texture2D(screenTexture, TexCoords + vec2(0.0, -texelSize.y)).rgb;
            vec3 left = texture2D(screenTexture, TexCoords + vec2(-texelSize.x, 0.0)).rgb;
            vec3 right = texture2D(screenTexture, TexCoords + vec2(texelSize.x, 0.0)).rgb;
            vec3 topLeft = texture2D(screenTexture, TexCoords + vec2(-texelSize.x, texelSize.y)).rgb;
            vec3 topRight = texture2D(screenTexture, TexCoords + vec2(texelSize.x, texelSize.y)).rgb;
            vec3 bottomLeft = texture2D(screenTexture, TexCoords + vec2(-texelSize.x, -texelSize.y)).rgb;
            vec3 bottomRight = texture2D(screenTexture, TexCoords + vec2(texelSize.x, -texelSize.y)).rgb;
            
            vec3 sx = (topRight + 2.0 * right + bottomRight) - (topLeft + 2.0 * left + bottomLeft);
            vec3 sy = (topLeft + 2.0 * top + topRight) - (bottomLeft + 2.0 * bottom + bottomRight);
            vec3 sobel = sqrt(sx * sx + sy * sy);
            
            color = vec4(sobel, 1.0);
        }
    )");
}

bool RenderingSystem::shouldContinue() {
    if (!initialized_ || !window_) {
        return false;
    }
    
    return !glfwWindowShouldClose(window_);
}

void RenderingSystem::beginFrame() {
    if (!initialized_) return;
    
    // Poll and handle events
    glfwPollEvents();

    // Check for window resize
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    if (display_w != post_processor_.getWidth() || display_h != post_processor_.getHeight()) {
        post_processor_.resize(display_w, display_h);
    }

    // Begin rendering to framebuffer
    post_processor_.beginRender();
    
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void RenderingSystem::endFrame() {
    if (!initialized_) return;
    
    // Rendering - render ImGui to the framebuffer first
    ImGui::Render();
    
    // Clear the framebuffer and render ImGui to it
    glClearColor(clear_color_.x * clear_color_.w, clear_color_.y * clear_color_.w, 
                 clear_color_.z * clear_color_.w, clear_color_.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Apply post-processing effects and render to screen
    post_processor_.endRender();

    glfwSwapBuffers(window_);
}

ImDrawList* RenderingSystem::getDrawList() {
    return ImGui::GetWindowDrawList();
}

void RenderingSystem::beginCanvas() {
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("Canvas", nullptr, 
                 ImGuiWindowFlags_NoDecoration | 
                 ImGuiWindowFlags_NoMove | 
                 ImGuiWindowFlags_NoResize | 
                 ImGuiWindowFlags_NoSavedSettings |
                 ImGuiWindowFlags_NoBringToFrontOnFocus);
}

void RenderingSystem::endCanvas() {
    ImGui::End();
}

ImGuiIO& RenderingSystem::getIO() {
    return ImGui::GetIO();
}

void RenderingSystem::setClearColor(const ImVec4& color) {
    clear_color_ = color;
}

void RenderingSystem::shutdown() {
    if (!initialized_) return;
    
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
    
    initialized_ = false;
}