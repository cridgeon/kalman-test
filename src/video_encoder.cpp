#include "video_encoder.hpp"
#include <iostream>
#include <stdexcept>

// Static member initialization
bool VideoEncoder::s_av_initialized = false;
std::mutex VideoEncoder::s_init_mutex;

VideoEncoder::VideoEncoder(const Config& config) 
    : m_config(config), m_initialized(false) {
    initializeAVCpp(m_config.log_level);
}

void VideoEncoder::setConfig(const Config& config) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_initialized) {  // Check directly instead of calling isRecording()
        throw std::runtime_error("Cannot change configuration while recording");
    }
    
    m_config = config;
    m_initialized = false; // Force re-initialization with new config
}

bool VideoEncoder::startRecording() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_initialized) {
        return true; // Already recording
    }
    
    try {
        m_frames.clear();
        m_frame_timestamps.clear();
        m_recording_start_time = std::chrono::steady_clock::now();
        m_initialized = true;
        
        std::cout << "Video encoder started: " << m_config.width << "x" << m_config.height 
                  << " @ " << m_config.framerate_num << "/" << m_config.framerate_den 
                  << " fps, " << m_config.format << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start video recording: " << e.what() << std::endl;
        return false;
    }
}

bool VideoEncoder::stopRecording() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_initialized = false;
    return true;
}

bool VideoEncoder::isRecording() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool VideoEncoder::addFrame(const std::vector<unsigned char>& rgba_data) {
    return addFrame(rgba_data, m_config.width, m_config.height);
}

bool VideoEncoder::addFrame(const std::vector<unsigned char>& rgba_data, int width, int height) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return false;
    }
    
    if (rgba_data.size() < width * height * 4) {
        std::cerr << "Invalid RGBA data size: " << rgba_data.size() 
                  << " (expected: " << width * height * 4 << ")" << std::endl;
        return false;
    }
    
    try {
        av::VideoFrame frame = createRGBFrame(rgba_data, width, height);
        
        if (!frame.isValid()) {
            std::cerr << "Failed to create valid VideoFrame" << std::endl;
            return false;
        }
        
        // Calculate timestamp since recording started
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_recording_start_time);
        
        m_frames.push_back(std::move(frame));
        m_frame_timestamps.push_back(elapsed.count());
        
        std::cout << "Captured frame " << m_frames.size() << " at " << elapsed.count() << "ms" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error adding frame to video: " << e.what() << std::endl;
        return false;
    }
}

size_t VideoEncoder::getFrameCount() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_frames.size();
}

double VideoEncoder::getRecordingDuration() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized || m_frame_timestamps.empty()) {
        return 0.0;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_recording_start_time);
    return elapsed.count() / 1000.0;
}

bool VideoEncoder::saveToFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_frames.empty()) {
        std::cout << "No frames to save" << std::endl;
        return false;
    }
    
    std::cout << "Saving " << m_frames.size() << " frames to " << filename << std::endl;
    
    try {
        std::error_code ec;
        
        // Create output format context
        av::OutputFormat ofrmt;
        ofrmt.setFormat(m_config.format, filename);
        
        av::FormatContext octx;
        octx.setFormat(ofrmt);
        
        // Set up encoder  
        av::Codec codec;
        if (m_config.codec == "h264") {
            codec = av::findEncodingCodec(AVCodecID::AV_CODEC_ID_H264);
        } else {
            // Add support for other codecs as needed
            codec = av::findEncodingCodec(AVCodecID::AV_CODEC_ID_H264);
        }
        
        if (codec.isNull()) {
            std::cerr << "Codec " << m_config.codec << " not found" << std::endl;
            return false;
        }
        
        av::VideoEncoderContext encoder(codec);
        
        // Configure encoder settings
        encoder.setWidth(m_config.width);
        encoder.setHeight(m_config.height);
        encoder.setPixelFormat(m_config.output_pixfmt);
        encoder.setTimeBase(av::Rational{1, 1000});    // 1ms time base
        encoder.setBitRate(m_config.bitrate);
        
        // Open the encoder
        encoder.open(codec, ec);
        if (ec) {
            std::cerr << "Failed to open encoder: " << ec.message() << std::endl;
            return false;
        }
        
        // Add video stream
        av::Stream ost = octx.addStream(encoder);
        ost.setFrameRate(av::Rational{m_config.framerate_num, m_config.framerate_den});
        
        // Open output file
        octx.openOutput(filename, ec);
        if (ec) {
            std::cerr << "Failed to open output file: " << ec.message() << std::endl;
            return false;
        }
        
        // Write header
        octx.writeHeader(ec);
        if (ec) {
            std::cerr << "Failed to write header: " << ec.message() << std::endl;
            return false;
        }
        
        // Create video rescaler to convert RGB24 to output format if needed
        av::VideoRescaler rescaler;
        
        // Process each frame
        for (size_t i = 0; i < m_frames.size(); ++i) {
            auto& rgb_frame = m_frames[i];
            
            av::VideoFrame output_frame;
            
            // Convert pixel format if necessary
            if (m_config.input_pixfmt != m_config.output_pixfmt) {
                output_frame = av::VideoFrame(m_config.output_pixfmt, m_config.width, m_config.height, 1);
                rescaler.rescale(output_frame, rgb_frame, ec);
                if (ec) {
                    std::cerr << "Failed to rescale frame " << i << ": " << ec.message() << std::endl;
                    continue;
                }
            } else {
                output_frame = rgb_frame;
            }
            
            // Set frame properties
            output_frame.setTimeBase(encoder.timeBase());
            output_frame.setPts(av::Timestamp{static_cast<int64_t>(m_frame_timestamps[i]), encoder.timeBase()});
            output_frame.setStreamIndex(0);
            
            // Encode the frame
            av::Packet packet = encoder.encode(output_frame, ec);
            if (ec) {
                std::cerr << "Encoding error for frame " << i << ": " << ec.message() << std::endl;
                continue;
            }
            
            if (packet) {
                packet.setStreamIndex(0);
                octx.writePacket(packet, ec);
                if (ec) {
                    std::cerr << "Failed to write packet for frame " << i << ": " << ec.message() << std::endl;
                }
            }
        }
        
        // Flush encoder
        while (true) {
            av::Packet packet = encoder.encode(ec);
            if (ec || !packet) {
                break;
            }
            
            packet.setStreamIndex(0);
            octx.writePacket(packet, ec);
            if (ec) {
                std::cerr << "Failed to write flush packet: " << ec.message() << std::endl;
                break;
            }
        }
        
        // Write trailer
        octx.writeTrailer(ec);
        if (ec) {
            std::cerr << "Failed to write trailer: " << ec.message() << std::endl;
        }
        
        std::cout << "Video saved successfully: " << filename 
                  << " (" << m_frames.size() << " frames)" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving video: " << e.what() << std::endl;
        return false;
    }
}

void VideoEncoder::clearFrames() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_frames.clear();
    m_frame_timestamps.clear();
}

std::string VideoEncoder::getStatusString() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return "Not recording";
    }
    
    // Calculate duration directly without calling getRecordingDuration()
    double duration = 0.0;
    if (!m_frame_timestamps.empty()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_recording_start_time);
        duration = elapsed.count() / 1000.0;
    }
    
    return "Recording: " + std::to_string(m_frames.size()) + " frames (" 
           + std::to_string(duration) + "s)";
}

void VideoEncoder::initializeAVCpp(int log_level) {
    std::lock_guard<std::mutex> lock(s_init_mutex);
    
    if (s_av_initialized) {
        return;
    }
    
    av::init();
    av::setFFmpegLoggingLevel(log_level);
    s_av_initialized = true;
    
    std::cout << "AVCpp library initialized" << std::endl;
}

av::VideoFrame VideoEncoder::createRGBFrame(const std::vector<unsigned char>& rgba_data, int width, int height) {
    // Create a properly allocated VideoFrame for RGB24
    av::VideoFrame frame(m_config.input_pixfmt, width, height, 1);
    
    if (!frame.isValid()) {
        throw std::runtime_error("Failed to create VideoFrame");
    }
    
    // Get the frame data pointers and linesize
    uint8_t* frame_data = frame.data(0);
    int linesize = frame.bufferSize() / frame.height();  // Calculate linesize from buffer size
    
    // Convert RGBA to RGB24 and copy directly to frame buffer
    for (int y = 0; y < height; y++) {
        uint8_t* dst_row = frame_data + y * linesize;
        
        for (int x = 0; x < width; x++) {
            // Flip vertically (OpenGL reads from bottom-left, video expects top-left)
            int rgba_index = ((height - y - 1) * width + x) * 4;
            int rgb_index = x * 3;
            
            dst_row[rgb_index]     = rgba_data[rgba_index];     // R
            dst_row[rgb_index + 1] = rgba_data[rgba_index + 1]; // G
            dst_row[rgb_index + 2] = rgba_data[rgba_index + 2]; // B
            // Skip alpha channel
        }
    }
    
    return frame;
}