#include "video_encoder.hpp"
#include <iostream>
#include <stdexcept>

// Static member initialization
bool VideoEncoder::s_av_initialized = false;
std::mutex VideoEncoder::s_init_mutex;

VideoEncoder::VideoEncoder(const Config& config) 
    : m_config(config), m_initialized(false), m_stream_opened(false), m_frame_count(0) {
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

bool VideoEncoder::startRecording(std::unique_ptr<OutputStream> output_stream) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_initialized) {
        return true; // Already recording
    }
    
    if (!output_stream) {
        std::cerr << "No output stream provided" << std::endl;
        return false;
    }
    
    try {
        m_output_stream = std::move(output_stream);
        m_frame_count = 0;
        m_recording_start_time = std::chrono::steady_clock::now();
        
        // Set up the encoder and format context
        if (!setupEncoder()) {
            cleanup();
            return false;
        }
        
        m_initialized = true;
        
        std::cout << "Video encoder started: " << m_config.width << "x" << m_config.height 
                  << " @ " << m_config.framerate_num << "/" << m_config.framerate_den 
                  << " fps, " << m_config.format << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start video recording: " << e.what() << std::endl;
        cleanup();
        return false;
    }
}

bool VideoEncoder::stopRecording() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return finalize();
}

bool VideoEncoder::finalize() {
    if (!m_initialized) {
        return true;
    }
    
    try {
        // Flush encoder
        if (m_encoder && m_format_context) {
            std::error_code ec;
            while (true) {
                av::Packet packet = m_encoder->encode(ec);
                if (ec || !packet) {
                    break;
                }
                
                packet.setStreamIndex(0);
                m_format_context->writePacket(packet, ec);
                if (ec) {
                    std::cerr << "Failed to write flush packet: " << ec.message() << std::endl;
                    break;
                }
            }
            
            // Write trailer
            m_format_context->writeTrailer(ec);
            if (ec) {
                std::cerr << "Failed to write trailer: " << ec.message() << std::endl;
            }
        }
        
        cleanup();
        std::cout << "Video recording finalized (" << m_frame_count << " frames)" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error finalizing video: " << e.what() << std::endl;
        cleanup();
        return false;
    }
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
        av::VideoFrame rgb_frame = createRGBFrame(rgba_data, width, height);
        
        if (!rgb_frame.isValid()) {
            std::cerr << "Failed to create valid VideoFrame" << std::endl;
            return false;
        }
        
        // Calculate timestamp since recording started
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_recording_start_time);
        
        av::VideoFrame output_frame;
        std::error_code ec;
        
        // Convert pixel format if necessary
        if (m_config.input_pixfmt != m_config.output_pixfmt) {
            output_frame = av::VideoFrame(m_config.output_pixfmt, m_config.width, m_config.height, 1);
            m_rescaler->rescale(output_frame, rgb_frame, ec);
            if (ec) {
                std::cerr << "Failed to rescale frame: " << ec.message() << std::endl;
                return false;
            }
        } else {
            output_frame = rgb_frame;
        }
        
        // Set frame properties
        output_frame.setTimeBase(m_encoder->timeBase());
        output_frame.setPts(av::Timestamp{static_cast<int64_t>(elapsed.count()), m_encoder->timeBase()});
        output_frame.setStreamIndex(0);
        
        // Encode the frame
        av::Packet packet = m_encoder->encode(output_frame, ec);
        if (ec) {
            std::cerr << "Encoding error: " << ec.message() << std::endl;
            return false;
        }
        
        if (packet) {
            packet.setStreamIndex(0);
            m_format_context->writePacket(packet, ec);
            if (ec) {
                std::cerr << "Failed to write packet: " << ec.message() << std::endl;
                return false;
            }
        }
        
        m_frame_count++;
        std::cout << "Encoded and wrote frame " << m_frame_count << " at " << elapsed.count() << "ms" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error adding frame to video: " << e.what() << std::endl;
        return false;
    }
}

size_t VideoEncoder::getFrameCount() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_frame_count;
}

double VideoEncoder::getRecordingDuration() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized) {
        return 0.0;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_recording_start_time);
    return elapsed.count() / 1000.0;
}

bool VideoEncoder::startRecordingToFile(const std::string& filename) {
    auto file_stream = std::make_unique<FileOutputStream>(filename);
    return startRecording(std::move(file_stream));
}

// FileOutputStream implementation
VideoEncoder::FileOutputStream::FileOutputStream(const std::string& filename) : m_filename(filename) {}

bool VideoEncoder::FileOutputStream::open() {
    // The actual file opening is handled by avcpp FormatContext
    return true;
}

bool VideoEncoder::FileOutputStream::write(const uint8_t* data, size_t size) {
    // For file output, writing is handled by avcpp FormatContext
    // This method is here for interface completeness
    return true;
}

bool VideoEncoder::FileOutputStream::close() {
    // Closing is handled by avcpp FormatContext
    return true;
}

bool VideoEncoder::setupEncoder() {
    try {
        std::error_code ec;
        
        // Create output format context
        av::OutputFormat ofrmt;
        ofrmt.setFormat(m_config.format, m_output_stream->getUri());
        
        m_format_context = std::make_unique<av::FormatContext>();
        m_format_context->setFormat(ofrmt);
        
        // Set up encoder  
        av::Codec codec;
        if (m_config.codec == "h264") {
            codec = av::findEncodingCodec(AVCodecID::AV_CODEC_ID_H264);
        } else {
            codec = av::findEncodingCodec(AVCodecID::AV_CODEC_ID_H264);
        }
        
        if (codec.isNull()) {
            std::cerr << "Codec " << m_config.codec << " not found" << std::endl;
            return false;
        }
        
        m_encoder = std::make_unique<av::VideoEncoderContext>(codec);
        
        // Configure encoder settings
        m_encoder->setWidth(m_config.width);
        m_encoder->setHeight(m_config.height);
        m_encoder->setPixelFormat(m_config.output_pixfmt);
        m_encoder->setTimeBase(av::Rational{1, 1000});    // 1ms time base
        m_encoder->setBitRate(m_config.bitrate);
        
        // Open the encoder
        m_encoder->open(codec, ec);
        if (ec) {
            std::cerr << "Failed to open encoder: " << ec.message() << std::endl;
            return false;
        }
        
        // Add video stream
        auto stream = m_format_context->addStream(*m_encoder);
        stream.setFrameRate(av::Rational{m_config.framerate_num, m_config.framerate_den});
        
        // Open output file/stream
        m_format_context->openOutput(m_output_stream->getUri(), ec);
        if (ec) {
            std::cerr << "Failed to open output: " << ec.message() << std::endl;
            return false;
        }
        
        // Write header
        m_format_context->writeHeader(ec);
        if (ec) {
            std::cerr << "Failed to write header: " << ec.message() << std::endl;
            return false;
        }
        
        // Create video rescaler if needed
        if (m_config.input_pixfmt != m_config.output_pixfmt) {
            m_rescaler = std::make_unique<av::VideoRescaler>();
        }
        
        m_stream_opened = true;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error setting up encoder: " << e.what() << std::endl;
        return false;
    }
}

void VideoEncoder::cleanup() {
    if (m_output_stream) {
        m_output_stream->close();
        m_output_stream.reset();
    }
    
    m_encoder.reset();
    m_format_context.reset();
    m_rescaler.reset();
    
    m_initialized = false;
    m_stream_opened = false;
}


std::string VideoEncoder::getStatusString() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_initialized || !m_stream_opened) {
        return "Not recording";
    }
    
    // Calculate duration directly without calling getRecordingDuration()
    double duration = 0.0;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_recording_start_time);
    duration = elapsed.count() / 1000.0;
    
    return "Recording: " + std::to_string(m_frame_count) + " frames (" 
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