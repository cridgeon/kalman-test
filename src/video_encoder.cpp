#include "video_encoder.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cctype>

extern "C" {
#include <libavformat/avformat.h>
}

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
        // Don't set m_recording_start_time here - we'll set it on first frame
        // This ensures timestamps match when the first frame is actually written
        
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
            
            // Same offset as in addFrame for consistency (in encoder timebase frames)
            static const int64_t INITIAL_OFFSET_FRAMES = 120;
            
            while (true) {
                av::Packet packet = m_encoder->encode(ec);
                if (ec || !packet) {
                    break;
                }
                
                packet.setStreamIndex(0);
                
                // Add offset in encoder timebase - avcpp handles rescaling
                auto pts = packet.pts();
                auto dts = packet.dts();
                auto tb = packet.timeBase();
                
                if (pts.isValid()) {
                    packet.setPts(av::Timestamp{pts.timestamp() + INITIAL_OFFSET_FRAMES, tb});
                }
                if (dts.isValid()) {
                    packet.setDts(av::Timestamp{dts.timestamp() + INITIAL_OFFSET_FRAMES, tb});
                }
                
                m_format_context->writePacket(packet, ec);
                if (ec) {
                    std::cerr << "Failed to write flush packet: " << ec.message() << std::endl;
                    break;
                }
            }
            
            // Flush interleaved queue
            m_format_context->flush();
            
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
        
        // Set frame properties - use frame count for PTS (simpler and more reliable)
        // Set recording start time on first frame to ensure timing is correct
        if (m_frame_count == 0) {
            m_recording_start_time = std::chrono::steady_clock::now();
        }
        output_frame.setTimeBase(m_encoder->timeBase());
        output_frame.setPts(av::Timestamp{static_cast<int64_t>(m_frame_count), m_encoder->timeBase()});
        output_frame.setStreamIndex(0);
        
        // Encode the frame
        av::Packet packet = m_encoder->encode(output_frame, ec);
        if (ec) {
            std::cerr << "Encoding error: " << ec.message() << std::endl;
            return false;
        }
        
        if (packet) {
            packet.setStreamIndex(0);
            
            // Add offset to PTS/DTS in encoder timebase BEFORE avcpp rescales them
            // This ensures DTS >= PCR when MPEG-TS muxer converts to 90kHz
            // avcpp's writePacket calls av_packet_rescale_ts automatically
            // PCR typically starts ~63000 (0.7s in 90kHz), so 2s offset should be safe
            static const int64_t INITIAL_OFFSET_FRAMES = 120; // 2 seconds at 60fps in encoder timebase
            
            auto pts = packet.pts();
            auto dts = packet.dts();
            auto tb = packet.timeBase();
            
            if (pts.isValid()) {
                packet.setPts(av::Timestamp{pts.timestamp() + INITIAL_OFFSET_FRAMES, tb});
            }
            if (dts.isValid()) {
                packet.setDts(av::Timestamp{dts.timestamp() + INITIAL_OFFSET_FRAMES, tb});
            }
            
            // Use interleaved write for proper streaming (writePacket handles timestamp rescaling)
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

bool VideoEncoder::startRecordingToNetwork(const std::string& uri) {
    auto network_stream = std::make_unique<NetworkOutputStream>(uri);
    return startRecording(std::move(network_stream));
}

bool VideoEncoder::startRecordingToNetwork(const std::string& uri, const NetworkOutputStream::NetworkConfig& config) {
    auto network_stream = std::make_unique<NetworkOutputStream>(uri, config);
    return startRecording(std::move(network_stream));
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

// NetworkOutputStream implementation
VideoEncoder::NetworkOutputStream::NetworkOutputStream(const std::string& uri) 
    : m_uri(uri) {
    parseUri();
}

VideoEncoder::NetworkOutputStream::NetworkOutputStream(const std::string& uri, const NetworkConfig& config) 
    : m_uri(uri), m_config(config) {
    parseUri();
}

bool VideoEncoder::NetworkOutputStream::parseUri() {
    // Extract protocol from URI
    size_t proto_end = m_uri.find("://");
    if (proto_end == std::string::npos) {
        std::cerr << "Invalid URI format: " << m_uri << std::endl;
        return false;
    }
    
    m_protocol = m_uri.substr(0, proto_end);
    
    // Convert to lowercase for comparison
    std::transform(m_protocol.begin(), m_protocol.end(), m_protocol.begin(), ::tolower);
    
    if (!validateProtocol()) {
        std::cerr << "Unsupported protocol: " << m_protocol << std::endl;
        return false;
    }
    
    return true;
}

bool VideoEncoder::NetworkOutputStream::validateProtocol() const {
    return isProtocolAvailable(m_protocol);
}

std::vector<std::string> VideoEncoder::NetworkOutputStream::getAvailableProtocols() {
    std::vector<std::string> available;
    
    // Check common protocols that are usually available
    std::vector<std::string> test_protocols = {
        "tcp", "udp", "http", "https", "file", "pipe", "rtp"
    };
    
    for (const auto& protocol : test_protocols) {
        if (isProtocolAvailable(protocol)) {
            available.push_back(protocol);
        }
    }
    
    return available;
}

bool VideoEncoder::NetworkOutputStream::isProtocolAvailable(const std::string& protocol) {
    // For now, we'll check against a list of commonly supported protocols
    // This could be enhanced to query FFmpeg's available protocols at runtime
    static const std::vector<std::string> common_protocols = {
        "tcp", "udp", "http", "https", "file", "pipe", "rtp"
    };
    
    return std::find(common_protocols.begin(), common_protocols.end(), protocol) 
           != common_protocols.end();
}

bool VideoEncoder::NetworkOutputStream::open() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_connected.load()) {
        return true; // Already connected
    }
    
    try {
        // Validate protocol availability
        if (!isProtocolAvailable(m_protocol)) {
            std::cerr << "Protocol '" << m_protocol << "' is not available." << std::endl;
            std::cerr << "Available protocols: ";
            auto available = getAvailableProtocols();
            for (size_t i = 0; i < available.size(); ++i) {
                std::cerr << available[i];
                if (i < available.size() - 1) std::cerr << ", ";
            }
            std::cerr << std::endl;
            return false;
        }
        
        std::cout << "Opening network stream: " << m_uri << " (Protocol: " << m_protocol << ")" << std::endl;
        
        // Additional validation for specific protocols
        if (m_protocol == "http" || m_protocol == "https") {
            // For HTTP streaming, suggest using a streaming-friendly format
            std::cout << "Note: For HTTP streaming, ensure the server supports chunked transfer encoding" << std::endl;
        } else if (m_protocol == "tcp" || m_protocol == "udp") {
            // For raw TCP/UDP, warn about format compatibility
            std::cout << "Note: Using raw " << m_protocol << " - ensure receiver supports MPEG-TS format" << std::endl;
        }
        
        m_connected.store(true);
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to open network stream: " << e.what() << std::endl;
        return false;
    }
}

bool VideoEncoder::NetworkOutputStream::write(const uint8_t* data, size_t size) {
    // For network streams, writing is handled by avcpp FormatContext
    // This method is here for interface completeness but could be used
    // for custom network implementations in the future
    if (!m_connected.load()) {
        return false;
    }
    return true;
}

bool VideoEncoder::NetworkOutputStream::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (!m_connected.load()) {
        return true; // Already closed
    }
    
    try {
        std::cout << "Closing network stream: " << m_uri << std::endl;
        
        // Signal stop and wait for any background operations
        m_should_stop.store(true);
        
        // Closing is primarily handled by avcpp FormatContext
        m_connected.store(false);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error closing network stream: " << e.what() << std::endl;
        m_connected.store(false);
        return false;
    }
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
        m_encoder->setTimeBase(av::Rational{1, m_config.framerate_num});  // Use framerate as time base
        m_encoder->setBitRate(m_config.bitrate);
        
        // Set streaming-friendly options for H.264
        av::Dictionary encoder_options;
        encoder_options.set("preset", "ultrafast");      // Fast encoding for low latency
        encoder_options.set("tune", "zerolatency");      // Zero latency tuning
        encoder_options.set("profile", "baseline");      // Baseline profile for compatibility
        encoder_options.set("g", std::to_string(m_config.framerate_num)); // Keyframe every 1 second
        encoder_options.set("bf", "0");                  // No B-frames for lower latency
        
        // Open the encoder with options
        m_encoder->open(encoder_options, codec, ec);
        if (ec) {
            std::cerr << "Failed to open encoder: " << ec.message() << std::endl;
            return false;
        }
        
        // Add video stream
        auto stream = m_format_context->addStream(*m_encoder);
        stream.setFrameRate(av::Rational{m_config.framerate_num, m_config.framerate_den});
        
        // Configure network options if using NetworkOutputStream
        auto* network_stream = dynamic_cast<NetworkOutputStream*>(m_output_stream.get());
        if (network_stream) {
            const std::string& protocol = network_stream->getProtocol();
            std::cout << "Configuring network stream options for protocol: " << protocol << std::endl;
            
            // Validate protocol is available
            if (!NetworkOutputStream::isProtocolAvailable(protocol)) {
                std::cerr << "Protocol '" << protocol << "' is not available in this FFmpeg build" << std::endl;
                auto available = NetworkOutputStream::getAvailableProtocols();
                std::cerr << "Available protocols: ";
                for (size_t i = 0; i < available.size(); ++i) {
                    std::cerr << available[i];
                    if (i < available.size() - 1) std::cerr << ", ";
                }
                std::cerr << std::endl;
                return false;
            }
            
            // Set network-specific options via FFmpeg dictionary
            av::Dictionary options;
            const auto& config = network_stream->getConfig();
            
            // Common network options
            if (config.timeout_ms > 0) {
                // Convert timeout to seconds for most protocols
                options.set("timeout", std::to_string(config.timeout_ms / 1000));
            }
            
            // Protocol-specific options
            if (protocol == "tcp") {
                options.set("listen", "0"); // Client mode
                if (config.tcp_nodelay) {
                    options.set("tcp_nodelay", "1");
                }
            } else if (protocol == "udp") {
                options.set("buffer_size", std::to_string(config.buffer_size));
                options.set("pkt_size", "1316"); // Standard for MPEG-TS over UDP
            } else if (protocol == "http" || protocol == "https") {
                options.set("method", "POST");
                options.set("chunked_post", "1");
                options.set("content_type", "video/mp2t"); // MPEG-TS MIME type
                options.set("multiple_requests", "1");     // Keep connection alive
                options.set("reconnect", "1");             // Reconnect on failure
                options.set("reconnect_streamed", "1");    // Reconnect for streaming
                if (!config.user_agent.empty()) {
                    options.set("user_agent", config.user_agent);
                }
            } else if (protocol == "rtp") {
                options.set("rtpflags", "latm");
                options.set("payload_type", "33"); // MP2T payload type
            }
            
            // Open output with options
            m_format_context->openOutput(m_output_stream->getUri(), options, ec);
            
            if (options.count() > 0) {
                std::cout << "Applied network options for " << protocol << " protocol" << std::endl;
            }
        } else {
            // Open output without options for file streams
            m_format_context->openOutput(m_output_stream->getUri(), ec);
        }
        
        if (ec) {
            std::cerr << "Failed to open output: " << ec.message() << std::endl;
            return false;
        }
        
        // Set avoid_negative_ts to ensure timestamps are valid for streaming
        // This tells FFmpeg to adjust timestamps to be non-negative
        if (m_format_context->raw()) {
            m_format_context->raw()->avoid_negative_ts = AVFMT_AVOID_NEG_TS_MAKE_NON_NEGATIVE;
        }
        
        // Write header with MPEG-TS muxer options for streaming
        av::Dictionary muxer_options;
        if (network_stream && m_config.format == "mpegts") {
            // MPEG-TS streaming options for low latency
            // initial_discontinuity tells receiver to reset PCR/DTS relationship at start
            muxer_options.set("mpegts_flags", "resend_headers+latm+initial_discontinuity");
            muxer_options.set("pcr_period", "40");         // PCR every 40ms (reasonable for streaming)
            muxer_options.set("muxrate", std::to_string(m_config.bitrate * 2)); // Higher mux rate
            muxer_options.set("flush_packets", "1");       // Flush packets immediately
        }
        
        if (muxer_options.count() > 0) {
            m_format_context->writeHeader(muxer_options, ec);
        } else {
            m_format_context->writeHeader(ec);
        }
        
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