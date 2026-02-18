#pragma once

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

// AVCpp includes
#include "../avcpp/src/av.h"
#include "../avcpp/src/ffmpeg.h"
#include "../avcpp/src/format.h"
#include "../avcpp/src/formatcontext.h"
#include "../avcpp/src/codec.h"
#include "../avcpp/src/codeccontext.h"
#include "../avcpp/src/frame.h"
#include "../avcpp/src/packet.h"
#include "../avcpp/src/pixelformat.h"
#include "../avcpp/src/videorescaler.h"
#include "../avcpp/src/averror.h"

class VideoEncoder {
public:
    // Forward declaration for output stream interface
    class OutputStream {
    public:
        virtual ~OutputStream() = default;
        virtual bool open() = 0;
        virtual bool write(const uint8_t* data, size_t size) = 0;
        virtual bool close() = 0;
        virtual std::string getUri() const = 0;
    };

    // File output stream implementation
    class FileOutputStream : public OutputStream {
    private:
        std::string m_filename;
        
    public:
        explicit FileOutputStream(const std::string& filename);
        bool open() override;
        bool write(const uint8_t* data, size_t size) override;
        bool close() override;
        std::string getUri() const override { return m_filename; }
    };

    struct Config {
        int width;
        int height;
        int bitrate;  // 2 Mbps
        int framerate_num;
        int framerate_den;
        std::string format;  // MPEG-TS format
        std::string codec;     // H.264 codec
        AVPixelFormat output_pixfmt;  // Standard for H.264
        AVPixelFormat input_pixfmt;     // Input from RGBA conversion
        int log_level;
        
        // Default constructor
        Config() : width(1280), height(720), bitrate(2000000), framerate_num(15), 
                  framerate_den(1), format("mpegts"), codec("h264"), 
                  output_pixfmt(AV_PIX_FMT_YUV420P), input_pixfmt(AV_PIX_FMT_RGB24), 
                  log_level(AV_LOG_WARNING) {}
    };

private:
    Config m_config;
    std::unique_ptr<OutputStream> m_output_stream;
    bool m_initialized;
    bool m_stream_opened;
    std::chrono::steady_clock::time_point m_recording_start_time;
    mutable std::mutex m_mutex;
    
    // Encoding context
    std::unique_ptr<av::FormatContext> m_format_context;
    std::unique_ptr<av::VideoEncoderContext> m_encoder;
    std::unique_ptr<av::VideoRescaler> m_rescaler;
    std::unique_ptr<av::Stream> m_stream;
    uint64_t m_frame_count;

    static bool s_av_initialized;
    static std::mutex s_init_mutex;

public:
    explicit VideoEncoder(const Config& config = Config());
    ~VideoEncoder() = default;

    // Configuration
    void setConfig(const Config& config);
    const Config& getConfig() const { return m_config; }

    // Recording control
    bool startRecording(std::unique_ptr<OutputStream> output_stream);
    bool stopRecording();
    bool isRecording() const;
    
    // Frame management
    bool addFrame(const std::vector<unsigned char>& rgba_data);
    bool addFrame(const std::vector<unsigned char>& rgba_data, int width, int height);
    size_t getFrameCount() const;
    double getRecordingDuration() const; // in seconds

    // Convenience method for file output
    bool startRecordingToFile(const std::string& filename);
    bool finalize(); // Finalize and close the stream

    // Status
    bool isInitialized() const { return m_initialized; }
    std::string getStatusString() const;

private:
    static void initializeAVCpp(int log_level = AV_LOG_WARNING);
    av::VideoFrame createRGBFrame(const std::vector<unsigned char>& rgba_data, int width, int height);
    bool setupEncoder();
    void cleanup();
};