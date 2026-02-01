#include "pidog_ros/audio_node.hpp"
#include <chrono>
#include <cstring>

namespace pidog_ros {

AudioNode::AudioNode(const rclcpp::NodeOptions & options)
    : Node("pidog_audio_node", options),
      capture_handle_(nullptr),
      playback_handle_(nullptr),
      running_(true),
      capture_frames_total_(0),
      playback_frames_total_(0),
      capture_errors_(0),
      playback_errors_(0) {

    declare_parameters();
    load_parameters();

    // Allocate capture buffer
    capture_buffer_.resize(CHUNK_BYTES);

    // Initialize ROS 2 publishers/subscribers
    auto qos_realtime = rclcpp::QoS(rclcpp::KeepLast(2))
        .best_effort()
        .durability_volatile();

    capture_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "pidog/audio/capture", qos_realtime);

    playback_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "pidog/audio/playback", qos_realtime,
        std::bind(&AudioNode::playback_callback, this, std::placeholders::_1));

    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "pidog/audio/status", rclcpp::QoS(10));

    // Initialize ALSA devices
    bool capture_ok = false;
    bool playback_ok = false;

    if (enable_capture_) {
        capture_ok = init_capture();
        if (capture_ok) {
            // Create capture timer (runs at ~50Hz for 20ms chunks)
            auto capture_period = std::chrono::milliseconds(CHUNK_MS);
            capture_timer_ = this->create_wall_timer(
                capture_period,
                std::bind(&AudioNode::capture_timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "AudioNode: Capture enabled on %s", capture_device_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "AudioNode: Capture initialization failed, continuing without mic");
        }
    }

    if (enable_playback_) {
        playback_ok = init_playback();
        if (playback_ok) {
            // Start playback thread
            playback_thread_ = std::thread(&AudioNode::playback_thread_func, this);
            RCLCPP_INFO(this->get_logger(), "AudioNode: Playback enabled on %s", playback_device_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "AudioNode: Playback initialization failed, continuing without speaker");
        }
    }

    // Status timer (every 10 seconds)
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&AudioNode::status_timer_callback, this));

    start_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "AudioNode initialized");
    RCLCPP_INFO(this->get_logger(), "  Format: %dHz, %d-bit, %s",
                SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS == 1 ? "mono" : "stereo");
    RCLCPP_INFO(this->get_logger(), "  Chunk: %dms (%d frames, %d bytes)",
                CHUNK_MS, CHUNK_FRAMES, CHUNK_BYTES);
    RCLCPP_INFO(this->get_logger(), "  Publishing to: pidog/audio/capture");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: pidog/audio/playback");
}

AudioNode::~AudioNode() {
    RCLCPP_INFO(this->get_logger(), "AudioNode shutting down...");

    running_ = false;

    if (capture_timer_) {
        capture_timer_->cancel();
    }
    if (status_timer_) {
        status_timer_->cancel();
    }

    // Wait for playback thread to finish
    if (playback_thread_.joinable()) {
        // Wake up playback thread if it's waiting
        {
            std::lock_guard<std::mutex> lock(playback_mutex_);
            // Push empty chunk to unblock
        }
        playback_thread_.join();
    }

    cleanup_capture();
    cleanup_playback();

    RCLCPP_INFO(this->get_logger(), "AudioNode shutdown complete.");
}

void AudioNode::declare_parameters() {
    // ALSA device names
    // Fusion HAT+ uses Google VoiceHAT compatible card
    this->declare_parameter<std::string>("audio.capture_device", "plughw:2,0");
    this->declare_parameter<std::string>("audio.playback_device", "plughw:2,0");

    // Enable/disable
    this->declare_parameter<bool>("audio.enable_capture", true);
    this->declare_parameter<bool>("audio.enable_playback", true);

    // Volume (0-100)
    this->declare_parameter<int>("audio.capture_volume", 80);
    this->declare_parameter<int>("audio.playback_volume", 80);
}

void AudioNode::load_parameters() {
    this->get_parameter("audio.capture_device", capture_device_);
    this->get_parameter("audio.playback_device", playback_device_);
    this->get_parameter("audio.enable_capture", enable_capture_);
    this->get_parameter("audio.enable_playback", enable_playback_);
    this->get_parameter("audio.capture_volume", capture_volume_);
    this->get_parameter("audio.playback_volume", playback_volume_);
}

bool AudioNode::init_capture() {
    int err;

    // Open ALSA capture device
    err = snd_pcm_open(&capture_handle_, capture_device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot open capture device '%s': %s",
                     capture_device_.c_str(), snd_strerror(err));
        return false;
    }

    return configure_alsa_device(capture_handle_, capture_device_, SND_PCM_STREAM_CAPTURE);
}

bool AudioNode::init_playback() {
    int err;

    // Open ALSA playback device
    err = snd_pcm_open(&playback_handle_, playback_device_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot open playback device '%s': %s",
                     playback_device_.c_str(), snd_strerror(err));
        return false;
    }

    return configure_alsa_device(playback_handle_, playback_device_, SND_PCM_STREAM_PLAYBACK);
}

bool AudioNode::configure_alsa_device(snd_pcm_t* handle, const std::string& device_name,
                                       snd_pcm_stream_t stream_type) {
    int err;
    snd_pcm_hw_params_t* hw_params;

    snd_pcm_hw_params_alloca(&hw_params);

    // Get default hardware parameters
    err = snd_pcm_hw_params_any(handle, hw_params);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot get hw params for %s: %s",
                     device_name.c_str(), snd_strerror(err));
        return false;
    }

    // Set access type (interleaved)
    err = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot set access type: %s", snd_strerror(err));
        return false;
    }

    // Set format (16-bit signed little-endian)
    err = snd_pcm_hw_params_set_format(handle, hw_params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot set format: %s", snd_strerror(err));
        return false;
    }

    // Set channels (mono)
    err = snd_pcm_hw_params_set_channels(handle, hw_params, CHANNELS);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot set channels: %s", snd_strerror(err));
        return false;
    }

    // Set sample rate
    unsigned int rate = SAMPLE_RATE;
    err = snd_pcm_hw_params_set_rate_near(handle, hw_params, &rate, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot set sample rate: %s", snd_strerror(err));
        return false;
    }
    if (rate != SAMPLE_RATE) {
        RCLCPP_WARN(this->get_logger(), "AudioNode: Sample rate %d not supported, using %d",
                    SAMPLE_RATE, rate);
    }

    // Set buffer size (larger buffer = more latency but fewer underruns)
    snd_pcm_uframes_t buffer_frames = CHUNK_FRAMES * 4;  // 4 chunks buffer
    err = snd_pcm_hw_params_set_buffer_size_near(handle, hw_params, &buffer_frames);
    if (err < 0) {
        RCLCPP_WARN(this->get_logger(), "AudioNode: Cannot set buffer size: %s", snd_strerror(err));
    }

    // Set period size (chunk size)
    snd_pcm_uframes_t period_frames = CHUNK_FRAMES;
    err = snd_pcm_hw_params_set_period_size_near(handle, hw_params, &period_frames, nullptr);
    if (err < 0) {
        RCLCPP_WARN(this->get_logger(), "AudioNode: Cannot set period size: %s", snd_strerror(err));
    }

    // Apply hardware parameters
    err = snd_pcm_hw_params(handle, hw_params);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot set hw params: %s", snd_strerror(err));
        return false;
    }

    // Prepare the device
    err = snd_pcm_prepare(handle);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "AudioNode: Cannot prepare device: %s", snd_strerror(err));
        return false;
    }

    const char* stream_name = (stream_type == SND_PCM_STREAM_CAPTURE) ? "capture" : "playback";
    RCLCPP_INFO(this->get_logger(), "AudioNode: ALSA %s device '%s' configured successfully",
                stream_name, device_name.c_str());

    return true;
}

void AudioNode::cleanup_capture() {
    if (capture_handle_) {
        snd_pcm_drain(capture_handle_);
        snd_pcm_close(capture_handle_);
        capture_handle_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "AudioNode: Capture device closed.");
    }
}

void AudioNode::cleanup_playback() {
    if (playback_handle_) {
        snd_pcm_drain(playback_handle_);
        snd_pcm_close(playback_handle_);
        playback_handle_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "AudioNode: Playback device closed.");
    }
}

void AudioNode::capture_timer_callback() {
    if (!capture_handle_ || !running_) {
        return;
    }

    // Read audio data from microphone
    snd_pcm_sframes_t frames_read = snd_pcm_readi(capture_handle_, capture_buffer_.data(), CHUNK_FRAMES);

    if (frames_read < 0) {
        // Handle underrun/overrun
        if (frames_read == -EPIPE) {
            RCLCPP_WARN(this->get_logger(), "AudioNode: Capture overrun, recovering...");
            snd_pcm_prepare(capture_handle_);
        } else if (frames_read == -EAGAIN) {
            // No data available, try again later
            return;
        } else {
            RCLCPP_ERROR(this->get_logger(), "AudioNode: Capture error: %s", snd_strerror(frames_read));
            capture_errors_++;
        }
        return;
    }

    capture_frames_total_ += frames_read;

    // Create and publish message
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.resize(frames_read * BYTES_PER_FRAME);
    std::memcpy(msg.data.data(), capture_buffer_.data(), frames_read * BYTES_PER_FRAME);

    capture_publisher_->publish(msg);
}

void AudioNode::playback_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (!enable_playback_ || !running_) {
        return;
    }

    // Add to playback queue
    std::lock_guard<std::mutex> lock(playback_mutex_);
    playback_queue_.push(msg->data);
}

void AudioNode::playback_thread_func() {
    RCLCPP_INFO(this->get_logger(), "AudioNode: Playback thread started");

    while (running_) {
        std::vector<uint8_t> audio_data;

        // Get data from queue
        {
            std::lock_guard<std::mutex> lock(playback_mutex_);
            if (playback_queue_.empty()) {
                // No data, sleep briefly
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            audio_data = std::move(playback_queue_.front());
            playback_queue_.pop();
        }

        if (!playback_handle_ || audio_data.empty()) {
            continue;
        }

        // Write to ALSA
        snd_pcm_sframes_t frames_to_write = audio_data.size() / BYTES_PER_FRAME;
        snd_pcm_sframes_t frames_written = snd_pcm_writei(playback_handle_, audio_data.data(), frames_to_write);

        if (frames_written < 0) {
            // Handle underrun
            if (frames_written == -EPIPE) {
                RCLCPP_WARN(this->get_logger(), "AudioNode: Playback underrun, recovering...");
                snd_pcm_prepare(playback_handle_);
                // Retry
                frames_written = snd_pcm_writei(playback_handle_, audio_data.data(), frames_to_write);
            } else if (frames_written == -EAGAIN) {
                // Device busy, retry later
                continue;
            } else {
                RCLCPP_ERROR(this->get_logger(), "AudioNode: Playback error: %s", snd_strerror(frames_written));
                playback_errors_++;
                continue;
            }
        }

        if (frames_written > 0) {
            playback_frames_total_ += frames_written;
        }
    }

    RCLCPP_INFO(this->get_logger(), "AudioNode: Playback thread exiting");
}

void AudioNode::status_timer_callback() {
    auto now = this->get_clock()->now();
    double elapsed = (now - start_time_).seconds();

    std::stringstream ss;
    ss << "AudioNode Status: "
       << "uptime=" << static_cast<int>(elapsed) << "s, "
       << "captured=" << capture_frames_total_ << " frames, "
       << "played=" << playback_frames_total_ << " frames, "
       << "cap_errors=" << capture_errors_ << ", "
       << "play_errors=" << playback_errors_;

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    status_publisher_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
}

std::string AudioNode::get_alsa_error_string(int err) {
    return std::string(snd_strerror(err));
}

}  // namespace pidog_ros
