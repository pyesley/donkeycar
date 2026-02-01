#ifndef PIDOG_ROS_AUDIO_NODE_HPP_
#define PIDOG_ROS_AUDIO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <alsa/asoundlib.h>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <queue>

namespace pidog_ros {

/**
 * @brief AudioNode - Real-time audio capture and playback for PiDog
 *
 * This node provides bidirectional audio streaming:
 * - Captures audio from Fusion HAT+ microphone and publishes to /pidog/audio/capture
 * - Subscribes to /pidog/audio/playback and plays through Fusion HAT+ speaker
 *
 * Audio format: 24kHz, 16-bit signed, mono (compatible with NVIDIA PersonaPlex)
 */
class AudioNode : public rclcpp::Node {
public:
    explicit AudioNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~AudioNode();

    // Audio format constants (matching PersonaPlex requirements)
    static constexpr unsigned int SAMPLE_RATE = 24000;      // 24kHz for PersonaPlex
    static constexpr unsigned int CHANNELS = 1;              // Mono
    static constexpr unsigned int BITS_PER_SAMPLE = 16;      // 16-bit signed
    static constexpr unsigned int CHUNK_MS = 20;             // 20ms chunks for low latency
    static constexpr unsigned int CHUNK_FRAMES = SAMPLE_RATE * CHUNK_MS / 1000;  // 480 frames per chunk
    static constexpr unsigned int BYTES_PER_FRAME = CHANNELS * (BITS_PER_SAMPLE / 8);  // 2 bytes
    static constexpr unsigned int CHUNK_BYTES = CHUNK_FRAMES * BYTES_PER_FRAME;  // 960 bytes per chunk

private:
    // ALSA handles
    snd_pcm_t* capture_handle_;
    snd_pcm_t* playback_handle_;

    // ALSA device names (configurable via parameters)
    std::string capture_device_;
    std::string playback_device_;

    // ROS 2 interfaces
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr capture_publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr playback_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Capture buffer
    std::vector<uint8_t> capture_buffer_;

    // Playback queue (thread-safe)
    std::queue<std::vector<uint8_t>> playback_queue_;
    std::mutex playback_mutex_;
    std::thread playback_thread_;
    std::atomic<bool> running_;

    // Statistics
    std::atomic<uint64_t> capture_frames_total_;
    std::atomic<uint64_t> playback_frames_total_;
    std::atomic<uint64_t> capture_errors_;
    std::atomic<uint64_t> playback_errors_;
    rclcpp::Time start_time_;

    // Parameters
    bool enable_capture_;
    bool enable_playback_;
    int capture_volume_;
    int playback_volume_;

    // Methods
    void declare_parameters();
    void load_parameters();

    bool init_capture();
    bool init_playback();
    void cleanup_capture();
    void cleanup_playback();

    void capture_timer_callback();
    void playback_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void playback_thread_func();
    void status_timer_callback();

    bool configure_alsa_device(snd_pcm_t* handle, const std::string& device_name, snd_pcm_stream_t stream_type);
    std::string get_alsa_error_string(int err);
};

}  // namespace pidog_ros

#endif  // PIDOG_ROS_AUDIO_NODE_HPP_
