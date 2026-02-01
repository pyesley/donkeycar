#ifndef PERSONAPLEX_BRIDGE_NODE_HPP_
#define PERSONAPLEX_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ssl/context.hpp>

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <memory>
#include <functional>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
namespace ssl = boost::asio::ssl;
using tcp = boost::asio::ip::tcp;

namespace personaplex_bridge {

/**
 * @brief PersonaPlexBridgeNode - ROS2 bridge to NVIDIA PersonaPlex
 *
 * This node connects to a PersonaPlex WebSocket server and bridges audio:
 * - Subscribes to /pidog/audio/capture (from PiDog mic)
 * - Sends audio to PersonaPlex
 * - Receives PersonaPlex responses
 * - Publishes to /pidog/audio/playback (to PiDog speaker)
 *
 * Audio format: 24kHz, 16-bit signed, mono (matching PersonaPlex requirements)
 */
class PersonaPlexBridgeNode : public rclcpp::Node {
public:
    explicit PersonaPlexBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PersonaPlexBridgeNode();

private:
    // WebSocket connection state
    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        ERROR
    };

    // Parameters
    std::string server_host_;
    int server_port_;
    bool use_ssl_;
    std::string voice_prompt_;      // PersonaPlex voice (e.g., "NATF2")
    std::string system_prompt_;     // PersonaPlex system/role prompt

    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr audio_capture_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr audio_playback_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Boost.Asio and WebSocket
    std::unique_ptr<net::io_context> ioc_;
    std::unique_ptr<ssl::context> ssl_ctx_;
    std::unique_ptr<websocket::stream<beast::ssl_stream<tcp::socket>>> wss_;
    std::unique_ptr<websocket::stream<tcp::socket>> ws_;
    std::thread io_thread_;

    // Audio queues (thread-safe)
    std::queue<std::vector<uint8_t>> send_queue_;
    std::mutex send_mutex_;
    std::atomic<bool> running_;
    std::atomic<ConnectionState> connection_state_;

    // Statistics
    std::atomic<uint64_t> audio_sent_bytes_;
    std::atomic<uint64_t> audio_received_bytes_;
    std::atomic<uint64_t> messages_sent_;
    std::atomic<uint64_t> messages_received_;
    rclcpp::Time start_time_;

    // Read buffer
    beast::flat_buffer read_buffer_;

    // Methods
    void declare_parameters();
    void load_parameters();

    void connect_to_server();
    void disconnect();
    void on_connect(beast::error_code ec);
    void on_handshake(beast::error_code ec);
    void on_ssl_handshake(beast::error_code ec);

    void start_reading();
    void on_read(beast::error_code ec, std::size_t bytes_transferred);
    void send_audio(const std::vector<uint8_t>& audio_data);
    void on_write(beast::error_code ec, std::size_t bytes_transferred);

    void audio_capture_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void reconnect_timer_callback();
    void status_timer_callback();

    void publish_status(const std::string& status);
    void run_io_context();
};

}  // namespace personaplex_bridge

#endif  // PERSONAPLEX_BRIDGE_NODE_HPP_
