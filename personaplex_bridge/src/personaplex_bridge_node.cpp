#include "personaplex_bridge/personaplex_bridge_node.hpp"
#include <chrono>
#include <sstream>

namespace personaplex_bridge {

PersonaPlexBridgeNode::PersonaPlexBridgeNode(const rclcpp::NodeOptions& options)
    : Node("personaplex_bridge_node", options),
      running_(true),
      connection_state_(ConnectionState::DISCONNECTED),
      audio_sent_bytes_(0),
      audio_received_bytes_(0),
      messages_sent_(0),
      messages_received_(0) {

    declare_parameters();
    load_parameters();

    // Initialize Boost.Asio
    ioc_ = std::make_unique<net::io_context>();

    // Initialize SSL context if needed
    if (use_ssl_) {
        ssl_ctx_ = std::make_unique<ssl::context>(ssl::context::tlsv12_client);
        // Accept self-signed certificates (PersonaPlex uses self-signed)
        ssl_ctx_->set_verify_mode(ssl::verify_none);
    }

    // ROS2 QoS for real-time audio
    auto qos_realtime = rclcpp::QoS(rclcpp::KeepLast(2))
        .best_effort()
        .durability_volatile();

    // Subscribe to audio from PiDog
    audio_capture_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "pidog/audio/capture", qos_realtime,
        std::bind(&PersonaPlexBridgeNode::audio_capture_callback, this, std::placeholders::_1));

    // Publish audio to PiDog
    audio_playback_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "pidog/audio/playback", qos_realtime);

    // Status publisher
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "personaplex/status", rclcpp::QoS(10));

    // Reconnect timer (every 5 seconds if disconnected)
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&PersonaPlexBridgeNode::reconnect_timer_callback, this));

    // Status timer (every 10 seconds)
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&PersonaPlexBridgeNode::status_timer_callback, this));

    start_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "PersonaPlexBridgeNode initialized");
    RCLCPP_INFO(this->get_logger(), "  Server: %s:%d (SSL: %s)",
                server_host_.c_str(), server_port_, use_ssl_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "  Voice: %s", voice_prompt_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: pidog/audio/capture");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: pidog/audio/playback");

    // Start IO context thread
    io_thread_ = std::thread(&PersonaPlexBridgeNode::run_io_context, this);

    // Initial connection attempt
    connect_to_server();
}

PersonaPlexBridgeNode::~PersonaPlexBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "PersonaPlexBridgeNode shutting down...");

    running_ = false;

    if (reconnect_timer_) {
        reconnect_timer_->cancel();
    }
    if (status_timer_) {
        status_timer_->cancel();
    }

    disconnect();

    // Stop IO context
    if (ioc_) {
        ioc_->stop();
    }

    // Wait for IO thread
    if (io_thread_.joinable()) {
        io_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "PersonaPlexBridgeNode shutdown complete.");
}

void PersonaPlexBridgeNode::declare_parameters() {
    // Server connection
    this->declare_parameter<std::string>("server.host", "localhost");
    this->declare_parameter<int>("server.port", 8998);
    this->declare_parameter<bool>("server.use_ssl", true);

    // PersonaPlex configuration
    this->declare_parameter<std::string>("personaplex.voice", "NATF2");
    this->declare_parameter<std::string>("personaplex.system_prompt",
        "You are a helpful assistant on a robot dog called PiDog. "
        "Keep responses brief and friendly.");
}

void PersonaPlexBridgeNode::load_parameters() {
    this->get_parameter("server.host", server_host_);
    this->get_parameter("server.port", server_port_);
    this->get_parameter("server.use_ssl", use_ssl_);
    this->get_parameter("personaplex.voice", voice_prompt_);
    this->get_parameter("personaplex.system_prompt", system_prompt_);
}

void PersonaPlexBridgeNode::run_io_context() {
    RCLCPP_INFO(this->get_logger(), "IO context thread started");

    while (running_) {
        try {
            ioc_->run();
            // If run() returns, reset and restart (unless we're shutting down)
            if (running_) {
                ioc_->restart();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IO context error: %s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_INFO(this->get_logger(), "IO context thread exiting");
}

void PersonaPlexBridgeNode::connect_to_server() {
    if (connection_state_ == ConnectionState::CONNECTING ||
        connection_state_ == ConnectionState::CONNECTED) {
        return;
    }

    connection_state_ = ConnectionState::CONNECTING;
    publish_status("Connecting to PersonaPlex...");

    RCLCPP_INFO(this->get_logger(), "Connecting to PersonaPlex at %s:%d...",
                server_host_.c_str(), server_port_);

    try {
        // Resolve the host
        tcp::resolver resolver(*ioc_);
        auto const results = resolver.resolve(server_host_, std::to_string(server_port_));

        if (use_ssl_) {
            // Create SSL WebSocket
            wss_ = std::make_unique<websocket::stream<beast::ssl_stream<tcp::socket>>>(
                net::make_strand(*ioc_), *ssl_ctx_);

            // Set SNI hostname
            if (!SSL_set_tlsext_host_name(wss_->next_layer().native_handle(), server_host_.c_str())) {
                RCLCPP_WARN(this->get_logger(), "Failed to set SNI hostname");
            }

            // Connect
            beast::get_lowest_layer(*wss_).connect(results);

            // SSL handshake
            wss_->next_layer().handshake(ssl::stream_base::client);

            // WebSocket handshake
            wss_->handshake(server_host_, "/");

            RCLCPP_INFO(this->get_logger(), "Connected to PersonaPlex (SSL)");

        } else {
            // Create plain WebSocket
            ws_ = std::make_unique<websocket::stream<tcp::socket>>(
                net::make_strand(*ioc_));

            // Connect
            beast::get_lowest_layer(*ws_).connect(results);

            // WebSocket handshake
            ws_->handshake(server_host_, "/");

            RCLCPP_INFO(this->get_logger(), "Connected to PersonaPlex (plain)");
        }

        connection_state_ = ConnectionState::CONNECTED;
        publish_status("Connected to PersonaPlex");

        // Start reading responses
        start_reading();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", e.what());
        connection_state_ = ConnectionState::ERROR;
        publish_status(std::string("Connection failed: ") + e.what());
        disconnect();
    }
}

void PersonaPlexBridgeNode::disconnect() {
    try {
        if (use_ssl_ && wss_) {
            beast::error_code ec;
            wss_->close(websocket::close_code::normal, ec);
            wss_.reset();
        } else if (ws_) {
            beast::error_code ec;
            ws_->close(websocket::close_code::normal, ec);
            ws_.reset();
        }
    } catch (...) {
        // Ignore errors during disconnect
    }

    connection_state_ = ConnectionState::DISCONNECTED;
}

void PersonaPlexBridgeNode::start_reading() {
    if (!running_) return;

    read_buffer_.clear();

    auto read_handler = [this](beast::error_code ec, std::size_t bytes_transferred) {
        on_read(ec, bytes_transferred);
    };

    if (use_ssl_ && wss_) {
        wss_->async_read(read_buffer_, read_handler);
    } else if (ws_) {
        ws_->async_read(read_buffer_, read_handler);
    }
}

void PersonaPlexBridgeNode::on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
        if (ec != websocket::error::closed) {
            RCLCPP_ERROR(this->get_logger(), "Read error: %s", ec.message().c_str());
        }
        connection_state_ = ConnectionState::DISCONNECTED;
        publish_status("Disconnected from PersonaPlex");
        return;
    }

    messages_received_++;
    audio_received_bytes_ += bytes_transferred;

    // Get the data from the buffer
    auto data = read_buffer_.data();
    std::vector<uint8_t> audio_data(
        static_cast<const uint8_t*>(data.data()),
        static_cast<const uint8_t*>(data.data()) + data.size());

    // Publish to PiDog speaker
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data = std::move(audio_data);
    audio_playback_pub_->publish(msg);

    // Continue reading
    start_reading();
}

void PersonaPlexBridgeNode::send_audio(const std::vector<uint8_t>& audio_data) {
    if (connection_state_ != ConnectionState::CONNECTED) {
        return;
    }

    try {
        if (use_ssl_ && wss_) {
            wss_->binary(true);
            wss_->write(net::buffer(audio_data));
        } else if (ws_) {
            ws_->binary(true);
            ws_->write(net::buffer(audio_data));
        }

        messages_sent_++;
        audio_sent_bytes_ += audio_data.size();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Send error: %s", e.what());
        connection_state_ = ConnectionState::ERROR;
        disconnect();
    }
}

void PersonaPlexBridgeNode::audio_capture_callback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {

    if (connection_state_ != ConnectionState::CONNECTED) {
        return;
    }

    // Forward audio to PersonaPlex
    send_audio(msg->data);
}

void PersonaPlexBridgeNode::reconnect_timer_callback() {
    if (connection_state_ == ConnectionState::DISCONNECTED ||
        connection_state_ == ConnectionState::ERROR) {
        RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
        connect_to_server();
    }
}

void PersonaPlexBridgeNode::status_timer_callback() {
    auto now = this->get_clock()->now();
    double elapsed = (now - start_time_).seconds();

    std::string state_str;
    switch (connection_state_.load()) {
        case ConnectionState::DISCONNECTED: state_str = "DISCONNECTED"; break;
        case ConnectionState::CONNECTING: state_str = "CONNECTING"; break;
        case ConnectionState::CONNECTED: state_str = "CONNECTED"; break;
        case ConnectionState::ERROR: state_str = "ERROR"; break;
    }

    std::stringstream ss;
    ss << "PersonaPlex Bridge Status: "
       << "state=" << state_str << ", "
       << "uptime=" << static_cast<int>(elapsed) << "s, "
       << "sent=" << audio_sent_bytes_ << " bytes (" << messages_sent_ << " msgs), "
       << "recv=" << audio_received_bytes_ << " bytes (" << messages_received_ << " msgs)";

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    publish_status(ss.str());
}

void PersonaPlexBridgeNode::publish_status(const std::string& status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
}

}  // namespace personaplex_bridge
