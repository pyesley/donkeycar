# PersonaPlex Bridge

ROS2 bridge node that connects PiDog audio to NVIDIA PersonaPlex for real-time voice AI.

**Run this on your desktop machine with the RTX 5090.**

## Architecture

```
┌─────────────────────┐         ROS2 DDS          ┌─────────────────────┐
│   PiDog (Pi 5)      │◄─────── WiFi ────────────►│  Desktop (RTX 5090) │
│                     │                            │                     │
│  pidog_node         │  /pidog/audio/capture ──►  │  personaplex_bridge │
│  - mic capture      │                            │         │           │
│  - speaker playback │  ◄── /pidog/audio/playback │         ▼           │
│                     │                            │  ┌─────────────┐    │
└─────────────────────┘                            │  │ PersonaPlex │    │
                                                   │  │   Server    │    │
                                                   │  │ (WebSocket) │    │
                                                   │  └─────────────┘    │
                                                   └─────────────────────┘
```

## Prerequisites

### On Desktop

1. **ROS2 Jazzy** installed
2. **NVIDIA PersonaPlex** installed and running:
   ```bash
   # Install PersonaPlex
   pip install moshi/.

   # For RTX 5090 (Blackwell)
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130

   # Start PersonaPlex server
   export HF_TOKEN=<your-huggingface-token>
   SSL_DIR=$(mktemp -d)
   python -m moshi.server --ssl "$SSL_DIR"
   ```

3. **Boost and OpenSSL** development libraries:
   ```bash
   sudo apt install libboost-all-dev libssl-dev
   ```

### On PiDog

1. `pidog_ros` node running:
   ```bash
   ros2 run pidog_ros pidog_node
   ```

### Network

Both machines must be on the same network with ROS2 DDS discovery working:
```bash
# Set same domain ID on both machines (optional, default is 0)
export ROS_DOMAIN_ID=42
```

## Building

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select personaplex_bridge
source install/setup.bash
```

## Running

### Quick Start

1. **Start PersonaPlex server** (in one terminal):
   ```bash
   SSL_DIR=$(mktemp -d)
   python -m moshi.server --ssl "$SSL_DIR"
   ```

2. **Start the bridge** (in another terminal):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run personaplex_bridge personaplex_bridge_node
   ```

### With Launch File

```bash
ros2 launch personaplex_bridge personaplex_bridge.launch.py
```

### Custom Parameters

```bash
ros2 launch personaplex_bridge personaplex_bridge.launch.py \
    server_host:=localhost \
    server_port:=8998 \
    voice:=NATM1 \
    system_prompt:="You are a friendly robot dog named Spark."
```

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `pidog/audio/capture` | `std_msgs/UInt8MultiArray` | Audio from PiDog microphone |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `pidog/audio/playback` | `std_msgs/UInt8MultiArray` | Audio to PiDog speaker |
| `personaplex/status` | `std_msgs/String` | Connection status |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `server.host` | localhost | PersonaPlex server hostname |
| `server.port` | 8998 | PersonaPlex server port |
| `server.use_ssl` | true | Use SSL/TLS connection |
| `personaplex.voice` | NATF2 | Voice prompt |
| `personaplex.system_prompt` | (see below) | System/role prompt |

### Available Voices

**Natural Female:** NATF0, NATF1, NATF2, NATF3
**Natural Male:** NATM0, NATM1, NATM2, NATM3
**Variety Female:** VARF0, VARF1, VARF2, VARF3, VARF4
**Variety Male:** VARM0, VARM1, VARM2, VARM3, VARM4

## Audio Format

The bridge uses audio format compatible with PersonaPlex:

- **Sample Rate**: 24000 Hz
- **Channels**: 1 (mono)
- **Bits per Sample**: 16 (signed little-endian)
- **Chunk Size**: 20ms (480 frames, 960 bytes)

## Troubleshooting

### Cannot connect to PersonaPlex

1. Ensure PersonaPlex server is running:
   ```bash
   curl -k https://localhost:8998/
   ```

2. Check firewall allows port 8998

3. Verify SSL certificates (PersonaPlex uses self-signed)

### No audio from PiDog

1. Check ROS2 topics are visible:
   ```bash
   ros2 topic list | grep pidog
   ```

2. Verify PiDog node is publishing:
   ```bash
   ros2 topic hz /pidog/audio/capture
   ```

3. Check ROS_DOMAIN_ID matches on both machines

### High latency

1. Ensure WiFi connection is stable
2. Check CPU/GPU usage on desktop
3. Consider wired Ethernet connection

## Files

```
personaplex_bridge/
├── CMakeLists.txt
├── package.xml
├── include/personaplex_bridge/
│   └── personaplex_bridge_node.hpp
├── src/
│   ├── main.cpp
│   └── personaplex_bridge_node.cpp
├── launch/
│   └── personaplex_bridge.launch.py
└── README.md
```

## Full System Startup

### Terminal 1 (Desktop) - PersonaPlex Server
```bash
export HF_TOKEN=<your-token>
SSL_DIR=$(mktemp -d)
python -m moshi.server --ssl "$SSL_DIR"
```

### Terminal 2 (Desktop) - PersonaPlex Bridge
```bash
source ~/ros2_ws/install/setup.bash
ros2 run personaplex_bridge personaplex_bridge_node
```

### Terminal 3 (PiDog) - PiDog Node
```bash
cd ~/ros2_ws/donkeycar/pidog
./launch_pidog.sh
```

Now speak to PiDog's microphone and hear PersonaPlex respond through the speaker!
