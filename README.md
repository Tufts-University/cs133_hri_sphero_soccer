# CS133 HRI Sphero Soccer

A comprehensive ROS2-based multi-robot platform for controlling Sphero BOLT robots in a soccer field environment. This project was developed for CS133 Human-Robot Interaction at Tufts University.

## Overview

This platform enables coordinated control of multiple Sphero BOLT robots using ROS2, with features including:

- **Multi-Robot Control**: Manage and control multiple Sphero robots simultaneously with isolated topic namespaces
- **ArUco-based Localization**: Real-time robot tracking on a virtual soccer field using ArUco markers and computer vision
- **Web Interface**: Modern, responsive web dashboard for managing robot fleet
- **Hierarchical Control Architecture**: From low-level motor commands to high-level autonomous behaviors and state machines
- **WebSocket Communication**: Real-time bidirectional communication between web interface and robots

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Multi-Robot Web Application                       │
│                         (Flask - Port 5000)                          │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
           ┌───────────────────┼───────────────────┐
           ▼                   ▼                   ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  WebSocket Server│  │  WebSocket Server│  │  WebSocket Server│
│    (Port 5001)   │  │    (Port 5002)   │  │    (Port 5003)   │
│   Sphero SB-3660 │  │   Sphero SB-1234 │  │   Sphero SB-9999 │
└────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
         │                     │                     │
         ▼                     ▼                     ▼
┌──────────────────────────────────────────────────────────────────┐
│                     ROS2 Controller Nodes                         │
│  • Device Controller (LED, motors, sensors)                       │
│  • Task Controller (move_to, patrol, circle, etc.)               │
│  • State Machine Controller (behavior orchestration)              │
└──────────────────────────────────────────────────────────────────┘
         │                     │                     │
         ▼                     ▼                     ▼
┌──────────────────────────────────────────────────────────────────┐
│                      ArUco SLAM Node                              │
│  • Camera-based marker detection                                  │
│  • Field calibration with corner markers                          │
│  • Real-time position tracking                                    │
└──────────────────────────────────────────────────────────────────┘
         │                     │                     │
         ▼                     ▼                     ▼
    Sphero BOLT          Sphero BOLT          Sphero BOLT
     (SB-3660)            (SB-1234)            (SB-9999)
```

## Repository Structure

```
cs133_hri_sphero_soccer/
├── README.md                           # This file
├── src/
│   ├── sphero_instance_controller/     # Core Sphero control package
│   │   ├── sphero_instance_controller/
│   │   │   ├── core/                   # ROS-independent core classes
│   │   │   ├── sphero_instance_device_controller_node.py
│   │   │   ├── sphero_instance_task_controller_node.py
│   │   │   ├── sphero_instance_statemachine_controller_node.py
│   │   │   └── sphero_instance_websocket_server.py
│   │   ├── msg/                        # Custom ROS2 message definitions
│   │   ├── README.md                   # Package documentation
│   │   └── ARCHITECTURE.md             # Architecture diagrams
│   │
│   ├── aruco_slam/                     # Vision-based localization
│   │   ├── aruco_slam/
│   │   │   ├── aruco_detector.py       # ArUco marker detection
│   │   │   ├── aruco_slam_node.py      # ROS2 SLAM node
│   │   │   ├── field_mapper.py         # Coordinate transformation
│   │   │   └── marker_generator.py     # Generate printable markers
│   │   └── README.md                   # Package documentation
│   │
│   ├── multirobot_webserver/           # Web interface
│   │   ├── multirobot_webserver/
│   │   │   ├── multirobot_webapp.py    # Flask web application
│   │   │   └── sphero_instance_websocket_server.py
│   │   ├── templates/                  # HTML templates
│   │   ├── static/                     # CSS and JavaScript
│   │   ├── README.md                   # Package documentation
│   │   └── QUICKSTART.md               # Quick start guide
│   │
│   └── images/                         # Visual resources
│       ├── Football_field.svg
│       └── aruco/                      # ArUco marker SVGs
│
├── test_aruco_detector.py              # ArUco detection test script
├── test_sphero_commands.py             # Sphero command test script
├── test_multi_sphero.py                # Multi-robot test script
├── test_sphero_roll.py                 # Roll command test script
└── test_sphero_commands.sh             # Shell-based test script
```

## Packages

### 1. Sphero Instance Controller

The core package for controlling Sphero robots with three-tiered architecture:

| Controller | Purpose | Features |
|------------|---------|----------|
| **Device Controller** | Low-level hardware control | LED, roll, spin, heading, speed, raw motors, collision detection |
| **Task Controller** | High-level autonomous behaviors | move_to, patrol, circle, square, LED sequences |
| **State Machine Controller** | Behavior orchestration | Configurable states, transitions, conditions |

**Key Features:**
- Per-instance topic namespacing (`sphero/<name>/*`)
- Custom ROS2 message definitions
- Support for Sphero BOLT LED matrix display
- Collision and tap detection
- Battery monitoring

📖 [Full Documentation](./src/sphero_instance_controller/README.md)

### 2. ArUco SLAM

Vision-based localization system using ArUco markers:

- Field calibration using 4 corner markers (IDs 0-3)
- Robot tracking using attached markers (IDs 10-13)
- Perspective transformation to field coordinates
- Real-time position publishing as PoseStamped messages

📖 [Full Documentation](./src/aruco_slam/README.md)

### 3. Multi-Robot Webserver

Web-based fleet management interface:

- Modern card-based dashboard UI
- Dynamic robot addition/removal
- Per-robot WebSocket servers
- Real-time status monitoring
- Individual controller interfaces

📖 [Full Documentation](./src/multirobot_webserver/README.md)

## Prerequisites

### Hardware
- Sphero BOLT robots (one or more)
- Webcam for ArUco tracking
- Computer with Bluetooth support

### Software
- **ROS2 Humble** (or later)
- **Python 3.10+**
- **OpenCV** with ArUco support

### Python Dependencies
```bash
pip install spherov2 opencv-python opencv-contrib-python numpy
pip install flask flask-socketio flask-cors eventlet
```

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/Tufts-University/cs133_hri_sphero_soccer.git
cd ..
```

### 2. Install Dependencies

```bash
# Python packages
pip install spherov2 opencv-python opencv-contrib-python numpy flask flask-socketio flask-cors eventlet

# ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select sphero_instance_controller aruco_slam multirobot_webserver
source install/setup.bash
```

## Quick Start

### Option 1: Web Interface (Recommended)

1. **Start the Multi-Robot Web Server:**
   ```bash
   ros2 run multirobot_webserver multirobot_webapp
   ```

2. **Open the Dashboard:**
   Navigate to http://localhost:5000

3. **Add Sphero:**
   - Click "Add Sphero"
   - Enter Sphero name (e.g., `SB-3660`)
   - Controllers start automatically

4. **Control:**
   Click "Open Controller" on any Sphero card

### Option 2: Command Line

1. **Start Device Controller:**
   ```bash
   ros2 run sphero_instance_controller sphero_instance_device_controller_node.py \
     --ros-args -p sphero_name:=SB-3660
   ```

2. **Send Commands:**
   ```bash
   # Set LED to red
   ros2 topic pub /sphero/SB_3660/led std_msgs/msg/String \
     '{data: "{\"red\": 255, \"green\": 0, \"blue\": 0}"}'
   
   # Roll forward
   ros2 topic pub /sphero/SB_3660/roll std_msgs/msg/String \
     '{data: "{\"heading\": 0, \"speed\": 100, \"duration\": 2.0}"}'
   ```

### Option 3: ArUco Tracking

1. **Generate and Print Markers:**
   ```bash
   ros2 run aruco_slam marker_generator
   ```

2. **Start SLAM Node:**
   ```bash
   ros2 run aruco_slam aruco_slam_node --ros-args \
     -p camera_id:=0 \
     -p field_width_cm:=600.0 \
     -p field_height_cm:=400.0
   ```

3. **Monitor Position:**
   ```bash
   ros2 topic echo /aruco_slam/SB-3660/position
   ```

## Topic Naming Convention

All topics are namespaced under `sphero/<sphero_name>/`:

> **Note:** ROS2 topic names cannot contain hyphens. Sphero names like `SB-3660` are automatically sanitized to `SB_3660` in topic names.

### Command Topics (Subscribe)
| Topic | Type | Description |
|-------|------|-------------|
| `/sphero/<name>/led` | `String` | LED color control |
| `/sphero/<name>/roll` | `String` | Roll movement |
| `/sphero/<name>/spin` | `String` | Spin in place |
| `/sphero/<name>/stop` | `String` | Stop movement |
| `/sphero/<name>/reset_aim` | `String` | Reset orientation |
| `/sphero/<name>/matrix` | `String` | LED matrix display |
| `/sphero/<name>/raw_motor` | `String` | Independent motor control |
| `/sphero/<name>/task` | `String` | High-level tasks |
| `/sphero/<name>/state_machine/config` | `String` | State machine configuration |

### Status Topics (Publish)
| Topic | Type | Description |
|-------|------|-------------|
| `/sphero/<name>/sensors` | `SpheroSensor` | Sensor data |
| `/sphero/<name>/state` | `String` | Complete state JSON |
| `/sphero/<name>/battery` | `BatteryState` | Battery status |
| `/sphero/<name>/status` | `String` | Health heartbeat |
| `/sphero/<name>/task/status` | `String` | Task execution status |
| `/sphero/<name>/state_machine/status` | `String` | State machine status |

## Supported Task Types

The task controller supports the following high-level behaviors:

| Task Type | Description | Parameters |
|-----------|-------------|------------|
| `move_to` | Move to coordinates | `x`, `y`, `speed` |
| `patrol` | Follow waypoints | `waypoints`, `speed`, `loop` |
| `circle` | Circular motion | `radius`, `speed`, `duration`, `direction` |
| `square` | Square pattern | `size`, `speed` |
| `led_sequence` | LED color sequence | `sequence`, `interval`, `loop` |
| `spin` | Spin in place | `angle`, `duration` |
| `stop` | Stop all movement | - |
| `set_led` | Set LED color | `red`, `green`, `blue` |

## Examples

### Python Example: Control Multiple Spheros

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MultiSpheroController(Node):
    def __init__(self):
        super().__init__('multi_sphero_controller')
        
        # Publishers for two Spheros
        self.sphero1_led = self.create_publisher(String, 'sphero/SB_3660/led', 10)
        self.sphero2_led = self.create_publisher(String, 'sphero/SB_1234/led', 10)
    
    def set_colors(self):
        # Sphero 1: Red
        msg1 = String()
        msg1.data = json.dumps({"red": 255, "green": 0, "blue": 0})
        self.sphero1_led.publish(msg1)
        
        # Sphero 2: Blue
        msg2 = String()
        msg2.data = json.dumps({"red": 0, "green": 0, "blue": 255})
        self.sphero2_led.publish(msg2)

def main():
    rclpy.init()
    controller = MultiSpheroController()
    controller.set_colors()
    rclpy.spin_once(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### HTTP API Example: Add Sphero via REST

```bash
# Add a Sphero
curl -X POST http://localhost:5000/api/spheros \
  -H "Content-Type: application/json" \
  -d '{"sphero_name": "SB-3660"}'

# Get all Spheros
curl http://localhost:5000/api/spheros

# Remove a Sphero
curl -X DELETE http://localhost:5000/api/spheros/SB-3660
```

## Test Scripts

The repository includes several test scripts:

| Script | Description |
|--------|-------------|
| `test_aruco_detector.py` | Test ArUco marker detection |
| `test_sphero_commands.py` | Test various Sphero commands |
| `test_multi_sphero.py` | Test multi-robot coordination |
| `test_sphero_roll.py` | Test roll commands with strategies |
| `test_sphero_commands.sh` | Shell-based command testing |

## Troubleshooting

### Sphero Connection Issues
- Ensure Bluetooth is enabled
- Verify Sphero is charged and powered on
- Check Sphero name matches exactly (case-sensitive)
- Ensure Sphero is not connected to another device

### Topics Not Appearing
- Verify controller started successfully
- Check topic namespace matches sphero_name
- Use `ros2 topic list | grep sphero` to see all topics

### ArUco Detection Issues
- Ensure adequate lighting
- Print markers at sufficient size (5cm×5cm minimum)
- Check marker orientation and flatness
- Verify correct ArUco dictionary (4x4_1000)

### Web Interface Issues
- Check if port 5000 is available
- Verify all Python dependencies are installed
- Check browser console for errors

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Authors

- **Siddharth Vaghela** - Tufts University

## License

This project is part of the CS133 Human-Robot Interaction course at Tufts University.

## Acknowledgments

- [spherov2](https://github.com/artificial-intelligence-class/spherov2.py) - Python library for Sphero robots
- [OpenCV ArUco](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html) - ArUco marker detection
- ROS2 community for the robotics middleware

## References

- [Sphero BOLT Technical Documentation](https://sdk.sphero.com/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [OpenCV ArUco Tutorial](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
