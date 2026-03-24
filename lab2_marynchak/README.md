# Lab 2 – ROS2 Integration with Gazebo and LiDAR Processing

## 1. Project Overview
This repository contains the ROS2 package `lab2_marynchak`, developed to integrate **ROS2 Jazzy** with the **Gazebo Harmonic** simulator. The project implements mobile robot control and real-time LiDAR data processing.

## 2. Package Structure
The package follows the `ament_python` build system and is organized as follows:

* **`lab2_marynchak/`**: Core Python package directory.
    * `robot_controller.py`: Publisher node for robot movement logic.
    * `lidar_subscriber.py`: Subscriber node for LiDAR data processing.
* **`launch/`**: Contains `gazebo_ros2.launch.py` (starts Gazebo, Bridge, and RViz2).
* **`worlds/`**: Contains `robot.sdf` (robot model and environment).
* **`config/`**: Contains `robot.rviz` (visualization settings).
* **`setup.py`**: Configuration for installation and console scripts.

## 3. Implemented Nodes

### Robot Control Node (`robot_controller`)
* **Topic**: Publishes to `/cmd_vel`.
* **Behavior**: Moves the robot forward ($0.5\,m/s$) with a sinusoidal angular velocity ($0.3 \cdot \sin(counter \cdot 0.1)$).
* **Logging**: Reports velocity commands every 5 seconds.

### LiDAR Processing Node (`lidar_subscriber`)
* **Input**: Subscribes to `/lidar` (`sensor_msgs/msg/LaserScan`).
* **Processing**: Filters invalid readings and splits data into three sectors (Left, Front, Right).
* **Obstacle Detection**: Logs a warning if an obstacle is within $1\,m$ in the front sector.
* **Output**: Publishes statistics to `/lidar_processed`.

## 4. ROS-Gazebo Integration
Integration is managed via `ros_gz_bridge`. The bridge maps:
1.  **`/lidar`**: Gazebo LiDAR $\rightarrow$ ROS2 `LaserScan`.
2.  **`/cmd_vel`**: ROS2 `Twist` $\rightarrow$ Gazebo movement plugin.

> **Note:** The LiDAR sensor in `robot.sdf` is configured as a standard `lidar` type (CPU-based) for compatibility with Docker.

## 5. Build and Run Instructions

### 5.1. Workspace Setup
Navigate to the repository and start the Docker container:
```bash
cd ~/robotics_lpnu
./scripts/cmd run
./scripts/cmd bash
5.2. Build the Package
Bash
cd /opt/ws
colcon build --packages-select lab2_marynchak --symlink-install
source install/setup.bash
5.3. Launch Simulation
Bash
ros2 launch lab2_marynchak gazebo_ros2.launch.py
5.4. Execute Nodes (Separate Terminals)
Terminal 2 (Controller):

Bash
source /opt/ws/install/setup.bash
ros2 run lab2_marynchak robot_controller
Terminal 3 (LiDAR Processing):

Bash
source /opt/ws/install/setup.bash
ros2 run lab2_marynchak lidar_subscriber
6. Visualization
The robot.rviz configuration is used to visualize the robot's perception.

Fixed Frame: Set to world or odom.

Reliability Policy: Set to Best Effort to see the point cloud.