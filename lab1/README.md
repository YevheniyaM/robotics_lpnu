# Lab 1 – Mobile Robot with LiDAR in Gazebo

This document summarizes the implementation of Lab 1 in the `worlds/robot.sdf` file. It describes the robot's configuration, the environment setup, and instructions on how to build, launch, and interact with the simulation.

## 1. World and Environment Setup

* **Physics and Plugins:** The `car_world` uses a standard physics configuration with a 1ms step size. It includes essential system plugins: `Physics`, `UserCommands`, `SceneBroadcaster`, and `Sensors` (configured with the `ogre2` engine for GPU-accelerated sensor simulation).
* **Lighting:** A directional light named `sun` provides realistic illumination and shadows.
* **Ground Plane:** A static `ground_plane` (100x100m) serves as the floor for the robot and obstacles.

## 2. Robot Model Configuration

The robot, named `vehicle_blue`, is a 4-wheeled mobile platform:

* **Chassis:** A blue rectangular link with calculated inertial properties.
* **Wheels:** Four wheels (front-left, front-right, rear-left, rear-right) connected via `revolute` joints.
* **Differential Drive Plugin:** The `gz::sim::systems::DiffDrive` plugin manages the movement. It is configured to control all four joints simultaneously:
* **Left Joints:** `left_front_wheel_joint`, `left_rear_wheel_joint`.
* **Right Joints:** `right_front_wheel_joint`, `right_rear_wheel_joint`.
* **Parameters:** Wheel separation is **1.2m**, and wheel radius is **0.4m**.
* **Command Topic:** Subscribes to `/cmd_vel` for velocity commands.



## 3. LiDAR Sensor Configuration

The robot is equipped with a high-performance GPU-based LiDAR:

* **Mounting:** A `lidar_link` is fixed to the front of the chassis.
* **Specifications:**
* **Topic:** `/lidar`.
* **Samples:** 640 horizontal beams.
* **Field of View:** Approximately ±80° (2.79 radians).
* **Range:** 0.08m to 10.0m.


* **Visualization:** `<visualize>true</visualize>` is enabled to show the laser rays in the Gazebo GUI.

## 4. Obstacles

Three static models are placed in the world to test the LiDAR detection:

1. **Red Box** (`box_obstacle`) at (5, 2).
2. **Green Cylinder** (`cylinder_obstacle`) at (5, -2).
3. **Grey Wall** (`wall_obstacle`) located behind the objects to act as a background.

---

## 5. Build and Launch Instructions

### 5.1. Start the Environment

Run and enter the Docker container from the repository root:

```bash
./scripts/cmd run
./scripts/cmd bash

```

### 5.2. Build the Workspace

```bash
colcon build
source install/setup.bash

```

### 5.3. Launch Simulation

```bash
gz sim install/lab1/share/lab1/worlds/robot.sdf

```

---

## 6. Interacting with the Robot

### 6.1. Terminal Teleoperation

Since the robot listens to the `/cmd_vel` topic, you can control it by publishing messages from the terminal.

**Move Forward:**

```bash
gz topic -t "/cmd_vel" -m "gz.msgs.Twist" -p "linear: {x: 0.5}, angular: {z: 0.0}"

```

**Rotate Left:**

```bash
gz topic -t "/cmd_vel" -m "gz.msgs.Twist" -p "linear: {x: 0.0}, angular: {z: 0.5}"

```

**Stop:**

```bash
gz topic -t "/cmd_vel" -m "gz.msgs.Twist" -p "linear: {x: 0.0}, angular: {z: 0.0}"

```

### 6.2. Monitoring LiDAR Data

To inspect the raw distance measurements in the terminal:

```bash
gz topic -e -t /lidar | grep -v "intensities"

```

---

**Note:** Ensure the simulation is running by clicking the **Play** button in the Gazebo GUI, otherwise, the robot will not respond to terminal commands.
