# Laboratory 7 Report

## Coordinate Transforms (TF2), URDF/Xacro, and RTR Manipulator

**Course:** Robotics (ROS 2)  
**Package:** `lab7`

---

## 1. Objective

This laboratory focuses on implementing TF2-based pose handling for an RTR manipulator, describing the robot structure using URDF/Xacro, and integrating with ROS 2 Control. Key tasks include:

- Broadcasting and listening to dynamic TF transforms with analytical validation
- Creating and visualizing robot models in URDF/Xacro format using RViz2
- Setting up ROS 2 Control with mock hardware, state broadcasting, and position control
- Validating kinematics through automated tests

---

## 2. Theory

The RTR manipulator features three degrees of freedom:

- \(\theta_1\): base rotation (revolute)
- \(\theta_2\): vertical movement (prismatic)
- \(\theta_3\): elbow rotation (revolute)

End-effector position \(\mathbf{p} = (x,y,z)^T\) is computed via forward kinematics:

\[
x = \cos\theta_1\,(l_3\cos\theta_3 + l_2), \quad
y = \sin\theta_1\,(l_3\cos\theta_3 + l_2), \quad
z = l_3\sin\theta_3 + \theta_2
\]

TF2 manages coordinate frame transformations, while URDF/Xacro specifies the robot's kinematic structure.

---

## 3. Package Structure

| Component | Description |
|-----------|-------------|
| `lab7/rtr_kinematics.py` | Forward kinematics computation and pose handling |
| `lab7/tf2_demo_cli.py` | Argument parser for joint angles and link lengths |
| `lab7/tf2_broadcaster_demo.py` | TF publisher for `world → rtr_ee_demo` |
| `lab7/tf2_listener_demo.py` | TF reader with validation against analytical results |
| `urdf/rtr_manipulator.xacro` | Robot model with ROS 2 Control integration |
| `launch/rtr_visualize.launch.py` | Visualization pipeline with joint GUI and RViz2 |
| `launch/rtr_ros2_control.launch.py` | Control system with spawner nodes |
| `config/rtr_controllers.yaml` | Controller configuration |
| `tests/test_rtr_kinematics.py` | Kinematics unit tests |
| `tests/test_tf2_analytic_agreement.py` | TF/analytical comparison tests |

---

## 4. Implementation and Results

### 4.1 Setup

```bash
cd /opt/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lab7 --symlink-install
source install/setup.bash
```

### 4.2 TF2 Validation

Test commands:
```bash
ros2 run lab7 tf2_broadcaster_demo -- 0.2 0.5 0.35
ros2 run lab7 tf2_listener_demo -- 0.2 0.5 0.35
ros2 run tf2_ros tf2_echo world rtr_ee_demo
```

For \(\theta_1=0.2, \theta_2=0.5, \theta_3=0.35, l_2=0.9, l_3=1.0\):

| Parameter | Analytical | TF Echo | Difference |
|-----------|------------|---------|-----------|
| x | 1.802701 | 1.803 | 0.000299 |
| y | 0.365445 | 0.365 | 0.000445 |
| z | 0.842898 | 0.843 | 0.000102 |

Values align within rounding tolerance.

### 4.3 URDF/Xacro Refinement

Updated `urdf/rtr_manipulator.xacro`:
- Parameters: `l2=1.0`, `l3=1.1`, `width=0.07`
- Added collision geometry to five links

Verification:
```bash
grep -n "<collision>" /opt/ws/src/code/lab7/urdf/rtr_manipulator.xacro
```

### 4.4 ROS 2 Control Integration

```bash
ros2 launch lab7 rtr_ros2_control.launch.py
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2, 0.6, 0.4]}"
ros2 run tf2_ros tf2_echo base_link tool0
```

Control chain: mock hardware → joint_state_broadcaster → robot_state_publisher → TF

### 4.5 Test Results

```bash
colcon test --packages-select lab7
```

All tests passed successfully.

---

## 5. Analysis

Two configurations validated kinematic correctness:

1. **Config A** - Demo pipeline with updated analytical parameters
2. **Config B** - Control system with URDF-based kinematics

Model constraints:
- Ideal rigid-body dynamics
- Mock hardware simulation
- Simplified collision geometry

---

## 6. Figures (to attach)

- RTR manipulator kinematic diagram
- TF2 broadcaster/listener outputs
- `tf2_echo` transform data
- `joint_state_publisher_gui` interface with joint sliders
- RViz2 visualization of TF tree
- Xacro parameter updates with collision blocks
- Post-refinement RViz representation
- Control command execution and TF response

---

## 7. Conclusion

TF2 broadcasting/listening, URDF/Xacro design, and ROS 2 Control implementation were successfully demonstrated. Analytical kinematics and TF outputs matched across test cases. Model parameters were refined and collision geometry added. All validation tests passed, confirming functional integration of kinematics, transforms, and control in ROS 2.

---



