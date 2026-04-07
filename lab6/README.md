# Lab 6 — Motion Planning with Nav2 (TurtleBot3, `room_nav2`)

This report summarizes loading the stack, tasks completed, and expected results.

---

## 1. Project description

Lab 6 uses the **Nav2** stack in Gazebo with **map_server**, **AMCL**, **global planner**, **local controller** (DWB by default), **costmaps**, and **bt_navigator**. The world is **`room_nav2`** (8×8 m); the robot is **TurtleBot3 Burger**; goals are sent from **RViz** after localization.

### 1.1 Learning goals
- Start Nav2 with a saved map and AMCL.
- Set **2D Pose Estimate** and **Nav2 Goal** in RViz.
- Tune `nav2_params` for smoother motion and clearer costmaps.
- *(Optional)* Compare **global planners** (NavFn, Smac 2D) and **local controllers** (DWB, Regulated Pure Pursuit).

### 1.2 References
- [Nav2 documentation](https://docs.nav2.org/)
- [Smac 2D Planner](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html)
- [Regulated Pure Pursuit](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)

---

## 2. Package layout

```
lab6/
├── config/
│   ├── nav2_params.yaml              # Default: NavFn (A*) + DWB
│   ├── nav2_params_navfn_dijkstra.yaml
│   ├── nav2_params_smac2d.yaml
│   └── nav2_params_rpp.yaml
├── launch/
│   └── nav2_room_bringup.launch.py
├── maps/
│   ├── room_nav2.yaml / room_nav2.pgm
└── worlds/
  └── room_nav2.sdf
```

**Global vs local:** Global planner finds paths on the saved map; local controller avoids nearby obstacles using the rolling local costmap and outputs `/cmd_vel`.

---

## 3. Setup and launch

```bash
colcon build --packages-select lab6
source install/setup.bash
ros2 launch lab6 nav2_room_bringup.launch.py
```

Optional (increase delay if needed):
```bash
ros2 launch lab6 nav2_room_bringup.launch.py nav2_startup_delay:=12.0
```

Wait for **`lifecycle_manager_navigation`** to complete, then in RViz set **Fixed Frame** to `map`, use **2D Pose Estimate**, then **Nav2 Goal**.

---

## 4. Task completion

### Task 1 — First navigation run
Launch bringup, confirm Gazebo and RViz. Use **2D Pose Estimate**, then **Nav2 Goal**; observe global path, local plan, and costmaps.

### Task 2 — Parameter tuning

| Parameter | Before | After | Effect |
|-----------|--------|-------|--------|
| Local costmap **update/publish freq** | 2 / 1 Hz | **12 / 10 Hz** | Less blur of obstacles |
| Local costmap **resolution** | 0.2 m | **0.05 m** | Finer cells, matches map |
| Controller **frequency** | 4 Hz | **20 Hz** | Snappier control |
| **max_vel_x** / **max_vel_theta** | 1.2 m/s / 2.5 rad/s | **0.45 m/s / 1.2 rad/s** | Calmer, smoother motion |
| **goal_checker** tolerances | 0.75 m / 1.25 rad | **0.2 m / 0.35 rad** | Meaningful final pose |
| **inflation_radius** | 1.0 | **0.65 & 0.55** | Smoother cost falloff |

**Key improvements:** Higher local costmap rates reduce drift/blur; lower speeds improve stability; tighter goal tolerances ensure proper alignment.

### Task 3 — Global planner comparison

| Planner | Config | Command |
|---------|--------|---------|
| NavFn + A* | `nav2_params.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py` |
| NavFn Dijkstra | `nav2_params_navfn_dijkstra.yaml` | `params_profile:=navfn_dijkstra` |
| Smac 2D | `nav2_params_smac2d.yaml` | `params_profile:=smac2d` |

Compare the **shape of global paths** in RViz.

### Task 4 — Local controller comparison

| Controller | Config | Command |
|-----------|--------|---------|
| DWB | `nav2_params.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py` |
| Regulated Pure Pursuit | `nav2_params_rpp.yaml` | `params_profile:=rpp` |

Compare **path tracking smoothness** and **final approach behavior**.

---

## 5. Expected results

1. **Gazebo** — TurtleBot3 executes planned motion toward goal with minimal recoveries.
2. **RViz** — Stable global path; local costmap tracks obstacles without excessive smearing; LaserScan and Footprint consistent with environment.
3. **Terminal** — `bt_navigator` reports start/finish; no persistent timeouts.

---

## Summary

Lab 6 demonstrates the full Nav2 pipeline in simulation. Rebuild **`lab6`**, source the workspace, run **`nav2_room_bringup.launch.py`**, localize with **2D Pose Estimate**, then send goals. Tuning focuses on costmap update rates, resolutiongit remote add origin https://github.com/ваш_логін/назва_репозиторію.git, controller speed, and goal tolerances. Optional tasks include comparing global planners and local controllers via separate YAML profiles.

