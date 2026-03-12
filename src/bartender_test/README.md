# Bartender Test Package (Robotender)

This package provides ROS 2 nodes for controlling a Doosan E0509 robot for automated beverage pouring.

---

## 🍹 Pouring Implementation Detail

### 1. `action pour move` (Circular)
- **Implementation:** Uses `movec` to define a circular arc.
- **Trajectory:** split into Horizontal (Cheers -> Pour_Horizontal) and Vertical (Pour_Horizontal -> Pour_Vertical).
- **Why `movec` is not sufficient:** Circular motion is constrained to a perfect geometric arc. A bottle's center of mass and mouth do not naturally follow a perfect circle during a pour; attempting to force this often leads to inconsistent flow and awkward joint orientations.

### 2. `action pour orbit` (Spline)
- **Implementation:** Uses `movesx` (Spline) to guide the TCP through a smooth 5-point path.
- **Why `orbit` is better:**
  - **Natural Curve:** Spline motion fits the complex, non-circular path required for pouring liquid from a bottle.
  - **Continuous Flow:** Ensures the robot doesn't pause between points, maintaining a steady stream.
  - **Orientation:** Provides superior control over the bottle's tilt angle throughout the motion.

---

## 📁 File Descriptions

| File | Purpose |
| :--- | :--- |
| `action.py` | Core node for high-level tasks (`pour move`, `pour orbit`, `warmup`). |
| `defines.py` | Constants for Joint poses and Cartesian target coordinates (POS1-POS5). |
| `monitor.py` | 1Hz monitor for live Joint and XYZ feedback. |
| `pose.py` | Simple CLI to move to named poses (home, cheers, etc.). |
| `gripper_command.py` | CLI for opening and closing the gripper with force control. |
| `gripper_controller.py` | Internal library for gripper communication. |
| `check_tcp.py` | Utility to print current Tool Center Point coordinates. |
| `calc_mouth_world.py` | **(Utility)** Calculates bottle mouth position in world frame. |
| `calc_marker_pos1.py` | **(Utility)** Calculates simulation marker positions based on joint angles. |
| `movej.py` / `movec.py` | Basic testing scripts for joint and circular moves. |

---

## 🚀 Usage

> **Note:** All commands must be run from the workspace root: `cd ~/bartender-robot/robot` (or your local workspace folder).

### **Setup**
```bash
# 1. Install dependencies
rosdep install --from-paths src --ignore-src -y

# 2. Build the workspace (First-time or All)
colcon build --symlink-install --allow-overriding dsr_description2

# 3. Source the environment
source install/setup.bash

# Incremental Build (Recommended for changes in bartender_test)
colcon build --symlink-install --packages-select bartender_test dsr_description2 --allow-overriding dsr_description2
```

### **Run Simulation**
```bash
ros2 launch dsr_bringup2 dsr_bringup2.launch.py mode:=virtual model:=e0509 gripper:=rh_p12_rn object:=bottle
```

### **Run Real Robot**
```bash
ros2 launch dsr_bringup2 dsr_bringup2.launch.py mode:=real host:=110.120.1.59 model:=e0509
```

### **Main Actions**
- `ros2 run bartender_test action pour move [horizontal|vertical] [repeat=n]`
- `ros2 run bartender_test action pour orbit [<start_idx>] [<end_idx>] [repeat=n]`
- `ros2 run bartender_test action warmup [repeat=n]`
- `ros2 run bartender_test monitor [xyz|j]`
