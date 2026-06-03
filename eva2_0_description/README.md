# eva2_0_description

**Owner:** Jai Bhugra  
**Role:** Arm Control Lead

---

## Contents

- `urdf/eva2_0.urdf` — Complete EVA 2.0 robot URDF (base, torso, both arms 5-DOF each, head 3-DOF + Kinect)
- `scripts/fk_l_node.py` — Forward kinematics node for left arm using DH parameters (P4-T1) ✅
- `scripts/fk_r_node.py` — Forward kinematics node for right arm using DH parameters (P4-T1) ✅
- `scripts/fk_h_node.py` — Forward kinematics node for head + Kinect (P4-T1) ✅
- `scripts/servo_test.py` — Basic servo control test node (P1-T2)
- `setup.py` — Python package config with ROS2 entry points
- `package.xml` — ROS2 package metadata

---

## Quick Start

### Setup Workspace (First Time)

```bash
# Create and navigate to workspace
mkdir -p ~/eva2_0_ws/src
cd ~/eva2_0_ws/src

# Get the package
git clone https://github.com/JaideepChouhan/eva2.git
cp -r eva2/eva2_0_description .

# Build
cd ~/eva2_0_ws
colcon build
source install/setup.bash
```

### Run FK Nodes

**Terminal 1 — Left Arm:**
```bash
source ~/eva2_0_ws/install/setup.bash
~/eva2_0_ws/install/eva2_0_description/bin/fk_l_node
```

**Terminal 2 — Right Arm:**
```bash
source ~/eva2_0_ws/install/setup.bash
~/eva2_0_ws/install/eva2_0_description/bin/fk_r_node
```

**Terminal 3 — Head + Kinect:**
```bash
source ~/eva2_0_ws/install/setup.bash
~/eva2_0_ws/install/eva2_0_description/bin/fk_h_node
```

### Publish Test Joint States

```bash
source ~/eva2_0_ws/install/setup.bash
python3 test_joint_states.py
```

### Echo Published Poses

```bash
source ~/eva2_0_ws/install/setup.bash

# Left arm end effector
ros2 topic echo /eva2_0/left_arm/end_effector_pose

# Right arm end effector
ros2 topic echo /eva2_0/right_arm/end_effector_pose

# Head + Kinect camera
ros2 topic echo /eva2_0/head/kinect_node
```

---

## Joint Structure

**Left Arm (5-DOF):**
- `left_shoulder_pan_joint` — Z-axis rotation
- `left_shoulder_lift_joint` — Y-axis rotation
- `left_elbow_joint` — Y-axis rotation
- `left_wrist_roll_joint` — Z-axis rotation
- `left_wrist_pitch_joint` — Y-axis rotation

**Right Arm (5-DOF):** Mirror of left arm

**Head (3-DOF):**
- `head_pan_joint` — Z-axis (yaw)
- `head_tilt_joint` — Y-axis (pitch)
- `head_roll_joint` — X-axis (roll)
- `kinect_link` — Fixed to head_roll

---

## Forward Kinematics

### Left Arm DH Parameters

| Joint | θ (rad) | d | a | α |
|---|---|---|---|---|
| shoulder_pan | q₁ | 0.06 | 0 | -π/2 |
| shoulder_lift | q₂ | 0 | 0 | 0 |
| elbow | q₃ | 0.18 | 0 | +π/2 |
| wrist_roll | q₄ | 0.16 | 0 | -π/2 |
| wrist_pitch | q₅ | 0.04 | 0 | 0 |

### Right Arm DH Parameters
Mirrored left arm (X-axis negated for shoulder offset)

### Head Kinematics
Direct axis-angle chaining: `base_link → torso → neck → pan → tilt → roll → kinect`

### Computation Flow

1. **Input:** `/joint_states` topic (all 13 joint angles)
2. **Extract:** Joint angles by name (name-based filtering, order-independent)
3. **FK Computation:** Chain transformation matrices
4. **Output:** Pose (position + orientation as quaternion)

---

## Phase Status

### P4 — Kinematics & Motion Planning

- [x] **P4-T1:** URDF model complete
- [x] **P4-T2:** FK node for left arm (DH-based)
- [x] **P4-T2:** FK node for right arm (DH-based, mirrored)
- [x] **P4-T2:** FK node for head + Kinect
- [ ] **P4-T3:** Motion Planning node (IK + trajectory generation)
- [ ] **P4-T4:** Left arm controller
- [ ] **P4-T5:** Right arm controller
- [ ] **P4-T6:** Head controller
- [ ] **P4-T7:** A* path planning

---

## Build & Development

```bash
# Full clean rebuild
cd ~/eva2_0_ws
rm -rf build install log
colcon build
source install/setup.bash

# Check installed executables
ls ~/eva2_0_ws/install/eva2_0_description/bin/

# List all ROS2 topics
ros2 topic list

# Check package info
ros2 pkg prefix eva2_0_description
```

---

## Notes

- All joint angles are read from `/joint_states` using joint name matching (not order-dependent)
- FK chains include fixed transforms from `base_link → torso → [arm/head base]`
- Poses are published in `base_link` frame
- All nodes use `scipy.spatial.transform.Rotation` for quaternion conversion

---

> **Last Updated:** June 3, 2026  
> **Status:** All FK nodes working and tested ✅  
> **Next:** Motion Planning node (IK) — JB-03
