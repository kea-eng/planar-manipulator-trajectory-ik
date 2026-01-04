# planar-manipulator-trajectory-ik
# Actuator-Space Inverse Kinematics for a Planar Manipulator

This repository demonstrates a generic actuator-space inverse kinematics (IK) 
framework for a 3-DOF planar manipulator driven by linear actuators.

The system generates smooth task-space trajectories using cubic polynomial 
profiles and maps them to actuator stroke setpoints via inverse kinematics.

> **Note:** All parameters, geometries, and test scenarios are synthetic and do 
> not represent any proprietary or real-world system.

---

## Key Features

- 3-DOF planar manipulator model
- Task-space trajectory generation using cubic polynomial profiles
- Fixed-slope straight-line tracking (tool follows a single direction)
- Inverse kinematics with continuity and joint limit checks
- Synthetic actuator (stroke) mapping with constraints
- Mechanism animation and video export (MATLAB)

---

## Trajectory Definition

The end-effector follows a straight line defined as:

\[
p(t) = p_0 + d(t)\,\hat{u}
\]

where:
- \(p_0\) is the start point
- \(\hat{u}\) is the unit direction vector
- \(d(t)\) is a cubic polynomial ensuring smooth velocity and acceleration
  profiles with zero start/end velocity.

---

## How to Run

1. Open MATLAB
2. Navigate to the `src` directory
3. Run:

```matlab
planar_actuated_manipulator_line_demo_with_video
