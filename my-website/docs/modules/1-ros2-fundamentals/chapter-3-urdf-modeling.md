---
title: Chapter 3 - Humanoid Modeling with URDF
sidebar_position: 4
description: Understanding and creating URDF files for humanoid robot modeling and simulation
tags: [urdf, modeling, humanoid, robot, simulation]
keywords: [urdf, robot modeling, humanoid robot, simulation, links, joints, robot description, xml]
---


# Chapter 3: Humanoid Modeling with URDF

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF files
- Interpret URDF files for humanoid robots
- Modify URDF files to change robot properties
- Create basic URDF models for simple robots

## Introduction

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, and other components. URDF is essential for robot simulation, visualization, and kinematic analysis in ROS-based systems.

## URDF Structure and Components

A URDF file describes a robot as a collection of links connected by joints. The main components are:

- **Links**: Rigid bodies with visual, collision, and inertial properties
- **Joints**: Connections between links with specific degrees of freedom
- **Materials**: Visual properties like color and texture
- **Gazebo plugins**: Simulation-specific properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have:

- **Visual**: How the link appears in visualization tools
- **Collision**: How the link interacts in collision detection
- **Inertial**: Physical properties for dynamics simulation

### Visual Properties

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
    <!-- or -->
    <cylinder radius="0.1" length="0.2"/>
    <!-- or -->
    <sphere radius="0.1"/>
    <!-- or -->
    <mesh filename="package://my_robot/meshes/link.stl"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Properties

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Properties

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

## Joints

Joints connect links and define their relative motion. Common joint types include:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (rigid connection)
- **floating**: 6 DOF with limits
- **planar**: Movement on a plane

### Joint Definition Example

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

## URDF for Humanoid Robots

Humanoid robots have a specific structure with body parts that include:

- **Torso**: Main body with head, arms, and legs attached
- **Head**: With sensors like cameras
- **Arms**: Shoulders, elbows, wrists, and hands
- **Legs**: Hips, knees, ankles, and feet

### Example Humanoid Link Structure

```xml
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Common URDF Tags and Attributes

- `<robot>`: Root element with the robot name
- `<link>`: Defines a rigid body with name attribute
- `<joint>`: Defines connection between links with name and type attributes
- `<visual>`: Defines how the link appears visually
- `<collision>`: Defines collision properties
- `<inertial>`: Defines mass and inertial properties
- `<geometry>`: Defines shape (box, cylinder, sphere, mesh)
- `<material>`: Defines visual appearance
- `<origin>`: Position and orientation (xyz coordinates and rpy angles)
- `<axis>`: Joint axis of rotation or translation
- `<limit>`: Joint limits for revolute and prismatic joints

## URDF Best Practices

1. **Use meaningful names**: Choose descriptive names for links and joints
2. **Organize hierarchically**: Structure the robot as a tree with a single base link
3. **Include inertial properties**: For proper simulation and dynamics
4. **Use consistent units**: Typically meters for length, kilograms for mass
5. **Validate your URDF**: Use tools like `check_urdf` to verify syntax

## Tools for Working with URDF

- **RViz**: Visualize URDF models in ROS
- **Gazebo**: Simulate robots with physics
- **URDF Tutorials**: Official ROS tutorials for learning URDF
- **xacro**: Macro language for URDF that allows parameterization

## Example: Simple Humanoid Robot

Here's a more complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.65" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </collision>
  </link>
</robot>
```

## Summary

URDF is a fundamental component of ROS-based robotics, providing a standardized way to describe robot models. Understanding URDF is crucial for:
- Robot simulation in Gazebo
- Robot visualization in RViz
- Kinematic analysis and motion planning
- Integration with robot middleware

## Summary

URDF is a fundamental component of ROS-based robotics, providing a standardized way to describe robot models. Understanding URDF is crucial for:
- Robot simulation in Gazebo
- Robot visualization in RViz
- Kinematic analysis and motion planning
- Integration with robot middleware

## Exercises/Assessment

1. Given a simple URDF file, identify the base link, end effectors, and joint types.
2. Modify the example URDF to add a right arm with the same structure as the left arm.
3. Explain the difference between visual, collision, and inertial properties in a link.
4. Create a simple URDF for a 2-wheeled robot with a caster wheel.
5. What would happen if you forgot to include inertial properties in your URDF?

## Next Steps

Return to the [Module Overview](./index.md) or explore other robotics topics in our curriculum.