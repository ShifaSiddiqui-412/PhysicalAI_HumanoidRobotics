---
title: Chapter 3 - Nav2 for Humanoid Navigation
sidebar_position: 3
description: Navigation systems and path planning for humanoid robots using Nav2
tags: [nav2, navigation, path planning, humanoid, robotics, autonomous navigation]
keywords: [nav2, navigation, path planning, humanoid, robotics, autonomous navigation, slam, localization]
---

# Chapter 3: Nav2 for Humanoid Navigation

This chapter introduces students to Nav2 (Navigation 2), ROS 2's navigation framework, specifically adapted for humanoid robot navigation challenges and path planning.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand Nav2 architecture and its application to humanoid robots
- Install and configure Nav2 for humanoid navigation
- Configure Nav2 behavior trees for complex navigation tasks
- Implement path planning algorithms for bipedal robots
- Create obstacle avoidance systems for humanoid navigation
- Integrate perception systems with navigation
- Understand safety considerations for humanoid navigation
- Implement dynamic obstacle handling techniques

## Introduction to Nav2

Nav2 (Navigation 2) is ROS 2's state-of-the-art navigation framework that provides comprehensive tools for autonomous robot navigation. For humanoid robots, Nav2 requires specialized configuration to handle the unique challenges of bipedal locomotion and human-like navigation patterns.

### Key Components

- **Costmap 2D**: Dynamic costmap management for navigation
- **Path Planner**: Global and local path planning algorithms
- **Controller**: Local trajectory control for robot movement
- **Behavior Trees**: Flexible navigation behavior composition
- **Recovery**: Automatic recovery from navigation failures

## Nav2 Architecture for Humanoid Robots

### Core Architecture

```
[Map] -> [Costmap 2D] -> [Global Planner] -> [Local Planner] -> [Controller] -> [Robot]
                    |-> [Behavior Tree]
                    |-> [Recovery Behaviors]
```

### Humanoid-Specific Considerations

Humanoid robots present unique navigation challenges:

- **Bipedal Locomotion**: Different movement patterns than wheeled robots
- **Balance Requirements**: Navigation must consider center of gravity
- **Step Planning**: May require discrete footstep planning
- **Human-like Paths**: More natural path following patterns

## Installation and Configuration

### System Requirements

- ROS 2 Humble Hawksbill or later
- Navigation2 packages
- Appropriate simulation or hardware platform
- Localization system (AMCL, SLAM)

### Installation Process

```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui

# Install additional dependencies
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-slam-toolbox
```

## Behavior Tree Configuration

### Behavior Tree Concepts

Nav2 uses behavior trees to compose navigation behaviors flexibly:

- **Sequences**: Execute nodes in order until one fails
- **Fallbacks**: Try nodes in order until one succeeds
- **Decorators**: Modify node behavior
- **Conditions**: Check for specific states
- **Actions**: Execute navigation tasks

### Humanoid-Specific Behavior Tree

```xml
<!-- Example humanoid navigation behavior tree -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="navigate_to_goal">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearGlobalCostmap-1" service_name="global_costmap/clear_entirely_global_costmap"/>
            <ClearEntireCostmap name="ClearLocalCostmap-1" service_name="local_costmap/clear_entirely_local_costmap"/>
            <GoalReached/>
            <PipelineSequence name="global_plan">
                <RateController hz="1.0">
                    <RecoveryNode number_of_retries="6" name="PlanRecovery">
                        <GlobalPlanner/>
                        <RecoveryActions>
                            <ClearEntireCostmap name="ClearGlobalCostmap-2" service_name="global_costmap/clear_entirely_global_costmap"/>
                        </RecoveryActions>
                    </RecoveryNode>
                </RateController>
            </PipelineSequence>
            <PipelineSequence name="local_plan">
                <RateController hz="20.0">
                    <RecoveryNode number_of_retries="6" name="ControlRecovery">
                        <ComputePathToPose goal="{goal}" path="{path}"/>
                        <LocalPlanner goal="{goal}" path="{path}" velocity="{velocity}"/>
                        <RecoveryActions>
                            <ClearEntireCostmap name="ClearLocalCostmap-2" service_name="local_costmap/clear_entirely_local_costmap"/>
                            <BackUp distance="0.15" speed="0.025" time_allowance="2.0"/>
                            <Spin spin_dist="1.57"/>
                            <Wait wait_duration="5"/>
                        </RecoveryActions>
                    </RecoveryNode>
                </RateController>
            </PipelineSequence>
        </Sequence>
    </BehaviorTree>
</root>
```

## Path Planning for Bipedal Robots

### Humanoid Path Planning Challenges

Bipedal robots have different path planning requirements:

- **Step Constraints**: Must consider discrete foot placement
- **Balance Preservation**: Paths must maintain center of gravity
- **Dynamic Stability**: Consider robot's dynamic balance during movement
- **Gait Patterns**: Integrate with specific walking patterns

### Path Planning Algorithms

```yaml
# Humanoid-specific path planner configuration
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  static_map: true
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 6
  height: 6
  resolution: 0.05
  plugins:
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

# Humanoid-specific planner configuration
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Humanoid-specific behavior tree
    default_nav_through_poses_bt_xml: "humanoid_nav_through_poses_bt.xml"
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose_bt.xml"
```

### Footstep Planning Integration

For advanced humanoid navigation, integrate with footstep planners:

```python
# Conceptual footstep planning integration
class HumanoidPathPlanner:
    def __init__(self):
        # Initialize standard path planner
        self.global_planner = GlobalPlanner()

        # Initialize footstep planner for bipedal navigation
        self.footstep_planner = FootstepPlanner()

    def plan_path(self, start, goal):
        # Plan high-level path using standard planner
        global_path = self.global_planner.plan(start, goal)

        # Convert to footstep plan for humanoid locomotion
        footstep_plan = self.footstep_planner.plan(global_path)

        return footstep_plan
```

## Obstacle Avoidance Implementation

### Dynamic Obstacle Handling

Humanoid robots must handle various obstacle types:

- **Static Obstacles**: Fixed environment obstacles
- **Dynamic Obstacles**: Moving humans, objects
- **Human-like Obstacles**: Other humanoid robots, pedestrians

### Local Planner Configuration

```yaml
# Humanoid-specific local planner configuration
local_planner:
  ros__parameters:
    # Humanoid-specific velocity constraints
    speed_limit_topic: speed_limit
    min_speed: 0.1  # Slower minimum for stability
    max_speed: 0.8  # Conservative maximum for humanoid stability

    # Humanoid-specific trajectory generation
    min_vel_x: 0.05
    max_vel_x: 0.5
    min_vel_theta: -0.3
    max_vel_theta: 0.3

    # Safety margins for humanoid balance
    acc_lim_x: 0.5
    acc_lim_theta: 0.5

    # Humanoid-specific path following
    xy_goal_tolerance: 0.2  # More precise for humanoid navigation
    yaw_goal_tolerance: 0.1
```

## Integration with Perception Systems

### Sensor Fusion for Navigation

Nav2 integrates with various sensors for navigation:

- **LIDAR**: Primary obstacle detection
- **Cameras**: Visual obstacle detection and mapping
- **IMU**: Balance and orientation information
- **Force/Torque**: Foot contact information

### Perception-to-Navigation Pipeline

```python
# Example perception-to-navigation integration
class PerceptionToNavigation:
    def __init__(self):
        # Subscribe to perception outputs
        self.detection_sub = rospy.Subscriber('/perception/detections', DetectionArray, self.detection_callback)
        self.pointcloud_sub = rospy.Subscriber('/perception/pointcloud', PointCloud2, self.pointcloud_callback)

        # Publish to navigation costmap
        self.obstacle_pub = rospy.Publisher('/local_costmap/obstacles', PointCloud2, queue_size=1)

    def detection_callback(self, detections):
        # Convert detections to navigation-relevant obstacles
        obstacles = self.process_detections(detections)
        self.update_costmap(obstacles)

    def pointcloud_callback(self, pointcloud):
        # Process point cloud for navigation obstacles
        processed_cloud = self.filter_for_navigation(pointcloud)
        self.obstacle_pub.publish(processed_cloud)
```

## Safety Considerations for Humanoid Navigation

### Safety Requirements

Humanoid navigation must prioritize safety:

- **Fall Prevention**: Avoid paths that risk robot falling
- **Human Safety**: Maintain safe distances from humans
- **Environmental Safety**: Avoid fragile or dangerous areas
- **Emergency Stops**: Immediate stopping capabilities

### Safety Configuration

```yaml
# Safety configuration for humanoid navigation
safety_controller:
  ros__parameters:
    # Emergency stop conditions
    max_tilt_angle: 15.0  # Maximum allowable tilt before stop
    min_distance_to_human: 1.0  # Minimum safe distance
    max_deceleration: 2.0  # Emergency stop deceleration

    # Safety monitoring
    safety_check_frequency: 10.0  # Hz
    enable_collision_detection: true
    collision_prediction_horizon: 1.0  # seconds
```

## Dynamic Obstacle Handling

### Human-Aware Navigation

Humanoid robots often navigate in human environments:

- **Social Navigation**: Follow social norms and etiquette
- **Predictive Avoidance**: Predict human movement patterns
- **Comfortable Distances**: Maintain appropriate personal space

### Dynamic Obstacle Configuration

```yaml
# Dynamic obstacle handling configuration
dynamic_obstacles:
  ros__parameters:
    # Human tracking and prediction
    human_tracking_enabled: true
    prediction_horizon: 2.0
    social_force_model: true

    # Dynamic costmap inflation
    dynamic_inflation_radius: 0.8
    temporal_decay_factor: 0.5

    # Human-aware navigation parameters
    prefer_keeping_right: true
    social_zone_radius: 1.2
```

## Practical Implementation Example

### Launching Humanoid Navigation

```bash
# Launch Nav2 for humanoid robot
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=install/nav2_hunterbot/share/nav2_hunterbot/config/nav2_params_humanoid.yaml
```

### Example Navigation Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Wait for navigation server
        self.nav_to_pose_client.wait_for_server()

    def navigate_to_pose(self, x, y, theta):
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        # Send navigation goal
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        return future

def main():
    rclpy.init()
    navigator = HumanoidNavigator()

    # Navigate to specific pose
    future = navigator.navigate_to_pose(5.0, 3.0, 0.0)

    rclpy.spin_until_future_complete(navigator, future)
    rclpy.shutdown()
```

## Troubleshooting and Best Practices

### Common Issues

- **Path Planning Failures**: Check costmap configuration and inflation parameters
- **Oscillation**: Adjust local planner parameters and goal tolerances
- **Performance**: Optimize costmap update frequencies and resolution

### Best Practices

- Start with conservative velocity and acceleration limits
- Test navigation in simulation before real-world deployment
- Regularly update and validate map data
- Implement proper logging and monitoring

## Summary

This chapter covered Nav2 for humanoid navigation, including architecture, behavior trees, path planning for bipedal robots, obstacle avoidance, and safety considerations. You learned how to configure Nav2 specifically for humanoid robots and integrate perception systems with navigation. This completes the Isaac AI Brain module, providing comprehensive coverage of NVIDIA Isaac technologies for humanoid robotics.

## Exercises

1. Configure and run Nav2 with behavior trees for a humanoid robot simulation
2. Implement dynamic obstacle avoidance in a humanoid navigation scenario
3. Integrate perception data with Nav2 costmaps for enhanced navigation