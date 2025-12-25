---
title: Chapter 2 - Python Agents with rclpy
sidebar_position: 3
description: Creating Python-based ROS 2 agents to bridge AI logic to robot controllers
tags: [python, rclpy, agents, ai, robot controllers]
keywords: [python, rclpy, ros2 python, ai robotics, robot controllers, publishers, subscribers, services]
---


# Chapter 2: Python Agents with rclpy

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the rclpy library and its role in ROS 2
- Create Python-based ROS 2 nodes
- Implement communication patterns (publishers, subscribers, services) in Python
- Bridge AI logic to robot controllers using Python agents

## Introduction

Python agents in ROS 2 are implemented using the `rclpy` library, which is the Python client library for ROS 2. Python is a popular choice for implementing AI and higher-level logic in robotic systems due to its simplicity and the availability of powerful libraries for machine learning, computer vision, and other AI-related tasks.

## Understanding rclpy

The `rclpy` library provides Python bindings for the ROS 2 client library (rcl). It allows Python programs to interact with the ROS 2 middleware, enabling communication with other nodes in the system.

### Key Components of rclpy

- **Node**: The basic execution unit in ROS 2
- **Publisher**: Used to send messages to topics
- **Subscriber**: Used to receive messages from topics
- **Service Client**: Used to make service requests
- **Service Server**: Used to provide services
- **Action Client**: Used to send action goals
- **Action Server**: Used to execute action goals

## Creating a Python Node

Let's look at the structure of a basic Python node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code goes here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services in Python

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future
```

## Practical Examples of Python Nodes

### Example 1: Simple Sensor Data Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(Float32, 'sensor_data', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 100.0)  # Simulate sensor reading
        self.publisher.publish(msg)
        self.get_logger().info('Sensor reading: %f' % msg.data)
```

### Example 2: Data Processing Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'processed_data', 10)

    def listener_callback(self, msg):
        # Process the incoming data
        if msg.data > 50.0:
            result = "High value detected: " + str(msg.data)
        else:
            result = "Normal value: " + str(msg.data)

        # Publish the processed result
        output_msg = String()
        output_msg.data = result
        self.publisher.publish(output_msg)
        self.get_logger().info('Processed: %s' % result)
```

## Bridging AI Logic to Robot Controllers

Python agents are particularly useful for bridging AI logic to robot controllers because:

1. **Rich Ecosystem**: Python has extensive libraries for AI, machine learning, and data processing
2. **Rapid Prototyping**: Python allows for quick development and testing of AI algorithms
3. **Integration**: Python can easily interface with both high-level AI systems and low-level robot controllers

### Example: AI-Based Decision Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # For robot movement commands

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            String,
            'sensor_input',
            self.sensor_callback,
            10)

        # Publish movement commands
        self.publisher = self.create_publisher(Twist, 'robot_cmd_vel', 10)

    def sensor_callback(self, msg):
        # Simple AI logic - in real applications, this could involve complex algorithms
        sensor_data = msg.data

        # Example decision logic
        cmd = Twist()
        if "obstacle" in sensor_data:
            # Stop the robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif "target" in sensor_data:
            # Move toward target
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        else:
            # Continue normal operation
            cmd.linear.x = 0.2
            cmd.angular.z = 0.1

        self.publisher.publish(cmd)
        self.get_logger().info(f'AI Decision: {cmd.linear.x}, {cmd.angular.z}')
```

## Best Practices for Python Agents

1. **Error Handling**: Always include proper error handling in your nodes
2. **Resource Management**: Clean up resources when nodes are destroyed
3. **Logging**: Use appropriate logging levels for debugging and monitoring
4. **Parameterization**: Use ROS 2 parameters for configurable values
5. **QoS Settings**: Configure Quality of Service settings appropriately for your use case

## Summary

Python agents with rclpy provide a powerful way to implement AI logic and high-level decision making in ROS 2 systems. They bridge the gap between sophisticated AI algorithms and low-level robot controllers, making them essential components in modern robotic systems.

## Exercises/Assessment

1. Create a Python node that subscribes to a topic called "input_numbers" and publishes the square of each number to a topic called "squared_numbers".
2. Design a Python node that implements a simple PID controller as a service. The service should take current value, target value, and PID parameters, and return the control output.
3. Explain the advantages of using Python for AI-robot controller bridging compared to other languages like C++.
4. What considerations should you take into account when designing a Python node for real-time control applications?

## Next Steps

Continue to [Chapter 3: Humanoid Modeling with URDF](./chapter-3-urdf-modeling.md) to learn about robot modeling and simulation.