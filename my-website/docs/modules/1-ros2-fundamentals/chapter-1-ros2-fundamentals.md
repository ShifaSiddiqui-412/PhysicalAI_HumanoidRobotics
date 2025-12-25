---
title: Chapter 1 - ROS 2 Fundamentals
sidebar_position: 2
description: Understanding ROS 2 communication primitives - nodes, topics, and services for humanoid robotics
tags: [ros2, fundamentals, nodes, topics, services, communication]
keywords: [ros2, nodes, topics, services, communication primitives, publish-subscribe, request-response, middleware]
---


# Chapter 1: ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the concept of nodes, topics, and services in ROS 2
- Identify nodes, topics, and services in a robotic system diagram
- Understand the publish-subscribe communication model
- Understand the request-response communication model

## Introduction

ROS 2 (Robot Operating System 2) is not an operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The communication layer in ROS 2 is what enables different software components to interact with each other.

## Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. They are organized in a distributed fashion, meaning they can run on different machines and still communicate with each other.

### Characteristics of Nodes

- Nodes are processes that perform computation
- Multiple nodes can run on a single machine or be distributed across multiple machines
- Nodes are typically organized around a single purpose (e.g., controlling a laser scanner, processing camera data)
- Nodes communicate with each other through messages

### Creating a Node

In ROS 2, nodes are created using client libraries such as `rclpy` (Python) or `rclcpp` (C++). Here's a simple example of a node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Messages

**Topics** are named buses over which nodes exchange messages. They use a publish-subscribe communication model where publishers send messages to topics, and subscribers receive messages from topics.

### Publish-Subscribe Model

- Publishers send messages to topics without knowing who will receive them
- Subscribers receive messages from topics without knowing who sent them
- This decouples the publishers and subscribers in time and space
- Multiple publishers can publish to the same topic
- Multiple subscribers can subscribe to the same topic

### Message Types

Messages are data structures that are passed between nodes. They have specific types that define their structure. Common message types include:
- `std_msgs/String` - for string data
- `std_msgs/Int32` - for integer data
- `sensor_msgs/Image` - for image data
- Custom message types for specific applications

## Services

**Services** provide a request-response communication model. A client sends a request to a service, and the service sends back a response. This is useful for operations that require a specific response or for actions that need to be completed before continuing.

### Request-Response Model

- Services are synchronous (the client waits for a response)
- Useful for operations that must complete before proceeding
- Good for configuration, control commands, or data requests

### Service Example

Here's an example of a service call in Python:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

**Actions** are another communication pattern in ROS 2 that provide a request-response model with feedback and goal preemption. They are useful for long-running tasks where you want to track progress and potentially cancel the operation.

## Architecture and Communication Patterns

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware. This provides:

- **Discovery**: Nodes automatically discover each other on the network
- **Transport**: Reliable and best-effort communication options
- **Quality of Service (QoS)**: Configurable communication policies for different requirements

## Summary

In this chapter, we've covered the fundamental communication primitives of ROS 2:
- **Nodes**: The basic computational units
- **Topics**: Publish-subscribe communication
- **Services**: Request-response communication
- **Actions**: Goal-oriented communication with feedback

These primitives form the foundation of how software components communicate in a ROS 2 system, enabling the creation of complex robotic applications.

## Summary

In this chapter, we've covered the fundamental communication primitives of ROS 2:
- **Nodes**: The basic computational units
- **Topics**: Publish-subscribe communication
- **Services**: Request-response communication
- **Actions**: Goal-oriented communication with feedback

These primitives form the foundation of how software components communicate in a ROS 2 system, enabling the creation of complex robotic applications.

## Exercises/Assessment

1. Draw a simple diagram showing 3 nodes communicating using topics. Label the publishers and subscribers.
2. Explain when you would use a service instead of a topic.
3. What is the difference between a message and a topic?
4. Describe a scenario in humanoid robotics where you might use each of the three communication patterns (topics, services, actions).

## Next Steps

Continue to [Chapter 2: Python Agents with rclpy](./chapter-2-python-agents.md) to learn how to implement these concepts in Python.