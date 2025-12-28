# Module 1: The Robotic Nervous System (ROS 2)

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is not actually an operating system, but rather a flexible framework for writing robot software. It's middleware that provides services designed specifically for robotics applications, such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

Think of ROS 2 as the "nervous system" of a robot. Just as your nervous system connects your brain to your limbs and sensory organs, allowing them to communicate and work together, ROS 2 connects different parts of a robot system to enable coordinated behavior.

### Why Use ROS 2 as Robot Middleware?

ROS 2 offers several key advantages:

- **Standardization**: Provides standardized interfaces for robot hardware and software components
- **Modularity**: Allows different robot functions to be developed as separate, reusable modules
- **Communication**: Enables seamless communication between different robot components
- **Community Support**: Large community contributing packages and solutions
- **Cross-Platform**: Works across different operating systems (Linux, Windows, macOS)
- **Scalability**: Supports distributed computing across multiple machines

## Core ROS 2 Concepts

### Nodes

A **Node** is the basic unit of computation in ROS 2. It's a process that performs computation and communicates with other nodes. Think of nodes as different departments in a company - each has its own responsibilities but needs to communicate with others to achieve the overall goal.

In a robot system, you might have:
- Sensor nodes (reading cameras, lidars, IMUs)
- Control nodes (managing motors, actuators)
- Planning nodes (path planning, navigation)
- Perception nodes (object detection, SLAM)

**Example Node Structure:**
```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        # Node initialization code here
```

### Topics

**Topics** enable asynchronous communication between nodes using a publish-subscribe pattern. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Communication is one-way (publisher → topic → subscriber)

**Real-world example**: Think of a topic like a radio station. The radio station (publisher) broadcasts music, and anyone with a radio (subscriber) can tune in to receive that broadcast. Multiple people can listen to the same station simultaneously.

Common topics in robot systems:
- `/camera/image_raw` - Raw camera images
- `/scan` - Laser scan data
- `/cmd_vel` - Velocity commands for robot movement
- `/tf` - Transform data between coordinate frames

### Services

**Services** enable synchronous, request-response communication between nodes. This follows a client-server model where a client sends a request and waits for a response from the server.

- Communication is synchronous (client waits for response)
- Request-response pattern
- Only one server typically handles requests

**Real-world example**: Like asking a question to a customer service representative. You ask your question (request) and wait for the response before continuing.

Common services in robot systems:
- `/get_map` - Request the current map
- `/set_parameters` - Set robot parameters
- `/navigate_to_pose` - Request navigation to a specific location

## Python AI Agents and rclpy

Python AI agents communicate with robots using `rclpy`, which is the Python client library for ROS 2. This library provides the interface between Python code and the ROS 2 system.

### Setting Up Communication

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
    def scan_callback(self, msg):
        # Process laser scan data
        self.get_logger().info(f'Distance to obstacle: {msg.ranges[0]}')
```

### Publishing Messages

```python
def send_velocity_command(self, linear_x, angular_z):
    msg = Twist()
    msg.linear.x = linear_x  # Forward/backward speed
    msg.angular.z = angular_z  # Rotation speed
    self.publisher.publish(msg)
```

### Using Services

```python
from example_interfaces.srv import AddTwoInts

def call_service(self, a, b):
    client = self.create_client(AddTwoInts, 'add_two_ints')
    if not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Service not available')
        return None
    
    request = AddTwoInts.Request()
    request.a = a
    request.b = b
    
    future = client.call_async(request)
    return future
```

## URDF Basics for Humanoid Robots

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF defines the physical structure, including links, joints, and sensors.

### Links

**Links** represent rigid bodies of the robot. Each link has physical properties and visual/collision representations.

**Properties of a link:**
- **Visual**: How the link looks (shape, color, mesh)
- **Collision**: How the link interacts with the environment
- **Inertial**: Mass, center of mass, and inertia properties

**Example Link Definition:**
```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="white">
      <color rgba="1 1 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.008"/>
  </inertial>
</link>
```

### Joints

**Joints** define how links connect and move relative to each other. Different joint types enable different types of motion.

**Common Joint Types:**
- **Fixed**: No movement (permanent connection)
- **Revolute**: Rotational movement around one axis (like an elbow)
- **Continuous**: Rotational movement without limits (like a wheel)
- **Prismatic**: Linear sliding movement (like a piston)

**Example Joint Definition:**
```xml
<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0.0 0.0 0.8" rpy="0.0 0.0 0.0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
</joint>
```

### Sensors

Sensors in URDF describe where sensors are located on the robot and their properties. This is crucial for simulation and real-world applications.

**Common Sensors in Humanoid Robots:**
- **Cameras**: For vision-based perception
- **IMU**: For balance and orientation
- **Force/Torque sensors**: For interaction with the environment
- **Lidar**: For distance measurement and mapping

**Example Sensor Definition:**
```xml
<gazebo reference="head">
  <sensor name="head_camera" type="camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## Practical Example: Simple Humanoid Robot URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0.0 0.0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>
</robot>
```

## Summary

ROS 2 serves as the nervous system for robots, connecting different components through Nodes, Topics, and Services. Python AI agents communicate with robots using the rclpy library, enabling sophisticated robot control and perception. URDF provides a standardized way to describe humanoid robot structures, defining how links connect through joints and where sensors are positioned.

Understanding these concepts is fundamental to working with modern robotic systems, providing the foundation for more advanced robot applications. As you continue learning robotics, you'll find that these core concepts are used across a wide range of robot platforms and applications.