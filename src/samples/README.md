# Monorepo Samples
These projects are to be referenced when developing code for the monorepo. It highlights coding conventions and testing practices.

## General Premise
This project contains three arbitrary ROS2 nodes which communicate with each other via ROS2 publishers and subscribers. The general communication pipeline can be summed up by the image below.

![Architecture](samples_diagram.svg)

Each ROS2 node is containerized([Producer](../../docker/samples/cpp/producer.Dockerfile), [Transformer](../../docker/samples/cpp/transformer.Dockerfile), [Aggregator](../../docker/samples/cpp/aggregator.Dockerfile)) and communicate with each other using ROS2 publishers and subscribers. 

### Core Logic and Node Logic
A single ROS2 node in our stack should contain two components, we call this the `WATO Core and Node paradigm`. The Core Logic of a node represents the crucial algorithms needed to augment data. This could be a neural network, control framework, RL algorithm, etc. On the other hand, the Node represents any ROS2 interfaces and practices which enable the node to communicate with the rest of the stack. This includes subscribers, publishers, buffers, etc. 

At times, the boundary between core logic and node logic may be unclear, so a good rule of thumb is that the core logic does not contain function calls from `rclpy` and `rclcpp`. It may contain message types. This rule may be broken in rare cases.

The reason why we do this is to make our code more intuitive to read. If any ROS2-related issues exist, we know to only look in the node component of your code, not the core logic. Another reason is for testing. Having core logic and node logic seperate enables us to not only do integration testing of the nodes, but also unit testing on only the node's logic, bypassing the need to spin up ROS2 within `pytest` and `gtest`. See more about testing [here](../../docs/dev/testing.md).

## Sample Node Descriptions

### Producer
Produces [unfiltered](../ros_msgs/sample_msgs/msg/Unfiltered.msg) coordinate data at 500ms intervals and publishes data to the [Transformer](#transformer) and [Aggregator](#aggregator) nodes. The coordinate data will be incremented according to the 'velocity' parameter. This can be dynamically adjusted with, `ros2 param set /producer velocity <some value>`.

### Transformer
Collects [unfiltered](../ros_msgs/sample_msgs/msg/Unfiltered.msg) coordinate data from the [Producer](#producer) at regular intervals and filters out messages containing invalid fields. Once 10 messages are collected they are packed into a [filtered_array](../ros_msgs/sample_msgs/msg/FilteredArray.msg) message and published to the [Aggregator](#aggregator).

### Aggregator
Listens to messages from the Producer and Transfomer nodes and logs the frequency of messages to the console.

## Usage
**Before proceding ensure that you have followed the setup guide([setup](../../docs/setup.md))**

To configure watod, update `watod-config.local.sh` to include the samples profile.
```bash
#!/bin/bash
from watod-config.sh

ACTIVE_MODULES="samples"
```

Then bring up the containers with,
```
watod up
```

### C++ and Python Samples
In the [Samples Module](../../modules/docker-compose.samples.yaml), you'll see that some of the services are commented out. The Python and C++ nodes are functionally equivalent. That is, if you were to switch out any of the C++ nodes with Python nodes by commenting and uncommenting their respective service, then the overall ROS2 communication pipeline will not change.

Editing the modules of the monorepo can be important during development, especially when you don't want to run the entire profile.

### Development
The development workflow in ROS2 is similar to ROS, however, it uses a different set
of tools. For developing and testing ROS2 nodes the process is as follows.
1. Start the service and open a shell into a running Docker container
```
watod up <SERVICE_NAME>
watod -t <SERVICE_NAME>
```
2. After making changes to source code rebuild the ROS2 package and source the install folder
```
colcon build
source install/setup.bash
```
3. Test that no breaking changes were introduced and that ROS2 coding conventions were followed
```
colcon test
colcon test-result --verbose // Get detailed information about failures
```
When developing your own node, use these samples as reference for setting up ROS2 constructs, configuration parameters, etc. 

### Testing
At this moment, the sample nodes contain unit tests, but not integration tests. As our codebase expands, we will update this section on how to do proper integration testing. See our [Testing Documentation](../../docs/dev/testing.md) for more details.