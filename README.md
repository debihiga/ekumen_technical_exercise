# Ekumen technical exercise

## Overview

Makes turtlesim follow specific paths and drawing them on the screen, and also, in a web browser.

The ekumen_technical_exercise package has been tested under [ROS] Kinetic and Ubuntu 16.04.

**Author: Emiliano Borghi
Affiliation: Ekumen, Buenos Aires, Argentina**

![Ekumen Logo](doc/ekumen-logo.png)



## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [roslibjs](core JavaScript library for interacting with ROS from the browser)

		sudo apt-get install ros-<rosdistro>-rosbridge-server


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd <your_catkin_workspace>/src
	git clone https://github.com/eborghi10/ekumen_technical_exercise.git
	cd ../
	catkin_make


## Usage

Run the package with

	roslaunch ekumen_technical_exercise turtlesim_demo.launch


## Nodes

The following description will be edited when the package works fine.

### turtlesim_node

turtlesim is a tool made for teaching ROS and ROS packages.


#### Subscribed Topics

* **`/topic_a`** ([type/Name])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`service_name`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ekumen_technical_exercise/service_name


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.

