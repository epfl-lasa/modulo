[![Codacy Badge](https://api.codacy.com/project/badge/Grade/a89a7401d0aa479db3db959625cdfd65)](https://www.codacy.com/manual/buschbapti/modulo?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=epfl-lasa/modulo&amp;utm_campaign=Badge_Grade)

# Modulo

Modulo is a software architecture to design control loop for robotic systems. It is based on the definition of modules communicating with each other to facilitate the implementation of closed loop control architecture. Each module has a specific function such as being an interface to a robot or a sensor, or modulate an input signal. The core library [modulo_core](./src/modulo_core) implements multiple abstract classes you can inherit from to develop your own module.

The communication between the modules is based on ROS2. Each module inherits from the `lifecycle` interface allowing the creation of state based control scheme. Exchanging messages has been simplified and is asynchronous (cf. [modulo_core](./src/modulo_core) documentation for control loop examples).

There are a few standalone libraries developed as helpers for communication and design of control loops. For example, the [state_representation](./lib/state_representation) library provides classes to represent states (pose, velocities, ...) in cartesian, joint and dual quaternion spaces. The [dynamical_systems](./lib/dynamical_systems) library is a set of templated classes representing dynamical systems. Used in complement of the [state_representation](./lib/state_representation) library it can generate dynamical system in the previously mentioned spaces.

You can build a complete working environment using `docker`. Simply run `sh build.sh` to build an image based on `ros2:nightly` docker image. This will create an image called `modulo`. Running `sh run.sh` opens an interactive shell with a `ros2_ws` allowing you to compile and run your modules. When creating a new module, we recommend you to create a new docker image on top of the `modulo` one. This way you will have all the libraries already installed while being able to create you own packages.

## Interfacing with ROS1

It is possible to interface ROS2 with ROS1 in order to publish/subscribe to topics from ROS1. To do so, you need to launch a ROS2/ROS1 bridge. You can use a ready to use docker image by issuing the command:

```bash
docker run -it --net=host osrf/ros:dashing-ros1-bridge
```

This will pull the bridge image and open interactive shell in which you can run the bridge:

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
