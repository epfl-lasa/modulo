
# Modulo

Modulo is an extension layer to ROS2 that adds support for [epfl-lasa/control-libraries](https://github.com/epfl-lasa/control-libraries), allowing parameters, subscriptions and publications to directly use custom `state_representation` types with a simplified asynchronous exchange.

The core package [modulo_core](./source/modulo_core) implements some abstract classes you can inherit from to develop your own module,
including the `Cell` and `Component` classes. These classes inherit from the `lifecycle` interface and additionally provide a customisable `step()` thread
that runs when activated, allowing the creation of state-based behavioural modules.

See also:
- [aica-technology/docker-images](https://github.com/aica-technology/docker-images)
- [epfl-lasa/control-libraries](https://github.com/epfl-lasa/control-libraries)


## Interfacing with ROS1

It is possible to interface ROS2 with ROS1 in order to publish/subscribe to topics from ROS1. To do so, you need to launch a ROS2/ROS1 bridge. You can use a ready to use docker image by issuing the command:

```bash
docker run --rm -it --net=host osrf/ros:dashing-ros1-bridge
```

This will pull the bridge image and open interactive shell in which you can run the bridge:

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
