# Modulo

Modulo is an extension layer to ROS2 that adds interoperability support for [epfl-lasa/control-libraries](https://github.com/epfl-lasa/control-libraries)
and provides a modular framework for [application composition](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
through custom component classes in both C++ and Python.

Documentation is available at [epfl-lasa.github.io/modulo](epfl-lasa.github.io/modulo).

## Modulo Core

The core package implements interoperability between ROS2 and state_representation data types,
allowing state data and parameters to be directly translated and handled as messages on the ROS2 interface layer.

## Modulo Components

Modulo components are wrappers for ROS2 Nodes which abstract low-level ROS methods for greater user convenience and
consistency between components.

While ROS2 Nodes provide a highly flexible and customizable interface, there is often a significant amount of 
boilerplate code that is duplicated with each new Node. In addition, the interoperability of multiple nodes in an
application depends on them using compatible message types and behaving in a generally consistent manner.

The component classes are intended to simplify the development of compatible application modules.
They provide more concise methods for adding parameters, services, transforms, topic subscriptions and publications.
They also introduce new concepts such as predicate broadcasting, error handling and the direct binding of
data attributes to their corresponding interface.

The package provides two variant classes: `modulo_components::Component` and `modulo_components::LifecycleComponent`.
See the package documentation for more information.

## Modulo Component Interfaces

This package defines custom standard interfaces for modulo components.

---

## Additional resources

### Interfacing with ROS1

It is possible to use a bridge service to interface ROS2 with ROS1 in order to publish/subscribe to topics from ROS1.

ROS provides a docker image with this service. Run the following lines to pull the bridge image, launch an interactive
shell, and subsequently start the bridge topic.

```bash
# run the bridge image interactively
docker run --rm -it --net=host ros:galactic-ros1-bridge
# start the bridge service in the container
root@docker-desktop:/# ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Further reading

- [epfl-lasa/control-libraries](https://github.com/epfl-lasa/control-libraries)
- [aica-technology/docker-images](https://github.com/aica-technology/docker-images)
- [aica-technology/component-sdk](https://github.com/aica-technology/component-sdk)
- [ROS2 Composition](https://github.com/aica-technology/docker-images)
- [ROS2 Managed Nodes](https://design.ros2.org/articles/node_lifecycle.html)

### Authors and maintainers

- [Baptiste Busch](https://github.com/buschbapti)
- [Enrico Eberhard](https://github.com/eeberhard)
- [Dominic Reber](https://github.com/domire8)