# Modulo

This is a ROS2 package that implements the communication protocol between the modules. Each module is able to send/receive states as they are implemented in the [state_representation](../../lib/state_representation) library, and also communicate with ros2 tf2 server to send/lookup tf transforms. To implement your own modules you have to inherit a new class from the `Modulo::Core::Cell`{:.cpp} base class

```cpp
class MyRobotInterface : public Modulo::Core::Cell
```

A module usually contains a set of publisher and subscriber. For example, to add a publisher to our `MyRobotInterface`{.cpp} class, that will publish the state of the robot, we do:

```cpp
class MyRobotInterface : public Modulo::Core::Cell
{
public:
	std::shared_ptr<state_representation::JointState> current_state;

	explicit MyRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period) :
	Cell(node_name, period),
	current_state(std::make_shared<state_representation::JointState>("my_robot"))
	{
		// a publisher contains a channel name and a shared_ptr to the state to publish
		// shared_ptr are used to enable asynchronous communication between the modules
		this->add_publisher<sensor_msgs::msg::JointState>("/robot/joint_state", this->current_state);
	}
}


```

As communication is asynchronous, the `JointState`{.cpp} will be published periodically at a frequency defined in the node under the `period` argument or at the `period` defined in the argument of the publisher. If no modifications has been made to the state it is considered as empty and will not be published. By default, such a timeout occurs after two `period`. You can change this behavior by adding an argument to the pubisher:

The core structure of the module should be implemented in the `step` function:

```cpp
class MyRobotInterface : public ModuloCore::RobotInterface
{
public:
	std::shared_ptr<state_representation::JointState> current_state;

	explicit MyRobotInterface(const std::string & node_name, const std::chrono::milliseconds & period) :
	Cell(node_name, period),
	current_state(std::make_shared<state_representation::JointState>("my_robot"))
	{
		// a publisher contains a channel name and a shared_ptr to the state to publish
		// shared_ptr are used to enable asynchronous communication between the modules
		this->add_publisher<sensor_msgs::msg::JointState>("/robot/joint_state", this->current_state);
	}

	void step()
	{
		// read your robot state and store it in the current position
		// this is an example where you should define the read state function on your own 
		this->current_position = {...}
	}
}
```

No need to do anything else. The step function is called periodically at a frequency defined in the node under the `period` argument. And the `current_state` variable will be published over `robot/joint_state` channel on ros2 network automatically.

We recommend keeping this step function short, especially if you use a small period. Otherwise, latency will occur in your control loop. Complete examples of control loops including multiple nodes are written in the [tests](examples/) for each representation spaces.
