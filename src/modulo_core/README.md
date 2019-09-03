# modulo_core

This is a ROS2 package that implements the communication protocol between the modules. Each module is able to send/receive states as they are implemented in the [state_representation](../../lib/state_representation) library, and also communicate with ros2 tf2 server to send/lookup tf transforms. To implement your own modules you can inherit from the abstract classes dfined in the library:

```
class MyRobotInterface : public ModuloCore::RobotInterface
```

A module usually contains a set of publisher and subscriber. For example, to add a publisher to our MyRobotInterface we do:

```
class MyRobotInterface : public ModuloCore::RobotInterface
{
public:
	std::shared_ptr<StateRepresentation::JointState> current_position;

	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	current_position = std::make_shared<StateRepresentation::JointState>("robot");
	{
		// a publisher contains a channel name and a shared_ptr to the state to publish
		// shared_ptr are used to enable asynchronous communication between the modules
		this->add_publisher<sensor_msgs::msg::JointState>("/robot/position", this->current_position);
	}
}


```

As communication is asynchronous, the JointState will be published periodically at a frequency defined in the node under the `period` argument. If no modifications has been made to the state it is considered as empty and will not be published. By default, such a timeout occurs after two `period`. You can change this behavior by adding an argument to the pubisher:

```
// This publisher will keep publishing for 5 period if the state has not been modified before considering it empty
this->add_publisher<sensor_msgs::msg::JointState>("/robot/position", this->current_position, 5 * period);
```

You can also set that the publisher will never timeout by setting a period of 0

```
// This publisher will never timeout and send the sate even if it has not been modified
this->add_publisher<sensor_msgs::msg::JointState>("/robot/position", this->current_position, std::chrono::milliseconds(0));
```

The core structure of your module should be implemented in the `step` function:

```
class MyRobotInterface : public ModuloCore::RobotInterface
{
public:
	std::shared_ptr<StateRepresentation::JointState> current_position;

	explicit LinearMotionGenerator(const std::string & node_name, const std::chrono::milliseconds & period) :
	current_position = std::make_shared<StateRepresentation::JointState>("robot");
	{
		// a publisher contains a channel name and a shared_ptr to the state to publish
		// shared_ptr are used to enable asynchronous communication between the modules
		this->add_publisher<sensor_msgs::msg::JointState>("/robot/position", this->current_position);
	}

	void step()
	{
		// read your robot state and store it in the current position
		// this is an example where you should define the read state function on your own 
		this->current_position = robot.read_state()
	}
}
```

The step function is called periodically at a frequency defined in the node under the `period` argument. We recommand to keep this step function short, especially if you use a small period. Otherwise, latency will occur in your control loop. Complete examples of control loop including multiple nodes are written in the [tests](./tests/) for each representation spaces.
