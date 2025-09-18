```
$ ros2 interface show autoware_control_msgs/msg/Control
# Control message with lateral and longitudinal components

# Time this message was created
builtin_interfaces/Time stamp
	int32 sec
	uint32 nanosec

# Time this configuration state is expected to be achieved in (optional)
builtin_interfaces/Time control_time
	int32 sec
	uint32 nanosec

# Lateral control command
Lateral lateral
	#
	#
	#
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	builtin_interfaces/Time control_time
		int32 sec
		uint32 nanosec
	float32 steering_tire_angle
	float32 steering_tire_rotation_rate
	bool is_defined_steering_tire_rotation_rate

# Longitudinal control command
Longitudinal longitudinal
	#
	#
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	builtin_interfaces/Time control_time
		int32 sec
		uint32 nanosec
	float32 velocity
	float32 acceleration
	float32 jerk
	bool is_defined_acceleration
	bool is_defined_jerk

```