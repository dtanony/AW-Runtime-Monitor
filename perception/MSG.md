```
$ros2 interface show autoware_perception_msgs/msg/PredictedObjects
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
PredictedObject[] objects
	unique_identifier_msgs/UUID object_id
		
		uint8[16] uuid
	float32 existence_probability
	ObjectClassification[] classification
		uint8 UNKNOWN = 0
		uint8 CAR = 1
		uint8 TRUCK = 2
		uint8 BUS = 3
		uint8 TRAILER = 4
		uint8 MOTORCYCLE = 5
		uint8 BICYCLE = 6
		uint8 PEDESTRIAN = 7
		uint8 label
		float32 probability
	PredictedObjectKinematics kinematics
		geometry_msgs/PoseWithCovariance initial_pose_with_covariance
			Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			float64[36] covariance
		geometry_msgs/TwistWithCovariance initial_twist_with_covariance
			Twist twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			float64[36] covariance
		geometry_msgs/AccelWithCovariance initial_acceleration_with_covariance
			Accel accel
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			float64[36] covariance
		PredictedPath[] predicted_paths
			geometry_msgs/Pose[] path
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			builtin_interfaces/Duration time_step
				int32 sec
				uint32 nanosec
			float32 confidence
	Shape shape
		uint8 BOUNDING_BOX=0
		uint8 CYLINDER=1
		uint8 POLYGON=2
		uint8 type
		geometry_msgs/Polygon footprint
			Point32[] points
				#
				#
				float32 x
				float32 y
				float32 z
		geometry_msgs/Vector3 dimensions
			float64 x
			float64 y
			float64 z
(.venv) duongtd@DuongPC:~/projects/AW-RuntimeMonitor$ 
```