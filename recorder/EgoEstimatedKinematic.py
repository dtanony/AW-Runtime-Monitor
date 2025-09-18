from recorder.utils import *
from recorder.Topic import *

EEK_TOPIC_NAME = '/api/vehicle/kinematics'
EEK_MSG_TYPE_STR = 'autoware_adapi_v1_msgs/msg/VehicleKinematics'

# Ego estimated kinematic
class EgoEstimatedKinematicTopic(Topic):
    def __init__(self):
        super(EgoEstimatedKinematicTopic, self).__init__(EEK_TOPIC_NAME, EEK_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        return {
            "timestamp": object2timestamp(msg.geographic_pose.header.stamp),
            "pose": {
                "position": object2dictpoint(msg.pose.pose.pose.position),
                "rotation": object2dicteulerangle(msg.pose.pose.pose.orientation),
            },
            "twist": {
                "linear": object2dictpoint(msg.twist.twist.twist.linear),
                "angular": object2dictpoint(msg.twist.twist.twist.angular),
            },
            "acceleration": {
                "linear": object2dictpoint(msg.accel.accel.accel.linear),
                "angular": object2dictpoint(msg.accel.accel.accel.angular),
            }
        }

    def trace_key(self):
        return 'ego_estimated_kinematic'