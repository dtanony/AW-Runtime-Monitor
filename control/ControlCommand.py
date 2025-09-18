from recorder.Topic import *
from recorder.utils import *

CONTROLCMD_TOPIC_NAME = '/control/command/control_cmd'
CONTROLCMD_MSG_TYPE_STR = 'autoware_control_msgs/msg/Control'

class ControlCommandTopic(Topic):
    def __init__(self):
        super(ControlCommandTopic, self).__init__(CONTROLCMD_TOPIC_NAME, CONTROLCMD_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        return {
            "timestamp": object2timestamp(msg.stamp),
            "control_time": object2timestamp(msg.control_time),
            "lateral": {
                "time": object2timestamp(msg.lateral.stamp),
                "control_time": object2timestamp(msg.lateral.control_time),
                "steering_tire_angle": round_float(msg.lateral.steering_tire_angle),
                "steering_tire_rotation_rate": round_float(msg.lateral.steering_tire_rotation_rate),
                "is_defined_rotation_rate": msg.lateral.is_defined_steering_tire_rotation_rate,
            },
            "longitudinal": {
                "time": object2timestamp(msg.longitudinal.stamp),
                "control_time": object2timestamp(msg.longitudinal.control_time),
                "velocity": round_float(msg.longitudinal.velocity),
                "acceleration": round_float(msg.longitudinal.acceleration),
                "jerk": round_float(msg.longitudinal.jerk),
                "is_defined_accel": msg.longitudinal.is_defined_acceleration,
                "is_defined_jerk": msg.longitudinal.is_defined_jerk,
            }
        }

    def trace_key(self):
        return "control_cmds"