from recorder.utils import *
from recorder.Topic import *

SCENARIO_PLTR_TOPIC_NAME = '/planning/scenario_planning/scenario_selector/trajectory'
SCENARIO_PLTR_MSG_TYPE_STR = 'autoware_planning_msgs/msg/Trajectory'

# Planning trajectory
class ScenarioPlanningTrajectoryTopic(Topic):
    def __init__(self):
        super(ScenarioPlanningTrajectoryTopic, self).__init__(SCENARIO_PLTR_TOPIC_NAME, SCENARIO_PLTR_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        result = {
            "timestamp": object2timestamp(msg.header.stamp),
            "points": []
        }
        for point in msg.points:
            result['points'].append({
                "time_from_start": object2timestamp(point.time_from_start),
                "pose": {
                    "position": object2dictpoint(point.pose.position),
                    "rotation": object2dicteulerangle(point.pose.orientation)
                },
                "longitudinal_velocity": round_float(point.longitudinal_velocity_mps),
                "lateral_velocity": round_float(point.lateral_velocity_mps),
                "acceleration": round_float(point.acceleration_mps2),
            })
        return result

    def trace_key(self):
        return "scenario_planning_trajectory"