from recorder.Topic import *
from recorder.utils import *

GRTR_TOPIC_NAME = '/simulation/gt/kinematic'
GTTR_MSG_TYPE_STR = 'aw_monitor/msg/GroundtruthKinematic'

# ground truth status of Ego and NPC vehicles, NPC pedestrians
class GroundTruthKinematicTopic(Topic):
    def __init__(self):
        super(GroundTruthKinematicTopic, self).__init__(GRTR_TOPIC_NAME, GTTR_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        result = {
            "timestamp": object2timestamp(msg.stamp),
            "groundtruth_ego": {
                "pose": {
                    "position": object2dictpoint(msg.groundtruth_ego.pose.position),
                    "rotation": object2dictpoint(msg.groundtruth_ego.pose.rotation),
                },
                "twist": {
                    "linear": object2dictpoint(msg.groundtruth_ego.twist.linear),
                    "angular": object2dictpoint(msg.groundtruth_ego.twist.angular),
                },
                "acceleration": {
                    "linear": object2dictpoint(msg.groundtruth_ego.accel.linear),
                    "angular": object2dictpoint(msg.groundtruth_ego.accel.angular),
                }
            },
            "groundtruth_vehicles": [],
            "groundtruth_pedestrians": [],
        }
        for vehicle in msg.groundtruth_vehicles:
            result['groundtruth_vehicles'].append({
                "name": vehicle.name,
                "pose": {
                    "position": object2dictpoint(vehicle.pose.position),
                    "rotation": object2dictpoint(vehicle.pose.rotation)
                },
                "twist": {
                    "linear": object2dictpoint(vehicle.twist.linear),
                    "angular": object2dictpoint(vehicle.twist.angular)
                },
                "accel": round_float(vehicle.accel),
                "bounding_box": {
                    "x": round_float(vehicle.bounding_box.x),
                    "y": round_float(vehicle.bounding_box.y),
                    "width": round_float(vehicle.bounding_box.width),
                    "height": round_float(vehicle.bounding_box.height),
                },
            })
        for pedestrian in msg.groundtruth_pedestrians:
            result['groundtruth_pedestrians'].append({
                "name": pedestrian.name,
                "pose": {
                    "position": object2dictpoint(pedestrian.pose.position),
                    "rotation": object2dictpoint(pedestrian.pose.rotation),
                },
                "speed": round_float(pedestrian.speed),
                "bounding_box": {
                    "x": round_float(pedestrian.bounding_box.x),
                    "y": round_float(pedestrian.bounding_box.y),
                    "width": round_float(pedestrian.bounding_box.width),
                    "height": round_float(pedestrian.bounding_box.height),
                }
            })
        return result

    def trace_key(self):
        return "groundtruth_kinematic"