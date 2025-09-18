from recorder.utils import *
from recorder.Topic import *

PREDICTED_OBJ_TOPIC_NAME = '/perception/object_recognition/objects'
PREDICTED_OBJ_MSG_TYPE_STR = 'autoware_perception_msgs/msg/PredictedObjects'

# Planning trajectory
class PerceptionObjectTopic(Topic):
    def __init__(self):
        super(PerceptionObjectTopic, self).__init__(PREDICTED_OBJ_TOPIC_NAME, PREDICTED_OBJ_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        result = {
            "timestamp": object2timestamp(msg.header.stamp),
            "objects": []
        }
        for entry in msg.objects:
            obj = {
                "id": uuidstr(entry.object_id.uuid),
                "existence_prob": round_float(entry.existence_probability),
                "classification": convert_classification(entry.classification),
                "pose": {
                    "position": object2dictpoint(entry.kinematics.initial_pose_with_covariance.pose.position),
                    "rotation": object2dicteulerangle(entry.kinematics.initial_pose_with_covariance.pose.orientation),
                },
                "twist": {
                    "linear": object2dictpoint(entry.kinematics.initial_twist_with_covariance.twist.linear),
                    "angular": object2dictpoint(entry.kinematics.initial_twist_with_covariance.twist.angular),
                },
                "acceleration": {
                    "linear": object2dictpoint(entry.kinematics.initial_acceleration_with_covariance.accel.linear),
                    "angular": object2dictpoint(entry.kinematics.initial_acceleration_with_covariance.accel.angular),
                },
                "shape": convert_shape(entry.shape),
                "predict_paths": [convert_predicted_path(ppath) for ppath in entry.kinematics.predicted_paths],
            }
            result['objects'].append(obj)
        return result

    def trace_key(self):
        return "perception_objects"

def convert_classification(input):
    return [{"label": c.label, "probability": round_float(c.probability)} for c in input]

SHAPE_BOUNDING_BOX = 0
SHAPE_CYLINDER = 1
SHAPE_POLYGON = 2
def convert_shape(input):
    if input.type == SHAPE_BOUNDING_BOX:
        return {
            "type": "box",
            "size": object2dictpoint(input.dimensions)
        }
    elif input.type == SHAPE_POLYGON:
        return {
            "type": "polygon",
            "footprint": [object2dictpoint(point) for point in input.footprint.points]
        }
    else:
        print("[WARNING] Shape type `Cylinder` is not handled for perception object.")
        return "Not yet handled Cylinder shape."

def convert_predicted_path(path):
    return {
        "confidence": round_float(path.confidence),
        "time_step": object2timestamp(path.time_step),
        "path": [{
            "position": object2dictpoint(entry.position),
            "rotation": object2dicteulerangle(entry.orientation),
        } for entry in path.path],
    }

