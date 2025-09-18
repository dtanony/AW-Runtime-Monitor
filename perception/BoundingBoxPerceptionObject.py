from recorder.utils import *
from recorder.Topic import *

BB_PERPCEPTION_OBJ_TOPIC_NAME = '/perception/object_recognition/detection/rois0'
BB_PERPCEPTION_OBJ_MSG_TYPE_STR = 'tier4_perception_msgs/msg/DetectedObjectsWithFeature'

# Planning trajectory
class BoundingBoxPerceptionObjectTopic(Topic):
    def __init__(self):
        super(BoundingBoxPerceptionObjectTopic, self).__init__(BB_PERPCEPTION_OBJ_TOPIC_NAME, BB_PERPCEPTION_OBJ_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        result = {
            "timestamp": object2timestamp(msg.header.stamp),
            "objects": []
        }
        for entry in msg.feature_objects:
            obj = {
                "existence_prob": round_float(entry.object.existence_probability),
                "classification": [{"label": c.label, "probability": round_float(c.probability)} for c in entry.object.classification],
                "bounding_box": {
                    "x": entry.feature.roi.x_offset,
                    "y": entry.feature.roi.y_offset,
                    "width": entry.feature.roi.width,
                    "height": entry.feature.roi.height,
                },
            }
            result['objects'].append(obj)
        return result

    def trace_key(self):
        return "boundingbox_perception_objects"