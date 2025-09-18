from recorder.Topic import *
from recorder.utils import *

GRTR_SIZE_TOPIC_NAME = '/simulation/gt/size'
GTTR_SIZE_MSG_TYPE_STR = 'aw_monitor/msg/GroundtruthSize'

# (ground truth) sizes of all vehicles
class GroundTruthSizeTopic(Topic):
    def __init__(self):
        super(GroundTruthSizeTopic, self).__init__(GRTR_SIZE_TOPIC_NAME, GTTR_SIZE_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        result = {
            "vehicle_sizes": [],
            "camera_screen_width": msg.camera_screen_width,
            "camera_screen_height": msg.camera_screen_height,
            "other_note": msg.other_note,
        }
        for vehicle in msg.vehicle_sizes:
            result['vehicle_sizes'].append({
                "name": vehicle.name,
                "center": object2dictpoint(vehicle.center),
                "size": object2dictpoint(vehicle.size)
            })
        return result

    def trace_key(self):
        return "groundtruth_size"