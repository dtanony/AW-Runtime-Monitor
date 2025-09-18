from recorder.Topic import *

OPERATION_MODE_TOPIC_NAME = '/autoware/engage'
OPERATION_MODE_TOPIC_TYPE_STR = 'autoware_vehicle_msgs/msg/Engage'

# Ego estimated kinematic
class OperationModeTrackerTopic(Topic):
    def __init__(self):
        super(OperationModeTrackerTopic, self).__init__(OPERATION_MODE_TOPIC_NAME, OPERATION_MODE_TOPIC_TYPE_STR)

    def _recursive_to_dict(self, msg):
        return {}

    def trace_key(self):
        return ''