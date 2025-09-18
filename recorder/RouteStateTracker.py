from recorder.Topic import *

ROUTE_STATE_TOPIC = "/api/routing/state"
ROUTE_STATE_MSG_TYPE_STR = 'autoware_adapi_v1_msgs/msg/RouteState'

# Ego estimated kinematic
class RouteStateTrackerTopic(Topic):
    def __init__(self):
        super(RouteStateTrackerTopic, self).__init__(ROUTE_STATE_TOPIC, ROUTE_STATE_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        return {}

    def trace_key(self):
        return ''