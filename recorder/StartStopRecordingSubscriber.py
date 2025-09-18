import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

OPERATION_MODE_TOPIC_NAME = '/autoware/engage'
OPERATION_MODE_TOPIC_TYPE_STR = 'autoware_vehicle_msgs/msg/Engage'
ROUTE_STATE_TOPIC = "/api/routing/state"
ROUTE_STATE_MSG_TYPE_STR = 'autoware_adapi_v1_msgs/msg/RouteState'

# OPERATION_MODE_TOPIC_NAME = '/api/operation_mode/state'
# OPERATION_MODE_TOPIC_TYPE_STR = 'autoware_adapi_v1_msgs/msg/OperationModeState'

class StartStopRecordingSubscriber(Node):
    def __init__(self, engage_cmd_received_callback, goal_arrived_callback):
        super().__init__('start_stop_events_recorder')
        try:
            opmode_msg_type = get_message(OPERATION_MODE_TOPIC_TYPE_STR)
            routestate_msg_type = get_message(ROUTE_STATE_MSG_TYPE_STR)
        except Exception as e:
            print(f"Failed to resolve message type: {e}")
            return

        self.subscription = self.create_subscription(
            opmode_msg_type,
            OPERATION_MODE_TOPIC_NAME,
            engage_cmd_received_callback,
            1
        )
        self.subscription = self.create_subscription(
            routestate_msg_type,
            ROUTE_STATE_TOPIC,
            goal_arrived_callback,
            1
        )

def subscribe(engage_cmd_received_callback, goal_arrived_callback):
    node = StartStopRecordingSubscriber(engage_cmd_received_callback, goal_arrived_callback)
    rclpy.spin(node)

if __name__ == '__main__':
    def engage_cmd_received_event(msg):
        print(msg)
    def goal_arrived_event(msg):
        print(msg)

    subscribe(engage_cmd_received_event, goal_arrived_event)

