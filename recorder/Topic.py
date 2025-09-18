from rosidl_runtime_py.utilities import get_message

# Represent a ROS topic
class Topic:
    def __init__(self, topic_name, msg_type_str, save_data=False):
        self.topic_name = topic_name
        self.msg_type_str = msg_type_str
        try:
            msg_type = get_message(msg_type_str)
        except Exception as e:
            print(f"Failed to resolve message type '{msg_type_str}': {e}")
            return
        self.msg_type = msg_type
        self.save_data = save_data

    # Convert ROS message to dictionary
    def message_to_dict(self, msg):
        try:
            return self._recursive_to_dict(msg)
        except Exception as e:
            print(f"[ERROR] Failed to convert message to dict: {e}")
            return {}
    def _recursive_to_dict(self, msg):
        pass

    def trace_key(self):
        pass