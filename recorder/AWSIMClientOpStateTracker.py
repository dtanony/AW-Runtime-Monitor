from recorder.Topic import *

AWSIM_CLIENT_OP_STATE_TOPIC_NAME = '/awsim_script_client/scenario_op_status'
AWSIM_CLIENT_OP_STATE_TOPIC_TYPE_STR = 'std_msgs/msg/Int32'

# tracking the simulation operation state.
# This is mainly to start/stop recording
class AWSIMClientOpStateTrackerTopic(Topic):
    def __init__(self):
        super(AWSIMClientOpStateTrackerTopic, self).__init__(AWSIM_CLIENT_OP_STATE_TOPIC_NAME, AWSIM_CLIENT_OP_STATE_TOPIC_TYPE_STR)

    def _recursive_to_dict(self, msg):
        return {}

    def trace_key(self):
        return ''