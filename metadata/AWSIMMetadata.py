from recorder.Topic import *
import json

AWSIM_METADATA_TOPIC_NAME = '/awsim/sim_metadata'
AWSIM_METADATA_TOPIC_TYPE_STR = 'std_msgs/msg/String'

class AWSIMMetadataTopic(Topic):
    def __init__(self):
        super(AWSIMMetadataTopic, self).__init__(AWSIM_METADATA_TOPIC_NAME, AWSIM_METADATA_TOPIC_TYPE_STR)

    def _recursive_to_dict(self, msg):
        # print(msg)
        json_string = msg.data.replace("'", "\"")
        obj = json.loads(json_string)
        return  obj

    def trace_key(self):
        return 'metadata'