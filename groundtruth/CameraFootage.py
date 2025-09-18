# Record camera images during driving
from recorder.Topic import *

CAM_IMG_TOPIC_NAME = '/sensing/camera/traffic_light/image_raw'

class CameraFootageTopic(Topic):
    def __init__(self):
        super(CameraFootageTopic, self).__init__(CAM_IMG_TOPIC_NAME, 'sensor_msgs/msg/Image')