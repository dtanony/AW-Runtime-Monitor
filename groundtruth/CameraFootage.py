# Record camera images during driving
from recorder.Topic import *

CAM_IMG_TOPIC_NAME = '/sensing/camera/camera0/image_rect_color'

class CameraFootageTopic(Topic):
    def __init__(self):
        super(CameraFootageTopic, self).__init__(CAM_IMG_TOPIC_NAME, 'sensor_msgs/msg/Image')
