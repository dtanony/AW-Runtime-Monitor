import rclpy
import sys, os
import yaml
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import argparse, time
import json
from aw_monitor.srv import MonitorRecordingState
import recorder.utils as utils
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from std_srvs.srv import Trigger

from control.ControlCommand import *
from groundtruth.CameraFootage import *
from groundtruth.GroundTruthSize import *
from groundtruth.GroundTruthKinematic import *
from perception.PerceptionObject import *
from perception.BoundingBoxPerceptionObject import *
from planning.PlanningTrajectory import *
from control.controller_shield import verify_control_cmd
from recorder.EgoEstimatedKinematic import *
from recorder.OperationModeTracker import *
from recorder.RouteStateTracker import *
from recorder.StartStopRecordingSubscriber import *
from recorder.Topic import *
from recorder.AWSIMClientOpStateTracker import *
from metadata.AWSIMMetadata import *

from cv_bridge import CvBridge
import cv2

AWSIM_CLIENT_OP_STATE_STOPPED = 1
AWSIM_CLIENT_OP_STATE_RUNNING = 2
AWSIM_CLIENT_OP_STATE_AUTO_MODE = 3
TTC_THRESHOLD = 1.5

class MultiTopicRecorder(Node):
    # $topics is a list of (class) Topic instances
    def __init__(self, topics, output_filename, format, no_sim=1, to_verify_control_cmd=True):
        super().__init__('fomaad_monitor')
        self.diff_time = -1.0
        self.output_filename = output_filename
        self.format = format
        self.to_verify_control_cmd = to_verify_control_cmd
        self.topics = topics
        self.finished = False
        self.no_sim = no_sim-1 # number of simulations have been done
        self.recording_state = MonitorRecordingState.Response.WAITING_RECORDING
        self.topics_to_record_data = [t for t in topics if
                                      not (isinstance(t, AWSIMClientOpStateTrackerTopic)
                                           or isinstance(t, CameraFootageTopic))]

        # to save text data
        self.messages = {}

        # to save image data
        self.bridge = CvBridge()
        self.frames = []

        # service server
        self.create_service(MonitorRecordingState, '/monitor/recording/state', self.recording_state_callback)
        # service client
        self.aeb_activation_client = self.create_client(
            Trigger,
            '/control/autonomous_emergency_braking/activate_aeb',
        )

        self.reset()

    def reset(self):
        self.finished = False
        self.no_sim += 1
        self.recording_state = MonitorRecordingState.Response.WAITING_RECORDING

        self.messages = {topic.trace_key(): [] for topic in self.topics_to_record_data}
        self.messages[GroundTruthSizeTopic().trace_key()] = {}
        self.messages[AWSIMMetadataTopic().trace_key()] = {}

        self.bridge = CvBridge()
        self.frames = []

    def subscribe(self):
        """
        Subscribe to the topics defined by self.topics_to_record_data
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        for topic in self.topics:
            self.create_subscription(
                topic.msg_type,
                topic.topic_name,
                self.listener_callback(topic),
                qos_profile
            )
            self.get_logger().info(f'Subscribed to {topic.topic_name}')

    def recording_state_callback(self, request, response):
        response.succeed = True
        response.state = self.recording_state
        return response

    def listener_callback(self, topic):
        if isinstance(topic, AWSIMClientOpStateTrackerTopic):
            def client_op_state_callback(msg):
                if msg.data == AWSIM_CLIENT_OP_STATE_AUTO_MODE:
                    print(f'Autonomous operation mode was started.')
                    self.recording_state = MonitorRecordingState.Response.RECORDING
                    for t in self.topics:
                        if not isinstance(t, AWSIMClientOpStateTrackerTopic):
                            t.save_data = True
                            print(f"[{t.__class__.__name__}] Started recording data.")

                elif msg.data == AWSIM_CLIENT_OP_STATE_STOPPED:
                    # stop recording
                    print(f'Simulation is terminating, saving recorded data...')
                    self.recording_state = MonitorRecordingState.Response.WRITING_DATA
                    for t in self.topics:
                        t.save_data = False
                    self.dump_data()
                    self.recording_state = MonitorRecordingState.Response.WRITING_DONE
                    self.reset()

            return client_op_state_callback

        def save_msg(msg):
            if self.to_verify_control_cmd and isinstance(topic, ControlCommandTopic):
                current_time = self.get_clock().now().nanoseconds/10**9
                msg_time = object2timestamp(msg.stamp, round=False)
                if self.diff_time and current_time <= msg_time + self.diff_time + 0.1:
                    # Verify control command is safe
                    no_collision, ttc = verify_control_cmd(msg, self.messages, self.get_logger())
                    if not no_collision:
                        self.get_logger().info(f"Unsafe control command, estimated TTC: {ttc}")
                        if ttc <= TTC_THRESHOLD:
                            self.get_logger().info("Activating AEB...")
                            self.activate_aeb()

            if isinstance(topic, GroundTruthKinematicTopic) and self.diff_time < 0:
                sim_time = object2timestamp(msg.stamp, round=False)
                sys_time = self.get_clock().now().nanoseconds / 10 ** 9
                self.diff_time = sys_time - sim_time

            if topic.save_data:
                if isinstance(topic, CameraFootageTopic):
                    # handle image data
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    self.frames.append( (cv_image, utils.object2timestamp(msg.header.stamp)) )
                else:
                    # handle text data
                    # for groundtruthSize: one message is enough
                    if isinstance(topic, GroundTruthSizeTopic):
                        if not self.messages[topic.trace_key()]:
                            self.messages[topic.trace_key()] = topic.message_to_dict(msg)
                    elif isinstance(topic, AWSIMMetadataTopic):
                        if not self.messages[topic.trace_key()]:
                            self.messages[topic.trace_key()] = topic.message_to_dict(msg)
                    else:
                        self.messages[topic.trace_key()].append(topic.message_to_dict(msg))
        return save_msg

    def dump_data(self):
        """
        Save recorded text data and video to disk
        """
        if self.finished:
            return
        if self.output_filename:
            output_path = self.output_filename
        else:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            output_path = f'tracedata_{timestamp}'

        output_path += f"_sim{self.no_sim}"
        print('Saving text data...')
        self.save_to_file(output_path)
        print('Saving camera images (if any) as footage...')
        self.save_video(output_path)
        self.finished = True

    def message_exist(self):
        for key, value in self.messages.items():
            if len(value) > 0:
                return True
        return False

    def save_to_file(self, output_path):
        if not self.message_exist():
            self.get_logger().error('No data recorded!')
            return

        if self.format == 'json':
            with open(output_path + '.json', 'w') as f:
                json.dump(self.messages, f, indent=None)
        else:
            with open(output_path + '.yaml', 'w') as f:
                yaml.dump(self.messages, f, sort_keys=False)
        print('Recorded text data saved to ' + output_path)

    # output_path does NOT end with .mp4
    def save_video(self, output_path):
        if not self.frames:
            self.get_logger().error('No frames recorded!')
            return

        # Get average FPS
        timestamps = [t for _, t in self.frames]
        duration = timestamps[-1] - timestamps[0]
        fps = len(self.frames) / duration if duration > 0 else 10
        self.get_logger().info(f'Approximated FPS: {fps:.2f}')

        # Get image dimensions from the first frame
        first_frame, _ = self.frames[0]
        height, width = first_frame.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path + '_footage.mp4', fourcc, fps, (width, height))

        # Write frames to video
        for frame, _ in self.frames:
            out.write(frame)
        out.release()

        # Write metadata
        metadata = {
            'frame_count': len(self.frames),
            'start_time': timestamps[0],
            'end_time': timestamps[-1],
            'timestamps': timestamps,
        }
        with open(output_path + '_footage.meta.json', 'w') as f:
            json.dump(metadata, f, indent=2)

        self.frames = []
        self.get_logger().info(f'Camera footage saved to {output_path}_footage.mp4.\n')
        self.frames = []
        self.messages = []

    def activate_aeb(self):
        """
        Sending ROS request to activate AEB
        """
        req = Trigger.Request()
        future = self.aeb_activation_client.call_async(req)
        future.add_done_callback(self.aeb_response_callback)

    def aeb_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("AEB activated!")
            else:
                self.get_logger().error(f"AEB activation failed, message: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")