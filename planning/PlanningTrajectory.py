from recorder.utils import *
from recorder.Topic import *
from control.shield_utils import *

PLTR_TOPIC_NAME = '/planning/scenario_planning/trajectory'
PLTR_UNVERIFIED_TOPIC_NAME = '/planning/scenario_planning/trajectory_unverified'
PLTR_MSG_TYPE_STR = 'autoware_planning_msgs/msg/Trajectory'

# Planning trajectory
class PlanningTrajectoryTopic(Topic):
    def __init__(self):
        super(PlanningTrajectoryTopic, self).__init__(PLTR_TOPIC_NAME, PLTR_MSG_TYPE_STR)

    def _recursive_to_dict(self, msg):
        result = {
            "timestamp": object2timestamp(msg.header.stamp),
            "points": []
        }
        for point in msg.points:
            result['points'].append({
                "time_from_start": object2timestamp(point.time_from_start),
                "pose": {
                    "position": object2dictpoint(point.pose.position),
                    "rotation": object2dicteulerangle(point.pose.orientation)
                },
                "longitudinal_velocity": round_float(point.longitudinal_velocity_mps),
                "lateral_velocity": round_float(point.lateral_velocity_mps),
                "acceleration": round_float(point.acceleration_mps2),
            })
        return result

    def trace_key(self):
        return "planning_trajectory"

# represent future points in planning trajectories
class KinematicState:
    def __init__(self, position, velocity, accel, time_to_next_step, time_from_start):
        """
        :param position: 2D
        :param velocity: scalar
        :param accel: scalar
        :param time_to_next_step: we estimate this time step
        """
        self.position = np.array(position)
        self.velocity = float(velocity)
        self.accel = float(accel)
        self.time_to_next_state = time_to_next_step
        self.time_from_start = time_from_start

    def __str__(self):
        return f'position: {self.position}, velocity: {self.velocity}, accel: {self.accel}, time_to_next_state: {self.time_to_next_state}'

def derive_timestep(position0, speed0, accel0, position1, speed1, accel1):
    if accel0+accel1 == 0:
        if speed0 + speed1 == 0:
            return 0
        return 2 * float(np.linalg.norm(np.array(position1) - np.array(position0))) / (speed0 + speed1)
    return 2 * (speed1 - speed0) / (accel0 + accel1)

def get_pos_and_vels(points_raw):
    positions = []
    long_vels = []
    accels = []
    for point in points_raw:
        if point.lateral_velocity_mps > 0.1:
            raise Exception('Not handle case when lateral velocity > 0')
        positions.append(point22Dpoint(point.pose.position))
        long_vels.append(point.longitudinal_velocity_mps)
        accels.append(point.acceleration_mps2)
    return positions, long_vels, accels

class ConstantHeadingVehicle:
    """
    represent ego vehicle moving with a constant heading angle
    """
    def __init__(self, vehicle_current_pose,
                 veh_length, veh_width, veh_center_x, veh_center_y,):
        self.init_pos = np.array((vehicle_current_pose['position']['x'],
                                  vehicle_current_pose['position']['y']))
        self.init_rot = vehicle_current_pose['rotation']['z']
        self.init_vertices = (
            get_ego_world_vertices(self.init_pos, self.init_rot,
                                (veh_length, veh_width),
                         (veh_center_x, veh_center_y)))
        heading_rad = np.deg2rad(self.init_rot)
        self.norm_vel = np.array((
            np.cos(heading_rad), np.sin(heading_rad)
        ))

    # get the 4 vertices when the veh moves to $position
    def get_vertices_whenat_pos(self, position):
        direction = position - self.init_pos
        return direction + self.init_vertices

    def get_vertices_at_time(self, time, speed):
        velocity = speed * self.norm_vel
        return self.init_vertices + time * velocity

class PlanningTrajectory:
    def __init__(self,
                 ego_current_state: ConstantHeadingVehicle,
                 past_states=None, future_states=None):
        """
        :param ego_current_state: represent the current state of the ego vehicle
        """
        if future_states is None:
            future_states = []
        if past_states is None:
            past_states = []
        self.ego_current_state = ego_current_state
        self.future_states = future_states
        self.past_states = past_states

    def parse_future_points(self, points_raw):
        """
        :param points_raw: in ROS msg format
        :return:
        """
        init_pos = self.ego_current_state.init_pos

        # find point entry in points_raw that has position closest to current position of Ego
        # because the planning trajectory also includes some previous positions
        min_dis = float('inf')
        min_dis_idx = -1
        for idx,point in enumerate(points_raw):
            pos = point22Dpoint(point.pose.position)
            dis = float(np.linalg.norm(np.array(pos) - np.array(init_pos) ))
            if dis < min_dis:
                min_dis = dis
                min_dis_idx = idx
            elif min_dis < dis < float('inf') and min_dis <= 0.3:
                break

        self.past_states = points_raw[:min_dis_idx]

        # extract planning trajectory
        positions, long_vels, accels = get_pos_and_vels(points_raw[min_dis_idx:])
        self.future_states = []
        time_from_start = 0.0
        for i in range(len(positions) - 1):
            time_step = derive_timestep(
                positions[i], long_vels[i], accels[i], positions[i + 1], long_vels[i + 1], accels[i + 1])
            self.future_states.append(KinematicState(
                positions[i], long_vels[i], accels[i], time_step, time_from_start
            ))
            time_from_start += time_step

    def __str__(self):
        result = ""
        for point in self.future_states:
            result += str(point) + "\n"
        return result