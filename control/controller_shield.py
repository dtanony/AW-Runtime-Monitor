from control.shield_utils import *
from planning.PlanningTrajectory import ConstantHeadingVehicle
from recorder.utils import *
import math

# Safety shield for control module.

def verify_control_cmd(control_cmd, recorded_messages, logger):
    """
    :param control_cmd:
    :param recorded_messages:
    :param logger:
    :return: {False, estimated TTC} if a collision is expected.
    Otherwise, {True, inf}.
    """
    timestamp = object2timestamp(control_cmd.stamp)
    ego_size = get_ego_size(recorded_messages)
    ego_kinematic,_ = get_ego_kinematics(recorded_messages, timestamp)
    perceived_objects, _id = get_perceived_objects(recorded_messages, timestamp)

    if not (ego_size and ego_kinematic and perceived_objects):
        return True, 1e9

    ego_length = ego_size['size']['x']
    ego_width = ego_size['size']['y']
    ego_center_x = ego_size['center']['x']
    ego_center_y = ego_size['center']['y']
    ego_current_state = ConstantHeadingVehicle(ego_kinematic['pose'],
                                               ego_length,ego_width,
                                               ego_center_x,ego_center_y)

    minTTC = 1e9

    # sampling Ego pose
    time_step = 0.1
    time_bound = 2
    sampled_ego_vertices = []
    for i in range(int(time_bound/time_step)):
        sampled_ego_vertices.append(ego_current_state.get_vertices_at_time(
            i*time_step, control_cmd.longitudinal.velocity))

    for perceived_obj in perceived_objects['objects']:
        prev_recorded_obj, prev_timestamp = find_latest_perp_obj(recorded_messages['perception_objects'],
                                                 perceived_obj['id'],
                                                 _id)

        # perceived object dimension
        if perceived_obj['shape']['type'] == 'box':
            length = perceived_obj['shape']['size']['x']
            width = perceived_obj['shape']['size']['y']
            npc_local_vertices = [
                np.array((width / 2, length / 2)),
                np.array((-width / 2, length / 2)),
                np.array((-width / 2, -length / 2)),
                np.array((width / 2, -length / 2)),
            ]
        elif perceived_obj['shape']['type'] == 'polygon':
            npc_local_vertices = \
                [np.array(([point['x'], point['y']])) for point in perceived_obj['shape']['footprint']]

        # Simulate a path in which the NPC keep going with current velocity
        # First step: check and correct velocity if needed. 
        # We calculate velocity of $perceived_obj based on the current and previous poses
        current_pose = perceived_obj['pose'].copy()
        current_orientation = current_pose['rotation']
        current_vel = dict_point_to_array(perceived_obj['twist']['linear'])
        current_position = dict_point_to_array(current_pose['position'])

        if prev_recorded_obj:
            cal_vel = (current_position - dict_point_to_array(prev_recorded_obj['pose']['position'])) \
                       / (timestamp - prev_timestamp)
            sign_str = lambda value: "+" if value >= 0 else "-"
            if abs(cal_vel[0]) >= 1e-2 and current_vel[0] * cal_vel[0] < 0:
                # logger.error(f"Sign of X-value of linear velocity of perceived object {perceived_obj['id']} should be {sign_str(cal_vel[0])}.")
                current_vel[0] = -current_vel[0]
            if abs(cal_vel[1]) >= 1e-2 and current_vel[1] * cal_vel[1] < 0:
                # logger.error(f"Sign of Y-value of linear velocity of perceived object {perceived_obj['id']} should be {sign_str(cal_vel[1])}.")
                current_vel[1] = -current_vel[1]
            
            ttc0 = ttc_with_simulated_npc_path(sampled_ego_vertices, current_position, current_orientation['z'], cal_vel,
                                               npc_local_vertices, time_step)
            minTTC = min(minTTC, ttc0)

        ttc1 = ttc_with_simulated_npc_path(sampled_ego_vertices, current_position, current_orientation['z'], current_vel,
                                           npc_local_vertices, time_step)
        minTTC = min(minTTC, ttc1)

        for i, predicted_path in enumerate(perceived_obj['predict_paths']):
            if predicted_path['confidence'] < 1e-2:
                # skip low-confidence paths
                logger.info(f"Skip predicted path {i} of perceived object {perceived_obj['id']} due to low confidence {predicted_path['confidence']}.")
                continue
            no_collision, ttc = check_collision_two_paths(sampled_ego_vertices, predicted_path, npc_local_vertices, time_step)
            if not no_collision:
                minTTC = min(minTTC, ttc)

    if minTTC < 1e9:
        return False, minTTC
    return True, 1e9

def ttc_with_simulated_npc_path(sampled_ego_vertices, current_position, current_heading, current_vel,
                                npc_local_vertices, time_step):
    """
    Assume the NPC keep going with given (current) velocity.
    Simulate the path and check the time to collision (TTC)
    """
    for i, ego_vertex in enumerate(sampled_ego_vertices):
        point = current_position + current_vel * i * time_step
        if check_collision_two_points(ego_vertex, point, current_heading, npc_local_vertices):
            return i * time_step
    return 1e9

def check_collision_two_paths(ego_vertices, predicted_path, npc_local_vertices, time_step):
    """
    Check collision between two paths.
    Return True if no collision.
    Return {False, TTC} if a collision is expected.
    """
    npc_path_time_step = predicted_path['time_step']
    if npc_path_time_step < time_step:
        raise Exception(f'Not handled NPC path time step smaller than {time_step}')
    dense = math.ceil(npc_path_time_step / time_step)

    ego_idx = 0
    for j, pobj_state in enumerate(predicted_path['path']):
        if ego_idx >= len(ego_vertices):
            break

        # calculate predicted position, heading angle, vertices
        pobj_position = np.array(
            (pobj_state['position']['x'], pobj_state['position']['y'])
        )
        if check_collision_two_points(ego_vertices[ego_idx], pobj_position, pobj_state['rotation']['z'], npc_local_vertices):
            return False, j * npc_path_time_step

        ego_idx += 1
        # sampling
        if j < len(predicted_path['path']) - 1:
            next_entry = predicted_path['path'][j+1]
            next_pos = np.array(
                (next_entry['position']['x'], next_entry['position']['y'])
            )
            segment = np.linalg.norm(pobj_position - next_pos) / dense
            for k in range(1, dense):
                pobj_position += segment
                if check_collision_two_points(ego_vertices[ego_idx], pobj_position, pobj_state['rotation']['z'],
                                              npc_local_vertices):
                    return False, j * npc_path_time_step + k * time_step
                ego_idx += 1

    return True, 1e9

def check_collision_two_points(ego_vertices, pobj_position, pobj_heading, pobj_local_vertices):
    pobj_vertices = get_object_world_vertices(
        pobj_position[:2],
        pobj_heading,
        pobj_local_vertices
    )
    return is_collision(ego_vertices, pobj_vertices)

def find_latest_perp_obj(recorded_perp_info, object_id, bounded_entry_id):
    """
    Find the latest perception object with id $object_id.
    The desired object must be in the entry with index before $bounded_entry_id
    :param recorded_perp_info:
    :param object_id:
    :param bounded_entry_id:
    :return:
    """
    max_id = min(bounded_entry_id - 1, len(recorded_perp_info) - 1)
    for i in range(max_id, -1, -1):
        perp_entry = recorded_perp_info[i]
        for obj in perp_entry['objects']:
            if obj['id'] == object_id:
                return obj, perp_entry['timestamp']
    return None, None