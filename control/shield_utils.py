import numpy as np

def dict_point_to_array(dict_point):
    return np.array((
        dict_point['x'],
        dict_point['y'],
        dict_point['z'],
    ))

def get_object_world_vertices(position, heading_angle_degree, local_vertices):
    theta = np.deg2rad(heading_angle_degree)
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # Rotate and translate to world frame
    world_vertices = (rot @ np.array(local_vertices).T).T + position
    return world_vertices

def get_center_rect(position, heading_angle_degree, center_offset):
    """Return the center point of the vehicle in world coordinates."""
    theta = np.deg2rad(heading_angle_degree)
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    return position + rot @ center_offset

def get_ego_world_vertices(position, heading_angle_degree, size, center_offset):
    """
    :param position:
    :param heading_angle_degree:
    :param size: (length, width)
    :param center_offset: (x,y)
    :return:
    """
    width, length = size
    dx = width / 2
    dy = length / 2

    # Local rectangle corners (FR, FL, RL, RR) relative to vehicle center
    local_vertices = np.array([
        [ dx,  dy],  # front-right
        [-dx,  dy],  # front-left
        [-dx, -dy],  # rear-left
        [ dx, -dy],  # rear-right
    ])

    # Rotation matrix (counter-clockwise heading)
    theta = np.deg2rad(heading_angle_degree)
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Rotate and translate to world frame
    center_world = get_center_rect(position, heading_angle_degree, center_offset)
    world_vertices = (rot @ local_vertices.T).T + center_world
    return world_vertices

def is_collision(ego_vertices, ppobject_vertices):
    A, B, C, D = ego_vertices
    for P in ppobject_vertices:
        if point_inside_rect(P, A,B,C,D):
            return True
    return point_inside_polygon(A, ppobject_vertices) or \
        point_inside_polygon(B, ppobject_vertices) or \
        point_inside_polygon(C, ppobject_vertices) or \
        point_inside_polygon(D, ppobject_vertices)

def sign_line_eq(P, A, B):
    """
    Suppose AB has the line equation ax+by+c=0.
    This func returns a*px+b*py+c, where P(px,py
    """
    ax, ay = A
    bx, by = B
    px,py = P
    m = float(ay - by)
    n = float(bx - ax)
    return m*(px - ax) + n*(py - ay)

def point_inside_rect(P, A,B,C,D):
    """
    Check if point P is inside or on the rectangle defined by points A, B, C, D.
    Points should be given as (x, y) tuples or numpy arrays.
    Assumes the points A, B, C, D are given in order (clockwise or counterclockwise).
    """
    signs = [
        sign_line_eq(P, A, B) >= 0,
        sign_line_eq(P, B, C) >= 0,
        sign_line_eq(P, C, D) >= 0,
        sign_line_eq(P, D, A) >= 0,
    ]
    all_left = not any(signs)
    all_right = all(signs)
    return all_left or all_right

def point_inside_polygon(P, polygon):
    """
    Check if point P is inside or on the polygon.
    :param P:
    :param polygon: numpy array of 2D point
    :return:
    """
    num_vertices = polygon.shape[0]
    signs = []

    for i in range(num_vertices):
        A = polygon[i]
        B = polygon[(i + 1) % num_vertices]
        s = sign_line_eq(P, A, B)
        signs.append(s)

    # Check if all signs are non-negative or all non-positive
    all_non_negative = all(s >= 0 for s in signs)
    all_non_positive = all(s <= 0 for s in signs)

    return all_non_negative or all_non_positive


def get_ego_size(recorded_messages):
    if "groundtruth_size" not in recorded_messages or\
        "vehicle_sizes" not in recorded_messages["groundtruth_size"]:
        return None

    vehicle_sizes = recorded_messages["groundtruth_size"]['vehicle_sizes']
    return next((entry for entry in vehicle_sizes if entry['name'] == 'ego'), None)

def get_perceived_objects(recorded_messages, timestamp):
    """
    get perceived objects information at $timestamp
    :param recorded_messages:
    :param timestamp:
    :return:
    """
    if 'perception_objects' not in recorded_messages:
        return None
    return get_recorded_info_at_time(recorded_messages['perception_objects'], timestamp)

def get_ego_kinematics(recorded_messages, timestamp):
    """
    get ego kinematics information at $timestamp
    :param recorded_messages:
    :param timestamp:
    :return:
    """
    if 'ego_estimated_kinematic' not in recorded_messages:
        return None
    return get_recorded_info_at_time(recorded_messages['ego_estimated_kinematic'], timestamp)

def get_recorded_info_at_time(states, target_time, time_step=0.1):
    window_start = target_time - time_step / 2
    window_end = target_time + time_step / 2
    closest_entry = None
    closest_distance = float('inf')
    _id = -1

    # Loop through data entries within the window
    # find the entry with the closest timestamp to the target time
    for i in range(len(states) - 1, -1, -1):
        state = states[i]
        ts = state["timestamp"]
        if ts < window_start:
            break
        if ts <= window_end:
            distance = abs(ts - target_time)
            if distance < closest_distance:
                closest_distance = distance
                closest_entry = state
                _id = i

    return closest_entry, _id
