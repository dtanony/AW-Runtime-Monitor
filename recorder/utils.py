from scipy.spatial.transform import Rotation as R

def round_float(num):
    return round(float(num), 3)

def object2dictpoint(obj, round=True):
    x, y, z = obj.x, obj.y, obj.z
    if round:
        x = round_float(x)
        y = round_float(y)
        z = round_float(z)
    return {
        "x": x,
        "y": y,
        "z": z,
    }

def object2dictquaternion(obj):
    return {
        "x": round_float(obj.x),
        "y": round_float(obj.y),
        "z": round_float(obj.z),
        "w": round_float(obj.w),
    }

def object2dicteulerangle(obj, round=True):
    roll, pitch, yaw = quaternion2eulerangle([obj.x, obj.y, obj.z, obj.w])
    if round:
        roll = round_float(roll)
        pitch = round_float(pitch)
        yaw = round_float(yaw)
    return {
        "x": roll,
        "y": pitch,
        "z": yaw,
    }

def object2timestamp(obj, round=True):
    result = obj.sec + obj.nanosec/10**9
    if round:
        return round_float(result)
    return result

def quaternion2eulerangle(quaternion):
    r = R.from_quat(quaternion)

    # roll, pitch, yaw <- euler_angles_deg
    return r.as_euler('xyz', degrees=True)

# uuid (int size of 16) to string, in order to reduce text size
def uuidstr(uuid):
    return " ".join(map(str, uuid))

def point22Dpoint(point):
    return (point.x, point.y)

if __name__ == '__main__':
    roll, pitch, yaw = quaternion2eulerangle([0,0,0.13,0.992])
    print(roll)
    print(pitch)
    print(yaw)