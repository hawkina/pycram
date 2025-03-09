import inspect
import rospy
from pycram.pose import Pose as PoseStamped
import matplotlib.colors as mcolors
import tf
import json
from typing import Callable

tf_l = tf.listener.TransformListener()
colors = mcolors.cnames
objects_path = '/home/hawkin/ros/pycram_ws/src/pycram/demos/pycram_gpsr_demo/objects.py'


# make rosout more colorful PC = PrintColors
class PC:
    PINK = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    GREY = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# generate a list of all plans instead of having to hardcode them
# this is required for the mapping between NLP and PyCRAM
def get_plans(module):  # works
    module_name = module.__name__
    return {name: obj for name, obj in inspect.getmembers(module)
            if inspect.isfunction(obj) and obj.__module__ == module_name}


def call_plan_by_name(plan_list, name, *args, **kwargs):  # works
    func = plan_list.get(name)
    if func:
        func(*args, **kwargs)
        rospy.loginfo(f"[CRAM] plan {name} found and executed.")
    else:
        rospy.logerr(f"[CRAM] Plan {name} not found.")


# this works for dicts
# k_pose = knowrob Pose
def kpose_to_pose_stamped(k_pose):  # works
    if k_pose is None or k_pose is []:
        rospy.logerr("[UTILS]: Got empty pose for conversion.")
        return PoseStamped()
    try:
        # find a better way? maybe via knowrob?
        pose = PoseStamped(frame=k_pose.get('Frame'), position=k_pose.get('Pose'), orientation=k_pose.get('Quaternion'))
        return pose
    except TypeError or ValueError:
        rospy.logerr("[UTILS] Got empty pose for conversion.")
        return PoseStamped()


# this works for lists
# l_pose = list_pose from knowrob....
def lpose_to_pose_stamped(list_pose):  # works
    if list_pose is None or list_pose is []:
        rospy.logerr("[UTILS] Got empty pose for conversion.")
        return PoseStamped()
    try:
        pose = PoseStamped(frame=list_pose[0],
                           position=list_pose[1],
                           orientation=list_pose[2])
        return pose
    except IndexError or ValueError:
        rospy.logerr("[UTILS]: Got empty pose for conversion.")
        return PoseStamped()  # identity pose


def cond_pairs(cond: bool, pairs: Callable[[], dict], ) -> dict:
    return pairs() if cond else {}


# this is a helper function to transform a pose into a different frame
# it is needed since otherwise tf will complain about interpolating into future for movable objects in semantic map
# since their updates are being published with a too low frequency
def transform_pose(tf_listener, target_frame, pose_stamped, max_attempts=10, attempt_delay=1):
    """
    Attempts to transform a pose to the specified target frame with retry logic.

    Args:
        tf_listener (tf.TransformListener): The TransformListener object.
        target_frame (str): The target frame to transform the pose into.
        pose_stamped (PoseStamped): The pose to transform.
        max_attempts (int): Maximum number of attempts to transform the pose.
        attempt_delay (int or float): Delay between attempts in seconds.

    Returns:
        PoseStamped or None: The transformed pose if successful, None otherwise.
    """
    #rospy.logerr(f"tf listener in transform pose {tf_listener}")
    #rospy.logerr(f"pose: " + str(pose_stamped))
    attempt_count = 0
    while attempt_count < max_attempts:
        try:
            if not tf_listener.canTransform(target_frame, pose_stamped.header.frame_id, rospy.Time(0)):
                rospy.logwarn("[TF]Transform not available. Attempt {} of {}".format(attempt_count + 1, max_attempts))
                rospy.sleep(attempt_delay)
            else:
                rospy.loginfo(f"[TF]Transform available. Attempting transformation to {target_frame}.")
                transformed_pose = tf_listener.transformPose(target_frame, pose_stamped)
                #rospy.logerr(PC.GREEN + f"pose: " + str(pose_stamped))
                return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("[TF]An error occurred during transformation attempt {}: {}".format(attempt_count + 1, e))
            rospy.sleep(attempt_delay)
        attempt_count += 1
    rospy.logerr("Failed to transform pose after {} attempts.".format(max_attempts))
    return None


def lookup_transform(tf_listener, target_frame, source_frame, max_attempts=10, attempt_delay=1):
    """
    Attempts to transform a pose to the specified target frame with retry logic.

    Args:
        tf_listener (tf.TransformListener): The TransformListener object.
        target_frame (str): The target frame to transform the pose into.
        pose_stamped (PoseStamped): The pose to transform.
        max_attempts (int): Maximum number of attempts to transform the pose.
        attempt_delay (int or float): Delay between attempts in seconds.

    Returns:
        PoseStamped or None: The transformed pose if successful, None otherwise.
    """
    attempt_count = 0
    while attempt_count < max_attempts:
        try:
            if not tf_listener.canTransform(target_frame, source_frame, rospy.Time(0)):
                rospy.logwarn("[TF]Transform not available. Attempt {} of {}".format(attempt_count + 1, max_attempts))
                rospy.sleep(attempt_delay)
            else:
                rospy.loginfo(f"[TF]Transform available. Attempting transformation to {target_frame}.")
                transformed_pose = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("[TF]An error occurred during transformation attempt {}: {}".format(attempt_count + 1, e))
            rospy.sleep(attempt_delay)
        attempt_count += 1
    rospy.logerr("Failed to transform pose after {} attempts.".format(max_attempts))
    return None


def knowrob_poses_result_to_list_dict(knowrob_output, nav_or_perc='nav'):  # works
    global tf_l
    poses_list = []
    for item in knowrob_output:
        for raw_pose in item.get('Pose'):
            pose = lpose_to_pose_stamped(raw_pose)
            # transform into map frame
            # pose = tf_l.transformPose("map", pose) # issues with time
            pose = transform_pose(tf_l, "map", pose)
            # ensure z is 0
            if pose is None:
                rospy.logerr("[UTILS] Could not transform pose. Aborting.")
                return None
            if nav_or_perc == 'nav':
                # make it naviagtion pose by ensuring you remove z
                pose.pose.position.z = 0

            # make sure only fields with data are filled
            # maybeh add more fields if needed
            poses_list.append({'Item': {k: v for k, v in {
                'value': item.get('Obj'),
                'name': item.get('Name'),
                'link': raw_pose[0],
                'room': item.get('Room'),
                'pose': pose}.items() if v is not None}})
    return poses_list


# if a color exists in the list, return it in front of a tuple,
# rest of tuple is list of previous items
def find_color(attribute_list):  # works
    for attribute in attribute_list:
        print(attribute)
        if attribute in colors.keys():
            color = attribute
            attribute_list.remove(color)
            return [color, attribute_list]
    return ['', attribute_list]


# for autogenerate a dict of knowledge objects from a file
def autogenerate_dict_from_file(file_path):
    ontology_dict = {}
    with open(file_path, 'r') as file:
        for line in file:
            # Assuming each line is in the format variable_name = 'ontology_URI'
            parts = line.strip().split(' = ')
            if len(parts) == 2:
                key = parts[0].strip().strip("'")
                value = parts[1].strip()
                ontology_dict[key] = value
    return ontology_dict


def write_json_to_file(json_data, file_path='result.json'):
    # Call the function to get the result

    # Convert the result to a JSON string
    result_json = json.dumps(json_data, indent=4)

    # Open the file in write mode and write the result
    with open(file_path, 'w') as file:
        file.write(result_json)

    print(f"Result written to {file_path}")


def pose_stamped_to_json(pose_stamped):
    """
    Convert a PoseStamped object to a JSON string.

    Args:
        pose_stamped (PoseStamped): The PoseStamped object to convert.

    Returns:
        str: The JSON string representation of the PoseStamped object.
    """
    pose_dict = {
        "header": {
            "seq": pose_stamped.header.seq,
            "stamp": {
                "secs": pose_stamped.header.stamp.secs,
                "nsecs": pose_stamped.header.stamp.nsecs
            },
            "frame_id": pose_stamped.header.frame_id
        },
        "pose": {
            "position": {
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y,
                "z": pose_stamped.pose.position.z
            },
            "orientation": {
                "x": pose_stamped.pose.orientation.x,
                "y": pose_stamped.pose.orientation.y,
                "z": pose_stamped.pose.orientation.z,
                "w": pose_stamped.pose.orientation.w
            }
        }
    }
    return json.dumps(pose_dict, indent=4)


def remap_source_to_destination(json_data):
    if json_data.get('Source'):
        json_data['Destination'] = json_data.pop('Source')
        rospy.loginfo(PC.GREY + '[UTILS] remapped Source to Destination')
    else:
        rospy.loginfo(PC.GREY + '[UTILS] no Source found')
    if json_data.get('SourceRoom'):
        json_data['DestinationRoom'] = json_data.pop('SourceRoom')
        rospy.loginfo(PC.GREY + '[UTILS] remapped SourceRoom to DestinationRoom')
    else:
        rospy.loginfo(PC.GREY + '[UTILS] no SourceRoom found')
    return json_data


def remove_prefix(text, prefix):
    if text.startswith(prefix):
        return text[len(prefix):]
    return text


# autogenerate a dict from all defined objects in the objects.py file
obj_dict = autogenerate_dict_from_file(objects_path)
