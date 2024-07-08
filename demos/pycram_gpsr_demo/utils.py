import inspect
import rospy
from pycram.pose import Pose
import matplotlib.colors as mcolors
import tf
from typing import Callable

tf_listener = tf.listener.TransformListener()
colors = mcolors.cnames
objects_path = 'demos/pycram_gpsr_demo/objects.py'


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
        return Pose()
    try:
        # find a better way? maybe via knowrob?
        pose = Pose(frame=k_pose.get('Frame'), position=k_pose.get('Pose'), orientation=k_pose.get('Quaternion'))
        return pose
    except TypeError or ValueError:
        rospy.logerr("[UTILS] Got empty pose for conversion.")
        return Pose()


# this works for lists
# l_pose = list_pose from knowrob....
def lpose_to_pose_stamped(list_pose):  # works
    if list_pose is None or list_pose is []:
        rospy.logerr("[UTILS] Got empty pose for conversion.")
        return Pose()
    try:
        pose = Pose(frame=list_pose[0],
                    position=list_pose[1],
                    orientation=list_pose[2])
        return pose
    except IndexError or ValueError:
        rospy.logerr("[UTILS]: Got empty pose for conversion.")
        return Pose()  # identity pose


def cond_pairs(cond: bool, pairs: Callable[[], dict], ) -> dict:
    return pairs() if cond else {}


def knowrob_poses_result_to_list_dict(knowrob_output):  # works
    poses_list = []
    for item in knowrob_output:
        for raw_pose in item.get('Pose'):
            pose = lpose_to_pose_stamped(raw_pose)
            # transform into map frame
            pose = tf_listener.transformPose("map", pose)
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


# autogenerate a dict from all defined objects in the objects.py file
obj_dict = autogenerate_dict_from_file(objects_path)
