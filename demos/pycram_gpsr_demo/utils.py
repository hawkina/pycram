import inspect

import rospy
from pycram.pose import Pose

from . import high_level_plans


# generate a list of all plans instead of having to hardcode them
# this is required for the mapping between NLP and PyCRAM
def get_plans(module):
    module_name = module.__name__
    return {name: obj for name, obj in inspect.getmembers(module)
            if inspect.isfunction(obj) and obj.__module__ == module_name}


def call_plan_by_name(plan_list, name, *args, **kwargs):
    func = plan_list.get(name)
    if func:
        func(*args, **kwargs)
        rospy.loginfo(f"[CRAM] plan {name} found and executed.")
    else:
        rospy.logerr(f"[CRAM] Plan {name} not found.")


# this works for dicts
# k_pose = knowrob Pose
def kpose_to_pose_stamped(k_pose):
    if k_pose is None or k_pose is []:
        rospy.logerr("CRAM-KnowRob: Got empty pose for conversion.")
        return Pose()
    try:
        # find a better way? maybe via knowrob?
        pose = Pose(frame=k_pose.get('Frame'), position=k_pose.get('Pose'), orientation=k_pose.get('Quaternion'))
        return pose
    except TypeError or ValueError:
        rospy.logerr("CRAM-KnowRob: Got empty pose for conversion.")
        return Pose()


# l_pose = list_pose from knowrob....
def lpose_to_pose_stamped(list_pose):
    if list_pose is None or list_pose is []:
        rospy.logerr("CRAM-KnowRob: Got empty pose for conversion.")
        return Pose()
    try:
        pose = Pose(frame=list_pose[0],
                    position=list_pose[1],
                    orientation=list_pose[2])
        return pose
    except IndexError or ValueError:
        rospy.logerr("[CRAM-KnowRob]: Got empty pose for conversion.")
        return Pose() # identity pose



# --- repl specific ---
def reset():
    #global todo_list, plans_list
    todo_list = []
    plans_list = get_plans(high_level_plans)
    rospy.loginfo("[CRAM] reset done.")