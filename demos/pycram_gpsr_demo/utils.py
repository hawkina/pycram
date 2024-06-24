import inspect

import rospy

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
        rospy.loginfo("plan found and executed")
    else:
        rospy.loginfo(f"Plan {name} not found.")

