# maps the result of nlp to high level plans
import rospy

from .import utils, high_level_plans, plan_list
plans_list = plan_list  # this is stupid...


def nlp_to_plans_mapping(todo_plans):
    global plans_list
    rospy.logdebug("plans to do are: " + str(todo_plans))

    if not plans_list:
        rospy.loginfo("plan list is empty. Initializing...\n")
        plans_list = utils.get_plans(high_level_plans)
        rospy.loginfo("Initialized plans list.")
        rospy.logdebug("Available plans are: \n" + str(plans_list) + "\n")

    if not todo_plans:
        rospy.loginfo("todo_plans is empty.")
        return

    if plans_list.get(todo_plans["intent"].lower()):  # ensure lower case for matching
        plan_name = todo_plans["intent"].lower()
        rospy.loginfo("found matching plan: " + plan_name)
        utils.call_plan_by_name(plans_list, plan_name, todo_plans)
    else:
        rospy.loginfo("no matching plan found")
