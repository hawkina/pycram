import rospy
import actionlib
from demos.pycram_gpsr_demo.utils import find_color
from pycram.designators.action_designator import LookAtAction, DetectAction
from pycram.process_module import real_robot
from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult, QueryActionResult

rk = actionlib.SimpleActionClient('robokudo/query', QueryAction)


def init_robokudo():
    global rk
    rk = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("[RK] Waiting for action server...")
    if rk.wait_for_server():
        rospy.loginfo("[RK] ready")
    else:
        rospy.loginfo("[RK] something went wrong during connection")


def make_robokudo_obj_msg(item_json):
    goal_msg = QueryGoal()
    goal_msg.type = 'detect'
    goal_msg.obj.type = item_json.get('value')
    color_results = find_color(item_json.get('propertyAttribute'))
    goal_msg.obj.color = color_results[0]
    goal_msg.obj.attribute = color_results[1]
    goal_msg.obj.pose_source = ''
    goal_msg.obj.description = ''
    return goal_msg


def ask_robokudo_for_all_objects():
    global rk
    goal_msg = QueryGoal()
    goal_msg.type = 'all'
    rk.send_goal(goal_msg)
    rospy.loginfo("[RK] goal sent... waiting for result")
    rospy.wait_for_message(topic='/robokudo/query/result', topic_type=QueryActionResult, timeout=15)
    result = rk.get_result()
    rospy.loginfo("[RK] result received")
    return result  # list of all perceived items or an empty list


# todo test from here ----
def ask_robokudo_for_object(obj_type):
    global rk
    goal_msg = QueryGoal()
    goal_msg.obj.type = obj_type  # object type
    goal_msg.type = 'all'
    rk.send_goal(goal_msg)
    rospy.loginfo("[RK] goal sent... waiting for result")
    rospy.wait_for_message(topic='/robokudo/query/result', topic_type=QueryActionResult, timeout=15)
    result = rk.get_result()
    rospy.loginfo("[RK] result received")
    return result  # list of all perceived items or an empty list


def send_robokudo_goal(goal_msg):
    global rk
    rk.send_goal(goal_msg)
    rospy.loginfo("[RK] goal sent... waiting for result")
    rospy.wait_for_message(topic='/robokudo/query/result', topic_type=QueryActionResult, timeout=15)
    result = rk.get_result()
    rospy.loginfo("[RK] result received")
    return result


# todo test end here------

### --- REPL testing ---
def test_pc():
    global rk
    rk = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    rk.wait_for_server()
    rospy.loginfo("You can start your demo now")
    goal_msg = QueryGoal()
    goal_msg.obj.type = 'Crackerbox'  # human doesn't work somehow? 'mueslibox'
    goal_msg.type = 'all'
    rk.send_goal(goal_msg)
    result = rk.get_result()
    print(result.res[0].type)


def test_lookat(pose):
    with real_robot:
        LookAtAction(targets=[pose]).resolve().perform()

# this would be the designator way, which we are currently not using since it is slow
#def test_perception():
#    # LookAtAction().resolve().perform()
#    with real_robot:
#        try:  # todo changed couch_table to pickup_location_name
#            object_desig = DetectAction(technique='all').resolve().perform()
#        except PerceptionObjectNotFound:
#            object_desig = {}
#            print(object_desig)
#        return object_desig
### to get the result I would need to do this: object_desig[1].get('Metalmug_1719825441.1798751')
