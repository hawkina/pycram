import rospy
import actionlib
from rospy import ROSException

from demos.pycram_gpsr_demo import sing_my_angel_of_music
from demos.pycram_gpsr_demo.utils import find_color
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


def ask_robokudo_for_all_objects(): # works
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


# person = attributes of person, lying, standing etc.
# faces = face detection
def ask_robokudo_for_humans():
    global rk
    goal_msg = QueryGoal()
    goal_msg.obj.type = 'person'
    goal_msg.type = 'person'
    rk.send_goal(goal_msg)
    rospy.loginfo("[RK] goal sent... waiting for result")
    rospy.wait_for_message(topic='/robokudo/query/result', topic_type=QueryActionResult, timeout=15)
    result = rk.get_result()
    rospy.loginfo("[RK] result received")
    return result  # list of all perceived items or an empty list


# DetectHumanAction ohne face.

def send_robokudo_goal(goal_msg):
    global rk
    rk.send_goal(goal_msg)
    rospy.loginfo("[RK] goal sent... waiting for result")
    rospy.wait_for_message(topic='/robokudo/query/result', topic_type=QueryActionResult, timeout=15)
    result = rk.get_result()
    rospy.loginfo("[RK] result received")
    return result


# all poses are returned in map frame
def process_robokudo_obj_result(result):
    result_dict = {}
    for msg in result.res:
        entry = {}
        entry = {
            'type': msg.type,
            'color': msg.color,
            'attribute': msg.attribute,
            'pose': msg.pose,
            'pose_source': msg.pose_source,
            'description': msg.description
        }
        result_dict[msg.type] = entry
    return result_dict


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



def looking_for_human():
    # loop until a human is seen
    check_human = False
    sing_my_angel_of_music("Please step infront of me.")
    while not check_human:
        try:
            rospy.loginfo("[CRAM] Looking for human...")
            result = ask_robokudo_for_humans()
            if result is not None:
                check_human = True
                rospy.loginfo("[CRAM] Human found")
            else:
                rospy.loginfo("[CRAM] No human found")
            rospy.sleep(3)
        except ROSException:
            rospy.loginfo("[CRAM] No human found and robokudo timed out")
    return check_human

# def test_lookat(pose):
#     with real_robot:
#         LookAtAction(targets=[pose]).resolve().perform()

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
