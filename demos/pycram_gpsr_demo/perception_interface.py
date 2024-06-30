import actionlib
import rospy

from pycram.designators.action_designator import LookAtAction, DetectAction
from pycram.plan_failures import PerceptionObjectNotFound
from pycram.process_modules import hsrb_process_modules
from pycram.process_module import real_robot
from pycram.external_interfaces import robokudo
from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult



def test_perception():
    # LookAtAction().resolve().perform()
    with real_robot:
        try:  # todo changed couch_table to pickup_location_name
            object_desig = DetectAction(technique='all').resolve().perform()
        except PerceptionObjectNotFound:
            object_desig = {}
            print(object_desig)
        return object_desig


def test_pc():
    robokudo_client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    robokudo_client.wait_for_server()
    rospy.loginfo("You can start your demo now")
    goal_msg = QueryGoal()
    goal_msg.obj.type = 'Crackerbox' # human doesn't work somehow? 'mueslibox'
    goal_msg.type='all'
    robokudo_client.send_goal(goal_msg)
    result = robokudo_client.get_result()
    print(result.res[0].type)



