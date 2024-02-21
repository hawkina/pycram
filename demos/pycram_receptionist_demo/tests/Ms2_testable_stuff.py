import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.fluent import Fluent
from demos.pycram_receptionist_demo.utils.misc import *
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.external_interfaces.knowrob import get_guest_info
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from demos.pycram_receptionist_demo.deprecated import talk_actions
import pycram.external_interfaces.navigate as moveBase

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# carefull that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 0)
pose_couch = Pose([3, 5, 0], robot_orientation_couch)

robot_orientation_from_couch = axis_angle_to_quaternion([0, 0, 1], -90)
pose_from_couch = Pose([4.2, 3.8, 0], robot_orientation_from_couch)

robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)

pose_home = Pose([3, 1.7, 0], robot_orientation)

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

data_received = False


############################



#################################################################

def demo_test(area):
    with real_robot:
        #guest_data = get_guest_info("5.0")
        global data_received
        data_received = False
        print("start demo")
        #host = HumanDescription("Bob", fav_drink="Coffee")
        #host.human_pose.set_value(False)
        #guest1 = HumanDescription("guest1")
        #guest1.human_pose.set_value(False)

        TalkingMotion("Hello").resolve().perform()


        # While loop, human is detected
        #while not guest1.human_pose.get_value():
         #   rospy.loginfo("value: " + str(guest1.human_pose.get_value()))
          #  rospy.loginfo("in while")
        DetectAction(technique='human', state='start').resolve().perform()
           # rospy.sleep(5)
            #TalkingMotion("please step in front of me").resolve().perform()
            # Perception, detect first guest


        rospy.loginfo("human detected")

        # look at guest and introduction
        giskardpy.move_head_to_human()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()

        rospy.sleep(1)

        # signal to start listening
        pub_nlp.publish("start listening")
        #wait for human to say something
        rospy.sleep(5)

        # TODO: might not work, if so use version from Ms1 in comments
        #guest_data = get_guest_info("1.0") # guest_data format is = ["name", "drink"]
        #print(str(guest_data))
        #while guest_data == "No name saved under this ID!":
        #    talk_error("no name")
        #    guest_data = get_guest_info(1)
        #    rospy.sleep(3)

        # failure handling
        rospy.Subscriber("nlp_feedback", Bool, talk_error)

        # receives name and drink via topic
        rospy.Subscriber("nlp_out", String, talk_request)
        #guest1.set_name(guest_data[0])
        #guest1.set_drink(guest_data[1])
        #talk_request(guest_data)

        # TODO: does the rest of the code waits for talk_request to be executed?
        # if not sleep has to stay
        rospy.sleep(5)

        while not data_received:
            rospy.sleep(0.5)

        #guest_data = get_guest_info("2.0")  # guest_data format is =
        #print(str(guest_data))
        #rospy.sleep(4)
        # lead human to living room
        TalkingMotion("i will stop looking now").resolve().perform()
        rospy.loginfo("stop looking now")
        giskardpy.stop_looking()

        # stop perceiving human
        rospy.loginfo("stop detecting")
        DetectAction(technique='human', state='stop').resolve().perform()

        #guest1.human_pose.set_value(False)
        #host.human_pose.set_value(False)



        if area == 'to_couch':
            rospy.loginfo("Navigating now")
            TalkingMotion("navigating to couch area now, please step away").resolve().perform()
            rospy.sleep(5)
            NavigateAction([pose_kitchen_to_couch]).resolve().perform()
            NavigateAction([pose_couch]).resolve().perform()
        elif area == 'from_couch':
            rospy.loginfo("Navigating now")
            TalkingMotion("navigating to couch area now, please step away").resolve().perform()
            rospy.sleep(5)
            NavigateAction([pose_from_couch]).resolve().perform()
            NavigateAction([pose_home]).resolve().perform()
        else:
            rospy.loginfo("in else")
            TalkingMotion("not navigating").resolve().perform()
            rospy.sleep(5)
            print("end")

        TalkingMotion("End of demo").resolve().perform()




def nav_test():
    with real_robot:
        robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
        test_pose1 = Pose([4.2, 3, 0], robot_orientation)
        test_pose = Pose([3, 5, 0], [0, 0, 0, 1])
        moveBase.queryPoseNav(test_pose1)
        moveBase.queryPoseNav(test_pose)


#demo_test('from_couch')
demo_test('now')
