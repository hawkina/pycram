from enum import Enum

from move_base_msgs.msg import MoveBaseAction
from roslibpy import actionlib

from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_hsrb_real_test_demos.utils.misc import *
from demos.pycram_hsrb_real_test_demos.utils.misc import sort_objects
# from pycram.external_interfaces.knowrob import get_table_pose
import pycram.helper as helper
import rospkg


# publish robot on param server for gripper tool frame
# rospack = rospkg.RosPack()
# name = "hsrb.urdf"
# package_path = rospack.get_path('pycram') + '/resources/' + name
# urdf_string = helper.urdf_to_string(package_path)
# rospy.set_param('robot_description', urdf_string)#

#move = PoseNavigator()

from geometry_msgs.msg import Point
# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning



#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

#giskardpy.removing_of_objects()
#giskardpy.sync_worlds()
robot.set_color([0.5, 0.5, 0.9, 1])


# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

giskardpy.init_giskard_interface()

giskardpy.sync_worlds()

#client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)

# Main interaction sequence with real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
    #ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([2, 5, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    #MoveTorsoAction([0.2]).resolve().perform()
    # LookAtAction(targets=[Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7])]).resolve().perform()
    object_desig = DetectAction(technique='all').resolve().perform()
    sort_objects = sort_objects(robot, object_desig, wished_sorted_obj_list=["Metalbowl"])
    for value in range(len(sort_objects)):
        #move.query_pose_nav(Pose([sort_objects[value].pose.position.x + 0.2,
                                               #robot.get_pose().pose.position.y, 0], [0, 0, 0.7, 0.7]))

        # NavigateAction(target_locations=[Pose([sort_objects[value].pose.position.x + 0.2,
                                               # robot.get_pose().pose.position.y, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
        # angle = 1.6 - robot.get_joint_state('arm_roll_joint')
        # print(f"arm_roll: {robot.get_joint_state('arm_roll_joint')}")
        # print(f"angle: {angle}")
        #lt = LocalTransformer()
        #print(lt.get_all_frames())
        #print(sort_objects[value])


        #print(frame_final)
        PouringAction([Pose([sort_objects[value].pose.position.x, sort_objects[value].pose.position.y,
                             sort_objects[value].pose.position.z])], ["left"], ["right"],
                      [115]).resolve().perform()
        #PouringAction([sort_objects[value]], ["left"], ["right"], [-angle]).resolve().perform()

    #PouringAction([Pose([4, 2, 0.75], [0, 0, 0, 1])],["left"],["right"], [-1.6]).resolve().perform()
    #PouringMotion("right", 0).resolve().perform()
    #MoveTCPMotion(Pose([4, 2, 0.85], [0, 0, 0, 1]), "left").resolve().perform()
