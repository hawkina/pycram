# setup the environment
from pycram.designators.action_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.pose import Pose as PoseStamped
from pycram.process_module import simulated_robot, real_robot
from pycram.robot_descriptions import robot_description
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher, SoundRequestPublisher, GraspListener, \
    StartSignalWaiter, HSRBMoveGripperReal
from . import utils, high_level_plans, knowrob_interface
import pycram.external_interfaces.giskard_new as giskard
import tf

with_real_robot = True
# initialize interfaces
instruction_point = PoseStamped([1.45, 4.5, 0], [0, 0, 1, 0])
world = None
robot = None
environment_raw = None
rviz = None
move = None
image_switch = None
sound_pub = None
tf_listener = None
grasp_listener = None
start_signal_waiter = None
lt = None
gripper = None
previous_value = None


# maybe move this into the setup function so that it doesn't get auto-executed?
# init demo in repl:  import demos.pycram_gpsr_demo as gpsr


def setup():
    with (real_robot):
        rospy.loginfo("[CRAM] initialize everything")
        global world, robot, environment_raw, rviz, plan_list, giskard, move, tts, image_switch, sound_pub
        global tf_listener, with_real_robot, grasp_listener, start_signal_waiter, lt, gripper, previous_value

        world = BulletWorld()

        environment_raw = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")
        environment_desig = ObjectDesignatorDescription(names=["kitchen"])

        move = PoseNavigator()
        image_switch = ImageSwitchPublisher()
        sound_pub = SoundRequestPublisher()
        tf_listener = tf.listener.TransformListener()
        grasp_listener = GraspListener()
        start_signal_waiter = StartSignalWaiter()
        lt = LocalTransformer()
        gripper = HSRBMoveGripperReal()
        knowrob_interface.init_knowrob()
        rviz = VizMarkerPublisher()

        robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
        robot.set_color([0.5, 0.0, 0.2, 1])
        robot_desig = ObjectDesignatorDescription(names=["hsrb"])
        RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
        KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

        # sync to kitchen and robot
        RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
        rospy.sleep(2)
        KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

        giskard.init_giskard_interface()
        giskard.clear()
        giskard.sync_worlds()

        previous_value = None

        rospy.loginfo(utils.colors.GREEN + "[CRAM] done with setup")


# def do_stuff():
#     with simulated_robot:
#         rospy.loginfo("now doing things")
#         pose1 = PoseStamped([1.45, 4.5, 0], [0, 0, 1, 0])
#         ParkArmsAction([Arms.LEFT]).resolve().perform()
#         NavigateAction([pose1]).resolve().perform()
#         MoveJointsMotion(["wrist_roll_joint"], [-1.57]).resolve().perform()
#         MoveTorsoAction([0.35]).resolve().perform()
#         MoveGripperMotion(motion="open", gripper="left").resolve().perform() #fails
#         rospy.loginfo("done")


# def test():
#     global plan_list
#     plan_list = utils.get_plans(high_level_plans)
#     rospy.loginfo("imported all plans: ")
#     rospy.loginfo(plan_list)
#     rospy.loginfo("attempt to call cleaning with param test")
#     utils.call_plan_by_name(plan_list, "cleaning", "test")
#     # rviz marker publisher
#     marker = ManualMarkerPublisher()
#     tp = PoseStamped(frame='map', position=[1, 1, 0], orientation=[1, 0, 0, 0])
#     marker.publish(tp)
