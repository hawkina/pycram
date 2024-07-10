# setup the environment
from pycram.designators.action_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.pose import Pose
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

# initialize interfaces
world = 0
robot = 0
environment_raw = 0
rviz = 0
instruction_point = Pose([1.45, 4.5, 0], [0, 0, 1, 0])
move = PoseNavigator()
image_switch = ImageSwitchPublisher()
sound_pub = SoundRequestPublisher()
tf_listener = tf.listener.TransformListener()
with_real_robot = True
grasp_listener = GraspListener()
start_signal_waiter = StartSignalWaiter()
lt = LocalTransformer()
gripper = HSRBMoveGripperReal()
previous_value = None


# maybe move this into the setup function so that it doesn't get auto-executed?
# init demo in repl:  import demos.pycram_gpsr_demo as gpsr


def setup():
    with (real_robot):
        rospy.loginfo("init")
        global world, robot, environment_raw, rviz, plan_list, giskard, move, tts, image_switch, sound_pub
        global tf_listener, with_real_robot
        with_real_robot = True
        giskard.init_giskard_interface()
        #world = BulletWorld("DIRECT") #rviz only, without any parameters, spawns bullet
        knowrob_interface.init_knowrob()
        rospy.loginfo("init setup")
        world = BulletWorld()
        environment_raw = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")
        environment_desig = ObjectDesignatorDescription(names=["kitchen"])

        robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
        robot.set_color([0.5, 0.0, 0.2, 1])
        robot_desig = ObjectDesignatorDescription(names=["hsrb"])

        #rviz = VizMarkerPublisher()





        # sync to kitchen and robot
        RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
        rospy.sleep(2)
        KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
        # giskard.sync_worlds()
        giskard.sync_worlds()

        rospy.loginfo("done with setup")


def do_stuff():
    with simulated_robot:
        rospy.loginfo("now doing things")
        pose1 = Pose([1.45, 4.5, 0], [0, 0, 1, 0])
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        NavigateAction([pose1]).resolve().perform()
        MoveJointsMotion(["wrist_roll_joint"], [-1.57]).resolve().perform()
        MoveTorsoAction([0.35]).resolve().perform()
        MoveGripperMotion(motion="open", gripper="left").resolve().perform() #fails
        rospy.loginfo("done")


def test():
    global plan_list
    plan_list = utils.get_plans(high_level_plans)
    rospy.loginfo("imported all plans: ")
    rospy.loginfo(plan_list)
    rospy.loginfo("attempt to call cleaning with param test")
    utils.call_plan_by_name(plan_list, "cleaning", "test")
    # rviz marker publisher
    marker = ManualMarkerPublisher()
    tp = Pose(frame='map', position=[1, 1, 0], orientation=[1, 0, 0, 0])
    marker.publish(tp)
