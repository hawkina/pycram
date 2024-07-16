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
from . import utils, high_level_plans, knowrob_interface, nlp_processing
import pycram.external_interfaces.giskard_new as giskard
import tf
import pycram.utilities.gpsr_utils as plans
from demos.pycram_gpsr_demo import tf_l

with_real_robot = False # CHANGE set to TRUE for real robot
# initialize interfaces
instruction_point = PoseStamped([1.45, 4.5, 0], [0, 0, 1, 0])
world = None
robot = None
environment_raw = None
environment_desig = None
rviz = None
move = None
image_switch = None
sound_pub = None
grasp_listener = None
start_signal_waiter = None
lt = None
gripper = None
previous_value = None
robot_desig = None


# maybe move this into the setup function so that it doesn't get auto-executed?
# init demo in repl:  import demos.pycram_gpsr_demo as gpsr


def setup():
    rospy.loginfo("[CRAM] initialize everything")
    global world, robot, environment_raw, rviz, plan_list, giskard, move, tts, image_switch, sound_pub
    global tf_listener, with_real_robot, grasp_listener, start_signal_waiter, lt, gripper, previous_value
    global robot_desig, environment_desig, tf_l

    #tf_l = tf.listener.TransformListener()

    world = BulletWorld('DIRECT')
    rospy.sleep(2)

    environment_raw = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_alina.urdf")
    environment_desig = ObjectDesignatorDescription(names=["kitchen"])
    move = PoseNavigator()
    image_switch = ImageSwitchPublisher()
    sound_pub = SoundRequestPublisher()
    grasp_listener = GraspListener()
    start_signal_waiter = StartSignalWaiter()
    lt = LocalTransformer()
    gripper = HSRBMoveGripperReal()
    knowrob_interface.init_knowrob()
    rviz = VizMarkerPublisher()

    robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
    robot.set_color([0.5, 0.0, 0.2, 1])
    robot_desig = ObjectDesignatorDescription(names=["hsrb"])

    # sync to kitchen and robot
    RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
    rospy.sleep(2)
    KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

    giskard.init_giskard_interface()
    giskard.clear()
    giskard.sync_worlds()

    # this is already done in listen to commands
    if nlp_processing.canSpeak:
         nlp_processing.nlp_subscribe()

    rospy.loginfo(utils.PC.GREEN + "[CRAM] done with setup")
    print("###################################################################")


