# setup the environment
from sympy.utilities.iterables import kbins

from pycram.datastructures.dataclasses import Color
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.designators.action_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.datastructures.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher, SoundRequestPublisher, GraspListener, \
    StartSignalWaiter, HSRBMoveGripperReal
from . import utils, high_level_plans, knowrob_interface, nlp_processing
import demos.pycram_gpsr_demo.perception_interface as perception_interface
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

with_real_robot = True # CHANGE set to TRUE for real robot
# initialize interfaces
#instruction_point = PoseStamped([1.45, 4.5, 0], [0, 0, 1, 0])
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
extension = None # what is this for?


# maybe move this into the setup function so that it doesn't get auto-executed?
# init demo in repl:  import demos.pycram_gpsr_demo as gpsr
def setup_bullet():
    global world, robot, environment_raw, rviz, plan_list, move, tts, image_switch, sound_pub
    global tf_listener, with_real_robot, grasp_listener, start_signal_waiter, lt, gripper, previous_value
    global robot_desig, environment_desig, tf_l, extension

    extension = ObjectDescription.get_file_extension()
    world = BulletWorld(WorldMode.DIRECT)

    rviz = VizMarkerPublisher()

    # init environment
    environment = Object("arena", ObjectType.ENVIRONMENT, "robocup_vanessa.urdf")
    environment_desig = ObjectDesignatorDescription(names=["arena"])
    extension = ObjectDescription.get_file_extension()

    # init robot
    robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}", pose=Pose([1, 2, 0]))
    #robot.set_color(rgba_color=Color().from_list([0.5, 0.5, 0.9, 1]))
    robot_desig = ObjectDesignatorDescription(names=["hsrb"])

    lt = LocalTransformer()

    # TODO add objects to the world
    # on top of dinner table
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([8.1, 4.2, 0.86]), color=[1, 0, 0, 1])
    # on top of dishwasher (predefined location)
    cup = Object("cup", ObjectType.JEROEN_CUP, "../../resources/jeroen_cup.stl", pose=Pose([10.3, 5.29, 0.9]),
                 color=[0, 0, 1, 0])

    # cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
    #                 color=[0, 1, 0, 1])
    # spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
    # bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
    # human_female = Object("human_female", ObjectType.HUMAN, "female_standing.stl", pose=Pose([3, 3, 0]),
    #                       color=[1, 1, 0, 1])

    world.simulate(seconds=1, real_time=True)

    # init knowrob
    knowrob_interface.init_knowrob()

    # No longer needed? at least not for simulation?
    # move = PoseNavigator()
    # image_switch = ImageSwitchPublisher()
    # sound_pub = SoundRequestPublisher()
    # grasp_listener = GraspListener()
    # start_signal_waiter = StartSignalWaiter()
    #
    # gripper = HSRBMoveGripperReal()
    # TODO add KnowRob back in
    # knowrob_interface.init_knowrob()
    # perception_interface.init_robokudo()
    #
    # # sync to kitchen and robot
    # RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
    # rospy.sleep(2)
    # KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
    #
    # giskard.init_giskard_interface()
    # giskard.clear()
    # giskard.sync_worlds()
    #
    # # this is already done in listen to commands
    # if nlp_processing.canSpeak:
    #      nlp_processing.nlp_subscribe()

    rospy.loginfo(utils.PC.GREEN + "[CRAM] done with setup")
    print("###################################################################")

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

    perception_interface.init_robokudo()

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


