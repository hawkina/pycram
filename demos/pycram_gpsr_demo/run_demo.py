from dynamic_reconfigure.msg import DoubleParameter, IntParameter, BoolParameter, StrParameter, GroupState, Config
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from pybullet_utils.transformations import quaternion_matrix

from pycram import failures
from pycram.datastructures.pose import Pose as PoseStamped
from pycram.process_module import real_robot
from pycram.utilities.robocup_utils import StartSignalWaiter, SoundRequestPublisher
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from . import utils, high_level_plans, perception_interface, knowrob_interface
from .perception_interface import *
from . import nlp_processing as nlp
from stringcase import snakecase
from . import llp_navigation as navi
from .nlp_processing import sing_my_angel_of_music
#import src.pycram.utilities.gpsr_utils as gpsr_utils
from pycram.datastructures.enums import ObjectType, ImageEnum
from pycram.language import Code, Monitor
from .utils import monitor_func
from . import setup_demo

#from demos.pycram_gpsr_demo.setup_demo import image_switch
fts = ForceTorqueSensor(robot_name='hsrb')
instruction_point = PoseStamped([6.19, 2.4, 0], [0, 0, 0, 1])
#instruction_point = PoseStamped([4.4, -0.5, 0], [0, 0, 0, 1])
#image_switch = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()


# todo: move to utils or smth
# broken ask David about it
def look_around():
    # move head around in order to detect a waving person
    #with real_robot:
    MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
    human_pose = []
    x = -0.5
    while x <= 1 and human_pose == []:
        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            #human_pose = DetectAction(technique='waving', state='start').resolve().perform()
            rk_result = ask_robokudo_for_waving_humans()
            print("rk result: ", rk_result)
            if rk_result or (rk_result is not None) or (rk_result is not []):
                human_pose = process_robokudo_obj_result(rk_result).get('human').get('pose')
            # or
            #human_pose = process_robokudo_obj_result(rk_result).get('pose')
        except failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if human_pose:
            break

        x += 0.35 # increase this step if you want larger moving head motions
        if x == 1:
            x = -0.5
    print("human Pose: ", human_pose)
    return human_pose

# calculates pose infront of person. From Meike
def set_pose_in_front(goalPose: PoseStamped, dist : float):
    rotation_matrix = quaternion_matrix([goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])

    forward_vector = rotation_matrix[:3, 0]
    distance = dist
    new_pos = np.array([goalPose.pose.position.x, goalPose.pose.position.y, goalPose.pose.position.z]) - distance * forward_vector
    adjusted_pose = PoseStamped(position=[new_pos[0], new_pos[1], 0.0], orientation=[goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])
    print(adjusted_pose)
    return adjusted_pose


def go_to_person(human_pose: PoseStamped, offset = 0.5):
    sing_my_angel_of_music("Found a person.")
    #goal_pose = PoseStamped(position= [human_pose.pose.position.x - offset,
    #                                   human_pose.pose.position.y - offset,
    #                                   0.0],
    #                        orientation=human_pose.pose.orientation)
    goal_pose = set_pose_in_front(human_pose, 1.0)
    sing_my_angel_of_music("Going to the person.")
    result = navi.go_to_pose(goal_pose) # Pose if successfully, None if not
    return result

# room = 'office'
# entry_or_exit = 'entry'
def look_for_person_in_room_and_go_to_them(room, entry_or_exit, hardcoded_pose=None):
    if hardcoded_pose:
        navi.go_to_pose(hardcoded_pose) # TODO PUT BACK
    else:
        navi.go_to_pose(knowrob_interface.get_room_pose(room, entry_or_exit)) # TODO PUT BACK
    # --- OFFICE ---
    # look for a person
    sing_my_angel_of_music(f"Looking for a waving person in {room}.")
    human_pose = look_around()
    if human_pose:
        go_to_person(human_pose[0])
        return True
    else:
        sing_my_angel_of_music("No person found.")
        return False

def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    print(der.wrench.force.x)
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception")
        return SensorMonitoringCondition

    return False

def wait_for_door():
    try:
        # image_switch.pub_now(ImageEnum.HI.value)  # hi im toya
        sing_my_angel_of_music("Push down my Hand, when you are Ready.")
        # image_switch.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        sing_my_angel_of_music("Starting EGPSR.")

# go to room
# look for person
# navigate to person
# look at them
# ask them how we can help
# repeat what they said

def ask_person_for_task_and_try_to_execute_it():
    plan_list = utils.get_plans(high_level_plans)
    perception_interface.looking_for_human()
    giskard.move_head_to_human()
    # move torso up
    MoveJointsMotion(["arm_flex_joint"], [-0.25]).perform()
    MoveJointsMotion(["torso_lift_joint"], [0.2]).perform()
    # listen to commands
    instruction_list = nlp.listen_to_commands()
    rospy.logwarn("[CRAM] instruction list: " + str(instruction_list))
    giskard.cancel_goal()

    # execute instructions
    # TODO iterate over list of instructions and do stuff
    while instruction_list:  # Test
        rospy.logwarn("[CRAM] in instruction loop")
        instruction = instruction_list.pop(0)
        rospy.loginfo(instruction)
        # do stuff
        # match instruction to plan
        #
        #utils.call_plan_by_name(plan_list, snakecase(instruction['intent']), instruction)
        sing_my_angel_of_music("Thank you for the task. I will first look for other tasks. You can stop waving for now. See you soon!")
        # if plan was successful, remove it from the list
        # instruction_list.remove(instruction) # if it gets poped then removal is not needed

def egpsr():
    # wait infront of door for it to open
    with real_robot:
        wait_for_door() # TODO PUT BACK IN
        sing_my_angel_of_music("Entering the Arena.") # TODO PUT BACK IN
        person_found = None

        # --- OFFICE ---
        person_found = look_for_person_in_room_and_go_to_them('office', 'entry', hardcoded_pose=None)
        if person_found:
            ask_person_for_task_and_try_to_execute_it()
            person_found = None

        # --- KITCHEN ---
        kitchen_pose = PoseStamped(position=[5.9, 0.27, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
        person = look_for_person_in_room_and_go_to_them('kitchen', 'exit', hardcoded_pose=kitchen_pose)
        if person_found:
            ask_person_for_task_and_try_to_execute_it()
            person_found = None

        # --- LIVING_ROOM ---
        living_room_pose = PoseStamped(position=[7.72, 2.63, 0.0], orientation=[0.0, 0.0, 0.70, 0.70])
        person = look_for_person_in_room_and_go_to_them('living_room', 'entry', hardcoded_pose=living_room_pose)
        if person_found:
            ask_person_for_task_and_try_to_execute_it()
            person_found = None

        # --- HALLWAY ---
        hallway_pose = PoseStamped(position=[3.8, 2.7, 0.0], orientation=[0.0, 1.0, 0.0, 0.0])
        person = look_for_person_in_room_and_go_to_them('hallway', 'entry', hardcoded_pose=hallway_pose)
        if person_found:
            ask_person_for_task_and_try_to_execute_it()
            person_found = None

        # --- BEDROOM ---
        bedroom_pose = PoseStamped(position=[2.55, 4.3, 0.0], orientation=[0.0, 0.0, 0.70, 0.70])
        person = look_for_person_in_room_and_go_to_them('hallway', 'entry', hardcoded_pose=bedroom_pose)
        if person_found:
            ask_person_for_task_and_try_to_execute_it()
            person_found = None
        # todo: do something else or exit the arena


# --- main control ---
# TODO: test on real robot
def gpsr():
    with real_robot:
        # try:
        #     # image_switch.pub_now(ImageEnum.HI.value)  # hi im toya
        #     sing_my_angel_of_music("Push down my Hand, when you are Ready.")
        #     # image_switch.pub_now(ImageEnum.PUSHBUTTONS.value)
        #     plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        #     plan.perform()
        # except SensorMonitoringCondition:
        #     sing_my_angel_of_music("Starting GPSR.")

        plan_list = utils.get_plans(high_level_plans)
        sound_pub = SoundRequestPublisher()
        #sound_pub.publish_sound_request()

        sing_my_angel_of_music("Going to the instruction point")
        #navi.go_to_pose(PoseStamped([2.99, 2.0, 0], [0, 0, 0, 1]))  # in door
        #navi.go_to_room_entry_or_exit('office', 'exit')
        navi.go_to_pose(instruction_point)
        # look at a person when listening to command

        # high_level_plans.prepare_for_commands()
        instruction_list = []
        for i in range(10):  # test
            # TODO add this to plan.
            #    DetectAction(technique='human').resolve().perform()
            #    giskardpy.move_head_to_human()
            #    giskardpy.cancel_all_goals() OR giskardpy.cancel_all_called_goals() to stop
            # listen to instructions
            perception_interface.looking_for_human()
            giskard.move_head_to_human()
            instruction_list = nlp.listen_to_commands()
            rospy.logwarn("[CRAM] instruction list: " + str(instruction_list))
            #giskard.cancel_all_called_goals()
            giskard.cancel_goal()

            # execute instructions
            # TODO iterate over list of instructions and do stuff
            while instruction_list:  # Test
                rospy.logwarn("[CRAM] in instruction loop")
                instruction = instruction_list.pop(0)
                rospy.loginfo(instruction)
                # do stuff
                # match instruction to plan
                utils.call_plan_by_name(plan_list, snakecase(instruction['intent']), instruction)
                # if plan was successful, remove it from the list
                #instruction_list.remove(instruction) # if it gets poped then removal is not needed
            instruction_list = []
            sing_my_angel_of_music("navigating to the instruction point")
            navi.go_to_pose(instruction_point)


# CHANGE WITH CARE THIS STUFF GETS ACTUALLY EXECUTED
def demo_plan(data):
    with real_robot:
        high_level_plans.transporting(data)
        print('--------------stahp----------------')
        return


# beginnings of EGPSR
def blub():
    with real_robot:
        #init_robokudo()
        #nav = NavigateAction(target_locations=[PoseStamped([2.99, 2.0, 0], [0, 0, 0, 1])])
        #nav.resolve().perform()
        #TalkingMotion("Hello my name is Toya.").perform()
        #result = DetectAction(technique='human',).resolve().perform()
        #result = ask_robokudo_for_waving_humans()
        #print(result)
        #rospy.sleep(1)
        #result = ask_robokudo_for_humans()
        #print(result)
        #rospy.sleep(1)
        #result = ask_robokudo_for_all_objects()
        #print(result)
        #rospy.sleep(1)
        #result = ask_robokudo_for_object("cup")
        #print(result)
        #rospy.sleep(1)
        #HeadFollowMotion(state="start").perform()
        #rospy.sleep(3)
        #DetectAction(technique='human', state="stop").resolve().perform()
        MoveTorsoAction([0.1]).resolve().perform()
        rospy.sleep(2)
        MoveTorsoAction([0.0]).resolve().perform()

#blub()
#setup()
#fake_pose_2 = Pose([2.88, 0.3, 0])
#pub_fake_pose(fake_pose_2)
#gpsr()
#demo_plan(data2)
#setup_demo.gripper.pub_now('open')


# marker publisher notes
# from pycram.ros_utils.viz_marker_publisher import ManualMarkerPublisher
# rviz = ManualMarkerPublisher()
# rviz.publish(PoseStamped(position=[2.55, 4.3, 0.0], orientation=[0.0, 0.0, 0.70, 0.70]))
