import rospy
from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.utilities.robocup_utils import StartSignalWaiter, TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher
from pycram.external_interfaces.navigate import PoseNavigator
from std_msgs.msg import String
import demos.pycram_gpsr_demo.setup_demo as setup_demo
from pycram.utilities.robocup_utils import StartSignalWaiter
from demos.pycram_gpsr_demo.knowrob_interface import KnowrobKnowledge
from demos.pycram_gpsr_demo import perception_interface
import demos.pycram_gpsr_demo.utils as utils
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music

from stringcase import snakecase

# these are all the high level plans, to which we map the NLP output.
# they should either connect to low level plans or be filled with data from knowledge


# navigate the robot to LOCATION
def moving_to(param_json):  # WIP Can also be funriture, or a person
    # ToDo: test
    global with_real_robot
    rospy.loginfo("[CRAM] MovingTo plan." + str(param_json))
    # get room pose from knowrob
    room_name = snakecase(param_json.get('DestinationRoom').get('value').lower())
    k_pose = setup_demo.kb.prolog_client.once(f"entry_pose('{room_name}', [Frame, Pose, Quaternion]).")

    if k_pose == [] or k_pose is None:
        rospy.loginfo("[CRAM] KnowRob result was empty.")
        sing_my_angel_of_music("I am sorry. I don't know where " + room_name + "is.")  # asking for help would be fun
        return  # abort mission
    # continue
    pose = utils.kpose_to_pose_stamped(k_pose)
    rospy.loginfo(f"[CRAM] Going to {room_name} Pose : " + str(pose))
    sing_my_angel_of_music("Going to the " + room_name)
    if setup_demo.with_real_robot:
        setup_demo.move.pub_now(pose)
    sing_my_angel_of_music("[CRAM] done")
    # ToDo: does it always make sense to use enter pose?


# also finding + searching
def looking_for(param_json): # WIP
    sing_my_angel_of_music("in looking for plan")
    rospy.loginfo("Looking For: " + str(param_json))
    physical_place, physical_artifact = None, None
    # step 0: go to the requested room - if it was mentioned explicitly OR
    if param_json.get('Location') and param_json.get('Location').get('entity') == 'PhysicalPlace':
        physical_place = snakecase(param_json.get('Location').get('value'))
    if param_json.get('Destination') and param_json.get('Destination').get('entity') == 'PhysicalArtifact':
        physical_artifact = param_json.get('Destination').get('value')
    # make msg for perception
    rk_msg = perception_interface.make_robokudo_obj_msg(param_json.get('Item'))
    # get navigation pose from knowrob depending on what info is known


    # step 1: go to the furniture/surface item that got mentioned and look on it for the specified obj


def picking(param_json):
    sing_my_angel_of_music("in picking up plan")
    rospy.loginfo("Picking: " + str(param_json))


def placing(param_json):
    sing_my_angel_of_music("in placing plan")
    rospy.loginfo("Place: " + str(param_json))


def fetching(param_json):
    # go to a target location, pick up object, bring it back
    sing_my_angel_of_music("in fetching plan")
    rospy.loginfo("fetching: " + str(param_json))
    # BeneficiaryRole: target
    if param_json.get('BeneficiaryRole').get('entity') == 'NaturalPerson':
        # Goal is natural pearson. Means we need to HRI
        # if person is 'me', save current pose and person name
        if param_json.get('BeneficiaryRole').get('value') == 'me': # or part of a list of names?
            # save current pose
            me_pose = setup_demo.tf_listener.lookupTransform(target_frame='map', source_frame='base_link', time=rospy.get_time())
            # alternatively, try to memorize the human person from perception? ... uff
            person = 'you'
    # go to the destination to pick the obj
    # --- Step 1 ---
    # get the pose of 'PhysicalPlace' = 'Room' (if available) CHANGE (Potentially)
    if param_json.get('Destination').get('entity') == 'PhysicalPlace':
        physical_place = param_json.get('Destination').get('value')  # CHANGE this actually should be Origin

    # perceive obj
    # TODO WIP




def cleaning(param_json):
    sing_my_angel_of_music("in cleaning plan")
    rospy.loginfo("cleaning: " + str(param_json))


def transporting(param_json):
    sing_my_angel_of_music("in transporting plan")
    rospy.loginfo("transporting: " + str(param_json))
    # from BeneficiaryRole


def arranging(param_json):
    sing_my_angel_of_music("in arranging plan")
    rospy.loginfo("arranging: " + str(param_json))


# count obj or person
def counting(param_json):
    sing_my_angel_of_music("in counting plan")
    rospy.loginfo("count: " + str(param_json))


def guiding(param_json):
    sing_my_angel_of_music("in guiding plan")
    rospy.loginfo("guide: " + str(param_json))


# --- utils plans ---
def track_human():#
    sing_my_angel_of_music("in track human plan")
    # look for a human
    DetectAction(technique='human').resolve().perform()
    # look at guest and introduce
    HeadFollowAction('start').resolve().perform()


def prepare_for_commands():
    global move, instruction_point
    # wait for start signal (laser scan)
    # Create an instance of the StartSignalWaiter
    start_signal_waiter = StartSignalWaiter()
    # Wait for the start signal
    start_signal_waiter.wait_for_startsignal()
    # Once the start signal is received, continue with the rest of the script
    rospy.loginfo("Start signal received, now proceeding with tasks.")

    # TODO this should be looped for multiple tasks
    # go to initial point
    move.query_pose_nav(instruction_point)
