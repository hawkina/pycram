from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.utilities.robocup_utils import StartSignalWaiter, TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher
from pycram.external_interfaces.navigate import PoseNavigator
from std_msgs.msg import String
import demos.pycram_gpsr_demo.setup_demo as setup_demo
from pycram.utilities.robocup_utils import StartSignalWaiter

# these are all the high level plans, to which we map the NLP output.
# they should either connect to low level plans or be filled with data from knowledge


# navigate the robot to LOCATION
def moving_to(param_json):
    # NavigateAction([pose1]).resolve().perform()
    setup_demo.tts.pub_now("in Moving-to plan.")
    rospy.loginfo("Moving To." + str(param_json))


# also finding + searching
def looking_for(param_json):
    setup_demo.tts.pub_now("in looking for plan")
    rospy.loginfo("Looking For: " + str(param_json))


def picking(param_json):
    setup_demo.tts.pub_now("in picking up plan")
    rospy.loginfo("Picking: " + str(param_json))


def placing(param_json):
    setup_demo.tts.pub_now("in placing plan")
    rospy.loginfo("Place: " + str(param_json))


def fetching(param_json):
    setup_demo.tts.pub_now("in fetching plan")
    rospy.loginfo("fetching: " + str(param_json))


def cleaning(param_json):
    setup_demo.tts.pub_now("in cleaning plan")
    rospy.loginfo("cleaning: " + str(param_json))


def transporting(param_json):
    setup_demo.tts.pub_now("in transporting plan")
    rospy.loginfo("transporting: " + str(param_json))


def arranging(param_json):
    setup_demo.tts.pub_now("in arranging plan")
    rospy.loginfo("arranging: " + str(param_json))


# count obj or person
def counting(param_json):
    setup_demo.tts.pub_now("in counting plan")
    rospy.loginfo("count: " + str(param_json))


def guiding(param_json):
    setup_demo.tts.pub_now("in guiding plan")
    rospy.loginfo("guide: " + str(param_json))


# --- utils plans ---
def track_human():#
    setup_demo.tts.pub_now("in track human plan")
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
