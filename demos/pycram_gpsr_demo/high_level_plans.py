from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.utilities.robocup_utils import StartSignalWaiter, TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher
from pycram.external_interfaces.navigate import PoseNavigator
from std_msgs.msg import String

# these are all the high level plans, to which we map the NLP output.
# they should either connect to low level plans or be filled with data from knowledge



# navigate the robot to LOCATION
def moving_to(param_json):
    # NavigateAction([pose1]).resolve().perform()
    rospy.loginfo("Moving To." + str(param_json))


# also finding + searching
def looking_for(param_json):
    rospy.loginfo("Looking For: " + str(param_json))


def picking(param_json):
    rospy.loginfo("Picking: " + str(param_json))


def placing(param_json):
    rospy.loginfo("Place: " + str(param_json))


def fetching(param_json):
    rospy.loginfo("fetching: " + str(param_json))


def cleaning(param_json):
    rospy.loginfo("cleaning: " + str(param_json))


def transporting(param_json):
    rospy.loginfo("transporting: " + str(param_json))


def arranging(param_json):
    rospy.loginfo("arranging: " + str(param_json))


# count obj or person
def counting(param_json):
    rospy.loginfo("count: " + str(param_json))


def guiding(param_json):
    rospy.loginfo("guide: " + str(param_json))


# --- utils plans ---
def track_human():
    # look for a human
    DetectAction(technique='human').resolve().perform()
    # look at guest and introduce
    HeadFollowAction('start').resolve().perform()



