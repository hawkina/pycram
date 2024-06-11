from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.utilities.robocup_utils import StartSignalWaiter, TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher
from pycram.external_interfaces.navigate import PoseNavigator
from std_msgs.msg import String

# these are all the high level plans, to which we map the NLP output.
# they should either connect to low level plans or be filled with data from knowledge



# navigate the robot to LOCATION
def moving_to(loc):
    # NavigateAction([pose1]).resolve().perform()
    rospy.loginfo("Moving To:  " + str(loc))


# also finding + searching
def looking_for(obj, loc):
    rospy.loginfo("Looking For: " + str(obj) + " at " + str(loc))


def picking(obj, from_location):
    rospy.loginfo("Picking: " + str(obj) + " from " + str(from_location))


def placing(obj, to_location):
    rospy.loginfo("Place: " + str(obj) + "at" + str(to_location))


def fetching(obj):
    rospy.loginfo("fetching: " + str(obj))


def cleaning(obj):
    rospy.loginfo("cleaning: " + str(obj))


def transporting(obj, from_loc, to_loc, from_person, to_person):
    rospy.loginfo("transporting: " + str(obj) + " from " + str(from_loc) + " to " + str(to_loc) + " from " + str(from_person) + " to " + str(to_person))


def arranging(obj, from_loc, to_loc):
    rospy.loginfo("arranging: " + str(obj) + " from " + str(from_loc) + " to " + str(to_loc))


# count obj or person
def counting(obj_or_pers):
    rospy.loginfo("count: " + str(obj_or_pers))


def guiding(person, from_loc, to_loc):
    rospy.loginfo("guide: " + str(person) + " from " + str(from_loc) + " to " + str(to_loc))


# --- utils plans ---
def track_human():
    # look for a human
    DetectAction(technique='human').resolve().perform()
    # look at guest and introduce
    HeadFollowAction('start').resolve().perform()



