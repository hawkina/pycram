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

# these are all the low level plans  which are used by the high level plans

def go_to_room_entry():
    # go to the room entry
    pass


def go_to_room_middle():
    # go to the room middle
    pass


def go_to_room_exit():
    # go to the room exit
    pass


def go_to_furniture():
    # go to the furniture
    pass


def go_to_furniture_in_room():
    # go to the furniture in the room
    pass


def go_to_person():
    # go to the person
    pass


def go_to_person_at_furniture():
    # go to the person at the furniture
    pass


def go_to_person_in_room():
    # go to the person in the room
    pass


def go_to_person_at_furniture_in_room():
    # go to the person at the furniture in the room
    pass