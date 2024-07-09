from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.utilities.robocup_utils import StartSignalWaiter, TextToSpeechPublisher, ImageSwitchPublisher, \
    SoundRequestPublisher
from pycram.external_interfaces.navigate import PoseNavigator
from std_msgs.msg import String
import demos.pycram_gpsr_demo.setup_demo as setup_demo
from pycram.utilities.robocup_utils import StartSignalWaiter
from demos.pycram_gpsr_demo import perception_interface
from demos.pycram_gpsr_demo import knowrob_interface as knowrob
import demos.pycram_gpsr_demo.utils as utils
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music
from stringcase import snakecase


# these are all the low level plans  which are used by the high level plans

def go_to_room_entry_or_exit(room_name, entry_exit):  # test
    # go to the room entry point
    pose = knowrob.get_room_pose(room=room_name, entry_or_exit=entry_exit)
    if pose is None:
        rospy.loginfo("[CRAM] KnowRob result was empty.")
        sing_my_angel_of_music("I am sorry. I don't know where " + room_name + "is.")
        return None  # abort mission
    else:
        if setup_demo.with_real_robot:
            sing_my_angel_of_music("Going to the " + room_name)
            setup_demo.move.pub_now(pose)
            return pose


def go_to_room_middle(room_name): # test
    # go to the room middle
    pose = knowrob.get_room_middle_pose(room=room_name)
    if pose is None:
        rospy.loginfo("[CRAM] KnowRob result was empty.")
        sing_my_angel_of_music("I am sorry. I don't know where " + room_name + "is.")
        return None  # abort mission
    else:
        if setup_demo.with_real_robot:
            sing_my_angel_of_music("Going to the " + room_name)
            setup_demo.move.pub_now(pose)
            return pose


def go_to_pose(pose):
    # go to the pose
    if pose is None or pose is []:
        rospy.loginfo("[CRAM] Empty pose received  :(.")
        sing_my_angel_of_music("I am sorry. I don't know where that is.")
        return None  # abort mission
    else:
        if setup_demo.with_real_robot:
            setup_demo.move.pub_now(pose)
            return pose


# f_class = soma:'DesignedFurniture'
# WIP how to handle mutliple poses in this case? should this function handle them at all?
def go_to_furniture_in_room(room_name='arena', f_class=None, f_name=f"Name"):  #  WIP
    pass
#    # go to the room middle WIP: Ensure only one pose gets returned
#    # Note ensure furniture class/name exists -> done on knowrob side
#    pose = knowrob.get_nav_poses_for_furniture_item(room=room_name, furniture_iri=f_class, furniture_name=f_name)
#    if pose is None or pose is []:
#        # this is failure handling for the name not being found
#        rospy.loginfo(f"[CRAM] KnowRob result was empty. I will retry once without the furniture name: {f_name}.")
#        pose = knowrob.get_nav_poses_for_furniture_item(room=room_name, furniture_iri=f_class, furniture_name=f"Name")
#    if pose is None or pose is []:
#        sing_my_angel_of_music("I am sorry. I don't know where " + room_name + "is.")
#        return None  # abort mission
#    else:
#        if setup_demo.with_real_robot:
#            sing_my_angel_of_music("Going to the " + room_name)
#            setup_demo.move.pub_now(pose)
#            return pose



#def go_to_furniture_in_room():
#    # go to the furniture in the room
#    pass


def go_to_person():
    # go to the person
    pass


def go_to_person_at_furniture():
    # go to the person at the furniture
    # go to furniture and rotate while looking for a person?
    pass


def go_to_person_in_room(room_name='living_room'):
    # go to the person in the room
    go_to_room_middle(room_name)
    # TODO look for person
    pass


def go_to_person_at_furniture_in_room(room_name, f_class, f_name):
    # go to the person at the furniture in the room
    #go_to_furniture_in_room(room_name, f_class, f_name)
    # todo look for person
    pass
