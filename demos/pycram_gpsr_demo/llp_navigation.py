from pycram.designators.action_designator import *
import demos.pycram_gpsr_demo.setup_demo as setup_demo
from demos.pycram_gpsr_demo import knowrob_interface as knowrob
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music
import demos.pycram_gpsr_demo.utils as utils


# these are all the low level plans  which are used by the high level plans

def go_to_room_entry_or_exit(room_name, entry_exit):  # test
    # go to the room entry point
    pose = knowrob.get_room_pose(room=room_name, entry_or_exit=entry_exit)
    rospy.loginfo(utils.PC.GREEN + f"[Go-To-Pose] Going to: {pose}.")
    if pose is None:
        rospy.loginfo(utils.PC.GREEN + "[CRAM] KnowRob result was empty.")
        sing_my_angel_of_music("I am sorry. I don't know where " + room_name + "is.")
        return None  # abort mission
    else:
        if setup_demo.with_real_robot:
            sing_my_angel_of_music("Going to the " + room_name)
            rospy.loginfo(utils.PC.GREEN + f"[CRAM] Going to: {pose}.")
            setup_demo.move.pub_now(pose)
            return pose


def go_to_room_middle(room_name):  # test
    # go to the room middle
    pose = knowrob.get_room_middle_pose(room=room_name)
    rospy.loginfo(utils.PC.GREEN + f"[Go-To-Pose] Going to: {pose}.")
    if pose is None:
        rospy.loginfo(utils.PC.GREEN +"[CRAM] KnowRob result was empty.")
        sing_my_angel_of_music("I am sorry. I don't know where " + room_name + "is.")
        return None  # abort mission
    else:
        if setup_demo.with_real_robot:
            sing_my_angel_of_music(utils.PC.GREEN + "Going to the " + room_name)
            rospy.loginfo(utils.PC.GREEN + f"[CRAM] Going to: {pose}.")
            setup_demo.move.pub_now(pose)
            return pose


def go_to_pose(pose):
    rospy.loginfo(utils.PC.GREEN + f"[Go-To-Pose] Going to: {pose}.")
    # go to the pose
    if pose is None or pose is []:
        rospy.logwarn(utils.PC.GREEN + "[CRAM] Empty pose received  :(.")
        sing_my_angel_of_music("I am sorry. I don't know where that is.")
        return None  # abort mission
    else:
        if setup_demo.with_real_robot:
            rospy.loginfo(utils.PC.GREEN + f"[CRAM] Going to: {pose}.")
            setup_demo.move.pub_now(pose)
            return pose


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
