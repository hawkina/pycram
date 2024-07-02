from demos.pycram_gpsr_demo import setup_demo
from pycram.knowledge.knowrob_knowledge import KnowrobKnowledge
import rospy
import demos.pycram_gpsr_demo.utils as utils


def init_knowrob():
    retry = 7
    while (not setup_demo.kb.is_connected) and retry > 0:
        rospy.loginfo(f"[CRAM-KNOW] Waiting for knowrob connection... {retry} retries left." )
        setup_demo.kb.connect()
        rospy.sleep(1)
        retry -= 1
    rospy.loginfo("[CRAM-KNOW] Connected.")


def get_room_entry_pose(Room):
    result = setup_demo.kb.prolog_client.once(f"entry_pose('{Room}', PoseStamped).")
    pose = utils.kpose_to_pose_stamped(result)
    return pose


def test_queries():
    setup_demo.kb.prolog_client.once("findall(Room, has_type(Room, soma:'Room'), RoomList).")
    setup_demo.kb.prolog_client.once("member(X,[1,2,3]).")
    setup_demo.kb.prolog_client.all_solutions("member(X,[1,2,3]).")
    setup_demo.kb.prolog_client.all_solutions("entry_pose(Rooms, PoseStamped).")
    setup_demo.kb.prolog_client.all_solutions("middle(Rooms, PoseStamped).")
    setup_demo.kb.prolog_client.once("entry_pose('kitchen', PoseStamped).")  # this works!
    setup_demo.kb.prolog_client.once("entry_pose('kitchen', [Frame, Pose, Quaternion]).") # this is better
    setup_demo.kb.prolog_client.all_solutions("grasp_pose(ObjectType, Pose).") # returns bowl = above
    setup_demo.kb.prolog_client.all_solutions("has_value(Objname, Property, Value).")
    setup_demo.kb.prolog_client.all_solutions("predefined_origin_location(Class, OriginLocation).")
    setup_demo.kb.prolog_client.all_solutions("is_inside_of(Obj, Room).")
    setup_demo.kb.prolog_client.all_solutions("tf:tf_get_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_WDOVGYLZ', ['map', Pos, Rot]).")
    setup_demo.kb.prolog_client.all_solutions("tf:tf_get_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_WDOVGYLZ', ['map', Pos, Rot]).")

