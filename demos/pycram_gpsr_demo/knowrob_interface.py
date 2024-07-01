from pycram.knowledge.knowrob_knowledge import KnowrobKnowledge
import rospy
import demos.pycram_gpsr_demo.utils as utils

kb = KnowrobKnowledge()


def init_knowrob():
    retry = 7
    while (not kb.is_connected) and retry > 0:
        rospy.loginfo(f"[CRAM-KNOW] Waiting for knowrob connection... {retry} retries left." )
        kb.connect()
        rospy.sleep(1)
        retry -= 1
    rospy.loginfo("[CRAM-KNOW] Connected.")


def get_room_entry_pose(Room):
    result = kb.prolog_client.once(f"entry_pose('{Room}', PoseStamped).")
    pose = utils.kpose_to_pose_stamped(result)
    return pose


def test_queries():
    kb.prolog_client.once("findall(Room, has_type(Room, soma:'Room'), RoomList).")
    kb.prolog_client.once("member(X,[1,2,3]).")
    kb.prolog_client.all_solutions("member(X,[1,2,3]).")
    kb.prolog_client.all_solutions("entry_pose(Rooms, PoseStamped).")
    kb.prolog_client.all_solutions("middle(Rooms, PoseStamped).")
    kb.prolog_client.once("entry_pose('kitchen', PoseStamped).")  # this works!
    kb.prolog_client.once("entry_pose('kitchen', [Frame, Pose, Quaternion]).") # this is better