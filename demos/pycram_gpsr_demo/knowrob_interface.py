from demos.pycram_gpsr_demo import setup_demo
from pycram.knowledge.knowrob_knowledge import KnowrobKnowledge
import rospy
import demos.pycram_gpsr_demo.utils as utils
import tf

from pycram.pose import Pose

# available rooms iri types
kitchen = 'http://www.ease-crc.org/ont/SOMA.owl#Kitchen'
living_room = 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom'
arena = 'http://www.ease-crc.org/ont/SUTURO.owl#Arena'


def init_knowrob():
    retry = 7
    while (not setup_demo.kb.is_connected) and retry > 0:
        rospy.loginfo(f"[CRAM-KNOW] Waiting for knowrob connection... {retry} retries left.")
        setup_demo.kb.connect()
        rospy.sleep(1)
        retry -= 1
    rospy.loginfo("[CRAM-KNOW] Connected.")


def get_obj_instance_of_type(type_iri):
    # TODO test - works
    # returns the instance of smth given the type iri. e.g. 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom'
    tmp = setup_demo.kb.prolog_client.once(f"has_type(Instance, '{type_iri}').")
    if tmp is []:
        rospy.logwarn(f"[KnowRob] no room instance with iri {type_iri} found.")
        return None
    else:
        tmp = tmp.get('Instance')
        rospy.loginfo(f"[KnowRob] room instance {tmp} of type {type_iri} found")
        return tmp


def get_room_entry_pose(Room):
    result = setup_demo.kb.prolog_client.once(f"entry_pose('{Room}', PoseStamped).")
    pose = utils.kpose_to_pose_stamped(result)
    return pose


# TODO WIP
def get_navigation_pose_for_all_tables_in_room(room_iri, obj_name='p_table'):
    knowrob_poses_list = setup_demo.kb.prolog_client.all_solutions(f"has_type(Obj, soma:'Table'), "
                                                                   f"has_type(Room, '{room_iri}'), "
                                                                   f"is_inside_of(Obj, Room), "
                                                                   f"furniture_rel_pose(Obj, 'perceive', Pose),"
                                                                   f"has_robocup_name(Obj, '{obj_name}').")
    poses_list = []
    for p in knowrob_poses_list:
        for item in p.get('Pose'):
            pose = utils.lpose_to_pose_stamped(item)
            # transform into map frame
            pose = setup_demo.tf_listener.transformPose("map", pose)
            # item[p.get('Name')] = pose # TODO make this a class so that you have KnowRobName, Pos1, Pos2, and other stuff
            poses_list.append(pose)
    return poses_list


def test_queries():
    setup_demo.kb.prolog_client.once("findall(Room, has_type(Room, soma:'Room'), RoomList).")
    setup_demo.kb.prolog_client.once("member(X,[1,2,3]).")
    setup_demo.kb.prolog_client.all_solutions("member(X,[1,2,3]).")
    setup_demo.kb.prolog_client.all_solutions("entry_pose(Rooms, PoseStamped).")
    setup_demo.kb.prolog_client.all_solutions("middle(Rooms, PoseStamped).")
    setup_demo.kb.prolog_client.once("entry_pose('kitchen', PoseStamped).")  # this works!
    setup_demo.kb.prolog_client.once("entry_pose('kitchen', [Frame, Pose, Quaternion]).")  # this is better
    setup_demo.kb.prolog_client.all_solutions("grasp_pose(ObjectType, Pose).")  # returns bowl = above
    setup_demo.kb.prolog_client.all_solutions("has_value(Objname, Property, Value).")
    setup_demo.kb.prolog_client.all_solutions("predefined_origin_location(Class, OriginLocation).")
    setup_demo.kb.prolog_client.all_solutions("is_inside_of(Obj, Room).")
    # iris ändern sich bei jedem launch
    setup_demo.kb.prolog_client.all_solutions(
        "tf:tf_get_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_WDOVGYLZ', ['map', Pos, Rot]).")
    setup_demo.kb.prolog_client.all_solutions(
        "tf:tf_get_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_WDOVGYLZ', ['map', Pos, Rot]).")
    setup_demo.kb.prolog_client.all_solutions(
        "has_type(Table, 'http://www.ease-crc.org/ont/SOMA.owl#DesignedHandle'), is_inside_of(Table,Room), has_type(Room, suturo:'LivingRoom'), object_rel_pose(Table, 'perceive', Pose).")
    # check for navigation pose to furniture for e.g. searching
    setup_demo.kb.prolog_client.all_solutions(
        "has_type(Obj, 'http://www.ease-crc.org/ont/SOMA.owl#DesignedFurniture').")  # returns the instances of obj currently present
    setup_demo.kb.prolog_client.all_solutions(
        "triple(Obj, P, 'http://www.ease-crc.org/ont/SOMA.owl#DesignedFurniture').")  # returns the class names
    # get all obj of type table and their robocup names
    setup_demo.kb.prolog_client.all_solutions(
        "has_type(Obj, soma:'Table'), triple(Obj, suturo:'hasRobocupName', Result).")
    # all semantic map items
    setup_demo.kb.prolog_client.all_solutions("has_urdf_name(Furniture, UrdfLink).")
    # all robocup item names
    setup_demo.kb.prolog_client.all_solutions("has_robocup_name(Furniture, RobocupName).")
    # gets all shelf layers
    setup_demo.kb.prolog_client.all_solutions("has_type(Obj, suturo:'ShelfLayer').")

    # drop databases
    # setup_demo.kb.prolog_client.all_solutions("drop_graph(user), tf_mem_clear, mng_drop(roslog, tf).")
    # setup_demo.kb.prolog_client.all_solutions(f"reset_user_data.")

# debugging:
# importlib.reload(gpsr)
# from pycram.ros.viz_marker_publisher import ManualMarkerPublisher
# marker = ManualMarkerPublisher()
# marker.publish(Pose.from_pose_stamped(human_p))



