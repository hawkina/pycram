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
rooms = {'kitchen': kitchen, 'living_room': living_room, 'arena': arena}


def init_knowrob(): # works
    retry = 7
    while (not setup_demo.kb.is_connected) and retry > 0:
        rospy.loginfo(f"[CRAM-KNOW] Waiting for knowrob connection... {retry} retries left.")
        setup_demo.kb.connect()
        rospy.sleep(1)
        retry -= 1
    rospy.loginfo("[CRAM-KNOW] Connected.")


def get_obj_instance_of_type(type_iri): # test
    # returns the instance of smth given the type iri. e.g. 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom'
    tmp = setup_demo.kb.prolog_client.once(f"has_type(Instance, '{type_iri}').")
    if tmp is []:
        rospy.logwarn(f"[KnowRob] no room instance with iri {type_iri} found.")
        return None
    else:
        tmp = tmp.get('Instance')
        rospy.loginfo(f"[KnowRob] room instance {tmp} of type {type_iri} found")
        return tmp


# room = 'kitchen' but iri will get matched from knowrob
def get_room_entry_pose_class(room):  # works
    if rooms.get(room):
        result = setup_demo.kb.prolog_client.once(f"has_type(Room, '{rooms.get(room)}'), entry_pose(Room, PoseStamped).")
        if result is None or result == []:
            rospy.logerr(f"[KnowRob] No entry pose for {room} found. :(")
            return None
        pose = utils.lpose_to_pose_stamped(result)
        return pose
    else:
        rospy.logerr(f"[KnowRob] No Room with name {room} found. :(")
        return None


# room = 'kitchen'
# entry_or_exit = 'entry' | 'exit' > those are two different knowrob queries
def get_room_pose(room, entry_or_exit='entry'):  # Works
    if rooms.get(room):
        result = setup_demo.kb.prolog_client.once(f"{entry_or_exit}_pose('{room}', PoseStamped).")
        if result is None or result == []:
            rospy.logerr(f"[KnowRob] No entry pose for {room} found. :(")
            return None
        pose = utils.lpose_to_pose_stamped(result.get('PoseStamped'))
        return pose
    else:
        rospy.logerr(f"[KnowRob] No Room with name {room} found. :(")
        return None


# room = 'kitchen'
# obj_name = 'p_table' | robocup Name
# result: list of dicts. accessible like: res[0].get('Item').get('room')
# NOTE the result can contain multiple poses if the table is large enough to require this
def get_navigation_poses_for_all_tables_in_room(room='arena', obj_name=None):  # works
    if rooms.get(room):
        room_iri = rooms.get(room)
    else:
        rospy.logerr(f"[KnowRob] unknown room with name {room}.")
        return None
    if obj_name is not None:
        obj_name = "'%s'" % obj_name
    else:
        obj_name = 'Name'
    knowrob_poses_list = setup_demo.kb.prolog_client.all_solutions(f"has_type(Obj, soma:'Table'), "
                                                                   f"has_type(Room, '{room_iri}'), "
                                                                   f"is_inside_of(Obj, Room), "
                                                                   f"furniture_rel_pose(Obj, 'perceive', Pose),"
                                                                   f"has_robocup_name(Obj, {obj_name}).")
    poses_list = []
    if knowrob_poses_list:
        poses_list = utils.knowrob_poses_result_to_list_dict(knowrob_poses_list)
    else:
        rospy.logerr("[KnowRob] query returned empty :(")
    return poses_list


# for testing: pose_list = gpsr.get_nav_pose_for_furniture(furniture_name=f'p_table')
# for testing: gpsr.move.pub_now(pose_list[0])
# works: but might need more test-ing
def get_nav_poses_for_furniture_item(room='arena', furniture_class=None, furniture_name=f"Name"):  # works
    # unless another class is specified, ensure soma:'DesignedFurniture' is the default
    if furniture_class is not None:
        # ensure correct formatting
        if "'" not in furniture_class:
            furniture_class = f"{furniture_class.split(':', 1)[0]}:'{furniture_class.split(':', 1)[1]}'"
    else:
        furniture_class = f"soma:'DesignedFurniture'"
    # ensure room exists
    if rooms.get(room):
        room_iri = rooms.get(room)
    else:
        rospy.logerr(f"[KnowRob] unknown room with name {room}.")
        return None

    knowrob_poses_list = setup_demo.kb.prolog_client.all_solutions(f"has_type(Obj, {furniture_class}), "
                                                                   f"has_type(Room, '{room_iri}'), "
                                                                   f"is_inside_of(Obj, Room), "
                                                                   f"furniture_rel_pose(Obj, 'perceive', Pose),"
                                                                   f"has_robocup_name(Obj, {furniture_name}),"
                                                                   f"has_robocup_name(Obj, Name).")  # CHANGE find a prettier way?
    poses_list = []
    if knowrob_poses_list:
        poses_list = utils.knowrob_poses_result_to_list_dict(knowrob_poses_list)
    else:
        rospy.logerr("[KnowRob] query returned empty :(")
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
