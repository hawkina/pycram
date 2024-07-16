from stringcase import snakecase
from pycram.knowledge.knowrob_knowledge import KnowrobKnowledge
import rospy
from demos.pycram_gpsr_demo.perception_to_knowrob import perc_to_know
import demos.pycram_gpsr_demo.utils as utils

# available rooms iri types
kitchen = 'http://www.ease-crc.org/ont/SOMA.owl#Kitchen'
living_room = 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom'
arena = 'http://www.ease-crc.org/ont/SUTURO.owl#Arena'
dining_room = 'http://www.ease-crc.org/ont/SUTURO.owl#DiningRoom'
hallway = 'http://www.ease-crc.org/ont/SUTURO.owl#Hallway'
office = 'http://www.ease-crc.org/ont/SUTURO.owl#Office'
rooms = {'kitchen': kitchen, 'living_room': living_room, 'arena': arena, 'dining_room': dining_room,
         'hallway': hallway, 'office': office}
kb = KnowrobKnowledge()

#hopefully tmp
dishes = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupDishes'
snacks = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupSnacks'
fruits = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupFruits'
food = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupFood'
cleaning_supplies = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupCleaningSupplies'
toys = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupToys'
decorations = 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupDecorations'


def init_knowrob():  # works
    global kb
    retry = 7
    #    kb = KnowrobKnowledge()
    while (not kb.is_connected) and retry > 0:
        rospy.loginfo(f"[CRAM-KNOW] Waiting for knowrob connection... {retry} retries left.")
        kb.connect()
        rospy.sleep(1)
        retry -= 1
    kb.prolog_client.all_solutions(f"init_gpsr_2024.")
    rospy.loginfo("[CRAM-KNOW] Connected.")


def get_obj_instance_of_type(type_iri):  # test
    # returns the instance of smth given the type iri. e.g. 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom'
    tmp = kb.prolog_client.once(f"has_type(Instance, '{type_iri}').")
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
        result = kb.prolog_client.once(f"has_type(Room, '{rooms.get(room)}'), entry_pose(Room, PoseStamped).")
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
        result = kb.prolog_client.once(f"{entry_or_exit}_pose('{room}', PoseStamped).")
        if result is None or result == []:
            rospy.logerr(f"[KnowRob] No entry pose for {room} found. :(")
            return None
        pose = utils.lpose_to_pose_stamped(result.get('PoseStamped'))
        return pose
    else:
        rospy.logerr(f"[KnowRob] No Room with name {room} found. :(")
        return None


def get_room_middle_pose(room):  # Works
    if rooms.get(room):
        result = kb.prolog_client.once(f"middle('{room}', PoseStamped).")
        if result is None or result == []:
            rospy.logerr(f"[KnowRob] No entry pose for {room} found. :(")
            return None
        pose = utils.lpose_to_pose_stamped(result.get('PoseStamped'))
        return pose
    else:
        rospy.logerr(f"[KnowRob] No Room with name {room} found. :(")
        return None


# for testing: pose_list = gpsr.get_nav_pose_for_furniture(furniture_name=f'p_table')
# for testing: gpsr.move.pub_now(pose_list[0])
# furniture name = robocup name
# works: but might need more test-ing
def get_nav_poses_for_furniture_item(room='arena', furniture_iri=None, furniture_name=f"Name"):  # works
    rospy.logwarn(f"tf listener in knowrob{utils.tf_l}")
    # unless another class/iri is specified, ensure soma:'DesignedFurniture' is the default
    if furniture_iri is not None:
        # ensure correct formatting
        if "'" not in furniture_iri and 'http' not in furniture_iri:  # ensure class is formatted correctly
            furniture_iri = f"{furniture_iri.split(':', 1)[0]}:'{furniture_iri.split(':', 1)[1]}'"
        elif 'http' in furniture_iri:
            furniture_iri = "'" + furniture_iri + "'"
        else:
            furniture_iri = f"soma:'{furniture_iri}'"
        # check that the iri is actually a furniture item
        if kb.prolog_client.all_solutions(f"subclass_of({furniture_iri}, soma:'DesignedFurniture')."):
            pass
        else:
            rospy.logwarn(f"[KnowRob] unknown furniture class with name {furniture_iri}. "
                          f"looking for DesignedFurniture instead")
            furniture_iri = f"soma:'DesignedFurniture'"
    else:
        furniture_iri = f"soma:'DesignedFurniture'"
    # ensure room exists
    if rooms.get(snakecase(room)):
        room_iri = rooms.get(snakecase(room))
    else:
        rospy.logerr(f"[KnowRob] unknown room with name {room}.")
        return None
    # ensure correct formatting of name
    if "'" not in furniture_name and furniture_name != "Name":
        furniture_name = f"'{furniture_name}'"

    print(room, room_iri, furniture_iri, furniture_name)
    knowrob_poses_list = kb.prolog_client.all_solutions(f"has_type(Room, '{room_iri}'), "
                                                        f"(what_object_transitive({furniture_name}, Obj); "
                                                        f"has_robocup_name(Obj, {furniture_name})),"
                                                        f"has_type(Obj, {furniture_iri}), "
                                                        # f"instance_of(Inst, Obj),"
                                                        f"Inst = Obj,"
                                                        f"is_inside_of(Inst, Room),"
                                                        f"furniture_rel_pose(Inst, 'perceive', Pose).")  # CHANGE find a prettier way?
    poses_list = []
    if knowrob_poses_list:
        poses_list = utils.knowrob_poses_result_to_list_dict(knowrob_poses_list)
    else:
        rospy.logerr("[KnowRob] query returned empty :(")
    return poses_list


# mostly used to check if a furniture object exists based on name from nlp
def check_existence_of_instance(nlp_name):
    # check if an instance of the object exists
    # returns the instance name
    # nlp_name = 'table'
    tmp = kb.prolog_client.all_solutions(f"(what_object_transitive('{nlp_name}', Obj), instance_of(Inst, Obj)); "
                                         f"(has_robocup_name(Obj, '{nlp_name}')).")
    if tmp is None or tmp == []:
        rospy.logwarn(f"[KnowRob] no object instance with name {nlp_name} found.")
        return None
    else:
        rospy.loginfo(f"[KnowRob] object instance {tmp} of type {nlp_name} found")
        return tmp


def check_existence_of_class(nlp_name):
    # check if an instance of the object exists
    # returns the instance name
    # nlp_name = 'table'
    tmp = kb.prolog_client.all_solutions(f"what_object_transitive('{nlp_name}', Class).")
    if tmp is None or tmp == []:
        rospy.logwarn(f"[KnowRob] no object class with name {nlp_name} found.")
        return None
    else:
        rospy.loginfo(f"[KnowRob] object class {tmp} of type {nlp_name} found")
        return tmp


# get iri from objects.py mapping
def get_predefined_source_item_location(items_iri):
    if "'" not in items_iri:
        items_iri = "'" + items_iri + "'"
    knowrob_poses_list = kb.prolog_client.all_solutions(f"predefined_origin_location({items_iri}, Furniture), "
                                                        f"furniture_rel_pose(Furniture, 'perceive', Pose).")
    poses_list = []
    if knowrob_poses_list:
        poses_list = utils.knowrob_poses_result_to_list_dict(knowrob_poses_list)
    else:
        rospy.logerr("[KnowRob] query returned empty :(")
    return poses_list


def get_predefined_destination_item_location(items_iri):
    if "'" not in items_iri:
        items_iri = "'" + items_iri + "'"
    knowrob_poses_list = kb.prolog_client.all_solutions(f"predefined_destination_location({items_iri}, Furniture), "
                                                        f"furniture_rel_pose(Furniture, 'perceive', Pose).")
    poses_list = []
    if knowrob_poses_list:
        poses_list = utils.knowrob_poses_result_to_list_dict(knowrob_poses_list)
    else:
        rospy.logerr("[KnowRob] query returned empty :(")
    return poses_list


def check_existence_based_on_class(class_iri):
    # check if an instance of the object exists
    # returns the instance name
    # nlp_name = 'table'
    tmp = kb.prolog_client.all_solutions(f"what_object_transitive(Name, {class_iri}).")
    if tmp is None or tmp == []:
        rospy.logwarn(f"[KnowRob] no object class with name {class_iri} found.")
        return None
    else:
        rospy.loginfo(f"[KnowRob] object class {tmp} of type {class_iri} found")
        return tmp


# Test -----------------------------------------------------------------------------------------
def test_predefined_locations():
    source_list = []
    destination_list = []
    for item in utils.obj_dict.values():
        get_dest = get_predefined_destination_item_location(item)
        if get_dest:
            destination_list.append(get_dest)

        get_source = get_predefined_source_item_location(item)
        if get_source:
            source_list.append(get_source)
    return {'source': source_list, 'destination': destination_list}


perception_list = [
    "Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball",
    "Baseball", "Mustardbottle", "Jellochocolatepuddingbox", "Wineglass",
    "Orange", "Coffeepack", "Softball", "Metalplate", "Pringleschipscan",
    "Strawberry", "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug",
    "Abrasivesponge", "Jellobox", "Dishwashertab", "Knife", "Cerealbox",
    "Metalbowl", "Sugarbox", "Coffeecan", "Milkpackja", "Apple", "Tomatosoupcan",
    "Tunafishcan", "Gelatinebox", "Pear", "Lemon", "Banana", "Pottedmeatcan",
    "Peach", "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen",
    "Largemarker", "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes",
    "Cup_small", "screwdriver", "clamp", "hammer", "wooden_block", "Cornybox"
]

furniture_list = ['hallway cabinet', 'entrance', 'desk', 'shelf', 'coathanger',
                  'exit', 'TV table', 'lounge chair', 'lamp', 'couch', 'coffee table',
                  'trashcan', 'kitchen cabinet', 'dinner table', 'dishwasher', 'kitchen counter']


def check_existence_of_perception():
    results = []
    items_iri = None
    for item in perc_to_know:
        items_iri = perc_to_know.get(item)
        if "'" not in items_iri:
            items_iri = "'" + items_iri + "'"
        tmp = check_existence_based_on_class(items_iri)
        if tmp:
            results.append([item, tmp])
        else:
            results.append([item, None])
    return results


# check_existence_of_furniture(furniture_list)
def check_existence_of_furniture(tmp_list):
    results = []
    tmp = None
    for item in tmp_list:
        tmp = check_existence_of_instance(snakecase(item))
        if tmp:
            results.append([item, tmp])
        else:
            results.append([item, None])
        tmp = None
    return results


def test_queries():
    kb.prolog_client.once("findall(Room, has_type(Room, soma:'Room'), RoomList).")
    kb.prolog_client.once("member(X,[1,2,3]).")
    kb.prolog_client.all_solutions("member(X,[1,2,3]).")
    kb.prolog_client.all_solutions("entry_pose(Rooms, PoseStamped).")
    kb.prolog_client.all_solutions("middle(Rooms, PoseStamped).")
    kb.prolog_client.once("entry_pose('kitchen', PoseStamped).")  # this works!
    kb.prolog_client.once("entry_pose('kitchen', [Frame, Pose, Quaternion]).")  # this is better
    kb.prolog_client.all_solutions("grasp_pose(ObjectType, Pose).")  # returns bowl = above
    kb.prolog_client.all_solutions("has_value(Objname, Property, Value).")
    kb.prolog_client.all_solutions("predefined_origin_location(Class, OriginLocation).")
    kb.prolog_client.all_solutions("is_inside_of(Obj, Room).")
    # iris ändern sich bei jedem launch
    kb.prolog_client.all_solutions(
        "tf:tf_get_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_WDOVGYLZ', ['map', Pos, Rot]).")
    kb.prolog_client.all_solutions(
        "tf:tf_get_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_WDOVGYLZ', ['map', Pos, Rot]).")
    kb.prolog_client.all_solutions(
        "has_type(Table, 'http://www.ease-crc.org/ont/SOMA.owl#DesignedHandle'), is_inside_of(Table,Room), "
        "has_type(Room, suturo:'LivingRoom'), object_rel_pose(Table, 'perceive', Pose).")
    # check for navigation pose to furniture for e.g. searching
    kb.prolog_client.all_solutions(
        "has_type(Obj, 'http://www.ease-crc.org/ont/SOMA.owl#DesignedFurniture').")  # returns the instances of obj currently present
    kb.prolog_client.all_solutions(
        "triple(Obj, P, 'http://www.ease-crc.org/ont/SOMA.owl#DesignedFurniture').")  # returns the class names
    # get all obj of type table and their robocup names
    kb.prolog_client.all_solutions(
        "has_type(Obj, soma:'Table'), triple(Obj, suturo:'hasRobocupName', Result).")
    # all semantic map items
    kb.prolog_client.all_solutions("has_urdf_name(Furniture, UrdfLink).")
    # all robocup item names
    kb.prolog_client.all_solutions("has_robocup_name(Furniture, RobocupName).")
    # gets all shelf layers
    kb.prolog_client.all_solutions("has_type(Obj, suturo:'ShelfLayer').")
    # get predefined location
    kb.prolog_client.all_solutions(f"predefined_origin_location(X, Y).")
    # get predifined destination
    kb.prolog_client.all_solutions(f"predefined_destination_location(X, Y).")
    # get all fruits
    kb.prolog_client.all_solutions(f"subclass_of(X, 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupFruits').")
    # get obj of that instance by name
    kb.prolog_client.all_solutions(f"what_object('living room', Obj), instance_of(Inst, Obj).")
    # get all classes of obj name
    kb.prolog_client.all_solutions("what_object_transitive('cup', Obj).")
    # get pose of obj
    kb.prolog_client.all_solutions("what_object_transitive('table', Obj), instance_of(Inst, Obj),"
                                   " is_inside_of(Inst, Room), furniture_rel_pose(Inst, 'perceive', Pose).")
    # get poses based on (hopefully) nlp names
    kb.prolog_client.all_solutions(f"what_object_transitive('table', Obj), instance_of(Inst, Obj),"
                                   f"what_object_transitive('living room', Room), instance_of(RoomInst, Room), "
                                   f"is_inside_of(Inst, RoomInst), furniture_rel_pose(Inst, 'perceive', Pose).")

    # triple(Object, soma:isOntopOf, Furniture) # check if obj is on top of shelf layer

    # drop databases
    # kb.prolog_client.all_solutions("drop_graph(user), tf_mem_clear, mng_drop(roslog, tf).")
    # kb.prolog_client.all_solutions(f"reset_user_data.")

# debugging:
# importlib.reload(gpsr)
# from pycram.ros.viz_marker_publisher import ManualMarkerPublisher
# marker = ManualMarkerPublisher()
# marker.publish(Pose.from_pose_stamped(human_p))
