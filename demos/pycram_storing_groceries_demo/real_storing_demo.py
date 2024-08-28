from pycram.ros.robot_state_updater import KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.datastructures.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()

robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
# #robokudo.init_robokudo_interface()
# #rospy.sleep(2)
giskardpy.spawn_kitchen()
#
# # Shelf Variables
# shelf_pose = Pose([4.417880842356951,4.913135736923778, 0] ,[0, 0, 0, 1])
# lower_compartment = ShelfCompartmentDescription(height=0.09, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# middle_compartment = ShelfCompartmentDescription(height=0.37, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# upper_compartment = ShelfCompartmentDescription(height=0.71, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# shelves = [lower_compartment, middle_compartment, upper_compartment]


shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                          [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])

# List of objects
objects = [
    "Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball",
    "Baseball", "Mustardbottle", "Jellochocolatepuddingbox", "Wineglass", "Orange",
    "Coffeepack", "Softball", "Metalplate", "Pringleschipscan", "Strawberry",
    "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug", "Abrasivesponge",
    "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox",
    "Coffeecan", "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan",
    "Gelatinebox", "Pear", "Lemon", "Banana", "Pottedmeatcan", "Peach",
    "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
    "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small",
    "screwdriver", "clamp", "hammer", "wooden_block", "Cornybox*"
]

# Group objects by similarity
groups = {
    "Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
    "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue", "Cupgreen",
                                 "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
    "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                    "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                    "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                    "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"]
}

links_from_shelf = [
    'shelf:shelf:shelf_floor_0',
    'shelf:shelf:shelf_floor_1',
    'shelf:shelf:shelf_floor_2',
]


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
        link_pose = kitchen.get_link_pose(link)
        posez = obj_pose.position.z

        dis = abs(link_pose.pose.position.z - posez)
        if dis <= position_distance:
            position_distance = dis
            nearest_link = link
    return nearest_link


def get_closest_pose(obj_pose, pose_list):
    position_distance = float('inf')
    nearest_pose = None

    for pose in pose_list:
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 +
               (obj_pose.pose.position.y - pose.position.y) ** 2 +
               (obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


def pickupnew(object_desig, grasp, arm):
    # Initialize the local transformer and robot reference
    lt = LocalTransformer()
    robot = BulletWorld.robot
    # Retrieve object and robot from designators
    object = object_desig.bullet_world_object
    # Calculate the object's pose in the map frame
    oTm = object.get_pose()
    execute = True

    if grasp == "top":
        oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
    oTb.orientation = grasp_rotation
    oTmG = lt.transform_pose(oTb, "map")

    rospy.logwarn("Opening Gripper")
    MoveGripperMotion(motion="open", gripper=arm).resolve().perform()

    rospy.logwarn("Picking up now")
    BulletWorld.current_bullet_world.add_vis_axis(oTmG)
    #!!!!!!!!!!!!!
    MoveTCPMotion(oTmG, arm, allow_gripper_collision=False).resolve().perform()

    tool_frame = robot_description.get_tool_frame(arm)
    special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
    z = 0.04
    if grasp == "top":
        z = 0.025
    special_knowledge_offset.pose.position.z += z
    push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    liftingTm = push_baseTm
    liftingTm.pose.position.z += 0.03



    rospy.logwarn("Offset now")
    # !!!!!!!!!!!!!11
    MoveTCPMotion(special_knowledge_offsetTm, arm, allow_gripper_collision=False).resolve().perform()

    rospy.logwarn("Pushing now")
    # !!!!!!!!!!!!!11
    MoveTCPMotion(push_baseTm, arm, allow_gripper_collision=False).resolve().perform()

    # Finalize the pick-up by closing the gripper and lifting the object
    rospy.logwarn("Close Gripper")
    # !!!!!!!!!!!!!11
    MoveGripperMotion(motion="close", gripper=arm, allow_gripper_collision=True).resolve().perform()

    rospy.logwarn("Lifting now")

    # !!!!!!!!!!!!!11
    MoveTCPMotion(liftingTm, arm, allow_gripper_collision=False).resolve().perform()



    tool_frame = robot_description.get_tool_frame(arm)
    robot.attach(object=object_desig.bullet_world_object, link=tool_frame)


def demo(step):
    with real_robot:
        # Wait for the start signal
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        if step <= 0:
            start_signal_waiter.wait_for_startsignal()

            # continue with the rest of the script
            rospy.loginfo("Start signal received, now proceeding with tasks.")

        # move to shelf
        if step <= 1:
            rospy.logerr("driving")
            talk.pub_now("driving", True)
            move.query_pose_nav(rotated_shelf_pose)
            move.query_pose_nav(shelf_pose)

        # perceive shelf
        groups_in_shelf = {
            "Kitchen Utensils and Tools": None,
            "Containers and Drinkware": None,
            "Cleaning Supplies": None,
            "Packaged Food and Beverages": None,
            "Fruits": None,
            "Sports Equipment": None,
            "Miscellaneous": None
        }
        groups_on_table = {}

        if step <= 2:
            look_pose = kitchen.get_link_pose("shelf:shelf:shelf_floor_1")
            LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18
            shelf_obj = DetectAction(technique='all').resolve().perform()
            first, *remaining = shelf_obj
            for dictionary in remaining:
                for value in dictionary.values():

                    try:
                        group = find_group(value.type)
                        link = get_closet_link_to_pose(value.pose)
                        groups_in_shelf[group] = [value.pose, link, value]

                    except AttributeError:
                        pass
            print(groups_in_shelf)
        # if step <= 3:
        #     rospy.logerr("driving")
        #     talk.pub_now("driving", True)
        #     move.query_pose_nav(rotated_shelf_pose)
        #     move.query_pose_nav(table_pose)
        #     look_pose = kitchen.get_link_pose("popcorn_table:p_table:table_center")
        #     look_pose.pose.position.x+=0.5
        #     world.current_bullet_world.add_vis_axis(look_pose)
        #     LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18
        #
        #     table_obj = DetectAction(technique='all').resolve().perform()
        #     first, *remaining = table_obj
        #     for dictionary in remaining:
        #         for value in dictionary.values():
        #             try:
        #                 group = find_group(value.type)
        #                 groups_on_table[value.name] = [value, group]
        #
        #             except AttributeError:
        #                 pass
        #
        # if step <= 4:
        #     # print(groups_on_table.keys())
        #     for obj in groups_on_table.values():
        #         PickUpAction(obj[0], ["left"], ["front"]).resolve().perform()
        #
        #         rospy.logerr("driving")
        #         talk.pub_now("driving", True)
        #         move.query_pose_nav(rotated_shelf_pose)
        #         move.query_pose_nav(shelf_pose)
        #
        #         if groups_in_shelf[obj[1]] is not None:
        #             try:
        #                 link = groups_in_shelf[obj[1]][1]
        #                 group_pose = groups_in_shelf[obj[1]][0]
        #             except TypeError:
        #                 pass  # for now, later new group
        #             place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left",
        #                                               world, 0.2, object_desig=obj[0])
        #             if place_poses:
        #                 nearest_pose_to_group = get_closest_pose(group_pose, place_poses)
        #                 world.current_bullet_world.add_vis_axis(nearest_pose_to_group)
        #                 PlaceAction(obj[0], ["left"], ["front"], [nearest_pose_to_group]).resolve().perform()

        # if groups_in_shelf[group] is not None:
        #     posey = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left",
        #                                 world, 0, object_desig=value)
        #     world.current_bullet_world.add_vis_axis(posey)
        #     print("done im iter")
        print("done")
        # res = sort_new(shelf_obj, robot)
        #
        #
        #
        # # object sorted by distance to robot
        # sorted_objects = res[0]
        # print("sorted obj: " + str(sorted_objects))
        #
        # # dict with object poses
        # object_list_poses = res[1]
        # print("pose list: " + str(object_list_poses))
        #
        # # update perceived info to shelves
        # order_items_to_shelves(object_list_poses, shelves)

        # TODO: next steps

        # navigate to table and perceive

        # while objects on table:

        # pick up nearest object

        # navigate to shelf

        # place object depending on shelf areas

        # end?


# print(kitchen.get_link_pose("shelf:shelf:shelf_floor_2"))
# get_closet_link_to_pose()
demo(2)
# rospy.sleep(1)
# print(robot.get_pose())
# print(kitchen.links)
