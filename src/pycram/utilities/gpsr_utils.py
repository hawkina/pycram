from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code
from pycram.plan_failures import NoPlacePoseFoundCondition
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal, pakerino, GraspListener

world = BulletWorld()



environment_raw = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")
environment_desig = ObjectDesignatorDescription(names=["kitchen"])

grasp_listener = GraspListener()
talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()
lt = LocalTransformer()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
giskardpy.clear()
giskardpy.sync_worlds()


previous_value = None


# return a pose if pose found otherwise NoPlacePoseFoundCondition
def get_place_poses_for_surface(object_to_place,link):

    place_poses = find_placeable_pose(link, environment_desig.resolve(), robot_desig.resolve(), "left",
                                      world, 0.1,
                                      object_desig=object_to_place)
    print(place_poses)
    if place_poses:
        nearest_pose = place_poses[0]

        pose_in_shelf = lt.transform_pose(nearest_pose, environment_raw.get_link_tf_frame(link))

        pose_in_shelf.pose.position.x = -0.10
        pose_in_shelf.pose.position.z += 0.03
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")

        world.current_bullet_world.add_vis_axis(adjusted_pose_in_map)

        return adjusted_pose_in_map

    else:
        return NoPlacePoseFoundCondition


def place(object, grasp, link):
    def monitor_func_place():
        global previous_value
        der = fts.get_last_value()
        current_value = fts.get_last_value()

        prev_force_x = previous_value.wrench.force.x
        curr_force_x = current_value.wrench.force.x
        change_in_force_x = abs(curr_force_x - prev_force_x)
        print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")

        def calculate_dynamic_threshold(previous_force_x):
            # Placeholder for a dynamic threshold calculation based on previous values
            # This function can be enhanced to calculate a threshold based on the history of values or other logic
            return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5

        if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
            print("Significant change detected")

            return SensorMonitoringCondition

        return False

    global previous_value


    if grasp == "front":
        config_for_placing = {'arm_lift_joint': 0.20, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                              'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
    else:
        config_for_placing = {'arm_flex_joint': 0.20, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                              'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

    pakerino(config=config_for_placing)

    try:
        table_obj = DetectAction(technique='all').resolve().perform()
        found_object = None
        first, *remaining = table_obj
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        talk.pub_now("I was not able to perceive any objects")

    #todo ifno target_location  then do handover
    target_location = get_place_poses_for_surface(object, 'shelf:shelf:shelf_floor_2')

    oTm = target_location
    oTm.pose.position.z += 0.02

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, environment_raw.get_link_tf_frame(link))

    if grasp == "top":
        grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
        oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
    else:
        oTb.orientation = grasp_rotation

    oTmG = lt.transform_pose(oTb, "map")

    BulletWorld.current_bullet_world.add_vis_axis(oTmG)
    #talk.pub_now(f"Placing now! {object_name.split('_')[0]} from: {grasp}")
    giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
    while not giskard_return:
        rospy.sleep(0.1)
    config_after_place = {'arm_lift_joint': 0.0}
    talk.pub_now("Placing neatly")
    previous_value = fts.get_last_value()
    try:
        plan = Code(lambda: giskardpy.test(config_after_place)) >> Monitor(monitor_func_place)
        plan.perform()

    except Exception as e:
        print(f"Exception type: {type(e).__name__}")

    giskardpy.achieve_attached(object.bullet_world_object, tip_link="map")
    BulletWorld.robot.detach(object.bullet_world_object)
    gripper.pub_now("open")

    giskardpy.avoid_all_collisions()
    park = pakerino()
    while not park:
        print("waiting for park")
        rospy.sleep(0.1)
    return True


#return grasped_bool, grasp, found_object
def process_pick_up_objects(obj_type, link):
    look_pose = environment_raw.get_link_pose(link)
    # park
    perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    pakerino(config=perceive_conf)
    # look
    #todo here should be look pose
    locationtoplace = Pose([2.6868790796016738, 5.717920690528091, 0.715])
    giskardpy.move_head_to_pose(locationtoplace)

    talk.pub_now("perceiving", True)
    # noteme detect on table
    try:
        table_obj = DetectAction(technique='all').resolve().perform()
        found_object = None
        first, *remaining = table_obj
        print(first)
        for dictionary in remaining:
            for value in dictionary.values():
                if value.type == obj_type:
                    found_object = value
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        talk.pub_now("I was not able to perceive any objects")
        return

    # noteme if groups were found
    if found_object:
        obj_pose = found_object.bullet_world_object.pose
        object_raw = found_object
        tf_link = environment_raw.get_link_tf_frame(link)
        oTb = lt.transform_pose(obj_pose, tf_link)

        grasp_set = None
        # if to far behind the front face were robots look on the table
        if oTb.pose.position.x >= 0.20:
            grasp_set = "top"

        object_name = found_object.bullet_world_object.name
        object = found_object.bullet_world_object

        angle = helper.quaternion_to_angle(
            (oTb.pose.orientation.x, oTb.pose.orientation.y, oTb.pose.orientation.z, oTb.pose.orientation.w))
        object_dim = object.get_object_dimensions()
        print(f"obj dim von {object_name} {object_dim}")

        if grasp_set:
            grasp = "top"
        else:
            if object_dim[2] < 0.055:
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.075 and object_dim[1] > 0.075):
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            else:
                rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "front"

            if grasp == "top":
                print("pose adjusted with z")
                oTb.pose.position.z += (object_dim[2] / 10)
                if object_dim[2] < 0.02:
                    rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                    oTb.pose.position.z = 0.011
            else:
                oTb.pose.position.x += 0.03

        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
        if grasp == "top":
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
        else:
            oTb.orientation = grasp_rotation

        oTmG = lt.transform_pose(oTb, "map")
        after_pose = oTmG.copy()
        after_pose.pose.position.z += 0.02

        BulletWorld.current_bullet_world.add_vis_axis(oTmG)

        if grasp == "front":
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        else:
            config_for_placing = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                                  'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        pakerino(config=config_for_placing)

        gripper.pub_now("open")
        talk.pub_now(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
        giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
        while not giskard_return:
            rospy.sleep(0.1)

        giskardpy.achieve_attached(object)
        tip_link = 'hand_gripper_tool_frame'
        BulletWorld.robot.attach(object=object, link=tip_link)
        gripper.pub_now("close")
        giskardpy.avoid_all_collisions()
        park = pakerino()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        grasped_bool = None
        if grasp_listener.check_grasp():
            talk.pub_now("Grasped a object")
            grasped_bool = True
        else:
            talk.pub_now("I was not able to grasped a object")
            grasped_bool = False

        return grasped_bool, grasp, found_object



#
# def follow_human():


def pick_place_demo():
    # get 2 move poses
    with real_robot:
        pakerino()
        shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
        rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                                  [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
        table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])
        table_pose_pre = Pose([2.862644998141083, 4.946512935221523, 0.0],
                              [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])

        move.pub_now(table_pose)
        try:
            gasped_bool, grasp, found_object = process_pick_up_objects('cup_small',  "popcorn_table:p_table:table_front_edge_center")
        except TypeError:
            print("pick other iten")
            return

        move.pub_now(rotated_shelf_pose)
        move.pub_now(shelf_pose)
        place(found_object, "front", 'shelf:shelf:shelf_floor_2')

pick_place_demo()
