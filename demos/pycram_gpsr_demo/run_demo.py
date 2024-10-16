from dynamic_reconfigure.msg import DoubleParameter, IntParameter, BoolParameter, StrParameter, GroupState, Config
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from demos.pycram_gpsr_demo.setup_demo import *
from pycram.designators.action_designator import *
from demos.pycram_gpsr_demo import utils, setup_demo, perception_interface, ActionDesignator, Location, neem_interface, \
    generate_neem, enable_neem_generation
import demos.pycram_gpsr_demo.nlp_processing as nlp
from stringcase import snakecase
import demos.pycram_gpsr_demo.llp_navigation as navi
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import giskard
from pycram.process_module import real_robot, semi_real_robot


instruction_point = PoseStamped([6.12, 1.8, 0], [0, 0, 0, 1])
#instruction_point = PoseStamped([4.4, -0.5, 0], [0, 0, 0, 1])
#image_switch = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()


def move_vel(speed, distance, isForward, angle=0):
    # Starts a new node
    velocity_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    vel_msg = Twist()

    # Checking if the movement is forward or backwards
    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = 0
    if angle > 0:
        vel_msg.angular.z = angle
    else:
        vel_msg.angular.z = 0
    # Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Setting the current time for distance calculation
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    # Loop to move the turtle a specified distance
    while not rospy.is_shutdown() and current_distance < distance:
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Take actual time to velocity calculation
        t1 = rospy.Time.now().to_sec()
        # Calculate distance
        current_distance = speed * (t1 - t0)

    # After the loop, stop the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)


def set_parameters(new_parameters):
    rospy.wait_for_service('/tmc_map_merger/inputs/base_scan/obstacle_circle/set_parameters')
    try:
        reconfigure_service = rospy.ServiceProxy('/tmc_map_merger/inputs/base_scan/obstacle_circle/set_parameters',
                                                 Reconfigure)
        config = Config()

        # Set the new parameters
        if 'forbid_radius' in new_parameters:
            config.doubles.append(DoubleParameter(name='forbid_radius', value=new_parameters['forbid_radius']))
        if 'obstacle_occupancy' in new_parameters:
            config.ints.append(IntParameter(name='obstacle_occupancy', value=new_parameters['obstacle_occupancy']))
        if 'obstacle_radius' in new_parameters:
            config.doubles.append(DoubleParameter(name='obstacle_radius', value=new_parameters['obstacle_radius']))

        # Empty parameters that are not being set
        config.bools.append(BoolParameter(name='', value=False))
        config.strs.append(StrParameter(name='', value=''))
        config.groups.append(GroupState(name='', state=False, id=0, parent=0))

        req = ReconfigureRequest(config=config)
        reconfigure_service(req)
        rospy.loginfo("Parameters updated successfully")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def pub_fake_pose(fake_pose: PoseStamped):
    toya_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=100)
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position = fake_pose.pose.position
    msg.pose.pose.orientation = fake_pose.pose.orientation
    msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.06853892326654787]
    toya_pose_pub.publish(msg)


# DEPRECTAED / RoboCup Only
# def yeet_into_arena():
#     global image_switch
#     try:
#
#         #image_switch.pub_now(ImageEnum.HI.value)  # hi im toya
#         sing_my_angel_of_music("Push down my Hand, when you are Ready.")
#         #image_switch.pub_now(ImageEnum.PUSHBUTTONS.value)
#         plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
#         plan.perform()
#     except SensorMonitoringCondition:
#         sing_my_angel_of_music("Starting GPSR.")
#         # load everything world giskard robokudo....
#         # Wait for the start signal
#
#         #image_switch.pub_now(ImageEnum.HI.value)
#         start_signal_waiter.wait_for_startsignal()
#
#         # Once the start signal is received, continue with the rest of the script
#         rospy.loginfo("Start signal received, now proceeding with tasks.")
#         move_vel(speed=2, distance=4, isForward=True)
#         rospy.sleep(2)
#         new_parameters = {
#             'forbid_radius': 0.35,
#             'obstacle_occupancy': 10,
#             'obstacle_radius': 0.35
#         }
#
#         set_parameters(new_parameters)  # Once the start signal is received, continue with the rest of the script
#
#         rospy.sleep(1)
#
#         fake_pose_2 = Pose([2.88, 0.3, 0])
#         pub_fake_pose(fake_pose_2)
#         move_vel(0.2, 2, False, 0.02)
#         sing_my_angel_of_music("Driving.")
#         move_123 = Pose([4, -0.4, 0], [0, 0, 0, 1])
#         navi.go_to_pose(move_123)
#         #move_145 = Pose([4.8, 0.8, 0], [0, 0, 0.7, 0.7])
#         #navi.go_to_pose(move_145)
#


# --- main control ---
# TODO: test on real robot
def gpsr():
    with real_robot:
        plan_list = utils.get_plans(high_level_plans)
        #yeet_into_arena()
        #sound_pub = SoundRequestPublisher()
        #sound_pub.publish_sound_request()

        sing_my_angel_of_music("Going to the instruction point")
        navi.go_to_pose(PoseStamped([5.44, 0.2, 0.0], [0, 0, 0, 1]))  # in door
        #navi.go_to_room_entry_or_exit('office', 'exit')
        navi.go_to_pose(instruction_point)
        # look at a person when listening to command

        # high_level_plans.prepare_for_commands()
        instruction_list = []
        for i in range(10):  # test
            # TODO add this to plan.
            #    DetectAction(technique='human').resolve().perform()
            #    giskardpy.move_head_to_human()
            #    giskardpy.cancel_all_goals() OR giskardpy.cancel_all_called_goals() to stop
            # listen to instructions
            perception_interface.looking_for_human()
            giskard.move_head_to_human()
            instruction_list = nlp.listen_to_commands()
            rospy.logwarn("[CRAM] instruction list: " + str(instruction_list))
            giskard.cancel_all_called_goals()

            # execute instructions
            # TODO iterate over list of instructions and do stuff
            while instruction_list:  # Test
                rospy.logwarn("[CRAM] in instruction loop")
                instruction = instruction_list.pop(0)
                rospy.loginfo(instruction)
                # do stuff
                # match instruction to plan
                utils.call_plan_by_name(plan_list, snakecase(instruction['intent']), instruction)
                # if plan was successful, remove it from the list
                instruction_list.remove(instruction)
            instruction_list = []
            sing_my_angel_of_music("navigating to the instruction point")
            navi.go_to_pose(instruction_point)


# CHANGE CARE THIS STUFF GETS ACTUALLY EXECUTED
def demo_plan(data):
    with real_robot:
        high_level_plans.transporting(data)
        print('--------------stahp----------------')
        return


# --- TESTING FUNCTIONS ---
@giskard.init_giskard_interface
def test_move(pose=[Pose([1, 1, 0])]):
    print("in test move")
    #pose1 = Pose([1, 1, 0])
    #giskard.teleport_robot(pose1)
    with semi_real_robot:
        NavigateAction(pose).resolve().perform()
        #giskard.teleport_robot(pose1)


@giskard.init_giskard_interface
def test_move_pose(x, y):
    print("in test move")
    pose1 = Pose([1, 1, 0])
    giskard.teleport_robot(pose1)
    with semi_real_robot:
        NavigateAction([Pose([x, y, 0])]).resolve().perform()
        print("done with navigation")
        #giskard.teleport_robot(pose1)
    print("done?")


# --- test ActionDesignator ---
@giskard.init_giskard_interface
def test_desig():
    with semi_real_robot:
        NavigateAction([Pose([0, 0, 0])]).resolve().perform()
        print("--- done with navigation ---")
        rospy.sleep(2)
        # Test ActionDesignator
        action = ActionDesignator(type='navigate', target_locations=[Pose([2, 1, 0])])
        action.print_parameters()
        action.perform()


@giskard.init_giskard_interface
def test_nav_action(furniture_item=None, room=None):
    with semi_real_robot:
        action = ActionDesignator(type='navigate',
                                  target_locations=Location(furniture_item=furniture_item,
                                                            room=room))
        action.resolve().perform()


@giskard.init_giskard_interface
def test_perception_ref():
    with semi_real_robot:
        action = DetectAction(technique=PerceptionTechniques.ALL,
                              object_designator=ObjectDesignatorDescription(types=[ObjectType.JEROEN_CUP]))
        action = action.resolve().perform()
        for obj_desig in action.values():
            print(obj_desig)


@giskard.init_giskard_interface
def test_perception():
    with semi_real_robot:
        #perceived_obj = DetectAction(technique=PerceptionTechniques.ALL).resolve().perform()
        action = ActionDesignator(type='detect',
                                  technique=PerceptionTechniques.ALL,
                                  object_designator=ObjectDesignatorDescription(types=[ObjectType.JEROEN_CUP]))
        action = action.resolve().perform()
        print("----------------")
        print(action)
        result = None
        # for obj_desig in action.values():
        # print("-----------------")
        # print(obj_desig)
        # if obj_desig.obj_type == ObjectType.JEROEN_CUP:
        #     result = obj_desig
        print("##########################################################################")
        #print(action.values().get(ObjectType.JEROEN_CUP))
        return action


@giskard.init_giskard_interface
def test_pick_up():
    with semi_real_robot:
        #Perceive
        action = ActionDesignator(type='detect',
                                  technique=PerceptionTechniques.ALL,
                                  object_designator=ObjectDesignatorDescription(types=[ObjectType.JEROEN_CUP]))
        action = action.resolve().perform()
        # pick up
        cup_desig = action.values().get('cup')
        surface_loc = Location(furniture_item='kitchen counter', room='kitchen').ground()
        PickUpAction(object_designator_description=cup_desig,
                     arms=[Arms.LEFT],
                     grasps=[Grasp.FRONT],
                     source_location=surface_loc).resolve().perform()
        rospy.sleep(2)
        PlaceAction(cup_desig, target_locations=[Pose([10.3, 5.29, 0.9])], arms=[Arms.LEFT]).resolve().perform()


#@generate_neem
@giskard.init_giskard_interface
def test_plan(furniture_item='kitchen counter', room='kitchen', object_type='cup'):
    with semi_real_robot:
        ActionDesignator = enable_neem_generation()
        # look for jeroen_cup
        nav_location = Location(furniture_item=furniture_item, room=room)
        nav_action = ActionDesignator(type='navigate', target_locations=nav_location)

        nav_action.resolve().perform()

        object_desig = ObjectDesignatorDescription(types=[utils.match_string_to_enum(object_type, ObjectType)])

        detect_action = ActionDesignator(type='detect',
                                         technique=PerceptionTechniques.ALL,
                                         object_designator=object_desig)
        detect_action = detect_action.resolve().perform()
        detect_result = detect_action.get(object_type) # wip this dies?

        rospy.sleep(2)
        # look for milk
        # nav_location2 = Location(furniture_item='table', room='kitchen')
        # nav_action2 = ActionDesignator(type='navigate', target_locations=nav_location2)
        # nav_action2.resolve().perform()
        # object_desig2 = ObjectDesignatorDescription(types = [utils.match_string_to_enum('milk', ObjectType)])
        # detect_action2 = ActionDesignator(type='detect',
        #                           technique=PerceptionTechniques.ALL,
        #                           object_designator = object_desig2)
        # detect_action2 = detect_action2.resolve().perform()
        # detect_result2 = detect_action2.get('milk')
        return detect_result


#@generate_neem
def test_neem():
    ActionDesignator =  enable_neem_generation()
    #nav_action = ActionDesignator(type='navigate',
    #                              target_locations=Location(furniture_item='kitchen counter', room='kitchen'))
    furniture_designator = ObjectDesignatorDescription(names=["kitchen counter"])
    room_designator = ObjectDesignatorDescription(names=["kitchen"])

    nav_action = ActionDesignator(type='navigate',
                                  target_locations=Location(
                                      furniture_item=furniture_designator,
                                      room=room_designator))

    detect_action = ActionDesignator(type='detect', technique=PerceptionTechniques.ALL,
                                     object_designator=ObjectDesignatorDescription(types=[ObjectType.JEROEN_CUP]))
    nav_action.resolve()
    detect_action.resolve()
    print("--- done with execution ---")
    return [nav_action, detect_action]


# deprecated
def test_neem_generation():
    neem_interface.init_neem_interface()
    neem_interface.start_episode()

    #neem_interface.stop_episode()
