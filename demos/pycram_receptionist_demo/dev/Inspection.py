# from demos.pycram_carry_my_luggage.utils.cml_helper import fts
import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_receptionist_demo.utils.helper import detect_point_to_seat
from pycram.designators.action_designator import NavigateAction
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.failures import SensorMonitoringCondition
from pycram.language import Code, Monitor
from pycram.process_module import real_robot
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycram.utilities.robocup_utils import StartSignalWaiter, ImageSwitchPublisher
import rospy
from pycram.external_interfaces.navigate import PoseNavigator

start_signal = StartSignalWaiter()
# Initialize the Bullet world for simulation
fts = ForceTorqueSensor(robot_name='hsrb')
world = BulletWorld()
# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
RobotStateUpdater("/tf", "/giskard_joint_states")
img = ImageSwitchPublisher()
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
pose1 = Pose(position=[1.35, 0.1, 0], orientation=[0, 0, 0, 1])
pose2 = Pose(position=[4.3, 0.2, 0], orientation=[0, 0, 0, 1])
inspection_pose = Pose(position=[7.2, -0.3, 0], orientation=[0, 0, 1, 0])
# pose4 = Pose(position=[7.6, 3.15, 0])
back_pose1 = Pose(position=[8, 0.4, 0], orientation=[0, 0, 0.7, 0.7])
back_pose2 = Pose(position=[7.75, 3, 0], orientation=[0, 0, 0, 1])
back_pose3 = Pose(position=[1.8, 3, 0], orientation=[0, 0, 0, 1])
# end = Pose(position=[2.9, -0.5, 0], orientation=[0, 0, -0.7, 0.7])
navigation = PoseNavigator()

def demo(step: int):
    with real_robot:
        # start_pose = robot.get_pose()
        # print(start_pose)
        img.pub_now(ImageEnum.HI.value)
        print(robot.get_pose())
        print("done")


        # start_signal.wait_for_startsignal()
        # MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        TalkingMotion("push down my gripper").perform()
        try:
            plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
            plan.perform()

        except SensorMonitoringCondition:
            TalkingMotion("please open the door to let me in").perform()

            start_signal.wait_for_startsignal()
            start_pose = robot.get_pose()
            navigation.pub_fake_pose(start_pose)

            giskardpy.turning_left_and_back(45)

            if step <= 1:
                print("start")

                # MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
                # MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()


                TalkingMotion("Starting Inspection").perform()
                print("before nav")
                NavigateAction([pose1]).resolve().perform()
                TalkingMotion("on my way to the inspection point").perform()
                NavigateAction([pose2]).resolve().perform()
                NavigateAction([inspection_pose]).resolve().perform()
                TalkingMotion("i reached the inspection point").perform()
                rospy.sleep(6)
                TalkingMotion("driving to exit now").perform()
                NavigateAction([back_pose1]).resolve().perform()
                NavigateAction([back_pose2]).resolve().perform()
                NavigateAction([back_pose3]).resolve().perform()
                TalkingMotion("end of inspection").perform()


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception")
        return SensorMonitoringCondition

    return False




def demo_start():
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    try:

        TalkingMotion("Starting Inspection").perform()
        # img.pub_now(ImageEnum.HI.value)  # hi im toya
        TalkingMotion("Push down my Hand, when i should drive to the inspection point").perform()
        # img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        # img.pub_now(ImageEnum.HI.value)

        TalkingMotion("i will start driving now").perform()
        rospy.sleep(2)

        return






demo(0)
