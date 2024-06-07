#setup the environment
from pycram.designators.action_designator import *
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from pycram.robot_descriptions import robot_description
from pycram.enums import ObjectType
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from . import utils, high_level_plans

plan_list = {}

def setup():
    global world, robot, environment, rviz
    # world = BulletWorld("DIRECT") #rviz only, without any parameters, spawns bullet
    print("init setup")
    world = BulletWorld("DIRECT")
    rviz = VizMarkerPublisher()

    robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
    robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
    robot.set_color([0.5, 0.0, 0.2, 1])

    environment = Object("environment", ObjectType.ENVIRONMENT, "suturo_lab_version_small.urdf")
    environment_desig = ObjectDesignatorDescription(names=["environment"])
    print("done with setup")


def do_stuff():
    with simulated_robot:
        print("now doing things")
        pose1 = Pose([1.45, 4.5, 0], [0, 0, 1, 0])
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        NavigateAction([pose1]).resolve().perform()
        MoveJointsMotion(["wrist_roll_joint"], [-1.57]).resolve().perform()
        MoveTorsoAction([0.35]).resolve().perform()
        MoveGripperMotion(motion="open", gripper="left").resolve().perform() #fails
        print("done")


def test():
    plan_list = utils.get_plans(high_level_plans)
    print("imported all plans: ")
    print(plan_list)
    print("attempt to call cleaning with param test")
    utils.call_plan_by_name(plan_list, "cleaning", "test")

