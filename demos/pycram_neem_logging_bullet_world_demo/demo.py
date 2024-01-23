from neem_interface_python import neem
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import neem_interface_python
from neem_interface_python.neem_interface import Episode
from pycram.ros.tf_broadcaster import TFBroadcaster

# enable RviZ visualisation instead of BulletWorld
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
tf_broadcaster = TFBroadcaster()

robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
                color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

### set all necessary parameters for logging
neem_interface = neem_interface_python.neem_interface.NEEMInterface()
task_type = "BreakfastDemo"
env_owl = "package://iai_apartment/owl/iai-apartment.owl"
env_owl_ind_name = "http://knowrob.org/kb/iai-apartment.owl#apartment_root" # ind = individual
env_urdf = "package://iai_apartment/urdf/apartment.urdf"
env_urdf_prefix = "iai_apartment/"
agent_owl = "package://knowrob/owl/robots/PR2.owl"
agent_owl_ind_name = "http://knowrob.org/kb/PR2.owl#PR2_0"
agent_urdf = "package://knowrob/urdf/pr2.urdf"
neem_output_path = "/home/ahawkin/ros_ws/neems_library/BreakfastDemo"
start_time = None


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    with Episode(neem_interface, task_type, env_owl, env_owl_ind_name, env_urdf, env_urdf_prefix, agent_owl,
                 agent_owl_ind_name, agent_urdf, neem_output_path) as episode:

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.25]).resolve().perform()

        milk_desig = move_and_detect(ObjectType.MILK)

        TransportAction(milk_desig, ["left"], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

        cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)

        TransportAction(cereal_desig, ["right"], [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])]).resolve().perform()

        bowl_desig = move_and_detect(ObjectType.BOWL)

        TransportAction(bowl_desig, ["left"], [Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()

        # Finding and navigating to the drawer holding the spoon
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
        drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                        robot_desig=robot_desig.resolve()).resolve()

        NavigateAction([drawer_open_loc.pose]).resolve().perform()

        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        spoon.detach(apartment)

        # Detect and pickup the spoon
        LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

        spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()

        pickup_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
        PickUpAction(spoon_desig, [pickup_arm], ["top"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        close_loc = drawer_open_loc.pose
        close_loc.position.y += 0.1
        NavigateAction([close_loc]).resolve().perform()

        CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.15]).resolve().perform()

        # Find a pose to place the spoon, move and then place it
# BUG commented out due to not working
#        spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
#        placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

#        NavigateAction([placing_loc.pose]).resolve().perform()

#        PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
