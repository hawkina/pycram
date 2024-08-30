import rospy
from stringcase import snakecase

from pycram.designators.location_designator import *
from demos.pycram_gpsr_demo import knowrob_interface as knowrob, utils


@dataclasses.dataclass
class Location(LocationDesignatorDescription.Location):
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.print_desig()
        self.poses = []
        #self.ground() # commented out so we can differenceate between description + resolved designator

    def ground(self):
        furniture_item = None
        furniture_class = None
        room = None
        # check if the required items exist:
        # furniture item name
        # TODO fix the fallback of class. Since instances of class table also exist
        # problem case: table in living room
        if ('furniture_item' in self.kwargs and isinstance(self.kwargs['furniture_item'], str)
                and knowrob.check_existence_of_instance(snakecase(self.kwargs['furniture_item']))):
            rospy.loginfo(utils.PC.BLUE + f"[LOC] instance of furniture item {self.kwargs['furniture_item']} found")
            furniture_item = snakecase(self.kwargs['furniture_item'])

        # furniture class iri
        elif ('furniture_item' in self.kwargs and isinstance(self.kwargs['furniture_item'], str)
                and knowrob.check_existence_of_class(self.kwargs['furniture_item'])):
            rospy.loginfo(utils.PC.BLUE + f"[LOC] No instance of furniture item found. Fallback to class.")
            furniture_class = knowrob.check_existence_of_class(self.kwargs['furniture_item'])[0].get('Class')
            rospy.loginfo(utils.PC.BLUE + f"[LOC] class of furniture item {self.kwargs['furniture_item']} found")

        if ('room' in self.kwargs and isinstance(self.kwargs['room'], str)
                and knowrob.check_existence_of_instance(snakecase(self.kwargs['room']))):
            room = snakecase(self.kwargs['room'])

        # get poses
        if furniture_item and room:
            self.poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture_item,
                                                     room=room)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FIR] Location for {furniture_item} in {room} found.")


        elif furniture_class and room:
            self.poses = knowrob.get_nav_poses_for_furniture_item(furniture_iri=furniture_class,
                                                     room=room)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FCR] Location for {furniture_class} in {room} found.")

        elif furniture_item:
            self.poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture_item)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FI] Location for {furniture_item} found.")

        elif furniture_class:
            self.poses = knowrob.get_nav_poses_for_furniture_item(furniture_iri=furniture_class)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FC] Location for {furniture_class} found.")

        elif room:
            self.poses = knowrob.get_room_middle_pose(room=room)
            rospy.loginfo(utils.PC.BLUE + f"[LOC R] Location for {room} middle found")
        else:
            rospy.loginfo(utils.PC.BLUE + f"[LOC] Location designator could not be resolved.")
            pass
    # loc.poses works
    # TODO implement iterator?

    def print_desig(self):
        print(f"Location({self.kwargs})")




