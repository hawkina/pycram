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
        self.pose = None
        #self.ground() # commented out so we can differenceate between description + resolved designator

    def ground(self):
        furniture_item = None
        room = None
        # check if the required items exist:
        # furniture item name
        # TODO fix the fallback of class. Since instances of class table also exist
        # problem case: table in living room
        if ('furniture_item' in self.kwargs and isinstance(self.kwargs['furniture_item'], str)
                and knowrob.check_existence_of_instance(snakecase(self.kwargs['furniture_item']))):
            rospy.loginfo(utils.PC.BLUE + f"[LOC] instance of furniture item {self.kwargs['furniture_item']} found")
            furniture_item = self.kwargs['furniture_item']

        if ('room' in self.kwargs and isinstance(self.kwargs['room'], str)
                and knowrob.check_existence_of_instance(self.kwargs['room'])):
            room = self.kwargs['room']

        # get poses
        if furniture_item and room:
            self.poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture_item,
                                                     room=room)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FIR] Location for {furniture_item} in {room} found.")

        elif furniture_item:
            self.poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture_item)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FI] Location for {furniture_item} found.")

        elif room:
            self.poses = [knowrob.get_room_middle_pose(room=room)]
            rospy.loginfo(utils.PC.BLUE + f"[LOC R] Location for {room} middle found")
        else:
            rospy.loginfo(utils.PC.BLUE + f"[LOC] Location designator could not be resolved.")
            pass

        # pose contains the first pose of the list
        # poses contain the entire list result of knowrob
        if not self.poses:
            rospy.loginfo(utils.PC.BLUE + f"[LOC] Location designator could not be resolved.")
            return None
        else:
            self.pose = self.poses[0].get('Item').get('pose')
            return self.poses

    
    # loc.poses works
    # TODO implement iterator?

    def print_desig(self):
        print(f"Location({self.kwargs})")




