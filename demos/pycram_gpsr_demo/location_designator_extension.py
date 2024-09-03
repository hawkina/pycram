import rospy
from numpy.f2py.auxfuncs import throw_error
from stringcase import snakecase

from pycram.designator import ResolutionError
from pycram.designators.location_designator import *
from demos.pycram_gpsr_demo import knowrob_interface as knowrob, utils

class LocationDesignatorResolutionError(Exception):
    def __init__(self, message="An error occurred", *args, **kwargs):
        super().__init__(message, *args, **kwargs)
        self.message = message

    def __str__(self):
        return f"[LocationDesignatorResolutionError]: {self.message}"

@dataclasses.dataclass
class Location(LocationDesignatorDescription.Location):
    def __init__(self, *args, **kwargs):
        self.args = args
        self.semantic_poses = []
        self.poses = []
        self.pose = None
        self.kwargs = kwargs
        self.print()
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
        else:
            rospy.logerr(f"[LOC] no instance of furniture item found.")

        if ('room' in self.kwargs and isinstance(self.kwargs['room'], str)
                and knowrob.check_existence_of_instance(self.kwargs['room'])):
            room = self.kwargs['room']
        else:
            rospy.logerr(f"[LOC] no instance of room found.")

        if not furniture_item and not room:
            raise LocationDesignatorResolutionError(f"Location designator could not be resolved."
                                  f"No objects of type: {furniture_item} and: {room} found.")

        # get poses
        if furniture_item and room:
            self.semantic_poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture_item,
                                                                           room=room)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FIR] Location for {furniture_item} in {room} found.")

        elif furniture_item and not room:
            self.semantic_poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture_item)
            rospy.loginfo(utils.PC.BLUE + f"[LOC FI] Location for {furniture_item} found.")

        elif room and not furniture_item:
            self.semantic_poses = [knowrob.get_room_middle_pose(room=room)]
            rospy.loginfo(utils.PC.BLUE + f"[LOC R] Location for {room} middle found")
        else:
            rospy.loginfo(utils.PC.BLUE + f"[LOC] Location designator could not be resolved.")
            raise LocationDesignatorResolutionError(f"Location designator could not be resolved."
                                                    f"No position of: {furniture_item} and: {room} found.")


        # pose contains the first pose of the list
        # poses contain the entire list result of knowrob
        if self.semantic_poses is not []:
            if len(self.semantic_poses) == 1 and room:
                self.pose = Pose.from_pose_stamped(self.semantic_poses[0]) # room pose is returned differently
                self.poses = [self.pose]
                return self
            else:
                self.pose = self.semantic_poses[0].get('Item').get('pose')
                self.poses = utils.knowrob_dict_list_to_poses_list(self.semantic_poses)
                return self
        else:
            rospy.loginfo(utils.PC.BLUE + f"[LOC] Location designator could not be resolved.")
            return None


    # loc.poses works
    # TODO implement iterator?

    def print(self):
        print(f"Location({self.kwargs})")




