import rospy

from ..datastructures.knowledge_source import KnowledgeSource, QueryKnowledge, UpdateKnowledge
from ..datastructures.enums import ObjectType
from ..datastructures.pose import Pose
from ..plan_failures import KnowledgeNotAvailable

import rosservice

from ..designator import DesignatorDescription, ObjectDesignatorDescription
from ..designators.action_designator import PickUpAction

from typing import Optional, Iterator, Tuple

try:
    from rosprolog_client import Prolog, PrologQuery
except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import Prolog client from package rosprolog_client, Knowrob related features are not available.")

# for automtically closing queries on exit of pycram
import atexit

class KnowrobKnowledge(KnowledgeSource, QueryKnowledge, UpdateKnowledge):

    def __init__(self):
        super().__init__("Knowrob", 0)
        self.prolog_client = None

    @property
    def is_available(self) -> bool:
        return '/rosprolog/query' in rosservice.get_service_list()

    @property
    def is_connected(self) -> bool:
        return self.prolog_client is not None

    def connect(self):
        if self.is_available:
            self.prolog_client = Prolog()
            # TODO this line errors because tripledb_load is not found
            # self.prolog_client.once(f"tripledb_load('package://iai_apartment/owl/iai-apartment.owl').")


    def _query_prolog(self, query: str) -> PrologQuery:
        """
        Start a query to knowrob. This method alse does additional work.
        Currently this additional work is registering an atexit function,
        that finishes the query to free up resources in knowrob when pycram exists.

        :param query: the knowrob query string
        :return: the rosprolog_client PrologQuery object the current client returned
        """
        query = self.prolog_client.query(query)
        atexit.register(query.finish)
        return query

    def query(self, designator: DesignatorDescription) -> DesignatorDescription:
        pass

    def query_pose_for_object(self, designator: DesignatorDescription) -> DesignatorDescription:
        if isinstance(designator, PickUpAction):
            object_description = designator.object_designator_description
            if isinstance(object_description, ObjectDesignatorDescription.Object):
                # object is already resolved, don't change anything
                return designator
            else:
                # object_description is of type ObjectDesignatorDescription
                obj = self._internal_query_object_for_object_designator_description(object_description)
                if not obj:
                    raise KnowledgeNotAvailable(f"Pose for object {designator} not available in {self.name}")
                designator.object_designator_description = obj
                return designator
        # elif isinstance(designator, somethingelse):
        raise KnowledgeNotAvailable(f"Pose for object {designator} not available in {self.name}")

    def _internal_query_object_for_object_designator_description(self, object_description: ObjectDesignatorDescription) -> Optional[ObjectDesignatorDescription.Object]:
        query = "kb_call(has_type(Object,Type)),kb_call(is_at(Object,[Frame,Position,Rotation]))"
        # optimization assumtion: i assume that len(types)*len(names) is a lot less than the number of objects in the knowrob database.
        # if this assumtion doesn't hold, the code might be slower how it is
        # vs how it would be if the type and name check are between has_type and is_at
        if object_description.types is not None:
            # TODO convert pycram ObjectType into knowrob object types
            # TODO and prepend the type condition
            pass
        if object_description.names is not None:
            # TODO check if pycram object names are the same as knowrob ones
            query = f"member(Object, {object_description.names})," + query
        result = self.prolog_client.once(query)
        if not result:
            return None
        pose = Pose(
            position = result["Position"],
            orientation = result["Rotation"],
            frame = result["Frame"]
        )
        obj = ObjectDesignatorDescription.Object(
            name = result["Object"],
            # TODO map the knowrob type to the pycram type
            obj_type = ObjectType.GENERIC_OBJECT,
            # TODO search for the world object, if available
            world_object = None
        )
        obj.pose = lambda: pose
        return obj

    def get_object_pose(object_name: str) -> Optional[Pose]:
        """
        Query the pose for an object from the knowledge source

        :param object_name: IRI of the object
        :return: Pose with the pose of the object, or None if the object is not known
        """
        result = self.prolog_client.once(f"kb_call(is_at('{object_name}',[Frame,Position,Rotation]))")
        if not result:
            return None
        return Pose(
            position = result["Position"],
            orientation = result["Rotation"],
            frame = result["Frame"]
        )

    def get_object_pose_of_type(self, type_name: str) -> Iterator[Tuple[str,Pose]]:
        """
        Query poses and objects of a certain type from Knowrob.
        This might return multiple results, which are provided lazily.

        :param type_name: IRI of the type
        :return: Iterator of object IRI, object pose pairs
        """
        # with block for automatic finishing of the query once the generator is exhausted
        with self._query_prolog(f"kb_call(has_type(OBJECT,'{type_name}')),kb_call(is_at(OBJECT,[FRAME,POSITION,ROTATION]))") as query:
            # query.solutions() returns a generator, so this is lazy
            for result in query.solutions():
                yield (result["OBJECT"],Pose(
                    position = result["POSITION"],
                    orientation = result["ROTATION"],
                    frame = result["FRAME"]))
