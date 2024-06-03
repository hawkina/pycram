import rospy

from ..datastructures.knowledge_source import KnowledgeSource, QueryKnowledge, UpdateKnowledge
from ..datastructures.enums import ObjectType
from ..datastructures.pose import Pose
from ..plan_failures import KnowledgeNotAvailable

import rosservice

from ..designator import DesignatorDescription, ObjectDesignatorDescription
from ..designators.action_designator import PickUpAction

from typing import Optional

try:
    from rosprolog_client import Prolog
except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import Prolog client from package rosprolog_client, Knowrob related features are not available.")


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
            query = f"member(Object, {object_description.names})" + query
        result = self.prolog_client.once(query)
        if not result:
            return None
        pose = Pose(
            position = result["Position"],
            orientation = result["Rotation"],
            frame = result["Frame"]
        )
        return ObjectDesignatorDescription.Object(
            name = result["Object"],
            # TODO map the knowrob type to the pycram type
            obj_type = ObjectType.GENERIC_OBJECT,
            # TODO search for the world object, if available
            world_object = None,
            _pose = lambda: pose
        )
