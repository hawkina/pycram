import os
import sys
import rospy

from ..datastructures.knowledge_source import KnowledgeSource, QueryKnowledge, UpdateKnowledge
from ..datastructures.enums import ObjectType
from ..datastructures.pose import Pose
from ..datastructures.knowledge_source import KnowledgeNotAvailable

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

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, os.pardir, os.pardir, "neem-interface", "src"))


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
        rospy.loginfo("[KnowRob] query sent: " + str(query))
        query = self.prolog_client.query(query)
        atexit.register(query.finish)
        rospy.loginfo("[KnowRob] result: " + str(query))
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

    def get_object_pose(self, object_name: str) -> Optional[Pose]:
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


### DEPRECATED
#
# if 'rosprolog/query' in rosservice.get_service_list():
#     from neem_interface_python.neem_interface import NEEMInterface
#     from neem_interface_python.rosprolog_client import Prolog, PrologException, atom
#
#     neem_interface = NEEMInterface()
#     prolog = Prolog()
#
# else:
#     logging.warning("No KnowRob services found, knowrob is not available")
#
#
# #logging.setLoggerClass(logging.Logger)
# logger = logging.getLogger(__name__)
# from pycram import ch
#
# logger.addHandler(ch)
# logger.setLevel(logging.DEBUG)
#
#
# def init_knowrob_interface():
#     """
#     Initializes the knowrob interface.
#     Can not be used at the moment.
#     """
#     global interf
#     global is_init
#     if is_init:
#         return
#     try:
#         import rospy
#         from suturo_knowledge import interf_q
#         interf = interf_q.InterfacePlanningKnowledge()
#         is_init = True
#         rospy.loginfo("Successfully initialized Knowrob interface")
#
#     except ModuleNotFoundError as e:
#         rospy.logwarn("Failed to import Knowrob messages, knowrob interface could not be initialized")
#
#
# def all_solutions(q):
#     logging.info(q)
#     r = prolog.all_solutions(q)
#     return r
#
#
# def once(q) -> Union[List, Dict]:
#     r = all_solutions(q)
#     if len(r) == 0:
#         return []
#     return r[0]
#
#
# def load_beliefstate(path: str):
#     logging.info(f"Restoring beliefstate from {path}")
#     once(f"remember('{path}')")
#
#
# def clear_beliefstate():
#     logging.info("Clearing beliefstate")
#     once("mem_clear_memory")
#
#
# def load_owl(path, ns_alias=None, ns_url=None):
#     """
#     Example: load_owl("package://external_interfaces/owl/maps/iai_room_v1.owl", "map", "http://knowrob.org/kb/v1/IAI-Kitchen.owl#")
#     :param str path: path to log folder
#     :rtype: bool
#     """
#     if ns_alias is None or ns_url is None:            # Load without namespace
#         q = "load_owl('{}')".format(path)
#     else:
#         q = "load_owl('{0}', [namespace({1},'{2}')])".format(path, ns_alias, ns_url)
#     try:
#         once(q)
#         return True
#     except PrologException as e:
#         logging.warning(e)
#         return False
#
#
# def new_iri(owl_class: str):
#     res = once(f"kb_call(new_iri(IRI, {owl_class}))")
#     return res["IRI"]
#
#
# def object_type(object_iri: str) -> str:
#     """
#     :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
#     """
#     res = once(f"kb_call(instance_of({atom(object_iri)}, Class))")
#     return res["Class"]
#
#
# def instances_of(type_: str) -> List[str]:
#     """
#     :param type_: An object type (i.e. class)
#     """
#     all_sols = all_solutions(f"kb_call(instance_of(Individual, {atom(type_)}))")
#     return [sol["Individual"] for sol in all_sols]
#
#
# def object_pose(object_iri: str, reference_cs: str = "world", timestamp=None) -> List[float]:
#     """
#     :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
#     :param reference_cs: The coordinate system relative to which the pose should be defined
#     """
#     if timestamp is None:
#         res = once(f"mem_tf_get({atom(object_iri)}, {atom(reference_cs)}, Pos, Ori)")
#     else:
#         res = once(f"mem_tf_get({atom(object_iri)}, {atom(reference_cs)}, Pos, Ori, {timestamp})")
#     pos = res["Pos"]
#     ori = res["Ori"]
#     return pos + ori
#
#
# def grasp_pose(object_iri: str) -> List[float]:
#     query = f"""
#     kb_call(has_grasp_point({atom(object_iri)}, GraspPointName)),
#     mem_tf_get(GraspPointName, world, Pos, Ori)
#     """
#     res = once(query)
#     pos = res["Pos"]
#     ori = res["Ori"]
#     return pos + ori
#
#
# def knowrob_string_to_pose(pose_as_string: str) -> List[float]:
#     reference_frame = ""
#     for i, char in enumerate(pose_as_string[1:-1]):
#         if char == ",":
#             break
#         reference_frame += char
#     pos, ori = pose_as_string[1+i+2:-2].split("],[")
#     xyz = list(map(float, pos.split(",")))
#     qxyzw = list(map(float, ori.split(",")))
#     return xyz + qxyzw
#
# def get_guest_info(id):
#     """
#     function that uses Knowledge Service to get Name and drink from new guest via ID
#     :param id: integer for person
#     :return: ["name", "drink"]
#     """
#
#     rospy.wait_for_service('info_server')
#     try:
#         info_service = rospy.ServiceProxy('info_server', IsKnown)
#         # guest_data = person_infos: "name,drink"
#         guest_data = info_service(id)
#         # result = ['person_infos: "name', 'drink"']
#         result = str(guest_data).split(',')
#         result[0] = result[0][13:]
#         return result
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed")
#         pass
#
# def get_table_pose(table_name):
#     """
#     Get table pose from knowledge
#     :param table_name: predefined name for each table
#
#     :return: object pose of the given table
#     """
#     rospy.wait_for_service('pose_server')
#     try:
#         service = rospy.ServiceProxy('pose_server', ObjectPose)
#         table_pose = service(table_name)
#         return table_pose
#     except rospy.ServiceException:
#         rospy.logerr("Service call failed")
#         pass
#
#
# def get_location_pose(location_name):
#     """
#     Can not be used yet.
#     Gives the pose for a given location back for instance for perceiving, placing, opening etc.
#     """
#     try:
#         location_pos = interf.get_pose(location_name)
#         return location_pos
#     except:
#         rospy.logerr("Failed to contact knowrob")
#         pass
#
#
# def get_handle_pos(handle):
#     """
#       Can not be used yet.
#       Gives the pose for a given handle back for instance for opening a container.
#     """
#     try:
#         handle_pos = interf.get_pose_of_handle(handle)
#         return handle_pos
#     except:
#         rospy.logerr("Failed to contact knowrob")
#         pass
