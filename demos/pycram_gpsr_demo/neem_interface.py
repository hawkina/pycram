import functools
import inspect

import rospy

import neem_interface_python.neem_interface as neem_interface
from demos.pycram_gpsr_demo import utils
from pycram.designators.action_designator import NavigateAction, NavigateActionPerformable, DetectAction, \
    DetectActionPerformable
from demos.pycram_gpsr_demo.action_designator_parser import ActionDesignator, Location
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.object_designator import ObjectDesignatorDescription as Object
from pycram.ontology.ontology import OntologyManager
from demos.pycram_gpsr_demo import knowrob_interface as kb

nio = None # neem_interface.NEEMInterface()  # neem interface object TODO remove
ont = OntologyManager()  # ontology manager object
task_type = "GPSR"
# todo change from apartment to robocuop arena
env_owl = "package://iai_apartment/owl/iai-apartment.owl"
env_owl_ind_name = "http://knowrob.org/kb/iai-apartment.owl#apartment_root"  # ind = individual
env_urdf = "package://iai_apartment/urdf/apartment.urdf"
env_urdf_prefix = "iai_apartment/"
agent_owl = "package://knowrob/owl/robots/hsrb.owl"
agent_owl_ind_name = "http://knowrob.org/kb/hsrb.owl#hsrb_robot1"
agent_urdf = "package://knowrob/urdf/hsrb.urdf"
neem_output_path = "/home/hawkin/ros_ws/neems_library/GPSR_neems/"
start_time = None
root_action = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_ZBRTKSFH"  # remove later

#### temp. remove later ####
transporting = "http://www.ease-crc.org/ont/SOMA.owl#Transporting"
toya = "http://www.ease-crc.org/ont/SUTURO.owl#ToyotaHSR_IAMTOYA"


def init_neem_interface():
    global nio
    nio = neem_interface.NEEMInterface()


def start_episode():
    global nio, root_action

    res = neem_interface.NEEMInterface.start_episode(nio, task_type, env_owl, env_owl_ind_name, env_urdf,
                                                     env_urdf_prefix, agent_owl,
                                                     agent_owl_ind_name, agent_urdf)
    root_action = res
    return root_action


def stop_episode():
    global nio
    neem_interface.NEEMInterface.stop_episode(nio, neem_output_path)


def add_subaction_with_task(parent_action=root_action, action_type=transporting):  # TODO remnove transporting
    global nio
    res = neem_interface.NEEMInterface.add_subaction_with_task(nio, parent_action, action_type)
    return res


# result: the iri of the subaction. e.g. http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_PBKJOHZG'

# WIP needs testing
def belief_perceived_at(obj_type, mesh, position, rotation):
    global nio
    # TODO implement. also think about how to most cleverly pass data from perception here
    #belief_perceived_at()


# WIP needs testing
def add_participant_with_role(action, participant, role):
    global nio
    neem_interface.NEEMInterface.add_participant_with_role(nio, action, participant, role)

def action_begin(current_action):
    global nio
    neem_interface.NEEMInterface.action_begin(nio, current_action)

def action_end(current_action):
    global nio
    neem_interface.NEEMInterface.action_end(nio, current_action)

def triple(subj, pred, obj):
    global nio
    neem_interface.NEEMInterface.triple(nio, subj, pred, obj)

def make_instance_of(class_iri):
    global nio
    res = neem_interface.NEEMInterface.make_instance_of(nio, class_iri)
    return res

def add_pose_to_instance(instance, pose):
    global nio
    res = neem_interface.NEEMInterface.add_pose_to_instance(nio, instance, pose)
    return res

# ---------------- Automation of calling the NEEM interface ----------------
# Flag to track initialization state
initialized = False
parent_action = root_action
current_action = None


# Define the initialization function that establishes the connection
def initialize_neem(): # done
    global initialized
    if not initialized:
        rospy.loginfo(utils.PC.GREEN + "[NEEM] Initializing connection..." + utils.PC.GREY)
        # Your initialization logic here, e.g., establishing connection
        init_neem_interface()
        start_episode()
        initialized = True
        rospy.loginfo(utils.PC.GREEN + "[NEEM] Connection established." + utils.PC.GREY)


# Class-level decorator to handle object initialization
def neem_class_decorator(cls):
    original_init = cls.__init__

    @functools.wraps(original_init)
    def new_init(self, *args, **kwargs):
        # Ensures initialization is done once when an object is created
        initialize_neem()

        rospy.loginfo(utils.PC.PINK + f"[NEEM] Initializing object of class {cls.__name__}" + utils.PC.GREY)

        # wip todo: log object at generation time. Access obj here
        rospy.loginfo(utils.PC.PINK + f"[NEEM] ActionDesig at creation time self: {self}" + utils.PC.GREY)
        rospy.loginfo(utils.PC.RED + f"[NEEM] TODO log object at generation time." + utils.PC.GREY)
        # wip at this point it is the action designator description which needs to be logged
        # go back to original init after the created object has been logged?
        original_init(self, *args, **kwargs)  # Call the original __init__ method

    cls.__init__ = new_init
    return cls


# Method decorator to log function calls
def generate_neem(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        global parent_action, current_action, ont  # ont is an IRI
        rospy.loginfo(utils.PC.PINK + f"[NEEM] Function {func.__name__} called" + utils.PC.GREY)
        # WIP: put pre-resovle code here

        if isinstance(self, ObjectDesignatorDescription):
            rospy.loginfo(utils.PC.YELLOW + f"[NEEM] Processing ObjectDesignatorDescription:" + utils.PC.GREY)


        if isinstance(self, ActionDesignator):
            rospy.loginfo(utils.PC.PINK + f"[NEEM] Created ActionDesignator: {self}" + utils.PC.GREY)

            if isinstance(self.action_instance, NavigateAction) or isinstance(self, NavigateAction):
                rospy.loginfo(utils.PC.GREEN + "[NEEM] NavigateAction detected" + utils.PC.GREY)
                current_action = add_subaction_with_task(parent_action=parent_action,
                                                         action_type=ont.soma.Navigating.iri)
                add_participant_with_role(current_action, toya, ont.soma.AgentRole.iri)
                # todo add goal which is the location designator
                #self.target_locations
                # create an instance of a location
                loc_inst = make_instance_of(ont.soma.Location.iri)
                # connect location instance to action as goal
                triple(current_action, ont.soma.hasGoal.iri, loc_inst)  # add a pose

                # process location designator within the action designator
                for attr_name, attr_value in self.action_instance.__dict__.items():
                    # this is location designator specific todo test if only for my locdesig or generally for all of them
                    if isinstance(attr_value, Location):
                        rospy.loginfo(utils.PC.YELLOW + f"[NEEM] Found location designator: {attr_name} + {attr_value} " + utils.PC.GREY)
                        tmp = attr_value.ground()
                        attr_value = tmp.poses
                        pose_array = [attr_value[0].frame , attr_value[0].position_as_list(), attr_value[0].orientation_as_list()]
                        add_pose_to_instance(loc_inst, pose_array)
                        rospy.loginfo(utils.PC.YELLOW + f"[NEEM] Done processing LocationDesignator: {pose_array}" + utils.PC.GREY)

            # process object designator within the action designator
            if isinstance(self, Object):
                rospy.loginfo(
                    utils.PC.PINK + f"[NEEM] Processing ObjectDesignator:" + utils.PC.GREY)
            else:
                rospy.logerr(utils.PC.PINK + f"[NEEM] Invalid designator type: {self}" + utils.PC.GREY)

            # Wrap resolve and perform methods of ActionDesignator to log them
            self.resolve = log_method(self.resolve, self, 'resolve')
            self.perform = log_method(self.perform, self, 'perform')
        else:
            rospy.logerr(f"[NEEM] not an instance of ActionDesignator: {self}" + utils.PC.GREY)


        # wip resolve action desig
        # result = func(self, *args, **kwargs)

        # WIP: put past-resolve code here
        # rospy.loginfo(utils.PC.PINK + f"[NEEM] Action {result} returned from {func.__name__}" + utils.PC.GREY)

        return self # result

    return wrapper


# This decorator logs calls to the resolve and perform methods
def log_method(method, action_designator, method_name):
    @functools.wraps(method)
    def wrapper(*args, **kwargs):
        rospy.loginfo(utils.PC.PINK + f"[NEEM] In Log_method wrapper {method_name}() on {action_designator}" + utils.PC.GREY)

        if method_name == 'resolve':
            print("resolve")

        elif method_name == 'perform':
            # Log start time for perform
            action_begin(current_action)
            rospy.loginfo(utils.PC.PINK + f"[NEEM] Starting {method_name} on {action_designator}" + utils.PC.GREY)

            # Call the original method and log the result
            result = method(*args, **kwargs)
            rospy.loginfo(utils.PC.PINK + f"[NEEM] {method_name}() result: {result}" + utils.PC.GREY)

            # Log end time for perform
            action_end(current_action)
            rospy.loginfo(
                utils.PC.PINK + f"[NEEM] {method_name}() completed on {action_designator}, result: {result}" + utils.PC.GREY)
        else:
            result = method(*args, **kwargs)
            rospy.loginfo(utils.PC.PINK + f"[NEEM] Neither perform or resolve: {method_name}() result: {result}" + utils.PC.GREY)

        return result

    return wrapper

# Function to dynamically apply decorators to your ActionDesignator class
def apply_decorators_to_action_designator():
    class NeemClass:
        def __init__(self, type, **kwargs):
            self.type = type
            rospy.loginfo(utils.PC.PINK + f"[NEEM] ActionDesignator initialized with type {self.type}" + utils.PC.GREY) #?

        def resolve(self):
            # Logic for resolving designator
            rospy.loginfo(utils.PC.PINK + f"[NEEM] Resolving designator: {self}" + utils.PC.GREY)
            log_method(self.resolve, self, 'resolve')
            return "--- resolved_result ---"

        def perform(self):
            # Logic for performing action
            rospy.loginfo(utils.PC.PINK + f"[NEEM] Performing action: {self}" + utils.PC.GREY)
            log_method(self.perform, self, 'perform')
            return "--- action_performed ---"

    # Apply class-level decorator
    NeemClass = neem_class_decorator(ActionDesignator)

    # Apply method-level decorator to specific methods
    ActionDesignator.resolve = generate_neem(ActionDesignator.resolve)
    ActionDesignator.perform = generate_neem(ActionDesignator.perform)
    # TODO this should be dynamic
    NavigateActionPerformable.perform = generate_neem(NavigateActionPerformable.perform)
    DetectActionPerformable.perform = generate_neem(DetectActionPerformable.perform)

    return NeemClass

def reset_neem():
    global initialized, parent_action, current_action
    initialized = False
    parent_action = root_action
    current_action = None

# --- NEEM queries ----
def get_longest_event():
    result = kb.kb.prolog_client.all_solutions(f"findall([Duration, Evt],"
                                               f"(event_interval(Evt, Begin, End),"
                                               f"number(End),"
                                               f"Duration is End - Begin),"
                                               f"Durations),"
                                               f"max_member([MaxDuration, LongestEvt], Durations).")
    return result


