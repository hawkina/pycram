import neem_interface_python.neem_interface as neem_interface
from demos.pycram_gpsr_demo.knowrob_interface import kb

nio = neem_interface.NEEMInterface()  # neem interface object TODO remove
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
def add_participatn_with_role(action, participant, role):
    global nio
    neem_interface.NEEMInterface.add_participant_with_role(nio, action, participant, role)

