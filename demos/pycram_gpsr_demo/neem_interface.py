import neem_interface_python.neem_interface as neem_interface
from demos.pycram_gpsr_demo.knowrob_interface import kb

nio = None
task_type = "GPSR"
# todo change from apartment to robocuop arena
env_owl = "package://iai_apartment/owl/iai-apartment.owl"
env_owl_ind_name = "http://knowrob.org/kb/iai-apartment.owl#apartment_root" # ind = individual
env_urdf = "package://iai_apartment/urdf/apartment.urdf"
env_urdf_prefix = "iai_apartment/"
agent_owl = "package://knowrob/owl/robots/hsrb.owl"
agent_owl_ind_name = "http://knowrob.org/kb/hsrb.owl#hsrb_robot1"
agent_urdf = "package://knowrob/urdf/hsrb.urdf"
neem_output_path = "/home/ahawkin/ros_ws/neems_library/GPSR_neems/"
start_time = None
root_action = "http://www.ease-crc.org/ont/SOMA.owl#Transporting"

def init_neem_interface():
    global nio
    nio = neem_interface.NEEMInterface()

def start_episode():
    # nio.start_episode(task_type, env_owl, env_owl_ind_name, env_urdf, agent_owl,
    #                            agent_owl_ind_name, agent_urdf, start_time)
    kb.prolog_client.once(f"mem_episode_start('{root_action}', '{task_type}', '{env_owl}', "
                          f"'{env_owl_ind_name}', '{env_urdf}', '{agent_owl}', '{agent_owl_ind_name}', '{agent_urdf}').")