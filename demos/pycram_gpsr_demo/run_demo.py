from demos.pycram_gpsr_demo.setup_demo import *
from demos.pycram_gpsr_demo import utils, setup_demo
import demos.pycram_gpsr_demo.nlp_processing as nlp
from stringcase import snakecase
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music
import src.pycram.utilities.gpsr_utils as gpsr_utils


# --- main control ---
# TODO: test on real robot
def gpsr():
    with real_robot:
        plan_list = utils.get_plans(high_level_plans)
        # plan_list = utils.get_plans(high_level_plans) # get list of all available plans
        # init params
        # go to instruction point, look at human, etc.
        # high_level_plans.prepare_for_commands()

        for i in range(5):  # test
            # listen to instructions
            instruction_list = nlp.listen_to_commands()
            rospy.logwarn("[CRAM] instruction list: " + str(instruction_list))

            # execute instructions
            # TODO iterate over list of instructions and do stuff
            while instruction_list:  # Test
                rospy.logwarn("[CRAM] in instruction loop")
                instruction = instruction_list.pop(0)
                rospy.loginfo(instruction)
                # do stuff
                # match instruction to plan
                utils.call_plan_by_name(plan_list, snakecase(instruction['intent']), instruction)
                # if plan was successfull, remove it from the list


data = {
    "sentence": "Please bring the red cup from the kitchen table to the living room table",
    "intent": "Transporting",
    "Item": {
        "value": "cup",
        "entity": "Transportable",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "Source": {
        "value": "couch table",
        "entity": "DesignedFurniture",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "SourceRoom": {
        "value": "living room",
        "entity": "Room",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "DestinationRoom": {
        "value": "dining room",
        "entity": "Room",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "Destination": {
        "value": "shelf",
        "entity": "DesignedFurniture",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    }
}

data2 = {
    "sentence": "  Bring me the cup from the living room table .",
    "intent": "Transporting",
    "BeneficiaryRole": {
        "value": "me",
        "entity": "NaturalPerson",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "Item": {
        "value": "cup",
        "entity": "Transportable",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "SourceRoom": {
        "value": "living room",
        "entity": "Room",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    },
    "Source": {
        "value": "couch table",
        "entity": "DesignedFurniture",
        "propertyAttribute": [],
        "actionAttribute": [],
        "numberAttribute": []
    }
}




# CHANGE CARE THIS STUFF GETS ACTUALLY EXECUTED
def demo_plan(data):
    with real_robot:
        high_level_plans.transporting(data)
        print('--------------stahp----------------')
        return


#setup()
#gpsr()
#demo_plan(data2)
#setup_demo.gripper.pub_now('open')
