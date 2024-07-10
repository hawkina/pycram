import rospy
from demos.pycram_gpsr_demo.setup_demo import *
from demos.pycram_gpsr_demo import utils
import demos.pycram_gpsr_demo.nlp_processing as nlp
from stringcase import snakecase
import demos.pycram_gpsr_demo.perception_interface as pc
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music


# --- main control ---
# TODO: test on real robot
def gpsr():
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



def test_moving_to():
    param = {'sentence': '  Go to the kitchen .', 'intent': 'MovingTo',
             'DestinationRoom': {'value': 'kitchen', 'entity': 'Room', 'propertyAttribute': [], 'actionAttribute': [],
                                 'numberAttribute': []}}
    high_level_plans.moving_to(param)


def test_gpsr():
    setup()
    gpsr()
    sing_my_angel_of_music(f"I am done with your bs")


# CHANGE CARE THIS STUFF GETS ACTUALLY EXECUTED
# test_gpsr()
