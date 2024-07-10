import rospy
from demos.pycram_gpsr_demo.setup_demo import *
from demos.pycram_gpsr_demo import utils
import demos.pycram_gpsr_demo.nlp_processing as nlp
from stringcase import snakecase
import demos.pycram_gpsr_demo.perception_interface as pc
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music
import src.pycram.utilities.gpsr_utils as gpsr_utils


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


def test_pick_place():
    # get 2 move poses
    with real_robot:
        shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
        rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                                  [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
        table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])
        table_pose_pre = Pose([2.862644998141083, 4.946512935221523, 0.0],
                              [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])

        move.pub_now(table_pose)
        gasped_bool, grasp, found_object = gpsr_utils.process_pick_up_objects('cup',  "popcorn_table:p_table:table_front_edge_center")
        target_pose = gpsr_utils.get_place_poses_for_surface(found_object, 'shelf:shelf:shelf_floor_2')
        move.pub_now(rotated_shelf_pose)
        move.pub_now(shelf_pose)
        if target_pose:
            gpsr_utils.place(found_object, grasp, target_pose, 'shelf:shelf:shelf_floor_2')


# CHANGE CARE THIS STUFF GETS ACTUALLY EXECUTED
# test_gpsr()
setup()
test_pick_place()