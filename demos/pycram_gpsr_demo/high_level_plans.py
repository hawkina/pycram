import re
import rospy
from pycram.designators.action_designator import *
from pycram.utilities.robocup_utils import StartSignalWaiter, pakerino
from demos.pycram_gpsr_demo import perception_interface, llp_tell_stuff
import demos.pycram_gpsr_demo.perception_interface as robokudo
from demos.pycram_gpsr_demo import knowrob_interface as knowrob
from demos.pycram_gpsr_demo import llp_navigation as navi
import demos.pycram_gpsr_demo.utils as utils
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music
import pycram.utilities.gpsr_utils as plans
from demos.pycram_gpsr_demo import setup_demo as sd
from demos.pycram_gpsr_demo import nlp_processing as nlp
from stringcase import snakecase
from demos.pycram_gpsr_demo import perc_to_know
from pycram.pose import Pose as PoseStamped

object_in_hand = None
me_pose = None
found_obj = None


# these are all the high level plans, to which we map the NLP output.
# they should either connect to low level plans or be filled with data from knowledge

# todo replace obj_dict with the nlp to knowrob one
# navigate the robot to LOCATION
def moving_to(param_json):  # WIP Can also be funriture, or a person
    # ToDo: test
    # ToDo: does it always make sense to use enter pose?
    global with_real_robot
    rospy.loginfo("[CRAM] MovingTo plan." + str(param_json))
    # check which fields are filled:
    room, furniture, person, furniture_class = None, None, None, None
    nav_poses = []
    # get all parameters
    if param_json.get('Destination') is not None:
        # ensure furniture obj exists
        furniture = param_json.get('Destination').get('value').lower()
        if knowrob.check_existence_of_instance(snakecase(furniture)):
            rospy.loginfo(f"[CRAM] found instance of furniture item " + furniture)
            furniture = snakecase(furniture)
        # if instance does not exist,check if class does ?
        elif knowrob.check_existence_of_class(furniture):
            rospy.loginfo("[CRAM] found class of furniture item " + furniture)
            furniture_class = knowrob.check_existence_of_class(furniture)[0].get('Class')
            furniture = False
        elif utils.obj_dict.get(furniture) is not None:
            furniture = utils.obj_dict.get(furniture)
            rospy.loginfo("[CRAM] Fallback: of furniture item found in list. item : " + furniture)
        else:
            rospy.loginfo("[CRAM] could not find furniture item " + furniture)
            sing_my_angel_of_music(f"I am sorry. I don't know where {furniture} is.")
            return None

    if param_json.get('DestinationRoom') is not None:
        # ensure room exists
        if knowrob.rooms.get(snakecase(param_json.get('DestinationRoom').get('value').lower())) is not None:
            room = param_json.get('DestinationRoom').get('value').lower()
        else:
            room = 'arena'

    if param_json.get('BeneficiaryRole') is not None:
        person = param_json.get('BeneficiaryRole')

    rospy.logwarn(f"room: {room}, furniture: {furniture}, person: {person}, furniture_class: {furniture_class}")

    # --- Cases ---
    # Room + Furniture + Person WIP---------------------------------------
    if room and furniture and person:
        rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to person at furniture item in room")
        sing_my_angel_of_music(f"I will go to the {furniture} in {room} to look for a person.")
        #rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to furniture item in room")
        #sing_my_angel_of_music(f"I will go to the {furniture} in {room}.")
        if furniture:
            rospy.loginfo(f"[CRAM] found instance of furniture item " + str(furniture))
            nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_name=furniture)
        # if instance does not exist,check class?
        elif furniture_class:  # maybeh make this an and?
            if furniture == furniture_class:
                #  matched knowledge and nlp
                nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room,
                                                                     furniture_name=furniture)  # change might get removed
            else:
                nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_iri=furniture_class)
        # go to furniture item in room
        if nav_poses is not None and nav_poses != []:
            # go to furniture item
            rospy.loginfo(
                utils.PC.BLUE + f"[CRAM] going to furniture item in Room pose {nav_poses[0].get('Item').get('pose')}")
            result = navi.go_to_pose(nav_poses[0].get('Item').get('pose'))
            rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
            return nav_poses[0]
        # go to person at furniture item in room
    # Room + Person WIP---------------------------------------------------
    elif room and person:  # DONE one value
        # go to the person in the room
        rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to person in room")
        sing_my_angel_of_music(f"I will go to the {room} and look for the person.")
        result = navi.go_to_room_middle(room)
        # look for person in room TODO ------------------------------------------
        rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
        return result

    # Furniture + Person WIP----------------------------------------------
    elif furniture and person:  # TODO - remove duplicates
        rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to person at furniture item")
        sing_my_angel_of_music(f"I will go to the {furniture} and look for a person.")
        # can have multiple poses
        nav_poses = []
        # try matching furniture name to knowrob obj, if it doesn't exist fall back to class
        if furniture:
            nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture)
        # if instance does not exist,check class?
        elif furniture_class:
            if furniture == furniture_class:
                #  matched knowledge and nlp
                nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture,
                                                                     furniture_iri=furniture)  # change might get removed
            else:
                nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_iri=furniture_class)
        # go to person at furniture item TODO ----------------------------------------------------------
        # go to furniture item in room
        if nav_poses is not None and nav_poses != []:
            # go to furniture item
            rospy.loginfo(utils.PC.BLUE + "[CRAM] going to furniture item and person")
            result = navi.go_to_pose(nav_poses[0].get('Item').get('pose'))
            rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
            # TODO look for person
            return nav_poses[0]


    elif room and furniture:  # DONE has multiple poses TODO remove duplicate code
        rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to furniture item in room")
        sing_my_angel_of_music(f"I will go to the {furniture} in {room}.")
        if furniture:
            rospy.loginfo(f"[CRAM] found instance of furniture item " + str(furniture))
            nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_name=furniture)
        # if instance does not exist,check class?
        elif furniture_class:  # maybeh make this an and?
            if furniture == furniture_class:
                #  matched knowledge and nlp
                nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room,
                                                                     furniture_name=furniture)  # change might get removed
            else:
                nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_iri=furniture_class)
        # go to furniture item in room
        if nav_poses is not None and nav_poses != []:
            # go to furniture item
            rospy.loginfo(
                utils.PC.BLUE + f"[CRAM] going to furniture item in Room pose {nav_poses[0].get('Item').get('pose')}")
            result = navi.go_to_pose(nav_poses[0].get('Item').get('pose'))
            rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
            return nav_poses[0]

    elif furniture:  # DONE
        sing_my_angel_of_music(f"I will go to the {furniture}.")
        # try matching furniture name to knowrob obj, if it doesn't exist fall back to class
        if furniture:
            nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture)
        # if instance does not exist,check class?
        elif furniture_class:
            if furniture == furniture_class:
                #  matched knowledge and nlp
                nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_iri=furniture)
            else:
                nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_iri=furniture_class)
        # try to move -----------------------------------------------
        if nav_poses is not None and nav_poses != []:
            rospy.loginfo(utils.PC.BLUE + "[CRAM] in going to room")
            # go to furniture item
            result = navi.go_to_pose(nav_poses[0].get('Item').get('pose'))
            rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
            return nav_poses[0]  # needed for perception

        else:
            rospy.loginfo(utils.PC.BLUE + "[CRAM] could not find furniture item " + str(
                param_json.get('Destination').get('value')))
            sing_my_angel_of_music("I am sorry. I don't know where that is.")
            return None  # abort mission

    elif room:  # DONE
        # has only one pose
        sing_my_angel_of_music(f"I am going to {room}.")
        rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to room")
        # go to room
        result = navi.go_to_room_middle(snakecase(param_json.get('DestinationRoom').get('value').lower()))
        rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
        return result
    # fallback
    elif person:
        if person.get('value') == 'me' and me_pose is not None:
            sing_my_angel_of_music("I am going to you.")
            rospy.loginfo(utils.PC.BLUE + "[CRAM] moving to me")
            # go to me
            result = navi.go_to_pose(me_pose)
            rospy.loginfo(utils.PC.BLUE + "[CRAM] result: " + str(result))
            return result

    else:
        sing_my_angel_of_music(utils.PC.BLUE + f"I am sorry. I don't know where that is.")
        rospy.logerr("[CRAM]: MovingTo plan failed. No valid parameters found.")
        return None


# subplan of fetching and transporting
def pick_up(param_json):  # works testing
    # TODO add obj to knowrob?
    global object_in_hand
    item, source, source_room, look_at_link, look_at_pose = None, None, None, None, None
    obj_types_dict = utils.obj_dict
    sing_my_angel_of_music("in picking up plan")
    rospy.loginfo(utils.PC.BLUE + "Pick-up plan called with params: " + str(param_json))
    # TODO get data from knowrob
    # 1.navigate to object
    source_params = {'Source': param_json.get('Source'), 'SourceRoom': param_json.get('SourceRoom')}
    source_params = utils.remap_source_to_destination(source_params)
    result = moving_to(source_params)
    rospy.loginfo(utils.PC.BLUE + "[CRAM] Navigation done. result: " + str(result))
    # get values
    # get position of where robot should look at
    if param_json.get('Item') is not None:
        item = param_json.get('Item').get('value')
        # item = 'cup_small' # CHANGE add fancy filtering
    if param_json.get('Source') is not None:
        source = param_json.get('Source').get('value')
    if param_json.get('SourceRoom') is not None:
        source_room = param_json.get('SourceRoom').get('value')
    # get link of pose you went to
    if result is not None:
        # remove prefix for bullet
        look_at_link = result.get('Item').get('link')
        # get lookup pose
        look_at_pose = utils.tf_l.lookupTransform(target_frame='map',
                                                  source_frame=look_at_link,
                                                  time=rospy.get_rostime())
    # get link of pose you went to
    # 2. look at item and pick up
    pick_up_retries = 3
    grasped_bool, grasp, found_object = None, None, None
    while pick_up_retries > 0 and found_object is None:
        if item is not None and look_at_link is not None and grasped_bool is None:
            try:
                gasped_bool, grasp, found_object = plans.process_pick_up_objects(obj_type=item,
                                                                                 obj_types_dict=obj_types_dict,
                                                                                 link=utils.remove_prefix(look_at_link,
                                                                                                          'iai_kitchen/'),
                                                                                 look_pose_given=look_at_pose,
                                                                                 environment_raw=sd.environment_raw,
                                                                                 giskardpy=sd.giskard,
                                                                                 gripper=sd.gripper,
                                                                                 talk=nlp.tts,
                                                                                 lt=sd.lt,
                                                                                 robot_description=sd.robot_description,
                                                                                 BulletWorld=sd.world,
                                                                                 grasp_listener=sd.grasp_listener)
            except TypeError:
                rospy.logerr("[CRAM] Could not pick up object. Will retry...")
                sing_my_angel_of_music("Something went wrong during the pick up process. I will try again.")
                return grasped_bool, grasp, found_object
        pick_up_retries -= 1
        if found_object is not None:
            name = found_object_name = found_object.bullet_world_object.name.replace('_', ' ')
            found_object_name = re.sub(r'\d+', '', name)
            rospy.loginfo(utils.PC.BLUE + f"[CRAM] found object of type {name} and picked it up")
            pick_up_retries = 0
            object_in_hand = found_object
            return grasped_bool, grasp, found_object
        break
    else:
        if found_object:
            sing_my_angel_of_music("[CRAM] picked up object.")
            return found_object
        else:
            sing_my_angel_of_music("[CRAM]I am sorry. I seem to not have found the requested item.")
            return None


def placing(param_json):  # works testing
    global object_in_hand
    destination_link = None
    if not object_in_hand:
        rospy.logerr("[CRAM] I have no object in my hand so I cannot place :(")
        return None
    sing_my_angel_of_music("I'm going to do a placing action")
    # move to playing destination
    result = moving_to(param_json)
    if result is None:
        rospy.logerr("[CRAM] Navigation failed to Destination location.")
        return None
    obj_type = object_in_hand.type
    # match with knowrob
    # PLACE WHERE HUMAN TOLD U
    if param_json.get('Destination'):
        destination_link = utils.remove_prefix(result.get('Item').get('link'), 'iai_kitchen/')

    # PLACE at PREDEFINED LOCATION FROM KNOWROB
    if obj_type in perc_to_know.keys():
        knowrob_iri = perc_to_know.get('type')
        item_location = knowrob.get_predefined_destination_item_location(knowrob_iri)  # CHANGE check if this works
        destination_link = utils.remove_prefix(item_location.get('Item').get('link'), 'iai_kitchen/')

    result = plans.place(object=object_in_hand, grasp='front', link=destination_link, giskard=sd.giskard, talk=nlp.tts,
                         robot_description=sd.robot_description, lt=sd.lt, environment_raw=sd.environment_raw,
                         environment_desig=sd.environment_desig,
                         gripper=sd.gripper, world=sd.world, robot_desig=sd.robot_desig)

    if result:
        sing_my_angel_of_music("done with placing")
        object_in_hand = None
        return True


# also finding + searching
# Source
def looking_for(param_json):  # WIP TODO
    #sing_my_angel_of_music("in looking for plan")
    #robokudo.init_robokudo()
    rospy.loginfo("Looking For: " + str(param_json))
    # get vars
    item, person, furniture, room, result = None, None, None, None, None
    if param_json.get('Source') is not None:
        # ensure furniture obj exists
        furniture = param_json.get('Source').get('value').lower()
        if knowrob.check_existence_of_instance(snakecase(furniture)):
            rospy.loginfo(f"[CRAM] found instance of furniture item " + furniture)
            furniture = snakecase(furniture)
        # if instance does not exist,check if class does ?
        elif knowrob.check_existence_of_class(furniture):
            rospy.loginfo("[CRAM] found class of furniture item " + furniture)
            furniture_class = knowrob.check_existence_of_class(furniture)[0].get('Class')
            furniture = False
        elif utils.obj_dict.get(furniture) is not None:
            furniture = utils.obj_dict.get(furniture)
            rospy.loginfo("[CRAM] Fallback: of furniture item found in list. item : " + furniture)
        else:
            rospy.loginfo("[CRAM] could not find furniture item " + furniture)
            sing_my_angel_of_music(f"I am sorry. I don't know where {furniture} is.")
            return None

    if param_json.get('SourceRoom') is not None:
        # ensure room exists
        if knowrob.rooms.get(snakecase(param_json.get('SourceRoom').get('value').lower())) is not None:
            room = param_json.get('SourceRoom').get('value').lower()
        else:
            room = 'arena'

    if param_json.get('Item') is not None:
        item = param_json.get('Item').get('value')
        rospy.loginfo("[CRAM] looking for item: " + item)

    sing_my_angel_of_music("I'm going look for the " + str(item))
    # go to location which human said to look for items
    if furniture and room:
        rospy.loginfo("[CRAM] going to furniture in room")
        source_params = {'Source': param_json.get('Source'), 'SourceRoom': param_json.get('SourceRoom')}
        source_params = utils.remap_source_to_destination(source_params)
        result = moving_to(source_params)
    elif room:
        rospy.loginfo("[CRAM] going to room")
        source_params = {'SourceRoom': param_json.get('SourceRoom')}
        source_params = utils.remap_source_to_destination(source_params)
        result = moving_to(source_params)
    elif furniture:
        rospy.loginfo("[CRAM] going to furniture")
        source_params = {'Source': param_json.get('Source')}
        source_params = utils.remap_source_to_destination(source_params)
        result = moving_to(source_params)
    elif item:
        rospy.loginfo("[CRAM] going to item")
        # try anyway? get default location of the item
        # check if item is known
        if " " in item:  # ensure snake case if a space is present
            item = snakecase(item)

        if item:
            # ensure item exists
            rospy.loginfo("[CRAM] looking for item: " + item)
            result = knowrob.get_predefined_source_item_location_iri(utils.obj_dict.get(item))  #FIX THIS
            if result is None:
                result = knowrob.get_predefined_source_item_location_name(item)
            rospy.loginfo("[CRAM] result: " + str(result))
            if result:
                navi.go_to_pose(result[0].get('Item').get('pose'))
            else:
                rospy.logerr("[CRAM] Could not find predefined location for item.")
                return None

    # assume navigation was successful, and robot is in a perceiving pose...
    # point head at correct location
    if result:
        perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
        if sd.with_real_robot:
            plans.pakerino(config=perceive_conf)
        look_at_link = result.get('Item').get('link')  # this failed  at RoboCup CHANGE
        look_at_pose = utils.tf_l.lookupTransform(target_frame='map',
                                                  source_frame=look_at_link,
                                                  time=rospy.get_rostime())
        if sd.with_real_robot:
            sd.giskard.move_head_to_pose(look_at_pose)
    # perceive
    objects = robokudo.ask_robokudo_for_all_objects()

    for obj in objects:
        if obj.type == item:
            rospy.loginfo("[CRAM] found object of type " + item)
            found_obj = obj
            return found_obj
    # TODO return to person?


def transporting(param_json):
    # TODO add Saying what the robot does
    sing_my_angel_of_music("I'm going to do a transporting action")
    global me_pose
    person = None
    if param_json.get('BeneficiaryRole').get('value') == 'me':  # TODO handle other people too
        # save current pose
        rospy.loginfo("[CRAM] tf lookup: ")

        me_pose = utils.lookup_transform(target_frame='map', source_frame='base_link', tf_listener=utils.tf_l)
        me_pose = PoseStamped(me_pose[0], me_pose[1])
        rospy.loginfo("[CRAM] me pose: " + str(me_pose))
        # alternatively, try to memorize the human person from perception? ... uff
        person = 'me'
    # pick up item
    result = pick_up(param_json)
    rospy.loginfo("[CRAM] pick up result: " + str(result))
    if result is None:
        return None
    if person == 'me':
        navi.go_to_pose(me_pose)
        sing_my_angel_of_music("Here is your object. Please take it from me in 3.. 2.. 1.")
        sd.gripper.pub_now('open')
    else:
        placing(param_json)

# TODO add this to plan.
#    DetectAction(technique='human').resolve().perform()
#    giskardpy.move_head_to_human()
#    giskardpy.cancel_all_goals() OR giskardpy.cancel_all_called_goals() to stop



def arranging(param_json):
    sing_my_angel_of_music("I am sorry. I do not know how to arrange things yet.")
    rospy.loginfo("arranging: " + str(param_json))


# count obj or person
def count(param_json):
    sing_my_angel_of_music("I am sorry, I do not know how to count items yet.")
    rospy.loginfo("count: " + str(param_json))


def cleaning(param_json):
    sing_my_angel_of_music("I am sorry, I fo not know how to clean up yet. ")
    rospy.loginfo("cleaning: " + str(param_json))


def guide(param_json):
    sing_my_angel_of_music("please follow me")
    moving_to(param_json)
    sing_my_angel_of_music("I have arrived. thank you for following me. ")
    rospy.loginfo("guide: " + str(param_json))


def accompany(param_json):
    sing_my_angel_of_music("I am sorry. I do now know how to accompany yet.")
    rospy.loginfo("accompany: " + str(param_json))


# --- utils plans ---
def track_human():  # WIP
    sing_my_angel_of_music("in track human plan")
    # look for a human
    DetectAction(technique='human').resolve().perform()
    # look at guest and introduce
    HeadFollowAction('start').resolve().perform()


def assert_speaker(param_json):  # todo
    sing_my_angel_of_music("in assert speaker plan")
    rospy.loginfo("assert speaker: " + str(param_json))


def assert_preferece(param_json):  # todo
    sing_my_angel_of_music("in assert preference plan")
    rospy.loginfo("assert preference: " + str(param_json))


def greet(param_json):
    sing_my_angel_of_music("I'm going to greet a person")
    moving_to(param_json)
    # TODO add looking for human
    sing_my_angel_of_music("Hello and welcome to the RoboCup 2024 in Eindhoven. I wish you a great day and lots of fun.")
    rospy.loginfo("greeting")


def describe(param_json):
    sing_my_angel_of_music("I am sorry. I do not know how to describe someone yet.")
    rospy.loginfo("describe: " + str(param_json))


def offer(param_json):
    sing_my_angel_of_music("I am sorry. I do not know how to offer something yet.")
    rospy.loginfo("offer: " + str(param_json))


def follow(param_json):
    sing_my_angel_of_music("Sorry. I am unsure how to follow you. ")
    rospy.loginfo("follow: " + str(param_json))


# DONE --- telling plans -------------------------------------------------------------------------------------
def tell_time(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_time())
    rospy.loginfo("[TELL] time: " + str(param_json))


def tell_day(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_date())
    rospy.loginfo("tell day: " + str(param_json))


def tell_tomorrow(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_tomorrow_day_and_date())
    rospy.loginfo("tell tomorrow: " + str(param_json))


def tell_team_name(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_team_name())
    rospy.loginfo("tell team name: " + str(param_json))


def tell_team_country(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_team_country())
    rospy.loginfo("tell team country: " + str(param_json))


def tell_team_affiliation(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_team_affiliation())
    rospy.loginfo("tell team affiliation: " + str(param_json))


def tell_day_of_week(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_day_of_week())
    rospy.loginfo("tell day of week: " + str(param_json))


def tell_birthday(param_json):  # CHANGE?
    sing_my_angel_of_music(llp_tell_stuff.say_birthday())
    rospy.loginfo("tell birthday: " + str(param_json))


def tell_from(param_json):  # DONE
    sing_my_angel_of_music(llp_tell_stuff.say_from())
    rospy.loginfo("tell from: " + str(param_json))


def tell_something(param_json):  # CHANGE adapt tp cute toya stuff
    sing_my_angel_of_music(llp_tell_stuff.say_something())
    rospy.loginfo("tell something: " + str(param_json))


# DONE --- END telling plans -------------------------------------------------------------------------------------
# WIP ---- Eindhoven Specific Questions ------------------------------
def eindhoven_mountain(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_mountain())
    rospy.loginfo("say eindhoven mountain: " + str(param_json))


def eindhoven_painter(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_painter())
    rospy.loginfo("say eindhoven painter: " + str(param_json))


def eindhoven_lake(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_lake())
    rospy.loginfo("say eindhoven lake: " + str(param_json))


def eindhoven_baron(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_baron())
    rospy.loginfo("say eindhoven baron: " + str(param_json))


def eindhoven_created(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_created())
    rospy.loginfo("say eindhoven mountain: " + str(param_json))


def eindhoven_people(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_people())
    rospy.loginfo("say eindhoven people: " + str(param_json))


def eindhoven_mascot(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_mascot())
    rospy.loginfo("say eindhoven mascot: " + str(param_json))


def eindhoven_lowest_point(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_lowest_point())
    rospy.loginfo("say eindhoven lowest point: " + str(param_json))


def eindhoven_currency(param_json):
    sing_my_angel_of_music(llp_tell_stuff.say_eindhoven_currency())
    rospy.loginfo("say eindhoven currency: " + str(param_json))


# WIP ---- END Eindhoven Specific Questions ------------------------------

def prepare_for_commands():
    global move, instruction_point
    # wait for start signal (laser scan)
    # Create an instance of the StartSignalWaiter
    start_signal_waiter = StartSignalWaiter()
    # Wait for the start signal
    start_signal_waiter.wait_for_startsignal()
    # Once the start signal is received, continue with the rest of the script
    rospy.loginfo("Start signal received, now proceeding with tasks.")

    # TODO this should be looped for multiple tasks
    # go to initial point
    move.query_pose_nav(instruction_point)

# DEPRECTAED ---------------------------------------------------------------------------------------
# deprecated -> transporting
# def fetching(param_json):
#     global me_pose
#     # go to a target location, pick up object, bring it back
#     sing_my_angel_of_music("in fetching plan")
#     rospy.loginfo("fetching: " + str(param_json))
#     # BeneficiaryRole: target
#     if param_json.get('BeneficiaryRole').get('entity') == 'NaturalPerson':
#         # Goal is natural pearson. Means we need to HRI
#         # if person is 'me', save current pose and person name
#         if param_json.get('BeneficiaryRole').get('value') == 'me':  # or part of a list of names?
#             # save current pose
#             me_pose = utils.tf_l.lookupTransform(target_frame='map', source_frame='base_link',
#                                                  time=rospy.get_time())
#             # alternatively, try to memorize the human person from perception? ... uff
#             person = 'you'
#     # go to the destination to pick the obj
#     # --- Step 1 ---
#     # get the pose of 'PhysicalPlace' = 'Room' (if available) CHANGE (Potentially)
#     pick_up(param_json)
#     navi.go_to_pose(me_pose)
#     sing_my_angel_of_music("Here is your object. Please take it from me in 3.. 2.. 1.")
#     sd.gripper('open')
#
#     # perceive obj
