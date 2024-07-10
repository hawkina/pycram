import rospy
from pycram.designators.action_designator import *
from pycram.utilities.robocup_utils import StartSignalWaiter
from demos.pycram_gpsr_demo import perception_interface, llp_tell_stuff
from demos.pycram_gpsr_demo import knowrob_interface as knowrob
from demos.pycram_gpsr_demo import llp_navigation as navi
import demos.pycram_gpsr_demo.utils as utils
from demos.pycram_gpsr_demo.nlp_processing import sing_my_angel_of_music


from stringcase import snakecase


# these are all the high level plans, to which we map the NLP output.
# they should either connect to low level plans or be filled with data from knowledge


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
        rospy.loginfo("[CRAM] moving to person at furniture item in room")
        sing_my_angel_of_music(f"I will go to the {furniture} in {room} to look for a person.")
        # TODO
        # go to person at furniture item in room
    # Room + Person WIP---------------------------------------------------
    elif room and person:  # DONE one value
        # go to the person in the room
        rospy.loginfo("[CRAM] moving to person in room")
        sing_my_angel_of_music(f"I will go to the {room} and look for the person.")
        navi.go_to_room_middle(room)
        # look for person in room TODO ------------------------------------------

    # Furniture + Person WIP----------------------------------------------
    elif furniture and person: # TODO - remove duplicates
        rospy.loginfo("[CRAM] moving to person at furniture item")
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
                nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_name=furniture, furniture_iri=furniture)  # change might get removed
            else:
                nav_poses = knowrob.get_nav_poses_for_furniture_item(furniture_iri=furniture_class)
        # go to person at furniture item TODO ----------------------------------------------------------

    elif room and furniture:  # DONE has multiple poses TODO remove duplicate code
        rospy.loginfo("[CRAM] moving to furniture item in room")
        sing_my_angel_of_music(f"I will go to the {furniture} in {room}.")
        if furniture:
            rospy.loginfo(f"[CRAM] found instance of furniture item " + str(furniture))
            nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_name=furniture)
        # if instance does not exist,check class?
        elif furniture_class:  # maybeh make this an and?
            if furniture == furniture_class:
                #  matched knowledge and nlp
                nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_name=furniture)  # change might get removed
            else:
                nav_poses = knowrob.get_nav_poses_for_furniture_item(room=room, furniture_iri=furniture_class)
        # go to furniture item in room
        if nav_poses is not None and nav_poses != []:
            # go to furniture item
            navi.go_to_pose(nav_poses[0].get('pose'))

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
            rospy.logwarn("in going to room")
            # go to furniture item
            navi.go_to_pose(nav_poses[0].get('pose'))

        else:
            rospy.loginfo("[CRAM] could not find furniture item " + str(param_json.get('Destination').get('value')))
            sing_my_angel_of_music("I am sorry. I don't know where that is.")
            return None  # abort mission

        # drive
        navi.go_to_pose(nav_poses[0].get('pose'))
        # rospy.loginfo("[CRAM] moving to furniture item")
        # go to furniture item

    elif room:  # DONE
        # has only one pose
        sing_my_angel_of_music(f"I am going to {room}.")
        rospy.loginfo("[CRAM] moving to room")
        # go to room
        navi.go_to_room_middle(snakecase(param_json.get('DestinationRoom').get('value').lower()))
    # fallback
    else:
        sing_my_angel_of_music(f"I am sorry. I don't know where that is.")
        rospy.logerr("[CRAM]: MovingTo plan failed. No valid parameters found.")


# also finding + searching
def looking_for(param_json):  # WIP
    sing_my_angel_of_music("in looking for plan")
    rospy.loginfo("Looking For: " + str(param_json))
    physical_place, physical_artifact = None, None
    # step 0: go to the requested room - if it was mentioned explicitly OR
    if param_json.get('Location') and param_json.get('Location').get('entity') == 'PhysicalPlace':
        physical_place = snakecase(param_json.get('Location').get('value'))
    if param_json.get('Destination') and param_json.get('Destination').get('entity') == 'PhysicalArtifact':
        physical_artifact = param_json.get('Destination').get('value')
    # make msg for perception
    rk_msg = perception_interface.make_robokudo_obj_msg(param_json.get('Item'))
    # get navigation pose from knowrob depending on what info is known

    # step 1: go to the furniture/surface item that got mentioned and look on it for the specified obj


def picking(param_json):
    sing_my_angel_of_music("in picking up plan")
    rospy.loginfo("Picking: " + str(param_json))


def placing(param_json):
    sing_my_angel_of_music("in placing plan")
    rospy.loginfo("Place: " + str(param_json))


def fetching(param_json):
    # go to a target location, pick up object, bring it back
    sing_my_angel_of_music("in fetching plan")
    rospy.loginfo("fetching: " + str(param_json))
    # BeneficiaryRole: target
    if param_json.get('BeneficiaryRole').get('entity') == 'NaturalPerson':
        # Goal is natural pearson. Means we need to HRI
        # if person is 'me', save current pose and person name
        if param_json.get('BeneficiaryRole').get('value') == 'me':  # or part of a list of names?
            # save current pose
            me_pose = utils.tf_l.lookupTransform(target_frame='map', source_frame='base_link',
                                                             time=rospy.get_time())
            # alternatively, try to memorize the human person from perception? ... uff
            person = 'you'
    # go to the destination to pick the obj
    # --- Step 1 ---
    # get the pose of 'PhysicalPlace' = 'Room' (if available) CHANGE (Potentially)
    if param_json.get('Destination').get('entity') == 'PhysicalPlace':
        physical_place = param_json.get('Destination').get('value')  # CHANGE this actually should be Origin

    # perceive obj
    # TODO WIP


def cleaning(param_json):
    sing_my_angel_of_music("in cleaning plan")
    rospy.loginfo("cleaning: " + str(param_json))


def transporting(param_json):
    sing_my_angel_of_music("in transporting plan")
    rospy.loginfo("transporting: " + str(param_json))
    # from BeneficiaryRole


def arranging(param_json):
    sing_my_angel_of_music("in arranging plan")
    rospy.loginfo("arranging: " + str(param_json))


# count obj or person
def count(param_json):
    sing_my_angel_of_music("in counting plan")
    rospy.loginfo("count: " + str(param_json))


def guide(param_json):
    sing_my_angel_of_music("in guiding plan")
    rospy.loginfo("guide: " + str(param_json))


def accompany(param_json):
    sing_my_angel_of_music("in accompany plan")
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
    sing_my_angel_of_music("in greeting plan")
    rospy.loginfo("greeting")


def describe(param_json):
    sing_my_angel_of_music("in describe plan")
    rospy.loginfo("describe: " + str(param_json))


def offer(param_json):
    sing_my_angel_of_music("in offer plan")
    rospy.loginfo("offer: " + str(param_json))


def follow(param_json):
    sing_my_angel_of_music("in follow plan")
    rospy.loginfo("follow: " + str(param_json))


# WIP --- telling plans -------------------------------------------------------------------------------------
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


# WIP --- END telling plans -------------------------------------------------------------------------------------
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
