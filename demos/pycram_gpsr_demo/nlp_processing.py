import json
import rospy
from std_msgs.msg import String
from threading import Condition
from pycram.utilities.robocup_utils import TextToSpeechPublisher

haveNLPOutput = Condition()
tts = TextToSpeechPublisher()
nlp_pub = rospy.Publisher('/startListener', String, queue_size=10)
nlp_pub_test = rospy.Publisher('/nlp_test', String, queue_size=10)
nlp_sub = {}
talk_sub = {}
response = ""
callback = False
doorbell = False
confirm = {}
todo_plans = []
currentSpeech = ""
stoppedSpeaking = Condition()
canSpeak = True
canListen = False
canDisplay = False


def what_am_i_saying(msg):
    global currentSpeech
    if ("" != currentSpeech) and ("" == msg.data):
        with stoppedSpeaking:
            stoppedSpeaking.notify_all()
    currentSpeech = msg.data


def sing_my_angel_of_music(text):
    global tts, canSpeak, stoppedSpeaking
    rospy.loginfo("SPEAKING: " + text)
    #tts.pub_now(text)
    # handled on other side now
    if canSpeak:
        tts.pub_now(text)
        with stoppedSpeaking:
            stoppedSpeaking.wait()


# This is the function that processes/filters the NLP result
def intent_processing(msg):
    # convert result into json array for easier access. e.g. response["person-name"] would return 'me'
    global response, todo_plans, confirm, haveNLPOutput
    response = json.loads(msg.data)
    if response["intent"] == "Affirm":
        rospy.loginfo("[NLP] confirmation result: Yes")
        confirm = True
    elif response["intent"] == "Deny":
        rospy.loginfo("[NLP] confirmation result: No")
        confirm = False
        todo_plans = []
    else:
        print("XXXXXXXXXXXXXXXXXXXXX ", len(todo_plans))
        todo_plans.append(response)
        rospy.loginfo("Got Data from NLP: " + str(todo_plans))
        response = "None"
    with haveNLPOutput:
        rospy.loginfo("NLP Callback sent notification.")
        haveNLPOutput.notifyAll()
    #if response["intent"] in ["Agreement", "Disagreement"]:
    #    return confirm


# create a subscriber to the /nlp_out topic on which the result from NLP is published
def nlp_subscribe():
    global nlp_sub, talk_sub
    nlp_sub = rospy.Subscriber('nlp_out', String, intent_processing)
    talk_sub = rospy.Subscriber('talking_sentence', String, what_am_i_saying)
    rospy.loginfo("subscriber initialized: " + str(nlp_sub))


# delete the subscriber
def nlp_unsubscribe():
    global nlp_sub
    nlp_sub.unregister()
    rospy.loginfo("nlp unsubscribed: " + str(nlp_sub))


def test_nlp_string_based(text=None):
    if text is None:
        text = "bring me the red apple from the kitchen."  # and then look for all red cups in the kitchen.
    global response, todo_plans
    rospy.loginfo("test nlp")
    rospy.loginfo(text)
    nlp_pub_test.publish(text)
    rospy.loginfo("ToDo plans: " + str(todo_plans))
    return todo_plans


# listen to speech from human
def nlp_listening():
    # connect to global vars
    global callback, response, nlp_sub, todo_plans
    # start actually listening
    if canListen:
        nlp_pub.publish("start listening")  # initialize listening

    # setup_demo.sound_pub.publish_sound_request()  # beep
    rospy.loginfo("waiting for a message...")
    #rospy.wait_for_message('nlp_out', String, timeout=20)
    with haveNLPOutput:
        haveNLPOutput.wait(30)
    rospy.loginfo("NLP data notification received.")
    # process output from NLP
    rospy.loginfo("message received. todo_plans: " + str(todo_plans))
    return todo_plans


# todo: are there special functions for confirmation?
def confirm_nlp_output(received_output):
    global confirm, todo_plans
    # concatenate output into a whole sentence again
    whole_sentence = ''
    for sentence in received_output:
        whole_sentence += sentence['sentence']
        whole_sentence += ' '

    sing_my_angel_of_music("I have understood: " + whole_sentence + ".")
    sing_my_angel_of_music("Please say yes or no, if this is correct or not after the beep.")
    nlp_listening()
    rospy.loginfo("[NLP] Confirmation was: " + str(confirm))
    if confirm:
        sing_my_angel_of_music("Okay.")
        confirm = 0
        return True
    else:
        sing_my_angel_of_music("I am sorry I didn't hear you correctly. Let's try again.")
        confirm = 0
        todo_plans = []
        return False


# --- THIS IS THE MAIN FUNCTION FOR NLP ---
def listen_to_commands():
    #nlp_subscribe() done during setup
    global response, todo_plans
    while True:
        if canDisplay:
            #setup_demo.image_switch.pub_now(1) # CHANGE or FIX
            rospy.loginfo("[CRAM] displaying")
        sing_my_angel_of_music("Please tell me what to do after the beep.")
        # setup_demo.sound_pub.publish_sound_request()
        temp = nlp_listening()
        rospy.loginfo("[NLP] temp: " + str(temp) + "todo_plans:" + str(todo_plans))
        if confirm_nlp_output(temp):
            break
    rospy.loginfo("[CRAM] do stuff")
    nlp_unsubscribe()
    return todo_plans
