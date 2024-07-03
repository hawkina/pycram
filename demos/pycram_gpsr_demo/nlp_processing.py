import json
import rospy
from std_msgs.msg import String
import demos.pycram_gpsr_demo.setup_demo as setup_demo

nlp_pub = rospy.Publisher('/startListener', String, queue_size=10)
nlp_pub_test = rospy.Publisher('/nlp_test', String, queue_size=10)
nlp_sub = {}
response = ""
callback = False
doorbell = False
confirm = {}
todo_plans = []


# might be deprecated?
def data_cb(data):
    global response
    global callback
    global doorbell

    rospy.loginfo("in NLP callback")
    response = data.data.split(",")
    response.append("None")
    callback = True


# This is the function that processes/filters the NLP result
def intent_processing(msg):
    # convert result into json array for easier access. e.g. response["person-name"] would return 'me'
    global response, todo_plans, confirm
    response = msg.data.replace(", ", ",").split(",")
    print(response)
    if response[0] == "<CONFIRM>":
        if response[1] == "True":
            rospy.loginfo("[NLP] result: Yes")
            confirm = True
            return True
        else:
            rospy.loginfo("[NLP] result: No")
            confirm = False
            todo_plans = []
            return False
    else:
        # TODO replace with just json_loads
        response = msg.data.replace(": ", ":").replace("'", "\"")  # formatting for making json
        response = json.loads(response)
        todo_plans.append(response)
        rospy.loginfo("Got Data from NLP: " + str(todo_plans))
        response = "None"


# create a subscriber to the /nlp_out topic on which the result from NLP is published
def nlp_subscribe():
    global nlp_sub
    nlp_sub = rospy.Subscriber('nlp_out', String, intent_processing)
    rospy.loginfo("subscriber initialized: " + str(nlp_sub))


# delete the subscriber
def nlp_unsubscribe():
    global nlp_sub
    nlp_sub.unregister()
    rospy.loginfo("nlp unsubscribed: " + str(nlp_sub))


def test_nlp_string_based():
    global response, todo_plans
    rospy.loginfo("test nlp")
    test_string = "bring me the red apple from the kitchen."  # and then look for all red cups in the kitchen.
    rospy.loginfo(test_string)
    nlp_pub_test.publish(test_string)
    rospy.loginfo("ToDo plans: " + str(todo_plans))
    return todo_plans


# listen to speech from human
def nlp_listening():
    # connect to global vars
    global callback, response, nlp_sub, todo_plans
    # start actually listening
    nlp_pub.publish("start listening")  # initialize listening

    # setup_demo.sound_pub.publish_sound_request()  # beep
    rospy.loginfo("waiting for a message...")
    rospy.wait_for_message('nlp_out', String, timeout=20)

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

    setup_demo.tts.pub_now("I have understood: " + whole_sentence + ".")
    rospy.sleep(2)
    setup_demo.tts.pub_now("Please say yes or no, if this is correct or not after the beep.")
    rospy.sleep(3)
    nlp_listening()
    rospy.loginfo("[NLP] Confirmation was: " + str(confirm))
    if confirm:
        setup_demo.tts.pub_now("Okay. I will go do it.")
        confirm = 0
        return True
    else:
        setup_demo.tts.pub_now("I am sorry I didn't hear you correctly. Let's try again.")
        confirm = 0
        todo_plans = []
        return False


# --- THIS IS THE MAIN FUNCTION FOR NLP ---
def listen_to_commands():
    nlp_subscribe()
    global response, todo_plans
    setup_demo.tts.pub_now(" Please tell me what to do after the beep.")
    setup_demo.image_switch.pub_now(1)
    rospy.sleep(3)
    # setup_demo.sound_pub.publish_sound_request()
    temp = nlp_listening()
    rospy.loginfo("[NLP] temp: " + str(temp) + "todo_plans:" + str(todo_plans))
    if confirm_nlp_output(temp):
        rospy.loginfo("[CRAM] do stuff")
        return todo_plans
    else:
        listen_to_commands()

