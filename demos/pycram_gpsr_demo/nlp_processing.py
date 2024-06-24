import json
import rospy
from std_msgs.msg import String
import setup_demo

nlp_pub = rospy.Publisher('/startListener', String, queue_size=10)
nlp_pub_test = rospy.Publisher('/nlp_test', String, queue_size=10)
nlp_sub = rospy.Subscriber('nlp_out', String)
response = ""
callback = False
doorbell = False


def data_cb(data):
    global response
    global callback
    global doorbell

    response = data.data.split(",")
    response.append("None")
    callback = True


def intent_processing(msg):
    # convert result into json array for easier access. e.g. response["person-name"] would return 'me'
    global response, todo_plans
    response = msg.data.replace(": ", ":").replace("'", "\"")  # formatting for making json
    response = json.loads(response)
    todo_plans = response
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


def test_nlp():
    global response, todo_plans
    rospy.loginfo("test nlp")
    test_string = "Bring me the apple from the kitchen table."  # and then look for all red cups in the kitchen.
    rospy.loginfo(test_string)
    nlp_pub_test.publish(test_string)
    rospy.loginfo("ToDo plans: " + str(todo_plans))
    return todo_plans


def nlp_listening():
    # connect to global vars
    global callback, response, nlp_sub, todo_plans
    nlp_subscribe()
    # start actually listening
    nlp_pub.publish("start listening")  # initialize listening

    setup_demo.sound_pub.publish_sound_request()  # beep
    rospy.loginfo("waiting for a message...")
    rospy.wait_for_message('nlp_out', String, timeout=10)

    # process output from NLP
    rospy.loginfo("message received. todo_plans: " + str(todo_plans))
    return todo_plans
