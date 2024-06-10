import rospy
from std_msgs.msg import String

from pycram.utilities.robocup_utils import TextToSpeechPublisher, SoundRequestPublisher

nlp_pub = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False
doorbell = False
tts = TextToSpeechPublisher()
sound_pub = SoundRequestPublisher()

def data_cb(data):
    global response
    global callback
    global doorbell

    response = data.data.split(",")
    response.append("None")
    callback = True


def nlp_listening():
    # connect to global vars
    global callback
    global response
    nlp_sub = rospy.Subscriber("nlp_out", String, data_cb)
    # start actually listening
    nlp_pub.publish("start listening")

    #wait for response
    while not callback:
        rospy.sleep(1)
    callback = False

    # process output from NLP
    # TODO adapt to GPSR
    if response[0] == "<GUEST>":
        # success a name and intent was understood
        if response[1] != "<None>":
            tts.publish_text("I am sorry, I didn't quite get that. Could you please repeat your command?")
            # guest1.set_drink(response[2])
            # rospy.sleep(1)
            # guest1.set_name(name_confirm(response[1]))

    list_of_intents = response

    return list_of_intents
