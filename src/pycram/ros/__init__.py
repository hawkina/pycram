import rospy
from .ros_tools import is_master_online

# Check is for sphinx autoAPI to be able to work in a CI workflow
if is_master_online():
    print("initializing node...")
    rospy.init_node("pycram")
