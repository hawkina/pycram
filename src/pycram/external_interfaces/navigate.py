import math

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

from pycram.fluent import Fluent




class PoseNavigator():
    def __init__(self):
        rospy.loginfo("move_base init")
        global move_client
        self.client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        rospy.loginfo("Waiting for move_base ActionServer")
        if self.client.wait_for_server():
            rospy.loginfo("Done")
        self.pub = rospy.Publisher('goal', PoseStamped, queue_size=10, latch=True)
        self.toya_pose = None
        self.goal_pose = None
        self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)
        rospy.loginfo("move_base init construct done")

    def toya_pose_cb(self, msg):
        self.toya_pose = msg.pose.pose.position
        rospy.sleep(0.1)

    def interrupt(self):
        print("interrupting hehe")
        self.client.cancel_all_goals()

    def pub_now(self, navpose: PoseStamped, interrupt_bool: bool = True) -> bool:
        rospy.logerr("New implementation!")

        self.goal_pose = navpose
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 0
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = navpose.pose

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            return False

        rospy.loginfo(f"Publishing navigation pose")
        rospy.loginfo("Waiting for subscribers to connect...")
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(navpose)

        near_goal = False
        rospy.loginfo("Pose was published")
        if self.toya_pose is not None:
            dis = math.sqrt((self.goal_pose.pose.position.x - self.toya_pose.x) ** 2 +
                            (self.goal_pose.pose.position.y - self.toya_pose.y) ** 2)
            rospy.loginfo("Distance to goal: " + str(dis))
            if dis < 0.15:
                rospy.logwarn("Near Pose")
                if interrupt_bool:
                    self.interrupt()
                return True
            # else:
            #     rospy.logerr("pose is not near goal, try recalling")
            #     return False
