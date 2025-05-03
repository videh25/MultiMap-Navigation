import rospy
import actionlib
from multimap_msgs.msg import MultiMapNavigationAction, MultiMapNavigationGoal
from geometry_msgs.msg import Pose

class MultiMapNavigationActionClient:
    def __init__(self, action_name='multimap_navigation'):
        # Initialize ROS node (do this only in main if using multiple nodes)
        rospy.loginfo("Initializing Multimap Navigation Action Client...")

        # Create the SimpleActionClient
        self.client = actionlib.SimpleActionClient(action_name, MultiMapNavigationAction)

        # Wait until the action server is ready
        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, ready to send goals.")

    def send_goal(self, map_name, local_pose):
        # Create a goal to send to the action server
        goal = MultiMapNavigationGoal()
        
        goal.map_id = map_name
        goal.pose.pose = local_pose
        goal.pose.header.frame_id = "map"

        rospy.loginfo(f"Sending navigation request to: map:{map_name} & {(local_pose.position.x, local_pose.position.y)}")
        self.client.send_goal(goal,
                              done_cb=self.done_callback,
                              active_cb=self.active_callback,
                              feedback_cb=self.feedback_callback)

        # Wait for the result
        self.client.wait_for_result()
        return self.client.get_result()

    def done_callback(self, status, result):
        rospy.loginfo(f"[DONE] Finished with status: {status}")
        if (result is None):
            rospy.logwarn("Action server returned none object for feedback")
        rospy.loginfo(f"[DONE] Result: {result.data}")  

    def active_callback(self):
        rospy.loginfo("[ACTIVE] Goal just went active.")

    def feedback_callback(self, feedback):
        rospy.loginfo(f"[FEEDBACK]: {feedback.current_action}")


if __name__ == '__main__':
    try:
        rospy.init_node('multimap_navigation_client')
        client = MultiMapNavigationActionClient()
        goal_pose = Pose()
        
        # ROOM 1 to ROOM 1
        goal_pose.position.x = -1.0
        goal_pose.position.y = 2.0
        goal_pose.position.z = 0.0
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0
        result = client.send_goal("room1", goal_pose)

        # ROOM 1 to ROOM 2
        goal_pose.position.x = 5.0
        goal_pose.position.y = 3.0
        goal_pose.position.z = 0.0
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0

        result = client.send_goal("room2", goal_pose)

        #ROOM 2 to ROOM 1
        goal_pose.position.x = -1.0
        goal_pose.position.y = 2.0
        goal_pose.position.z = 0.0
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0
        result = client.send_goal("room1", goal_pose)

        rospy.loginfo(f"Final result: {result.data}")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
