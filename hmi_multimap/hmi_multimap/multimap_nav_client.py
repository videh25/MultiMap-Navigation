import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException

from multimap_msgs.action import MultiMapNavigation
from geometry_msgs.msg import Pose

class MultiMapNavigationActionClient(Node):
    def __init__(self, action_name='multimap_navigation'):
        # Initialize ROS node (do this only in main if using multiple nodes)
        super().__init__('multimap_navigation_client')
        self.get_logger().info("Initializing Multimap Navigation Action Client...")

        # Create the SimpleActionClient
        self.client = ActionClient(self, MultiMapNavigation, action_name)

        # Wait until the action server is ready
        self.get_logger().info("Waiting for action server to start...")
        self.client.wait_for_server()
        self.get_logger().info("Action server started, ready to send goals.")

    def send_goal(self, map_name, local_pose):
        # Create a goal to send to the action server
        goal = MultiMapNavigation.Goal()
        
        goal.map_id = map_name
        goal.pose.pose = local_pose
        goal.pose.header.frame_id = "map"

        self.get_logger().info(f"Sending navigation request to: map:{map_name} & {(local_pose.position.x, local_pose.position.y)}")
        result = self.client.send_goal(goal, feedback_callback=self.feedback_callback)

        return result

    def done_callback(self, status, result):
        self.get_logger().info(f"[DONE] Finished with status: {status}")
        if (result is None):
            self.get_logger().warn("Action server returned none object for feedback")
        self.get_logger().info(f"[DONE] Result: {result.data}")  

    def active_callback(self):
        self.get_logger().info("[ACTIVE] Goal just went active.")

    def feedback_callback(self, feedback):
        self.get_logger().info(f"[FEEDBACK]: {feedback.current_action}")


def main():
    try:
        rclpy.init()
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

        if result is not None:
            client.get_logger().info(f"Final result: {result.data}")

    except ROSInterruptException:
        client.get_logger().error(f"ROS node interrupted.")
