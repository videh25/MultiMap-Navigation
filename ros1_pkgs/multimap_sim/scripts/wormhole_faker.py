import sqlite3
import os
import cv2
import yaml
import numpy as np

import rospy
from multimap_msgs.srv import RequestTeleportation, RequestTeleportationResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap, SetMapRequest
from nav_msgs.srv import LoadMap, LoadMapRequest
from std_srvs.srv import Empty, EmptyRequest

def fetch_map_data_as_dict(db_path):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT * FROM map_data")
    rows = cursor.fetchall()

    # Get column names
    columns = [description[0] for description in cursor.description]

    # Index of 'map_id' column
    map_id_index = columns.index('map_id')

    # Build the dictionary
    result = {}
    for row in rows:
        map_id = row[map_id_index]
        value_dict = {
            columns[i]: row[i]
            for i in range(len(columns)) if i != map_id_index
        }
        result[map_id] = value_dict

    conn.close()
    return result

class OccupancyGridCreator:
    def load_map_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_metadata = yaml.safe_load(f)
        return map_metadata

    def load_map_image(self, image_path, negate, occupied_thresh, free_thresh):
        img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

        if img is None:
            raise IOError(f"Failed to load image file: {image_path}")

        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img = cv2.flip(img, 0)  # Flip vertically to match ROS coordinates

        if negate:
            img = 255 - img

        img = img.astype(np.float32) / 255.0

        grid = np.empty_like(img, dtype=np.int8)
        grid.fill(-1)  # Unknown

        grid[img >= occupied_thresh] = 100  # Occupied
        grid[img <= free_thresh] = 0        # Free

        return grid

    def create_occupancy_grid(self, grid, map_info):
        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.info.resolution = map_info['resolution']
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]

        origin = map_info['origin']  # [x, y, yaw]
        msg.info.origin = Pose()
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0  # No rotation

        msg.data = grid.flatten(order='C').tolist()  # row-major

        return msg
    
    def load(self, yaml_path):
        map_info = self.load_map_yaml(yaml_path)

        image_path = map_info['image']

        grid = self.load_map_image(image_path,
                            negate=map_info.get('negate', 0),
                            occupied_thresh=map_info.get('occupied_thresh', 0.65),
                            free_thresh=map_info.get('free_thresh', 0.196))
        
        return self.create_occupancy_grid(grid, map_info)

class WormholeFaker:
    def __init__(self):
        self.map_data = fetch_map_data_as_dict(os.path.join(os.environ.get("MULTIMAP_STORAGE_PATH"), "wormholes.db"))
        
        rospy.loginfo("Waiting for service: /gazebo/set_model_state")
        rospy.wait_for_service('/gazebo/set_model_state')

        rospy.loginfo("Waiting for service: /change_map")
        rospy.wait_for_service('/change_map')

        # rospy.loginfo("Waiting for service: move_base/clear_unknown_space")
        # rospy.wait_for_service('move_base/clear_unknown_space')
        
        rospy.loginfo("Waiting for service: move_base/clear_costmaps")
        rospy.wait_for_service('move_base/clear_costmaps')
        
        rospy.loginfo("Waiting for service: set_map")
        rospy.wait_for_service('set_map')

        rospy.loginfo("All services initialised!")

        self.gazebo_model_set_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.change_map_client = rospy.ServiceProxy('/change_map', LoadMap)
        # self.move_base_clear_space_client = rospy.ServiceProxy('move_base/clear_unknown_space', Empty)
        self.move_base_clear_costmap_client = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        self.amcl_reset_client = rospy.ServiceProxy('set_map', SetMap)

        self.multimap_storage_path = os.environ.get("MULTIMAP_STORAGE_PATH")
        self.teleport_server = rospy.Service('teleport', RequestTeleportation, self.request_cb)

    def request_cb(self, req):
        goal_map = req.goal_map_id
        rospy.loginfo(f"Received teleport request to {goal_map}")
        
        # Set Gazebo Model
        set_model_req = SetModelStateRequest()
        set_model_req.model_state.model_name = "turtlebot3_waffle"
        set_model_req.model_state.pose.position.x = self.map_data[goal_map]['x']
        set_model_req.model_state.pose.position.y = self.map_data[goal_map]['y']
        set_model_req.model_state.pose.position.z = 0.0
        set_model_req.model_state.pose.orientation.x = self.map_data[goal_map]['qx']
        set_model_req.model_state.pose.orientation.y = self.map_data[goal_map]['qy']
        set_model_req.model_state.pose.orientation.z = self.map_data[goal_map]['qz']
        set_model_req.model_state.pose.orientation.w = self.map_data[goal_map]['qw']
        set_model_res = self.gazebo_model_set_client(set_model_req)
        if (not set_model_res.success):
            rospy.logerror("Gazebo Set Model Failed")

        # Call Map server change map
        new_map_url = os.path.join(self.multimap_storage_path, goal_map + ".yaml")
        change_map_req = LoadMapRequest()
        change_map_req.map_url = new_map_url
        self.change_map_client(change_map_req)

        # Reset AMCL with new pose
        amcl_req = SetMapRequest()
        amcl_req.map = OccupancyGridCreator().load(new_map_url)
        amcl_req.initial_pose.header.frame_id = "map"
        amcl_req.initial_pose.pose.pose = set_model_req.model_state.pose
        self.amcl_reset_client(amcl_req)

        # Reset move_base {clear_unknown_space, clear_costmaps}
        # mb_req = EmptyRequest()
        # self.move_base_clear_space_client(mb_req)
        # self.move_base_clear_costmap_client(mb_req)
        
        return RequestTeleportationResponse(True)
    
if __name__ == "__main__":
    rospy.init_node("wormhole_faker")
    wmf = WormholeFaker()
    rospy.spin()
