import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import sqlite3

import rospy
import tf

class RobotMappingGUI:
    def __init__(self, master):
        self.master = master
        master.title("Robot Multi-Map Mapping Tool")

        # Label and Entry for map name
        self.label = tk.Label(master, text="Map Name:")
        self.label.grid(row=0, column=0, padx=10, pady=10, sticky="e")

        self.map_name_entry = tk.Entry(master, width=40)
        self.map_name_entry.grid(row=0, column=1, columnspan=2, padx=10, pady=10, sticky="w")

        # Buttons
        self.save_map_button = tk.Button(master, text="Save Map", command=self.save_map)
        self.save_map_button.grid(row=1, column=0, padx=10, pady=10)

        self.save_wormhole_button = tk.Button(master, text="Save Wormhole Location", command=self.save_wormhole_location)
        self.save_wormhole_button.grid(row=1, column=1, padx=10, pady=10)

        self.restart_slam_button = tk.Button(master, text="Start/Restart SLAM", command=self.restart_slam)
        self.restart_slam_button.grid(row=1, column=2, padx=10, pady=10)

        self.save_map_function = lambda map_name: 0
        self.save_wormhole_function = lambda map_name: 0
        self.restart_slam_function = lambda: 0

    def register_save_map_function(self, save_map_function_):
        self.save_map_function = save_map_function_

    def register_save_wormhole_function(self, save_wormhole_function_):
        self.save_wormhole_function = save_wormhole_function_

    def register_restart_slam_function(self, restart_slam_function_):
        self.restart_slam_function = restart_slam_function_

    # Empty binding functions
    def save_map(self):
        map_name = self.map_name_entry.get().strip()
        if not map_name:
            messagebox.showwarning("Input Error", "Please enter a map name before saving.")
            return
        print(f"Saving map with name: {map_name}")

        if (self.save_map_function(map_name) != 0):
            messagebox.showwarning("Error in saving the map.")
            return

    def save_wormhole_location(self):
        map_name = self.map_name_entry.get().strip()
        if not map_name:
            messagebox.showwarning("Input Error", "Please enter a map name before saving.")
            return
        print("Saving wormhole location.")
        self.save_wormhole_function(map_name)

    def restart_slam(self):
        print("Restarting SLAM...")
        self.restart_slam_function()

class RobotMappingLogic:
    def __init__(self):
        self.active_slam_tools_launch = None
        self.active_slam_launch = None

    def __del__(self):
        if self.active_slam_tools_launch is not None:
            self.active_slam_tools_launch.terminate()
        if self.active_slam_launch is not None:
            self.active_slam_launch.terminate()

    def save_map(self, map_name):
        multimap_storage = os.environ.get("MULTIMAP_STORAGE_PATH")

        map_saver_command = [f"rosrun map_server map_saver -f {multimap_storage + map_name}"]

        process = subprocess.Popen(map_saver_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()

        return process.returncode

    def restart_slam_function(self):
        slam_tools_command = ["roslaunch hmi_multimap mapping_tools.launch"]
        slam_command = ["roslaunch hmi_multimap slam_node.launch"]

        if (self.active_slam_launch is not None):
            self.active_slam_launch.terminate()
            self.active_slam_launch.wait()
        else:
            # self.active_slam_tools_launch = subprocess.Popen(slam_tools_command, shell=True)
            self.active_slam_tools_launch = subprocess.Popen(slam_tools_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # self.active_slam_launch = subprocess.Popen(slam_command, shell=True)
        self.active_slam_launch = subprocess.Popen(slam_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)



        return 0

    def save_wormhole_location(self, map_id):
        listener = tf.TransformListener()
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        del listener

        sql_tuple = (map_id, trans[0], trans[1], rot[0],  rot[1], rot[2], rot[3])
        print(f'Recorded wormhole location {sql_tuple[1:]} on map: {map_id}')

        wormhole_database_path = os.path.join(os.environ.get("MULTIMAP_STORAGE_PATH"), 'wormholes.db')

        with sqlite3.connect(wormhole_database_path) as conn:
            cursor = conn.cursor()
            cursor.execute("""
                            CREATE TABLE IF NOT EXISTS map_data (
                                map_id TEXT PRIMARY KEY,
                                x REAL,
                                y REAL,
                                qx REAL,
                                qy REAL,
                                qz REAL,
                                qw REAL
                            )
                            """)

            insert_query = """
                            INSERT OR REPLACE INTO map_data (map_id, x, y, qx, qy, qz, qw)
                            VALUES (?, ?, ?, ?, ?, ?, ?)
                            """
            cursor.execute(insert_query, sql_tuple)

            conn.commit()

if __name__ == "__main__":
    rospy.init_node('mapping_hmi')

    root = tk.Tk()
    gui = RobotMappingGUI(root)
    logic = RobotMappingLogic()

    gui.register_save_wormhole_function(logic.save_wormhole_location)
    gui.register_save_map_function(logic.save_map)
    gui.register_restart_slam_function(logic.restart_slam_function)

    root.mainloop()
    logic.__del__()
