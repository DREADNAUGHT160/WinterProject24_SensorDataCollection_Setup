import rospy
import tkinter as tk
from threading import Thread
from gui import DataCollectionGUI
from collection import DataCollection
import subprocess
import signal
import sys
import time


class DataCollectionManager:
    def __init__(self, gui, data_collection):
        self.gui = gui
        self.data_collection = data_collection
        self.force_save_interval = gui.current_force_save_interval * 60  # Convert minutes to seconds
        self.last_force_save_time = rospy.Time.now().to_sec()  # ROS time
        self.user_started = False  # Track if the user manually started collection
        self.processes = {}  # Dictionary to track subprocesses with their names

    def run(self):
        """Main execution loop for managing data collection intervals and syncing with ROS time."""
        rospy.loginfo("Starting Data Collection Manager...")
        rate = rospy.Rate(1)  # 1Hz loop rate

        # Start external programs
        self.start_external_programs()

        while not rospy.is_shutdown():
            current_ros_time = rospy.Time.now().to_sec()
            time_since_last_save = current_ros_time - self.last_force_save_time

            if self.user_started:
                # Manual start: collection runs continuously
                rospy.loginfo("Data collection running (manually started by user).")
                self.data_collection.start_collection()
            else:
                # Automatic control
                if time_since_last_save >= self.force_save_interval:
                    # Force-save logic bypasses rain status
                    rospy.loginfo("Force save interval reached. Triggering data collection regardless of rain status.")
                    self.last_force_save_time = current_ros_time
                    self.data_collection.start_collection()
                elif self.data_collection.should_save_data():
                    # Normal logic considers rain status
                    rospy.loginfo("Rain status indicates data collection should proceed.")
                    self.data_collection.start_collection()
                else:
                    rospy.loginfo("Data collection paused (rain status or user stop).")
                    self.data_collection.stop_collection()

            # Sync GUI parameters with live intervals
            self.sync_intervals()

            # Check the status of the external programs
            self.log_process_status()

            # Sleep to maintain loop rate
            rate.sleep()

    def sync_intervals(self):
        """Synchronize intervals from the GUI to data collection modules."""
        self.data_collection.camera.update_sample_interval(self.gui.current_camera_interval)
        self.data_collection.lidar.update_sample_interval(self.gui.current_lidar_interval)
        self.data_collection.radar.update_sample_interval(self.gui.current_radar_interval)
        rospy.loginfo("Intervals synced: Camera = {}s, LiDAR = {}s, Radar = {}s".format(
            self.gui.current_camera_interval,
            self.gui.current_lidar_interval,
            self.gui.current_radar_interval
        ))

    def start_external_programs(self):
        """Start disdor.py and scheduler.py as subprocesses."""
        try:
            disdor_path = "/home/carissma/new_ros-workspace/src/my_package/src/disdrometer.py"  # Replace with the actual path
            scheduler_path = "/home/carissma/new_ros-workspace/src/Scheduling/scheduler.py"  # Replace with the actual path

            disdor_process = subprocess.Popen(["/usr/bin/python3", disdor_path])
            self.processes["disdor"] = disdor_process
            rospy.loginfo(f"Disdor started with PID: {disdor_process.pid}")

            scheduler_process = subprocess.Popen(["/usr/bin/python3", scheduler_path])
            self.processes["scheduler"] = scheduler_process
            rospy.loginfo(f"Scheduler started with PID: {scheduler_process.pid}")

        except Exception as e:
            rospy.logerr(f"Error starting external programs: {e}")
            self.stop_external_programs()

    def stop_external_programs(self):
        """Terminate all subprocesses."""
        for name, process in self.processes.items():
            process.terminate()
            rospy.loginfo(f"Terminated {name} process with PID: {process.pid}")
        self.processes.clear()

    def log_process_status(self):
        """Log the running status of the external programs."""
        for name, process in self.processes.items():
            retcode = process.poll()
            if retcode is None:
                rospy.loginfo(f"{name.capitalize()} is running (PID: {process.pid})")
            else:
                rospy.logwarn(f"{name.capitalize()} is not running (Exited with code: {retcode})")

    def start_from_gui(self):
        """User-initiated start of data collection."""
        rospy.loginfo("User started data collection.")
        self.user_started = True
        self.data_collection.start_collection()

    def stop_from_gui(self):
        """User-initiated stop of data collection."""
        rospy.loginfo("User stopped data collection.")
        self.user_started = False
        self.data_collection.stop_collection()

    def __del__(self):
        """Ensure subprocesses are terminated when the manager is destroyed."""
        self.stop_external_programs()


if __name__ == "__main__":
    rospy.init_node("data_collection_node", anonymous=True)

    # Initialize GUI and data collection modules
    root = tk.Tk()
    gui = DataCollectionGUI(root)
    data_collection = DataCollection(gui)

    # Create the manager
    manager = DataCollectionManager(gui, data_collection)

    # Link GUI controls to manager methods
    gui.start_data_collection = manager.start_from_gui
    gui.stop_data_collection = manager.stop_from_gui
    gui.toggle_rain_status_callback = data_collection.disdrometer.toggle_rain_status

    # Run the ROS manager in a separate thread
    ros_thread = Thread(target=manager.run)
    ros_thread.daemon = True
    ros_thread.start()

    # Ensure subprocesses are terminated on exit
    def on_exit():
        rospy.loginfo("Exiting main program, stopping external programs...")
        manager.stop_external_programs()
        root.destroy()
        sys.exit(0)

    root.protocol("WM_DELETE_WINDOW", on_exit)

    # Start the GUI in the main thread
    rospy.loginfo("Starting the GUI...")
    root.mainloop()
