import os
import time
from camera import Camera
from lidar import LiDAR
from radar import Radar
from disdrometer import Disdrometer
import rospy

class DataCollection:
    def __init__(self, gui):
        self.gui = gui
        self.base_data_dir = os.path.join(os.getcwd(), "sensor_data")
        os.makedirs(self.base_data_dir, exist_ok=True)

        # Initialize each sensor module
        self.camera = Camera(self.base_data_dir, gui, self)
        self.lidar = LiDAR(self.base_data_dir, gui, self)
        self.radar = Radar(self.base_data_dir, gui, self)
        self.disdrometer = Disdrometer(self.base_data_dir, gui)

        # Control variables for running status and force save
        self.is_running = False
        self.last_save_time = time.time()  # Initialize force save timer

    def start_collection(self):
        self.is_running = True
        self.last_save_time = time.time()  # Reset the timer on start
        self.gui.update_status("Collecting Data", "green")
        rospy.loginfo("Data collection started.")

    def stop_collection(self):
        self.is_running = False
        self.gui.update_status("Stopped", "red")
        rospy.loginfo("Data collection stopped.")



    def should_save_data(self):
        """
        Determine if data collection should proceed based on disdrometer filtering and force-save interval.
        """
        if not self.is_running:
            return False  # Stop data collection if not running

        force_save_due = self.force_save_due()

         #Log the rain status for debugging
        rospy.loginfo(f"Rain status: {self.disdrometer.is_raining}. "
                        f"Use Disdrometer: {self.gui.use_disdrometer.get()}, "
                        f"Force Save Due: {force_save_due}")

        
        
        if self.gui.use_disdrometer.get() and not force_save_due:
            # Disdrometer filtering is enabled, but force-save interval is not due
            if self.disdrometer.is_raining:
                rospy.loginfo("Disdrometer indicates rain: data collection proceeding.")
            else:
                rospy.loginfo("Disdrometer indicates no rain: data collection paused.")
                rospy.loginfo(self.is_running)
            return self.disdrometer.is_raining
        else:
            # Either disdrometer filtering is disabled, or force-save interval is due
            if force_save_due:
                rospy.loginfo("Force-save interval reached: data collection proceeding regardless of disdrometer.")
            return True  # Proceed with saving data

    def force_save_due(self):
        """
        Check if the force-save interval has been reached.
        """
        current_time = time.time()
        force_save_interval = self.gui.current_force_save_interval * 60  # Convert minutes to seconds
        if current_time - self.last_save_time >= force_save_interval:
            self.last_save_time = current_time  # Reset the timer after a force save
            return True
        return False
