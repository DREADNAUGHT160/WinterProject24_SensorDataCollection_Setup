#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import signal

class SensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Sensor Setup")
        self.processes = []

        # Buttons to control the sensors and roscore
        self.roscore_button = tk.Button(root, text="Start ROSCore", command=self.start_roscore)
        self.roscore_button.pack(pady=10)

        self.lidar_button = tk.Button(root, text="Start Lidar", command=self.check_lidar)
        self.lidar_button.pack(pady=10)

        self.radar_button = tk.Button(root, text="Start Radar", command=self.check_radar)
        self.radar_button.pack(pady=10)

        self.camera_button = tk.Button(root, text="Start Camera", command=self.check_camera)
        self.camera_button.pack(pady=10)

        self.disdrometer_button = tk.Button(root, text="Check Disdrometer", command=self.check_disdrometer)
        self.disdrometer_button.pack(pady=10)

        # Button to start data collection after sensors
        self.data_collection_button = tk.Button(root, text="Start Data Collection", command=self.start_data_collection)
        self.data_collection_button.pack(pady=20)

        # Button to quit all processes
        self.quit_button = tk.Button(root, text="Quit All", command=self.quit_all_processes)
        self.quit_button.pack(pady=20)

    def open_terminal(self, command):
        # Opens a new terminal and runs the command, storing the process
        try:
            process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])
            self.processes.append(process)
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Failed to run the command: {str(e)}")
            return False

    def check_device(self, device_path):
        # Checks if a device file exists
        return os.path.exists(device_path)

    def check_lidar(self):
        lidar_device = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyACM0']  # Assuming Lidar is connected to this port
        for device in lidar_device:
            if self.check_device(device):
                self.start_lidar()
                return
        
        messagebox.showerror("Error", "Lidar is not connected")

    def check_radar(self):
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        if 'Texas Instruments, Inc. CC1352R1 Launchpad' in result.stdout:  # Replace with specific identifier for radar
            self.start_radar()
        else:
            messagebox.showerror("Error", "Radar is not connected")

    def check_camera(self):
        camera_device = "/dev/video2"
        if self.check_device(camera_device):
            self.start_camera()
        else:
            messagebox.showerror("Error", "Camera is not connected")

    def check_disdrometer(self):
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        if "0403:6001" in result.stdout:  # Replace with specific identifier
            messagebox.showinfo("Disdrometer", "Disdrometer is connected")
        else:
            messagebox.showerror("Error", "Disdrometer is not connected")

    def start_roscore(self):
        command = """
        roscore
        """
        if self.open_terminal(command):
            messagebox.showinfo("Success", "ROSCore started successfully")

    def start_lidar(self):
        command = """
        rosrun urg_node urg_node
        """
        if self.open_terminal(command):
            messagebox.showinfo("Success", "Lidar node started successfully")

    def start_radar(self):
        # Full command to run in the new terminal
        full_command = "bash -c 'cd ~/mmwave_ti_ros/ros1_driver && source devel/setup.bash && roslaunch ti_mmwave_rospkg 1443_multi_3d_0.launch; exec bash'"

        try:
            # Open a new terminal and execute the command
            result = subprocess.run(['gnome-terminal', '--', 'bash', '-c', full_command], check=True)
            
            # Check the return code
            if result.returncode == 0:
                messagebox.showinfo("Success", "Radar node started successfully")
            else:
                messagebox.showerror("Error", f"Radar node failed to start. Return code: {result.returncode}")
        except subprocess.CalledProcessError as e:
            # Handle subprocess errors
            messagebox.showerror("Error", f"Radar node failed to start. Error: {e}")
        except FileNotFoundError:
            # Handle missing gnome-terminal or other files
            messagebox.showerror("Error", "gnome-terminal not found. Please ensure it is installed.")

    def start_camera(self):
        command = """
        rosrun usb_cam usb_cam_node _video_device:="/dev/video2"
        """
        if self.open_terminal(command):
            messagebox.showinfo("Success", "Camera node started successfully")

    def start_data_collection(self):
        command = """
        cd /home/carissma/new_ros-workspace/src/my_package/src
        python3 main.py
        """
        if self.open_terminal(command):
            messagebox.showinfo("Success", "Data collection started successfully")

    def quit_all_processes(self):
        # Terminate all opened processes
        for process in self.processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to terminate process: {str(e)}")
        self.processes.clear()
        messagebox.showinfo("Quit", "All processes have been terminated")

# Initialize GUI
root = tk.Tk()
app = SensorApp(root)
root.mainloop()
