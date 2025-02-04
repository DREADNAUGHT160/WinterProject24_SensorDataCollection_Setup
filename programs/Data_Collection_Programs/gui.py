import tkinter as tk
from tkinter import messagebox
from datetime import datetime

class DataCollectionGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Data Collection Control Panel")
        
        # Variables for each parameter and internal tracking values
        self.lidar_sample_interval = tk.DoubleVar(value=10)
        self.radar_sample_interval = tk.DoubleVar(value=10)
        self.camera_interval = tk.DoubleVar(value=20)
        self.force_save_interval = tk.DoubleVar(value=59)  # Default force-save interval in minutes
        self.is_raining = False
        self.use_disdrometer = tk.BooleanVar(value=True)

        # Values that sensors use, only updated on Restart button click
        self.current_lidar_interval = self.lidar_sample_interval.get()
        self.current_radar_interval = self.radar_sample_interval.get()
        self.current_camera_interval = self.camera_interval.get()
        self.current_force_save_interval = self.force_save_interval.get()

        # GUI Elements
        self.create_widgets()

        # Linking start/stop methods
        self.start_data_collection = lambda: None
        self.stop_data_collection = lambda: None
        self.toggle_rain_status_callback = lambda: None  # Placeholder

    def create_widgets(self):
        # Sampling interval controls without Set buttons
        tk.Label(self.root, text="LiDAR Sample Interval (s)").grid(row=0, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.lidar_sample_interval).grid(row=0, column=1)

        tk.Label(self.root, text="Radar Sample Interval (s)").grid(row=1, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.radar_sample_interval).grid(row=1, column=1)

        tk.Label(self.root, text="Camera Interval (s)").grid(row=2, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.camera_interval).grid(row=2, column=1)

        # Force Save Interval Control
        tk.Label(self.root, text="Force Save Interval (min)").grid(row=3, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.force_save_interval).grid(row=3, column=1)

        # Checkbox for disdrometer filtering
        tk.Checkbutton(self.root, text="Use Disdrometer for Data Filtering", variable=self.use_disdrometer).grid(row=4, columnspan=3)

        # Start, Stop, and Restart Buttons
        tk.Button(self.root, text="Start", command=self.start_collection).grid(row=5, column=0, pady=10)
        tk.Button(self.root, text="Stop", command=self.stop_collection).grid(row=5, column=1, pady=10)
        tk.Button(self.root, text="Restart", command=self.restart_collection).grid(row=5, column=2, pady=10)

        # Manual toggle for rain status
        tk.Button(self.root, text="Toggle Rain Status", command=self.toggle_rain_status).grid(row=6, columnspan=3, pady=10)

        # Status Display
        self.status_label = tk.Label(self.root, text="Status: Stopped", fg="red")
        self.status_label.grid(row=7, columnspan=3)

        self.disdrometer_label = tk.Label(self.root, text="Disdrometer Status: No rain")
        self.disdrometer_label.grid(row=8, columnspan=3)

        # Sensor Saving Status Labels
        self.saving_status_labels = {
            "Camera": tk.Label(self.root, text="Camera: Idle", fg="blue"),
            "LiDAR": tk.Label(self.root, text="LiDAR: Idle", fg="blue"),
            "Radar": tk.Label(self.root, text="Radar: Idle", fg="blue")
        }
        self.saving_status_labels["Camera"].grid(row=9, columnspan=3)
        self.saving_status_labels["LiDAR"].grid(row=10, columnspan=3)
        self.saving_status_labels["Radar"].grid(row=11, columnspan=3)

        # Real-time clock display
        self.clock_label = tk.Label(self.root, text="", font=("Helvetica", 10))
        self.clock_label.grid(row=12, columnspan=3)
        self.update_clock()

    def restart_collection(self):
        # Update the actual intervals from input fields
        self.current_lidar_interval = self.lidar_sample_interval.get()
        self.current_radar_interval = self.radar_sample_interval.get()
        self.current_camera_interval = self.camera_interval.get()
        self.current_force_save_interval = self.force_save_interval.get()

        # Restart data collection
        self.stop_collection()
        self.start_collection()
        messagebox.showinfo("Restarted", "Parameters updated and data collection restarted.")

    def start_collection(self):
        self.start_data_collection()
        self.update_status("Collecting Data", "green")

    def stop_collection(self):
        self.stop_data_collection()
        self.update_status("Stopped", "red")

    def update_status(self, message, color="black"):
        self.status_label.config(text=f"Status: {message}", fg=color)

    def update_disdrometer_status(self, message):
        self.disdrometer_label.config(text=f"Disdrometer Status: {message}")

    # New method to update sensor saving status in the GUI
    def update_saving_status(self, sensor, status):
        color = "green" if status == "Saving" else "blue"
        if sensor in self.saving_status_labels:
            # Schedule the update on the main thread
            self.root.after(0, self._update_saving_status_label, sensor, status, color)

    def _update_saving_status_label(self, sensor, status, color):
        """Helper method to update the label safely on the main thread."""
        self.saving_status_labels[sensor].config(text=f"{sensor}: {status}", fg=color)

    def toggle_rain_status(self):
        new_status = self.toggle_rain_status_callback()  # Call the function from collection module
        messagebox.showinfo("Rain Status", f"Rain status set to: {new_status}")

    def update_clock(self):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.clock_label.config(text=f"Current Time: {current_time}")
        self.root.after(1000, self.update_clock)  # Update clock every second

if __name__ == "__main__":
    root = tk.Tk()
    app = DataCollectionGUI(root)
    root.mainloop()