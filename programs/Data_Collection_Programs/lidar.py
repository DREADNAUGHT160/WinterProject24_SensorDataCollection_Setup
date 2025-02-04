import os
import csv
import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime

class LiDAR:
    def __init__(self, base_dir, gui, collection):
        self.gui = gui
        self.collection = collection
        self.data_dir = os.path.join(base_dir, "lidar")
        os.makedirs(self.data_dir, exist_ok=True)
        self.status_pub = rospy.Publisher('/lidar/status', String, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Initialize CSV
        self.file_path_detailed = os.path.join(self.data_dir, "lidar_data_detailed.csv")
        with open(self.file_path_detailed, 'a', newline='') as f:
            writer = csv.writer(f)
            if os.stat(self.file_path_detailed).st_size == 0:  # Write header only if file is empty
                writer.writerow(["Timestamp", "Milliseconds", "Frame", "Point", "Distance", "Intensity"])

        # Sampling and sequence management
        self.last_save_time = time.time()  # Track last save time
        self.sample_interval = gui.current_lidar_interval  # Sampling interval from GUI
        self.current_seq = None  # To track sequence changes
        self.point_index = 0  # To reset points on sequence change

        # Initialize InfluxDB Client
        self.influx_client = InfluxDBClient(
            url="http://localhost:8086",
            token="knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw==",
            org="winterproject24"
        )
        self.write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
        self.influx_bucket = "lidar_bucket"
        self.influx_org = "winterproject24"

    def lidar_callback(self, msg):
        """Callback triggered when a new LaserScan message is received."""
        current_time = time.time()

        # Debug: Log the current sampling interval
        #rospy.loginfo(f"Current LiDAR sample interval: {self.sample_interval}")

        # Check time interval and force-save logic
        if current_time - self.last_save_time >= self.sample_interval and self.collection.should_save_data():
            self.last_save_time = current_time  # Update last save time
            timestampo = rospy.Time.now()
            timestamp = timestampo.to_sec()
            milliseconds = int(timestampo.to_nsec() / 1e6)
            readable_time = datetime.utcfromtimestamp(timestamp).strftime('%Y-%m-%dT%H:%M:%S.%fZ')

            # Skip duplicate sequences
            if self.current_seq == msg.header.seq:
                rospy.loginfo(f"Skipping duplicate sequence: {msg.header.seq}")
                return

            # Update sequence and reset point index
            self.current_seq = msg.header.seq
            self.point_index = 0

            # Process LiDAR points
            for i, distance in enumerate(msg.ranges):
                intensity = msg.intensities[i] if msg.intensities else 0  # Default to 0 if no intensity
                if not msg.intensities:
                    rospy.logwarn(f"LiDAR intensities missing for sequence: {msg.header.seq}")
                self.point_index += 1

                # Append to CSV
                self.append_to_detailed_csv([timestamp, milliseconds, self.current_seq, self.point_index, distance, intensity])

                # Upload to InfluxDB
                self.upload_to_influx(readable_time, self.current_seq, self.point_index, distance, intensity)

            # Update status
            self.status_pub.publish(String("Collecting"))
            rospy.loginfo("LiDAR data saved and uploaded to InfluxDB.")
            self.gui.update_saving_status("LiDAR", "Saving")
        else:
            # Update status when idle
            self.status_pub.publish(String("Idle"))
            self.gui.update_saving_status("LiDAR", "Idle")

    def append_to_detailed_csv(self, data):
        """Append processed data to the detailed CSV file."""
        try:
            with open(self.file_path_detailed, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
            #rospy.loginfo()
        except Exception as e:
            rospy.logerr(f"Failed to append to CSV: {e}")

    def upload_to_influx(self, readable_time, seq, point, distance, intensity):
        """Upload data to InfluxDB."""
        try:
            timestamp_1 = rospy.Time.now().to_sec()
            ns_timestamp = int(timestamp_1 * 1e9)
            point_data = (
                Point("lidar_data")
                .field("distance", distance)
                .field("intensity", intensity)
                .tag("frame", seq)
                .tag("point", point)
                .time(ns_timestamp)
            )
            self.write_api.write(bucket=self.influx_bucket, org=self.influx_org, record=point_data)
            #rospy.loginfo(f"Successfully uploaded to InfluxDB for sequence: {seq}, point: {point}")
        except Exception as e:
            rospy.logerr(f"Failed to upload to InfluxDB: {e}")

    def update_sample_interval(self, interval):
        """Update the sampling interval dynamically."""
        self.sample_interval = interval
        #rospy.loginfo(f"Updated LiDAR sample interval to {self.sample_interval} seconds.")
