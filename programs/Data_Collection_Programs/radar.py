import os
import csv
import rospy
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime

class Radar:
    def __init__(self, base_dir, gui, collection):
        # Initialize GUI and collection system
        self.gui = gui
        self.collection = collection

        # Create data directory
        self.data_dir = os.path.join(base_dir, "radar")
        os.makedirs(self.data_dir, exist_ok=True)

        # Set up ROS publisher and subscriber
        self.status_pub = rospy.Publisher('/radar/status', String, queue_size=10)
        rospy.Subscriber('/ti_mmwave/radar_scan_pcl_0', PointCloud2, self.radar_callback)

        # Initialize CSV file
        self.file_path = os.path.join(self.data_dir, "radar_data.csv")
        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if os.stat(self.file_path).st_size == 0:  # Write header only if file is empty
                writer.writerow(["Unix_Timestamp", "Frame_ID", "X", "Y", "Z", "Intensity"])

        # Sampling and status management
        self.last_save_time = time.time()  # Track last save time
        self.sample_interval = gui.current_radar_interval  # Sampling interval from GUI

        # Initialize InfluxDB Client
        self.influx_client = InfluxDBClient(
            url="http://localhost:8086", 
            token="knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw==", 
            org="winterproject24"
        )
        self.write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
        self.influx_bucket = "radar_bucket"
        self.influx_org = "winterproject24"

    def radar_callback(self, msg):
        """Callback triggered when a new PointCloud2 message is received."""
        current_time = time.time()

        # Debug log for interval
        rospy.logdebug(f"Current Radar sample interval: {self.sample_interval} seconds.")

        if current_time - self.last_save_time >= self.sample_interval:
            if self.collection.should_save_data():
                self.last_save_time = current_time  # Update last save time
                try:
                    # Get UNIX timestamp in seconds
                    if not hasattr(msg.header, "stamp"):
                        rospy.logerr("Radar message header is missing timestamp.")
                        return
                    
                    timestamp = msg.header.stamp.to_sec()
                    readable_time = datetime.utcfromtimestamp(timestamp).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
                    timestamp_1 = rospy.Time.now().to_sec()
                    ns_timestamp = int(timestamp_1 * 1e9)
            

                    # Get the frame ID
                    frame_id = msg.header.frame_id

                    # Read points from the PointCloud2 message
                    points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)

                    # Save points to CSV and upload to InfluxDB
                    batch_points = []  # For batch uploading
                    for point in points:
                        data = [timestamp, frame_id, *point]  # Include timestamp, frame ID, and all point fields
                        self.append_to_csv(data)  # Save to CSV
                        # Prepare batch for InfluxDB
                        batch_points.append(
                            Point("radar_data")
                            .tag("frame_id", frame_id)
                            .field("x", point[0])
                            .field("y", point[1])
                            .field("z", point[2])
                            .field("intensity", point[3])
                            .time(ns_timestamp)
                        )
                    self.upload_to_influx(batch_points)  # Batch upload to InfluxDB

                    # Update status and GUI
                    self.status_pub.publish(String("Collecting"))
                    rospy.loginfo(f"Radar data saved for frame: {frame_id}")
                    self.gui.update_saving_status("Radar", "Saving")

                except Exception as e:
                    rospy.logerr(f"Error processing radar data: {e}")
            else:
                self.status_pub.publish(String("Idle"))
                self.gui.update_saving_status("Radar", "Idle")

    def append_to_csv(self, data):
        """Write data to the CSV file."""
        try:
            with open(self.file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
            #rospy.loginfo(f"Successfully appended to CSV: {data}")
        except Exception as e:
            rospy.logerr(f"Failed to append to CSV: {e}")

    def upload_to_influx(self, batch_points):
        """Batch upload data to InfluxDB."""
        try:
            self.write_api.write(bucket=self.influx_bucket, org=self.influx_org, record=batch_points)
            rospy.loginfo("Successfully uploaded batch to InfluxDB.")
        except Exception as e:
            rospy.logerr(f"Failed to upload to InfluxDB: {e}")

    def update_sample_interval(self, interval):
        """Update the sampling interval dynamically."""
        self.sample_interval = interval
        #rospy.loginfo(f"Updated Radar sample interval to {interval} seconds.")
