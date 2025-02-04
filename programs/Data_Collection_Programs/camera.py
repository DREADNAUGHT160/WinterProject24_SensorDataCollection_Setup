import os
import csv
import time
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

class Camera:
    def __init__(self, base_dir, gui, collection):
        self.gui = gui
        self.collection = collection
        self.bridge = CvBridge()
        self.data_dir = os.path.join(base_dir, "camera")
        os.makedirs(self.data_dir, exist_ok=True)
        self.status_pub = rospy.Publisher('/camera/status', String, queue_size=10)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback)

        # Initialize CSV
        self.file_path = os.path.join(self.data_dir, "camera_data.csv")
        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if os.stat(self.file_path).st_size == 0:  # Write header only if file is empty
                writer.writerow(["Timestamp", "Filepath"])

        # Sampling interval
        self.sample_interval = getattr(gui, 'current_camera_interval', 10)  # Fallback to 10 seconds
        self.last_save_time = time.time()  # Track last save time

        # Initialize InfluxDB Client
        self.influx_client = InfluxDBClient(
            url="http://localhost:8086", 
            token="knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw==", 
            org="winterproject24"
        )
        self.write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
        self.influx_bucket = "camera_bucket"
        self.influx_org = "winterproject24"

    def camera_callback(self, msg):
        """Callback triggered when a new image message is received."""
        if not hasattr(self, 'sample_interval'):
            rospy.logwarn("Camera sample_interval is not initialized.")
            return

        current_time = time.time()
        rospy.logdebug(f"Current Camera sample interval: {self.sample_interval} seconds.")

        if current_time - self.last_save_time >= self.sample_interval:
            if self.collection.should_save_data():
                self.last_save_time = current_time  # Update last save time

                ros_time = msg.header.stamp.to_sec()
                timestamp = rospy.Time.now().to_sec()
                readable_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))
                file_path = os.path.join(self.data_dir, f"{int(timestamp)}.jpg")

                # Convert Image message to OpenCV format and save
                try:
                    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    rotated_image = cv2.rotate(image, cv2.ROTATE_180)
                    cv2.imwrite(file_path, rotated_image)
                except Exception as e:
                    rospy.logerr(f"Failed to save image: {e}")
                    return

                # Append to CSV
                self.append_to_csv([readable_time, file_path])

                # Upload to InfluxDB
                self.upload_to_influx(readable_time, ros_time, file_path)

                # Update status
                self.status_pub.publish(String("Collecting"))
                rospy.loginfo("Camera data saved and uploaded to InfluxDB.")
                self.gui.update_saving_status("Camera", "Saving")
            else:
                self.status_pub.publish(String("Idle"))
                self.gui.update_saving_status("Camera", "Idle")

    def append_to_csv(self, data):
        """Append data to the CSV file."""
        try:
            with open(self.file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
        except Exception as e:
            rospy.logerr(f"Failed to append to CSV: {e}")

    def upload_to_influx(self, readable_time, ros_time, file_path):
        """Upload data to InfluxDB."""
        try:
            
            timestamp_1 = rospy.Time.now().to_sec()
            ns_timestamp = int(timestamp_1 * 1e9)
            point_data = (
                Point("camera_data")
                .field("ros_time", ros_time)
                .field("file_path", file_path)
                .time(ns_timestamp, write_precision="ns")
            )
            self.write_api.write(bucket=self.influx_bucket, org=self.influx_org, record=point_data)
            #rospy.loginfo(f"Uploaded to InfluxDB: Timestamp={readable_time}, Filepath={file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to upload to InfluxDB: {e}")

    def update_sample_interval(self, interval):
        """Update the sampling interval dynamically."""
        self.sample_interval = interval
        #rospy.loginfo(f"Updated Camera sample interval to {interval} seconds.")
