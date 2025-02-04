import os
import csv
import time
import rospy
from std_msgs.msg import String
import serial
from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import SYNCHRONOUS

# Constants for classification and precipitation
DIAMETER_CLASSES = [
    0.125, 0.250, 0.375, 0.500, 0.750, 1.000, 1.250, 1.500, 1.750, 2.000,
    2.500, 3.000, 3.500, 4.000, 4.500, 5.000, 5.500, 6.000, 6.500, 7.000, 7.500, 8.000
]
SPEED_CLASSES = [
    0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.4, 1.8, 2.2, 2.6,
    3.0, 3.4, 4.2, 5.0, 5.8, 6.6, 7.4, 8.2, 9.0, 10.0
]
PRECIPITATION_CODES = {
    "00": "No Precipitation",
    "51": "Drizzle", "53": "Drizzle",
    "55": "Freezing Drizzle",
    "61": "Rain", "63": "Rain",
    "66": "Freezing Rain", "67": "Freezing Rain",
    "71": "Snow", "73": "Snow", "75": "Snow",
    "79": "Ice Pellets",
    "77": "Snow Grains",
    "89": "Hail", "90": "Hail",
    "01": "Precipitation Not Identified",
}


class Disdrometer:
    def __init__(self, base_dir, gui, csv_location=None, use_dummy_data=False):
        self.gui = gui
        self.is_raining = False
        self.use_dummy_data = use_dummy_data

        # Configurable thresholds for rain detection
        self.total_precipitation_threshold = 1.0  # mm
        self.visibility_threshold = 2000  # m

        # InfluxDB configuration
        self.influxdb_url = "http://localhost:8086"  # Adjust as needed
        self.influxdb_token = "OmUfpo7ecszypWgAEbPT5NRC1z10x7hMDziT5Ecqyv5FJejHubhfgUTrOClCegoYdA5lAZ0ZeTwI43mPZXyySg=="
        self.influxdb_org = "winterproject24"
        self.influxdb_bucket = "disdrometer_data"

        try:
            self.influx_client = InfluxDBClient(
                url=self.influxdb_url,
                token=self.influxdb_token,
                org=self.influxdb_org
            )
            self.write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
            rospy.loginfo("Connected to InfluxDB successfully.")
            print("Connected to InfluxDB successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to connect to InfluxDB: {e}")
            self.influx_client = None

        # Set CSV location: default or custom
        if csv_location:
            self.data_dir = os.path.dirname(csv_location)
            self.file_path = csv_location
        else:
            self.data_dir = os.path.join(base_dir, "/home/carissma/new_ros-workspace/src/my_package/src/sensor_data/disdrometer/disdrometer.csv")
            self.file_path = os.path.join(self.data_dir, "disdrometer.csv")

        os.makedirs(self.data_dir, exist_ok=True)

        # Initialize CSV file only if it's empty
        if not os.path.isfile(self.file_path) or os.stat(self.file_path).st_size == 0:
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Timestamp", "Visibility (m/h)", "Total Precipitation (mm)",
                    "Liquid Precipitation (mm/h)", "Solid Precipitation (mm/h)",
                    "Precipitation Type"
                ])
            rospy.loginfo(f"CSV header written at: {self.file_path}")
        else:
            rospy.loginfo(f"Appending to existing CSV file: {self.file_path}")

        # ROS setup
        self.status_pub = rospy.Publisher('/disdrometer/status', String, queue_size=10)
        rospy.Subscriber('/disdrometer/status', String, self.disdrometer_callback)

        # Initialize serial connection
        if not use_dummy_data:
            self.serial_port = '/dev/ttyUSB0'
            self.baud_rate = 9600
            try:
                self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=5)
                rospy.loginfo(f"Connected to serial port: {self.serial_port}")
            except serial.SerialException as e:
                rospy.logerr(f"Error opening serial port: {e}")
                self.serial_connection = None
        else:
            self.serial_connection = None

    def parse_disdrometer_data(self, raw_data):
        """Parse the raw data string into meaningful components."""
        try:
            rospy.loginfo(f"Parsing raw data: {raw_data}")
            fields = raw_data.split(';')

            # Extract relevant columns
            date = fields[3].strip()       # Date of the sensor
            time = fields[4].strip()       # Time of the sensor
            visibility = float(fields[15].strip())  # Visibility (m/h)
            total_precipitation = float(fields[12].strip())  # Total precipitation (mm)
            liquid_precipitation = float(fields[13].strip())  # Liquid precipitation intensity
            solid_precipitation = float(fields[14].strip())  # Solid precipitation intensity
            precipitation_type = fields[9].strip()    # synop code

            # Combine date and time into a timestamp
            ros_timestamp = rospy.Time.now()
            timestamp = ros_timestamp.to_sec()

            # Map precipitation type using code table
            precipitation_description = PRECIPITATION_CODES.get(precipitation_type, "Unknown")

            # Determine rain status based on thresholds
            self.is_raining = (
                total_precipitation > self.total_precipitation_threshold and
                visibility < self.visibility_threshold
            )
            status = "Raining" if self.is_raining else "No rain"
            rospy.loginfo(f"Rain status updated: {status} "
                          f"(Total Precipitation: {total_precipitation}, Visibility: {visibility})")

            # Write data to InfluxDB
            self.write_to_influxdb(
                timestamp, visibility, total_precipitation,
                liquid_precipitation, solid_precipitation, precipitation_description
            )

            # Return parsed data
            return [timestamp, visibility, total_precipitation, liquid_precipitation,
                    solid_precipitation, precipitation_description]
        except (IndexError, ValueError) as e:
            rospy.logerr(f"Error parsing data: {e}")
            return None

    def write_to_influxdb(self, timestamp, visibility, total_precipitation,
                          liquid_precipitation, solid_precipitation, precipitation_type):
        """Write data to InfluxDB."""
        if self.influx_client:
            try:
                timestamp_ns = int(timestamp * 1e9)
                point = Point("disdrometer") \
                    .field("visibility", visibility) \
                    .field("total_precipitation", total_precipitation) \
                    .field("liquid_precipitation", liquid_precipitation) \
                    .field("solid_precipitation", solid_precipitation) \
                    .tag("precipitation_type", precipitation_type) \
                    .time(timestamp_ns,write_precision="ns")
                self.write_api.write(bucket=self.influxdb_bucket, org=self.influxdb_org, record=point)
                rospy.loginfo(f"Data written to InfluxDB: {point}")
                print("influx")
            except Exception as e:
                rospy.logerr(f"Error writing to InfluxDB: {e}")

    def disdrometer_callback(self, msg):
        """Handle incoming ROS messages for disdrometer status."""
        try:
            rospy.loginfo(f"Received raw message on /disdrometer/status: {msg.data}")
            parsed_data = self.parse_disdrometer_data(msg.data)
            if parsed_data:
                # Update GUI with rain status
                self.gui.update_disdrometer_status("Raining" if self.is_raining else "No rain")

                # Append data to CSV
                self.append_to_csv(parsed_data)

        except Exception as e:
            rospy.logerr(f"Error in disdrometer_callback: {e}")

    def append_to_csv(self, data):
        """Append parsed data to the CSV file."""
        try:
            with open(self.file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
            rospy.loginfo(f"Data appended to CSV: {data}")
        except Exception as e:
            rospy.logerr(f"Error writing to CSV: {e}")

    def toggle_rain_status(self):
        """Toggle the rain status manually."""
        self.is_raining = not self.is_raining
        status = "Raining" if self.is_raining else "No rain"
        self.gui.update_disdrometer_status(status)
        rospy.loginfo(f"Rain status toggled: {status}")
        return status

    def run(self):
        """Run the disdrometer data processing loop."""
        rospy.loginfo("Starting Disdrometer data collection.")
        while not rospy.is_shutdown():
            if not self.use_dummy_data and self.serial_connection:
                try:
                    raw_data = self.serial_connection.readline().decode('utf-8').strip()
                    if raw_data:
                        rospy.loginfo(f"Raw data received: {raw_data}")
                        parsed_data = self.parse_disdrometer_data(raw_data)
                        if parsed_data:
                            # Append data to CSV and publish status
                            self.append_to_csv(parsed_data)
                            self.status_pub.publish(raw_data)
                except serial.SerialException as e:
                    rospy.logerr(f"Error reading serial data: {e}")
                except Exception as e:
                    rospy.logerr(f"Unexpected error: {e}")
            time.sleep(1)  # Simulate processing loop for real-time updates


if __name__ == "__main__":
    rospy.init_node('disdrometer_node', anonymous=True)
    gui_stub = type('GUI', (), {"update_disdrometer_status": lambda self, status: print(f"GUI Status: {status}")})()
    base_directory = os.path.expanduser("~")
    disdrometer = Disdrometer(base_directory, gui_stub)
    try:
        disdrometer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Disdrometer node interrupted.")
