import pandas as pd
from influxdb_client import InfluxDBClient, Point, WriteOptions

# InfluxDB Configuration
INFLUXDB_URL = "http://localhost:8086"  # Replace with your InfluxDB URL
INFLUXDB_TOKEN = "knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw=="  # Replace with your InfluxDB token
INFLUXDB_ORG = "winterproject24"               # Replace with your InfluxDB organization
INFLUXDB_BUCKET = "lidar_metrics"       # Replace with your InfluxDB bucket name
CSV_FILE_PATH = r"/home/carissma/new_ros-workspace/src/Scheduling/lidar/output/updated_dataset_with_nanoseconds.csv"  # Replace with your CSV file path
MEASUREMENT_NAME = "Lidar_Test"  # Replace with your Measurement name

# Columns used as fields and tags
FIELDS = ["x", "y", "intensity"]
TAGS = ["cluster"]
TIMESTAMP_COLUMN = "timestamp_ns"  # Column containing the nanosecond timestamp

def upload_csv_to_influxdb(csv_file_path, bucket, org, token, url, measurement_name, fields, tags, timestamp_column):
    """
    Uploads data from a CSV file to InfluxDB using Line Protocol format.

    Args:
        csv_file_path (str): Path to the CSV file.
        bucket (str): InfluxDB bucket name.
        org (str): InfluxDB organization name.
        token (str): Authentication token.
        url (str): URL of InfluxDB instance.
        measurement_name (str): Measurement name.
        fields (list): List of columns to use as fields.
        tags (list): List of columns to use as tags.
        timestamp_column (str): Name of the timestamp column.
    """
    # Initialize InfluxDB client
    client = InfluxDBClient(url=url, token=token, org=org)
    write_api = client.write_api(write_options=WriteOptions(batch_size=500, flush_interval=10_000))

    try:
        # Read the CSV file
        df = pd.read_csv(csv_file_path)
        print("CSV file loaded successfully.")

        # Ensure the timestamp column is valid
        if timestamp_column not in df.columns:
            raise ValueError(f"Timestamp column '{timestamp_column}' not found in the CSV.")

        # Convert timestamps to nanoseconds if necessary
        def convert_to_nanoseconds(ts):
            try:
                ts = int(ts)
                if len(str(ts)) <= 10:  # Assume seconds
                    return ts * (10**9)
                elif len(str(ts)) <= 13:  # Assume milliseconds
                    return ts * (10**6)
                elif len(str(ts)) <= 16:  # Assume microseconds
                    return ts * (10**3)
                return ts  # Already in nanoseconds
            except ValueError:
                return None

        df[timestamp_column] = df[timestamp_column].apply(convert_to_nanoseconds)

        # Filter invalid timestamps
        df = df[df[timestamp_column].notnull() & (df[timestamp_column] > 0)]

        # Iterate through rows and upload data
        for _, row in df.iterrows():
            try:
                timestamp = int(row[timestamp_column])

                # Create a Point object
                point = Point(measurement_name).time(timestamp)

                # Add tags
                for tag in tags:
                    if tag in row and pd.notna(row[tag]):
                        point = point.tag(tag, str(row[tag]))

                # Add fields
                for field in fields:
                    if field in row and pd.notna(row[field]):
                        point = point.field(field, float(row[field]))

                # Write the Point to InfluxDB
                write_api.write(bucket=bucket, record=point)

            except Exception as e:
                print(f"Skipping row due to error: {e}")
                print(f"Row data: {row}")

        print("Data uploaded successfully to InfluxDB.")

    except Exception as e:
        print(f"Error uploading data: {e}")

    finally:
        # Close the client
        write_api.close()
        client.close()
        print("InfluxDB client closed.")

# Call the function to upload data
upload_csv_to_influxdb(
    csv_file_path=CSV_FILE_PATH,
    bucket=INFLUXDB_BUCKET,
    org=INFLUXDB_ORG,
    token=INFLUXDB_TOKEN,
    url=INFLUXDB_URL,
    measurement_name=MEASUREMENT_NAME,
    fields=FIELDS,
    tags=TAGS,
    timestamp_column=TIMESTAMP_COLUMN
    )
    
