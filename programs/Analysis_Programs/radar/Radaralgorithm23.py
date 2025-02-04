import pandas as pd
from influxdb_client import InfluxDBClient, Point, WriteOptions
import os
import numpy as np

def convert_and_filter_radar_data(input_file_path, output_file_path, filtered_csv_path):
    """
    Converts radar data from Unix_Timestamp to range, azimuth, elevation, and actual time.
    Filters the data based on range and azimuth conditions and appends the results.

    Args:
        input_file_path (str): Path to the input radar.csv file.
        output_file_path (str): Path to save the converted data as an Excel file.
        filtered_csv_path (str): Path to save the filtered data as a CSV file.
    """
    # Load the radar.csv file
    radar_df = pd.read_csv(input_file_path)

    # Ensure the necessary columns exist
    if 'Unix_Timestamp' not in radar_df.columns or 'X' not in radar_df.columns or 'Y' not in radar_df.columns or 'Z' not in radar_df.columns:
        raise ValueError("The input file must contain 'Unix_Timestamp', 'X', 'Y', and 'Z' columns.")

    # Calculate Range, Azimuth, and Elevation
    radar_df['Range'] = np.sqrt(radar_df['X']**2 + radar_df['Y']**2 + radar_df['Z']**2)
    radar_df['Azimuth'] = np.degrees(np.arctan2(radar_df['Y'], radar_df['X']))
    radar_df['Elevation'] = np.degrees(np.arcsin(radar_df['Z'] / radar_df['Range']))

    # Convert Unix_Timestamp to actual time
    radar_df['Actual_Time'] = pd.to_datetime(radar_df['Unix_Timestamp'], unit='s')

    # Append to Excel file
    if os.path.exists(output_file_path):
        existing_df = pd.read_excel(output_file_path)
        radar_df = pd.concat([existing_df, radar_df], ignore_index=True)

    radar_df.to_excel(output_file_path, index=False)
    print(f"Converted radar data appended to '{output_file_path}'.")

    # Filter the data based on conditions
    filtered_df = radar_df[(radar_df['Range'] > 9) & (radar_df['Azimuth'].between(-4, 4))]

    # Append to CSV file
    if os.path.exists(filtered_csv_path):
        existing_filtered_df = pd.read_csv(filtered_csv_path)
        filtered_df = pd.concat([existing_filtered_df, filtered_df], ignore_index=True).drop_duplicates()

    filtered_df.to_csv(filtered_csv_path, index=False)
    print(f"Filtered radar data appended to '{filtered_csv_path}'.")

# Reads the filtered CSV file and retrieves the processed Unix_Timestamps.
def get_processed_timestamps(filtered_csv_path):
    """
    Reads the filtered CSV file and retrieves the processed Unix_Timestamps.

    Args:
        filtered_csv_path (str): Path to the filtered CSV file.

    Returns:
        set: Set of processed Unix_Timestamps.
    """
    if os.path.exists(filtered_csv_path):
        filtered_df = pd.read_csv(filtered_csv_path)
        return set(filtered_df['Unix_Timestamp'])
    return set()

# Uploads CSV data directly to InfluxDB without creating an intermediate .lp file.
def upload_csv_to_influxdb(csv_file_path, measurement_name, influx_url, token, org, bucket):
    """
    Uploads CSV data directly to InfluxDB without creating an intermediate .lp file.

    Args:
        csv_file_path (str): Path to the CSV file.
        measurement_name (str): Measurement name for InfluxDB.
        influx_url (str): InfluxDB URL.
        token (str): InfluxDB token.
        org (str): Organization name.
        bucket (str): Bucket name.
    """
    # Initialize InfluxDB client
    client = InfluxDBClient(url=influx_url, token=token, org=org)
    write_api = client.write_api(write_options=WriteOptions(batch_size=500, flush_interval=10_000))

    try:
        # Load the CSV file
        df = pd.read_csv(csv_file_path)
        print(f"CSV file '{csv_file_path}' loaded successfully.")

        # Ensure 'Unix_Timestamp' column exists and convert it to nanoseconds
        if 'Unix_Timestamp' not in df.columns:
            raise ValueError("The CSV file must contain a 'Unix_Timestamp' column.")

        # Iterate through rows and upload data
        for _, row in df.iterrows():
            try:
                # Convert Unix_Timestamp to nanoseconds
                timestamp = int(float(row['Unix_Timestamp']) * 1e9)

                # Create a Point object for InfluxDB
                point = Point(measurement_name).time(timestamp)

                # Add fields (numeric values)
                for col in df.columns:
                    if col == "Unix_Timestamp":
                        continue  # Skip the Unix_Timestamp column
                    try:
                        point = point.field(col, float(row[col]))
                    except ValueError:
                        point = point.field(col, str(row[col]))  # If not numeric, store as string

                # Write the point to InfluxDB
                write_api.write(bucket=bucket, record=point)

            except Exception as e:
                print(f"Skipping row due to error: {e}")
                print(f"Row data: {row}")

        print(f"Data from '{csv_file_path}' successfully uploaded to InfluxDB.")

    except Exception as e:
        print(f"Error processing file: {e}")
    finally:
        write_api.close()
        client.close()
        print("InfluxDB client closed.")

# Configuration
Input_file_path = "/home/carissma/new_ros-workspace/src/my_package/src/sensor_data/radar/radar_data.csv"  # Path to main radar.csv file
Output_file_path = "/home/carissma/new_ros-workspace/src/Scheduling/radar/output/divided_data/radar_data_converted.xlsx" #Radar data converted to metrics
csv_file_path = "/home/carissma/new_ros-workspace/src/Scheduling/radar/output/Filtered_output/Filtered_output.csv"  # Path of CSV file uploaded to Influxdb
measurement_name = "Radar_data"  # Replace with your measurement name
influx_url = "http://localhost:8086"  # Replace with your InfluxDB URL
token = "knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw=="  # Replace with your InfluxDB token
org = "winterproject24"  # Replace with your organization name
bucket = "radar_metrics_final"  # Replace with your bucket name

# Get processed timestamps
processed_timestamps = get_processed_timestamps(csv_file_path)

# Convert and filter radar data
convert_and_filter_radar_data(Input_file_path, Output_file_path, csv_file_path)

# Load filtered data
filtered_df = pd.read_csv(csv_file_path)

# Filter out already processed timestamps
new_data_df = filtered_df[~filtered_df['Unix_Timestamp'].isin(processed_timestamps)]

# Append the new data back to the filtered CSV file
if not new_data_df.empty:
    new_data_df.to_csv(csv_file_path, mode='a', header=False, index=False)

# Upload new data to InfluxDB
upload_csv_to_influxdb(csv_file_path, measurement_name, influx_url, token, org, bucket)
