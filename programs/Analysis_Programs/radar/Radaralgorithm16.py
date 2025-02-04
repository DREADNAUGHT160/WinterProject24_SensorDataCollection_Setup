import os
import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import imageio.v2 as imageio
import re


# Configuration and constants
# Define file paths for input radar data and output files
input_file = '/home/carissma/new_ros-workspace/src/my_package/src/sensor_data/radar/radar_data.csv'
output_file = '/home/carissma/new_ros-workspace/src/Scheduling/radar/output/divided_data/radar_data_divided_by_timestamp.xlsx'
output_folder = '/home/carissma/new_ros-workspace/src/Scheduling/radar/output/NewResults'

# Define DBSCAN clustering parameters
DBSCAN_EPS = 1.00
DBSCAN_MIN_SAMPLES = 1
INTENSITY_THRESHOLD = 20

# Ensure the output folder exists
os.makedirs(output_folder, exist_ok=True)

# Function to calculate range, azimuth, and elevation from X, Y, Z coordinates
def calculate_range_azimuth_elevation(df):
    df['range'] = np.sqrt(df['X']**2 + df['Y']**2 + df['Z']**2)
    df['azimuth'] = np.degrees(np.arctan2(df['Y'], df['X']))
    df['elevation'] = np.degrees(np.arctan2(df['Z'], np.sqrt(df['X']**2 + df['Y']**2)))
    return df

# Function to convert Unix timestamp to human-readable time
def convert_unix_to_actual_time(df):
    df['actualtime'] = pd.to_datetime(df['Unix_Timestamp'], unit='s')
    return df

# Function to process data using DBSCAN clustering
def process_sheet_with_dbscan(df, eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES, intensity_threshold=INTENSITY_THRESHOLD):
    clustering_features = df[['range', 'Intensity']].values
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(clustering_features)
    df['cluster'] = db.labels_
    reflectors = df[(df['cluster'] != -1) & (df['Intensity'] >= intensity_threshold)]
    reflector_info = reflectors.groupby('cluster').agg({
        'range': 'max', 'Intensity': 'mean', 'X': 'mean', 'Y': 'mean', 'Z': 'mean',
        'azimuth': 'mean', 'elevation': 'mean'
    }).reset_index()
    return df, reflectors, reflector_info

# Function to visualize reflectors in a 3D scatter plot
def visualize_clusters(reflectors, reflector_info, sheet_name, global_min_x, global_max_x, global_min_y, global_max_y, global_min_z, global_max_z):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    scatter = ax.scatter(reflectors['X'], reflectors['Y'], reflectors['Z'], 
                         c=reflectors['Intensity'], cmap='viridis', s=20)
    for _, reflector in reflector_info.iterrows():
        ax.text(reflector['X'], reflector['Y'], reflector['Z'], f"r{int(reflector['cluster'])}", 
                color='red', fontsize=10)

    # Apply consistent axis limits
    ax.set_xlim(global_min_x, global_max_x)
    ax.set_ylim(global_min_y, global_max_y)
    ax.set_zlim(global_min_z, global_max_z)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Reflector Clusters for {sheet_name}')
    plt.colorbar(scatter, label='Intensity')

    return fig

# Function to save processed reflector information to CSV
def save_results_to_csv(reflector_info, sheet_name):
    csv_path = os.path.join(output_folder, f"{sheet_name}_reflectors.csv")
    reflector_info.to_csv(csv_path, index=False)
    print(f"Reflector data saved for {sheet_name} to {csv_path}.")

# Function to create a GIF animation from processed images
def create_gif_from_processed_images(output_folder, gif_filename="reflector_clusters_animation.gif"):
    pattern = re.compile(r'(\d+)_clusters\.png$')
    image_files = [f for f in os.listdir(output_folder) if pattern.search(f)]
    sorted_images = sorted(image_files, key=lambda x: int(pattern.search(x).group(1)))
    frames = [imageio.imread(os.path.join(output_folder, img)) for img in sorted_images]
    
    if frames:
        gif_path = os.path.join(output_folder, gif_filename)
        imageio.mimsave(gif_path, frames, duration=0.5)
        print(f"GIF animation saved to {gif_path}")
    else:
        print("No images found to create GIF.")

if __name__ == "__main__":
    df = pd.read_csv(input_file)
    df['Unix_Timestamp'] = df['Unix_Timestamp'].astype(float).astype(int)
    
    df = convert_unix_to_actual_time(df)
    df = calculate_range_azimuth_elevation(df)

    # Calculate global axis limits
    global_min_x = df['X'].min()
    global_max_x = df['X'].max()
    global_min_y = df['Y'].min()
    global_max_y = df['Y'].max()
    global_min_z = df['Z'].min()
    global_max_z = df['Z'].max()

    grouped = df.groupby('Unix_Timestamp')
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        for timestamp, group in grouped:
            sheet_name = f'{timestamp}'
            if len(sheet_name) > 31:
                sheet_name = f'UTS_{timestamp}'
            group.to_excel(writer, sheet_name=sheet_name, index=False)

    print(f"Data has been divided by Unix_Timestamp, and processed data is saved to '{output_file}'.")

    # Load processed timestamps
    timestamp_log_path = os.path.join(output_folder, "processed_timestamps.txt")
    existing_timestamps = set()
    if os.path.exists(timestamp_log_path):
        with open(timestamp_log_path, 'r') as f:
            existing_timestamps = set(line.strip() for line in f.readlines())

    print(f"Processed timestamps already: {existing_timestamps}")

    xls = pd.ExcelFile(output_file)
    sheet_names = xls.sheet_names
    frames = []

    for sheet_name in sheet_names:
        print(f"Processing sheet: {sheet_name}")

        # Skip already processed sheets
        if sheet_name in existing_timestamps:
            print(f"Skipping already processed timestamp: {sheet_name}")
            continue

        df_sheet = xls.parse(sheet_name)

        required_columns = ['X', 'Y', 'Z', 'Intensity', 'range', 'azimuth', 'elevation']
        if not all(col in df_sheet.columns for col in required_columns):
            print(f"Skipping {sheet_name}: Missing required columns.")
            continue

        _, reflectors, reflector_info = process_sheet_with_dbscan(df_sheet)

        fig = visualize_clusters(reflectors, reflector_info, sheet_name, global_min_x, global_max_x, global_min_y, global_max_y, global_min_z, global_max_z)
        frame_path = os.path.join(output_folder, f"{sheet_name}_clusters.png")
        plt.savefig(frame_path)
        frames.append(imageio.imread(frame_path))
        plt.close(fig)

        save_results_to_csv(reflector_info, sheet_name)

        # Log the processed timestamp
        print(f"Logging processed timestamp: {sheet_name}")
        with open(timestamp_log_path, 'a') as f:
            f.write(f"{sheet_name}\n")

    create_gif_from_processed_images(output_folder)

    print("Processing completed.")

