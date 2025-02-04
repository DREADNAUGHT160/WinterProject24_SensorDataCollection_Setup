import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from matplotlib.colors import Normalize
from sklearn.cluster import MiniBatchKMeans, Birch
import os
import psutil

#############################################################################################################################################################
######### PROGRAM: Integrated LIDAR PROGRAM
######### Purpose: To Create the required angles, cartesian co ordinates implement BIRCH K MEANS ALGORITHM with frame by frame scan analysis.
######### PRE-REQUISITE: Lidar raw data csv
######### LAST CREATED DATE: 20/12/2024
######### LAST MODIFIED DATE: 12/01/2025
######### AUTHORS: AJAY|REEYAZ|SAGAR
#############################################################################################################################################################



# Define global output directory
OUTPUT_DIR = r"/home/carissma/new_ros-workspace/src/Scheduling/lidar/output"


# Ensure the output directory exists
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)



# 1 ANGLE CALCULATION
def calculate_angles(file_path, angular_resolution=0.25, start_angle=315, end_angle=225.25):
    """
    Calculate angles for LIDAR measurements from a CSV file and save a new file with the results.

    Parameters:
    - file_path (str): Path to the input .csv file.
    - angular_resolution (float): Angular resolution of the LIDAR (degrees).
    - start_angle (float): Starting angle in degrees (default is 315 degrees).
    - end_angle (float): Ending angle in degrees (default is 225 degrees).

    Returns:
    - str: Path to the updated file.
    """
    lidar_data = pd.read_csv(file_path)
    points_per_scan = len(lidar_data)

    angles_part_1 = np.arange(start_angle, 360, angular_resolution)
    angles_part_2 = np.arange(0, end_angle, angular_resolution)
    angles = np.concatenate((angles_part_1, angles_part_2))

    angle_repeats = points_per_scan // len(angles) + 1
    angles = np.tile(angles, angle_repeats)[:points_per_scan]
    angles = np.mod(angles, 360)

    lidar_data['Angle (degrees)'] = angles
    output_path = os.path.join(OUTPUT_DIR, os.path.basename(file_path).replace(".csv", "_with_angles.csv"))
    lidar_data.to_csv(output_path, index=False)

    print(f"New file with calculated angles saved to: {output_path}")
    return output_path

# 2 POLAR CORDINATES

def calculate_cartesian_coordinates(file_path):
    """
    Convert polar coordinates (Distance and Angle) to Cartesian coordinates (X, Y).

    Parameters:
    - file_path (str): Path to the input .csv file.

    Returns:
    - str: Path to the updated file.
    """
    lidar_data = pd.read_csv(file_path)

    # Debug: Print column names to verify existence of required columns
    print("Columns in the DataFrame:", lidar_data.columns)

    # Check for 'Distance' or 'Distance (m)' column
    if 'Distance (m)' in lidar_data.columns:
        distance_column = 'Distance (m)'
    elif 'Distance' in lidar_data.columns:
        distance_column = 'Distance'
    else:
        raise KeyError("The 'Distance' column is missing in the input file.")

    # Ensure 'Angle (degrees)' exists
    if 'Angle (degrees)' not in lidar_data.columns:
        raise KeyError("The 'Angle (degrees)' column is missing in the input file.")

    # Convert angles from degrees to radians
    lidar_data['Angle (radians)'] = np.radians(lidar_data['Angle (degrees)'])

    # Convert polar coordinates (r, θ) to Cartesian coordinates (x, y)
    lidar_data['X (m)'] = lidar_data[distance_column] * np.cos(lidar_data['Angle (radians)'])
    lidar_data['Y (m)'] = lidar_data[distance_column] * np.sin(lidar_data['Angle (radians)'])

    # Save to a new CSV file
    output_path = os.path.join(OUTPUT_DIR, os.path.basename(file_path).replace(".csv", "_with_cartesian.csv"))
    lidar_data.to_csv(output_path, index=False)

    print(f"New file with Cartesian coordinates saved to: {output_path}")
    return output_path


# Function to print memory usage
def print_memory_usage(step):
    process = psutil.Process(os.getpid())
    memory = process.memory_info().rss / (1024 ** 2)  # Convert to MB
    print(f"[Memory Usage] {step}: {memory:.2f} MB")

# 3 BIRCH- MiniBatchKMeans ALGORITHM
def birckmeans(file_path):
    
    df = pd.read_csv(cartesian_file)
    print("File loaded successfully FOR DB SCAN")
# Assume the CSV contains columns 'x', 'y', 'timestamp', and 'intensity'
    X = df[['X (m)', 'Y (m)']].values  # Extract 'x' and 'y' values
    timestamps = df['Timestamp'].values  # Extract the corresponding timestamps
    intensities = df['Intensity'].values  # Extract the intensity values

# Define Target Identification Criteria
    print("Target filtering...")
    target_condition = (df['X (m)'] > -6) & (df['X (m)'] < 6) & (df['Y (m)'] > -7) & (df['Y (m)'] < 7)
    target_points = df[target_condition]
    print_memory_usage("After filtering target points")

# Extract filtered points
    X_target = target_points[['X (m)', 'Y (m)']].values
    timestamps_target = target_points['Timestamp'].values
    intensities_target = target_points['Intensity'].values
    print("Target points filtered successfully.")

# Visualize Target Points
    plt.scatter(X[:, 0], X[:, 1], s=10, c='gray', label="All Points")
    plt.scatter(X_target[:, 0], X_target[:, 1], s=10, c='red', label="Target Points")
    plt.title("Identified Target Points")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    
    print("Visualization of target points completed.")
    target_viz_path = os.path.join(OUTPUT_DIR, "target_points.png")
    plt.savefig(target_viz_path, dpi=300)
    print(f"target_points clustering visualization saved to: {target_viz_path}")
    plt.show()

# Preprocess Target Points (Scaling)
    print("Scaling target points...")
    scaler = StandardScaler()
    X_target_scaled = scaler.fit_transform(X_target)
    print("Target points scaled successfully.")

#   Optionally Apply BIRCH for Dimensional Reduction
    use_birch = True  # Set to True if BIRCH is needed before KMeans
    if use_birch:
        print("Applying BIRCH clustering for pre-clustering...")
        birch = Birch(n_clusters=None, threshold=0.7, branching_factor=70)
        X_target_birch = birch.fit_transform(X_target_scaled)
        print("BIRCH clustering completed.")
        print_memory_usage("After BIRCH clustering")
    else:
        X_target_birch = X_target_scaled

#   Apply MiniBatchKMeans
    print("Applying MiniBatchKMeans clustering...")
    minibatch_kmeans = MiniBatchKMeans(n_clusters=10, batch_size=1000, random_state=42)
    minibatch_kmeans_labels = minibatch_kmeans.fit_predict(X_target_birch)
    print("MiniBatchKMeans clustering completed.")
    print_memory_usage("After MiniBatchKMeans clustering")

#   Map Cluster Labels to Target Points, Timestamps, and Intensities
    print("Mapping cluster labels to target points...")
    result_df = pd.DataFrame({
        'timestamp': timestamps_target,
        'x': X_target[:, 0],
        'y': X_target[:, 1],
        'intensity': intensities_target,
        'cluster': minibatch_kmeans_labels
    })
    print("Cluster labels mapped successfully.")

#   Visualize MiniBatchKMeans Clustering Results
    print("Visualizing MiniBatchKMeans clustering results...")
    unique_labels = set(minibatch_kmeans_labels)
    plt.figure(figsize=(10, 6))
    for label in unique_labels:
        color = plt.cm.Spectral(float(label) / len(unique_labels)) if label != -1 else 'black'
        label_name = f"Cluster {label}" if label != -1 else "Noise"
        cluster_points = result_df[result_df['cluster'] == label]
        plt.scatter(cluster_points['x'], cluster_points['y'], s=10, c=[color], label=label_name)

    plt.title("MiniBatchKMeans Clustering Results for Target Points")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    
    print("Visualization of MiniBatchKMeans clustering results completed.")
    clustering_viz_path = os.path.join(OUTPUT_DIR, "minibatchkmeans_clustering.png")
    plt.savefig(clustering_viz_path, dpi=300)
    print(f"MiniBatchKMeans clustering visualization saved to: {clustering_viz_path}")
    plt.show()


#   Save Results
    print("Saving the results to a new CSV file...")
    output_path = os.path.join(OUTPUT_DIR, "minibatchkmeans_target_clustered_output_with_intensity.csv")
    result_df.to_csv(output_path, index=False)
    print(f"Target clustering results saved to: {file_path}")
    print_memory_usage("After saving results")

# 4 ANIMATION
def visualize_lidar_data(file_path, rows_per_cycle=1080):
    """
    Visualize LiDAR data over time using an animated scatter plot.

    Parameters:
    - file_path (str): Path to the input .csv file.
    - rows_per_cycle (int): Number of rows per cycle.
    """
    df = pd.read_csv(cartesian_file)
    print("File read successfully")

    # Step 2: Define cycle parameters
    rows_per_cycle = 1080  # Number of rows in each cycle
    total_rows = len(df)   # Total number of rows in the dataset
    num_cycles = total_rows // rows_per_cycle  # Number of complete cycles

    print(f"Total rows: {total_rows}, Rows per cycle: {rows_per_cycle}, Total cycles: {num_cycles}")

    # Step 3: Prepare the plot
    fig, ax = plt.subplots(figsize=(16, 16))  # Increased the size to 16x16 inches

    # Define the axis limits for X and Y from -30 to 30 meters
    x_limit = (-30, 30)  # X-axis range in meters
    y_limit = (-30, 30)  # Y-axis range in meters

    # Create a Normalize object for the intensity values to control color mapping
    norm = Normalize(vmin=df['Intensity'].min(), vmax=df['Intensity'].max())

    # Create an initial scatter plot to define the colorbar
    scatter = ax.scatter([], [], c=[], cmap='inferno', s=10, norm=norm)
    cbar = fig.colorbar(scatter, ax=ax, label='Intensity')  # Add colorbar once
    print(" START ANIMATION")
# Step 4B: Define update function for animation
    def update_ani(frame_idx):
        #print(f"Animating frame {frame_idx}")
        ax.clear()  # Clear the plot for the next frame
        start_row = frame_idx * rows_per_cycle
        end_row = start_row + rows_per_cycle
        cycle_data = df.iloc[start_row:end_row]

    # Extract data for the current cycle
        frame_X = cycle_data[['X (m)', 'Y (m)']].values
        frame_intensity = cycle_data['Intensity'].values

    # Plot the current cycle
        scatter = ax.scatter(
        frame_X[:, 0], frame_X[:, 1], c=frame_intensity, cmap='inferno', s=10, norm=norm
        )

    # Plot the 270-degree field of view as a filled circle with light red color
        circle = patches.Wedge(
        (0, 0), 30, 315, 225,  # Center of the circle (0,0), radius 30m, angle from 315 to 225
        color='lightcoral', alpha=0.3  # Light red color with some transparency
        )
        ax.add_patch(circle)  # Add the circle to the plot

    # Set plot title and labels
        ax.set_title(f"LiDAR Data - Cycle {frame_idx + 1}")
        ax.set_xlabel("X Coordinate")
        ax.set_ylabel("Y Coordinate")
        ax.grid(True)

    # Set fixed axis limits
        ax.set_xlim(x_limit)
        ax.set_ylim(y_limit)

    # Set the aspect ratio to be equal (square graph)
        ax.set_aspect('equal', 'box')

    # Set the x and y ticks at intervals of 5 meters
        ax.set_xticks(np.arange(x_limit[0], x_limit[1] + 5, 5))
        ax.set_yticks(np.arange(y_limit[0], y_limit[1] + 5, 5))

# Step 5: Create the animation
    ani = FuncAnimation(fig, update_ani, frames=num_cycles, repeat=True, interval=500)
    
# Save the first frame as PNG
    update_ani(0)  # Render the first frame
    animation_frame_path = os.path.join(OUTPUT_DIR, "lidar_first_frame.png")
    plt.savefig(animation_frame_path, dpi=300)
    print(f"First animation frame saved to: {animation_frame_path}")
    #plt.show()
    print("ANIMATION CYCLE PLOTTED")
    print("ALL PROCESS COMPLETE")

# 5 TIMESTAMP CONVERSION
def convert_timestamp_to_nanoseconds(file_path):
    """
    Convert the timestamp column of a CSV file into nanoseconds since the epoch.

    Parameters:
    - file_path (str): Path to the input CSV file.
    - output_path (str): Path to save the output CSV file with the converted timestamps.

    Returns:
    - None
    """
    print("NANO TIMESTAMP CONVERSION STARTED")
    # Read the CSV file
    data = pd.read_csv(file_path)
    print(data['timestamp'].head())

    # Convert the 'timestamp' column to nanosecond precision
    if 'timestamp' in data.columns:
        data['timestamp_ns'] = pd.to_datetime(data['timestamp']).astype('int64')  # Convert to nanoseconds
        print("Converted 'timestamp' to 'timestamp_ns'.")
    else:
        print("Column 'timestamp' not found.")
    # Save to a new CSV file
    output_path = os.path.join(OUTPUT_DIR, os.path.basename(file_path).replace("minibatchkmeans_target_clustered_output_with_intensity.csv", "updated_dataset_with_nanoseconds.csv"))
    data.to_csv(output_path, index=False)
    print(f"Updated dataset saved to {output_path}")
    print("ALL PROCESS COMPLETE")

if __name__ == "__main__":
    input_file = r"/home/carissma/new_ros-workspace/src/my_package/src/sensor_data/lidar/lidar_data_detailed.csv"
    input_file_nano = r"/home/carissma/new_ros-workspace/src/Scheduling/lidar/output/minibatchkmeans_target_clustered_output_with_intensity.csv"
    #nanoseconds_file = input_file.replace(".csv", "_with_nanoseconds.csv")
    angles_file = calculate_angles(input_file)
    cartesian_file = calculate_cartesian_coordinates(angles_file)
    birckmeans(cartesian_file)
    visualize_lidar_data(cartesian_file)
    convert_timestamp_to_nanoseconds(input_file_nano)
    