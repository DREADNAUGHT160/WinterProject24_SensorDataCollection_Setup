import cv2
import numpy as np
from pupil_apriltags import Detector
from skimage.measure import shannon_entropy
import os
import csv
from influxdb_client import InfluxDBClient, Point, WriteApi
from influxdb_client.client.write_api import SYNCHRONOUS
import time

# Initialize the AprilTag detector
detector = Detector(families="tag36h11")  # Commonly used tag family

# InfluxDB Configuration
INFLUXDB_URL = "http://localhost:8086"  # Replace with your InfluxDB URL
INFLUXDB_TOKEN = "knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw=="  # Replace with your InfluxDB token
INFLUXDB_ORG = "winterproject24"               # Replace with your InfluxDB organization
INFLUXDB_BUCKET = "image_metrics"       # Replace with your InfluxDB bucket name

# Initialize the InfluxDB client
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)


def load_image(filepath):
    """Load an image from a file."""
    image = cv2.imread(filepath)
    if image is None:
        raise FileNotFoundError(f"Error: Could not load image at {filepath}.")
    return image


def compute_sharpness(image):
    """Calculate sharpness using the variance of the Laplacian."""
    laplacian_var = cv2.Laplacian(image, cv2.CV_64F).var()
    return laplacian_var


def compute_contrast(image):
    """Calculate image contrast."""
    contrast = image.max() - image.min()
    return contrast


def compute_colorfulness(image):
    """Calculate colorfulness of an image."""
    rg = image[..., 0] - image[..., 1]
    yb = 0.5 * (image[..., 0] + image[..., 1]) - image[..., 2]
    std_root = np.sqrt(np.mean(rg ** 2) + np.mean(yb ** 2))
    mean_root = np.mean(np.abs(rg)) + np.mean(np.abs(yb))
    return std_root + 0.3 * mean_root


def compute_entropy(image):
    """Calculate the Shannon entropy of an image."""
    entropy = shannon_entropy(image)
    return entropy


def compute_brightness(image):
    """Calculate the average brightness of an image."""
    brightness = np.mean(image)
    return brightness

def compute_noise(image):
    """
    Estimate the noise level of an image and scale it to range 0-9 for visualization.
    
    Parameters:
        image: Input image in BGR format.
        
    Returns:
        scaled_noise: Scaled noise level in the range 0-9.
    """
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Compute the Fourier Transform and shift zero frequency to the center
    fft_image = np.fft.fft2(gray_image)
    fft_shifted = np.fft.fftshift(fft_image)
    
    # Compute Power Spectral Density (PSD) and normalize
    psd = np.abs(fft_shifted) ** 2
    psd_normalized = psd / np.sum(psd)
    
    # Create a high-frequency mask by excluding the low-frequency region
    h, w = psd.shape
    center_x, center_y = w // 2, h // 2
    low_freq_radius = min(h, w) // 8
    y, x = np.ogrid[:h, :w]
    mask = ((x - center_x) ** 2 + (y - center_y) ** 2) >= low_freq_radius ** 2
    
    # Compute the high-frequency noise contribution
    high_freq_noise = psd_normalized[mask].sum()
    
    # Normalize the noise value to [0, 1] and scale to [0, 9]
    noise = high_freq_noise / high_freq_noise.max() * 9 if high_freq_noise > 0 else 0
    
    return noise




def enhance_and_sharpen(image):
    """Enhance and sharpen an image."""
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    merged_lab = cv2.merge((cl, a, b))
    image_clahe = cv2.cvtColor(merged_lab, cv2.COLOR_LAB2BGR)

    gray_image = cv2.cvtColor(image_clahe, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray_image, (9, 9), 10)
    sharpened = cv2.addWeighted(gray_image, 1.5, blurred, -0.5, 0)
    sharpened_color = cv2.cvtColor(sharpened, cv2.COLOR_GRAY2BGR)

    return sharpened_color


def annotate_image(image, detections, metrics, output_filepath):
    """Annotate the image with detected tags and metrics."""
    annotated_image = image.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    color = (0, 255, 0)  # Green for text and markers

    # Annotate detected tags
    for detection in detections:
        center_x, center_y = map(int, detection.center)
        tag_id = detection.tag_id
        cv2.circle(annotated_image, (center_x, center_y), radius=5, color=color, thickness=-1)
        cv2.putText(
            annotated_image, f"ID: {tag_id}", 
            (center_x + 10, center_y - 10), font, font_scale, color, thickness, cv2.LINE_AA
        )

    # Add metrics as a legend
    legend_x, legend_y = 10, 20  # Starting position for the legend
    line_spacing = 20
    metrics_text = [
        f"Sharpness: {metrics[0]:.2f}",
        f"Contrast: {metrics[1]:.2f}",
        f"Colorfulness: {metrics[2]:.2f}",
        f"Entropy: {metrics[3]:.2f}",
        f"Brightness: {metrics[4]:.2f}",
        f"Noise Level: {metrics[5]:.2f}",
        f"Total Tags: {metrics[6]}",
    ]

    for i, text in enumerate(metrics_text):
        y_position = legend_y + i * line_spacing
        cv2.putText(
            annotated_image, text, 
            (legend_x, y_position), font, font_scale, color, thickness, cv2.LINE_AA
        )

    # Save the annotated image
    cv2.imwrite(output_filepath, annotated_image)
    print(f"Annotated image saved to {output_filepath}")


def save_results_to_csv(image_name, metrics, target_scores, image_path, light_condition, output_csv):
    """Save results to a CSV file."""
    file_exists = os.path.exists(output_csv)

    with open(output_csv, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write the header if the file is new
        if not file_exists:
            writer.writerow([
                "Image Name",
                "Sharpness",
                "Contrast",
                "Colorfulness",
                "Entropy",
                "Brightness",
                "Noise Level",
                "Total Tags Detected",
                "Target1 Score",
                "Target2 Score",
                "Target3 Score",
                "Target4 Score",
                "Analyzed Image Path",
                "Light Condition"
            ])

        # Append the row with metrics, target scores, and image path
        writer.writerow([
            image_name,
            *metrics,
            target_scores.get("Target1", 0),
            target_scores.get("Target2", 0),
            target_scores.get("Target3", 0),
            target_scores.get("Target4", 0),
            image_path,
            light_condition
        ])


def main():
    input_folder = '/home/carissma/new_ros-workspace/src/my_package/src/sensor_data/camera'
    output_folder = '/home/carissma/new_ros-workspace/src/Scheduling/camera/output/output_images'
    output_csv = "/home/carissma/new_ros-workspace/src/Scheduling/camera/output/image_metrics.csv"
    track_file = "/home/carissma/new_ros-workspace/src/Scheduling/camera/output/processed_images.txt"

    # Brightness threshold for night/low-light detection
    LOW_LIGHT_THRESHOLD = 50

    # Create the output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)

    targets = {
        "Target1": {12,13,14,15,16,17,18,19,20,21,21,23},
        "Target2": {1},
        "Target3": {2},
        "Target4": {3}
    }

    processed_images = set()
    if os.path.exists(track_file):
        with open(track_file, "r") as f:
            processed_images = set(f.read().splitlines())

    for image_file in os.listdir(input_folder):
        if not image_file.lower().endswith(('png', 'jpg', 'jpeg')) or image_file in processed_images:
            print(f"Skipping already processed image: {image_file}")
            continue

        try:
            filepath = os.path.join(input_folder, image_file)
            image_name = os.path.splitext(image_file)[0]
            image = load_image(filepath)

            enhanced_image = enhance_and_sharpen(image)
            grayscale_image = cv2.cvtColor(enhanced_image, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(grayscale_image)

            detected_ids = {detection.tag_id for detection in detections}

            target_scores = {
                target: (10 if detected_ids & ids == ids else 5 if detected_ids & ids else 0)
                for target, ids in targets.items()
            }

            sharpness = compute_sharpness(image)
            contrast = compute_contrast(grayscale_image)
            colorfulness = compute_colorfulness(enhanced_image)
            entropy = compute_entropy(grayscale_image)
            brightness = compute_brightness(grayscale_image)
            noise_level = compute_noise(enhanced_image)

            unix_timestamp = image_name.replace("camera_", "").replace(".jpg", "")
            unixtime = int(unix_timestamp)

            #light_condition = "Night" if brightness < LOW_LIGHT_THRESHOLD else "Day"
            local_time = time.localtime(unixtime)

            # Extract the hour
            hour = local_time.tm_hour

            # Define day and night boundaries
            DAY_START = 6  # 6 AM
            DAY_END = 18   # 6 PM

            # Determine day or night based on the hour
            light_condition = "Day" if DAY_START <= hour < DAY_END else "Night"

            # Print the result
            print(f"Hour: {hour}")
            print(f"Light Condition: {light_condition}")

            metrics = [
                sharpness, contrast, colorfulness, entropy, brightness, noise_level, len(detections)
            ]

            output_filepath = os.path.join(output_folder, f"{image_name}_analyzed.png")

            # Annotate and save the image
            annotate_image(enhanced_image, detections, metrics, output_filepath)

            save_results_to_csv(image_name, metrics, target_scores, output_filepath, light_condition, output_csv)
            timestamp = image_name.replace("camera_", "").replace(".jpg", "")
            timestamp = int(timestamp)
            ns_timestamp = int(timestamp * 1e9)
            

            point = Point("image_metrics") \
                .time(ns_timestamp) \
                .tag("light_condition", light_condition) \
                .field("sharpness", sharpness) \
                .field("contrast", contrast) \
                .field("colorfulness", colorfulness) \
                .field("entropy", entropy) \
                .field("brightness", brightness) \
                .field("noise_level", noise_level) \
                .field("total_tags_detected", len(detections)) \
                .field("Target1_score", target_scores.get("Target1", 0)) \
                .field("Target2_score", target_scores.get("Target2", 0)) \
                .field("Target3_score", target_scores.get("Target3", 0)) \
                .field("Target4_score", target_scores.get("Target4", 0))

            write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)

            with open(track_file, "a") as f:
                f.write(image_file + "\n")

            print(f"Processed and saved: {image_file}")

        except Exception as e:
            print(f"Error processing {image_file}: {e}")


if __name__ == "__main__":
    main()
