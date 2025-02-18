# **Integrated LiDAR Processing and Clustering System**

![Python](https://img.shields.io/badge/Python-3.8+-yellow) ![LiDAR](https://img.shields.io/badge/LiDAR-Processing-green) ![InfluxDB](https://img.shields.io/badge/InfluxDB-Enabled-blue)

This **Integrated LiDAR Processing Program** performs **preprocessing, clustering, visualization, and database integration** for LiDAR sensor data. It calculates **angles**, converts **polar to Cartesian coordinates**, applies **BIRCH + MiniBatch K-Means clustering**, **visualizes LiDAR frames**, and **uploads the results to InfluxDB**.

---

## **Features**
✅ **Reads LiDAR raw data from CSV** and calculates necessary **angles**.  
✅ **Converts polar coordinates to Cartesian coordinates (X, Y)**.  
✅ **Filters target points based on predefined X, Y range criteria**.  
✅ **Applies BIRCH and MiniBatch K-Means clustering on LiDAR points**.  
✅ **Generates visualizations for clustered target points**.  
✅ **Creates an animated scatter plot for LiDAR scans**.  
✅ **Converts timestamps to nanosecond precision** for InfluxDB compatibility.  
✅ **Uploads processed LiDAR data to InfluxDB** for cloud storage.  

---

## **File Structure**
```
your_project/
├── lidar_processing.py             <- Main LiDAR processing script
├── upload_lidar_to_influxdb.py     <- Upload processed LiDAR data to InfluxDB
├── sensor_data/
│   ├── lidar_data_detailed.csv     <- Raw LiDAR input data
│   └── ...
├── output/
│   ├── lidar_with_angles.csv       <- LiDAR data with calculated angles
│   ├── lidar_with_cartesian.csv    <- Converted Cartesian coordinates
│   ├── minibatchkmeans_clusters.png  <- Clustering visualization
│   ├── target_points.png           <- Filtered target points visualization
│   ├── lidar_first_frame.png       <- First animation frame
│   ├── lidar_animation.gif         <- LiDAR scan animation
│   ├── updated_dataset_with_nanoseconds.csv  <- Final dataset with nanosecond timestamps
│   ├── processed_timestamps.txt    <- Processed timestamps log
└── ...
```

---

## **Installation & Dependencies**
### **1️⃣ Prerequisites**
- **Python 3.8+**
- **InfluxDB setup** (for cloud storage of processed LiDAR data)

### **2️⃣ Install Required Libraries**
Run the following command to install dependencies:
```bash
pip install pandas numpy matplotlib scikit-learn psutil influxdb-client
```

---

## **How It Works**

### **🔹 Step 1: Angle Calculation**
- Reads **LiDAR raw data** from `sensor_data/lidar_data_detailed.csv`.
- Calculates angles for each LiDAR measurement.
- Saves the updated dataset as `lidar_with_angles.csv`.

### **🔹 Step 2: Convert to Cartesian Coordinates**
- Uses **distance and angle** to compute **(X, Y) coordinates**.
- Saves the output as `lidar_with_cartesian.csv`.

### **🔹 Step 3: Clustering with BIRCH + MiniBatch K-Means**
- Filters **target points** within a predefined range.
- Applies **BIRCH clustering** for dimensionality reduction.
- Runs **MiniBatch K-Means clustering** for identifying **object groups**.
- Saves **clustering results** in CSV and image format.

### **🔹 Step 4: Animated LiDAR Visualization**
- Animates LiDAR scans over time as a **scatter plot**.
- Saves the animation as `lidar_animation.gif`.

### **🔹 Step 5: Convert Timestamps to Nanoseconds**
- Converts **timestamps** to **nanoseconds** for InfluxDB compatibility.
- Saves the output as `updated_dataset_with_nanoseconds.csv`.

### **🔹 Step 6: Upload to InfluxDB**
- Reads the **processed LiDAR dataset**.
- Uploads data to **InfluxDB** for real-time monitoring and cloud storage.

---

## **Data Processing Metrics**
| **Metric**       | **Description** |
|-----------------|----------------|
| **Angle (degrees)** | Angle of the LiDAR beam during a scan. |
| **X, Y Coordinates** | Cartesian representation of scanned objects. |
| **Intensity**   | Strength of LiDAR reflection. |
| **Cluster ID**  | Identified object groups using clustering. |
| **Timestamp (ns)** | Nanosecond precision timestamps for InfluxDB. |

---

## **How to Run the Program**
### **1️⃣ Start LiDAR Processing**
```bash
python lidar_processing.py
```
### **2️⃣ Upload Processed Data to InfluxDB**
```bash
python upload_lidar_to_influxdb.py
```

### **3️⃣ Outputs Generated**
- **Processed LiDAR data** saved in `/output/`  
- **Clustered LiDAR visualization** saved as `minibatchkmeans_clusters.png`  
- **Animated LiDAR scans** saved as `lidar_animation.gif`  
- **Final dataset with timestamps** saved as `updated_dataset_with_nanoseconds.csv`  
- **Uploaded to InfluxDB** for further analysis  

---

## **🔄 InfluxDB Integration**
This program uploads **LiDAR data** to **InfluxDB** for real-time analytics.

### **InfluxDB Setup**
1. Run InfluxDB (if using Docker):
   ```bash
   docker run -d -p 8086:8086 --name influxdb influxdb
   ```
2. Create a bucket: `lidar_metrics`
3. Update the script with:
   ```python
   INFLUXDB_URL = "http://localhost:8086"
   INFLUXDB_TOKEN = "YOUR_INFLUXDB_TOKEN"
   INFLUXDB_ORG = "winterproject24"
   INFLUXDB_BUCKET = "lidar_metrics"
   ```

---

## **Troubleshooting**
### **1️⃣ No angles calculated**
- Ensure the **input CSV file exists** in `sensor_data/lidar_data_detailed.csv`.

### **2️⃣ Clustering results not visible**
- Adjust **BIRCH threshold and K-Means clusters** for better grouping.

### **3️⃣ InfluxDB data not saving**
- Verify InfluxDB is running (`docker ps`).
- Check InfluxDB **bucket and token settings**.

### **4️⃣ Animation is blank**
- Ensure there are **enough LiDAR points** to animate.

---

## **Future Enhancements**
📌 **Real-time LiDAR streaming** with ROS.  
📌 **Advanced clustering techniques** (e.g., DBSCAN, HDBSCAN).  
📌 **Integration with 3D visualization tools** (e.g., Open3D, PCL).  
📌 **Enhance InfluxDB queries for real-time analytics**.  

---

## **License**
This project is open-source under the **MIT License**. Feel free to contribute!

---

### **Need Help?**
For questions or contributions, open an **issue** in the GitHub repository! 🚀

---

## **Program Breakdown**

### **🔹 PROGRAM 1: MASTER PROGRAM - `master_lidar.py`**
#### **Overview:**
This is the **main LiDAR processing program** that handles **data ingestion, preprocessing, target filtering, clustering, 2D visualization, and animation**.

#### **Input File:**
```
/home/carissma/new_rosworkspace/src/my_package/src/sensor_data/lidar/lidar_data_detailed.csv
```

#### **Functions & Outputs:**
| **Function** | **Description** | **Output File** |
|-------------|---------------|----------------|
| **calculate_angles** | Calculates angles for LiDAR measurements and saves the results. | `/home/carissma/new_rosworkspace/src/Scheduling/lidar/output/lidar_data_detailed_with_angles.csv` |
| **calculate_cartesian_coordinates** | Converts polar coordinates (Distance, Angle) to Cartesian (X, Y). | `/home/carissma/new_rosworkspace/src/Scheduling/lidar/output/lidar_data_detailed_with_angles_with_cartesian.csv` |
| **print_memory_usage** | Monitors memory usage of the process. | (No file output, prints to console) |
| **Birckmeans** | Applies **BIRCH and MiniBatchKMeans** clustering, filtering target points before clustering. | `/home/carissma/new_rosworkspace/src/Scheduling/lidar/output/minibatchkmeans_target_clustered_output_with_intensity.csv` |
| | | `/home/carissma/new_rosworkspace/src/Scheduling/lidar/output/minibatchkmeans_clustering.png` |
| **visualize_lidar_data** | Animates LiDAR scans over time using scatter plots. | `/home/carissma/new_rosworkspace/src/Scheduling/lidar/output/lidar_first_frame.png` |
| **convert_timestamp_to_nanoseconds** | Converts timestamps to **nanoseconds** since the epoch for database compatibility. | `/home/carissma/new_rosworkspace/src/Scheduling/lidar/output/lidar_data_detailed_with_nanoseconds.csv` |

---

### **PROGRAM 2: UPLOAD SCRIPT - `upload_lidar_data_script_to_influxdb.py`**
#### **Overview:**
This script uploads **processed LiDAR data** to an **InfluxDB server** using predefined credentials.

#### **Output:**
✔ **Data uploaded to InfluxDB for real-time analytics and cloud storage.**

---
