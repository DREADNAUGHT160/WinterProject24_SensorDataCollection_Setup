# **Radar Data Processing and Clustering System**

![Python](https://img.shields.io/badge/Python-3.8+-yellow) ![DBSCAN](https://img.shields.io/badge/DBSCAN-Clustering-green) ![InfluxDB](https://img.shields.io/badge/InfluxDB-Enabled-blue)

This repository contains **two Python programs** for **processing radar data**, applying **DBSCAN clustering**, **filtering data**, and **uploading to InfluxDB**.

1. **RadarAlgorithm16**: 
   - Applies **DBSCAN clustering** on radar data.
   - Generates **3D visualizations** of reflector clusters.
   - Saves processed data in CSV files and images.
   - Creates a **GIF animation** from processed frames.

2. **RadarAlgorithm23**: 
   - Converts **Unix timestamps** to **human-readable time**.
   - Filters radar data based on **range and azimuth conditions**.
   - Uploads **filtered data** directly to **InfluxDB**.

---

## **Features**
✅ **Processes radar sensor data** and calculates key metrics.  
✅ **Clusters reflectors using DBSCAN** based on **intensity and range**.  
✅ **Generates 3D scatter plots** of reflector clusters.  
✅ **Saves clustered radar data** in CSV format.  
✅ **Creates a GIF animation** from processed radar frames.  
✅ **Filters radar data** and uploads the results to **InfluxDB**.  
✅ **Prevents duplicate processing** by tracking timestamps.  

---

## **File Structure**
```
your_project/
├── radar_algorithm16.py        <- Radar clustering and visualization
├── radar_algorithm23.py        <- Radar data filtering and InfluxDB upload
├── sensor_data/
│   ├── radar_data.csv          <- Raw radar data input
│   └── ...
├── output/
│   ├── divided_data/           <- Processed radar data split by timestamp
│   ├── NewResults/             <- Folder for visualization outputs
│   ├── Filtered_output.csv     <- Final filtered radar data
│   ├── processed_timestamps.txt <- Track processed timestamps
│   ├── reflector_clusters_animation.gif <- Generated GIF animation
└── ...
```

---

## **Installation & Dependencies**
### **1️⃣ Prerequisites**
- **Python 3.8+**
- **InfluxDB setup** (for RadarAlgorithm23)

### **2️⃣ Install Required Libraries**
Run the following command to install dependencies:
```bash
pip install pandas numpy scikit-learn matplotlib imageio influxdb-client openpyxl
```

---

## **How It Works**

### **🔹 RadarAlgorithm16 (Clustering & Visualization)**
1. **Loads radar data** from `sensor_data/radar_data.csv`.
2. **Calculates range, azimuth, and elevation**.
3. **Applies DBSCAN clustering** on reflectors.
4. **Generates 3D scatter plots** for each timestamp.
5. **Saves reflector data to CSV and images**.
6. **Creates a GIF animation** from processed frames.
7. **Avoids redundant processing** by tracking timestamps.

### **🔹 RadarAlgorithm23 (Filtering & InfluxDB Upload)**
1. **Reads radar data and calculates range, azimuth, and elevation**.
2. **Filters data** based on range (`>9`) and azimuth (`-4 to 4` degrees).
3. **Checks for already processed timestamps**.
4. **Uploads new data to InfluxDB**.

---

## **Data Processing Metrics**
| **Metric**      | **Description** |
|----------------|----------------|
| **Range**      | Distance of the object from the radar. |
| **Azimuth**    | Horizontal angle of the object. |
| **Elevation**  | Vertical angle of the object. |
| **Intensity**  | Strength of the radar reflection. |
| **DBSCAN Cluster ID** | Identifies reflector clusters in RadarAlgorithm16. |

---

## **How to Run the Programs**
### **1️⃣ Run Radar Clustering & Visualization (RadarAlgorithm16)**
```bash
python radar_algorithm16.py
```
### **2️⃣ Run Radar Data Filtering & InfluxDB Upload (RadarAlgorithm23)**
```bash
python radar_algorithm23.py
```

### **3️⃣ Outputs Generated**
- **Clustered radar data** saved in `/output/NewResults/`
- **Filtered radar data** saved in `/output/Filtered_output.csv`
- **GIF animation of reflector clusters** created

---

## **🔄 InfluxDB Integration (RadarAlgorithm23)**
This program uploads **filtered radar data** to **InfluxDB** for visualization.

### **InfluxDB Setup**
1. Run InfluxDB (if using Docker):
   ```bash
   docker run -d -p 8086:8086 --name influxdb influxdb
   ```
2. Create a bucket: `radar_metrics_final`
3. Update the script with:
   ```python
   influx_url = "http://localhost:8086"
   token = "YOUR_INFLUXDB_TOKEN"
   org = "winterproject24"
   bucket = "radar_metrics_final"
   ```

---

## **Troubleshooting**
### **1️⃣ No clusters detected (RadarAlgorithm16)**
- Adjust **DBSCAN parameters (`eps`, `min_samples`)** to optimize clustering.

### **2️⃣ No new data uploaded (RadarAlgorithm23)**
- Ensure InfluxDB is running (`docker ps`).
- Check the filtered CSV file for new timestamps.

### **3️⃣ GIF not generated**
- Ensure at least **one processed frame** is saved in `/output/NewResults/`.

---

## **Future Enhancements**
📌 **Improve clustering** by dynamically tuning DBSCAN parameters.  
📌 **Integrate real-time radar processing** using ROS and live sensor feeds.  
📌 **Add support for additional clustering algorithms** (e.g., K-Means, HDBSCAN).  
📌 **Enhance visualization** with interactive 3D plotting tools.  

---

## **License**
This project is open-source under the **MIT License**. Feel free to contribute!

---

### **Need Help?**
For questions or contributions, open an **issue** in the GitHub repository! 🚀
