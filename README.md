# **Data Collection System (ROS, Radar, LiDAR, Camera, Disdrometer)**

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![Python](https://img.shields.io/badge/Python-3.8+-yellow) ![InfluxDB](https://img.shields.io/badge/InfluxDB-Enabled-green)  

A **ROS-based multi-sensor data collection system** integrating **Radar, LiDAR, Camera, and a Disdrometer** for real-time sensor fusion. The system:
- **Captures synchronized sensor data** and logs it to **CSV files** and **InfluxDB**.  
- Provides a **Tkinter GUI** for user-friendly control.  
- Allows **dynamic interval configuration** and **conditional data saving** based on weather (rain detection).  
- Includes **sensor health monitoring** and **automated background processes** for robust data collection.  

This system is **ideal for robotics, autonomous vehicles, weather monitoring, and AI-driven sensor fusion applications**.

---

## **Table of Contents**
- [Features](#features)  
- [Project Structure](#project-structure)  
- [Installation](#installation)  
- [Usage](#usage)  
- [How It Works](#how-it-works)  
- [Program Details](#program-details)  
- [Data Storage](#data-storage)
- [Disdrometer Filtering](#Disdrometer-Filtering)
- [Analysis Programs](#Analysis-Programs)  
  

---

## **Features**
✅ **Multi-Sensor Data Collection**: Integrates **Radar, LiDAR, Camera, and Disdrometer** for **synchronized** logging.  
✅ **User-Friendly GUI**: Control system start/stop, adjust sampling intervals, and view sensor status.  
✅ **Rain-Based Conditional Logging**: Uses **disdrometer readings** to decide whether data should be saved.  
✅ **Force-Save Intervals**: Ensures periodic data collection even if conditions don’t require it.  
✅ **CSV & InfluxDB Storage**: Supports both **local file storage** and **cloud-based InfluxDB integration**.  
✅ **Sensor Health Monitoring**: Continuously checks the status of sensors and logs their state.  
✅ **Modular Design**: Each sensor module is independent, allowing for future upgrades.  

---

## **Project Structure**
```
/home/carissma/new_ros-workspace/src/my_package/src/
├── README.md                 <- Documentation file
├── main.py                   <- Core data collection manager
├── start_switch.py           <- GUI to start ROS, sensors, and main.py
├── collection.py             <- Manages sensor instances and triggers data logging
├── camera.py                 <- Captures and saves camera images
├── lidar.py                  <- Collects LiDAR point cloud data
├── radar.py                  <- Logs radar point clouds and metadata
├── disdrometer.py            <- Reads rain status and logs precipitation data
├── gui.py                    <- Tkinter-based graphical control panel
├── sensor_data/              <- Directory storing collected data (CSV, images)

```

---

## **Installation**
### **1. Prerequisites**
#### **Operating System**  
- Ubuntu 20.04+ (ROS Noetic Recommended)  
- ROS installed (`roscore`, `ros-noetic-usb-cam`, `urg_node`, etc.)  
- Python 3.8+  

#### **Install Required ROS Packages**
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
rosdep update
```

#### **Install Required Python Libraries**
```bash
pip3 install opencv-python influxdb_client tkinter
```

#### **Set Up InfluxDB (Optional)**
```bash
docker run -d -p 8086:8086 --name influxdb influxdb
```
- Go to `http://localhost:8086`
- Create an **organization** (`winterproject24`) and a **bucket** (`sensor_data`)
- Copy your **authentication token** and update it in `disdrometer.py`, `lidar.py`, `radar.py`, and `camera.py`

---

## **Usage**
### **1. Launch the Sensor GUI**
Run the startup GUI to check if all sensors are connected:
```bash
python3 start_switch.py
```
- Click **Start ROSCore**  
- Click **Start Sensors (Lidar, Radar, Camera, Disdrometer)**  
- Click **Start Data Collection**  

### **2. Start the Data Collection System**
If you don’t use `start_switch.py`, you can start the system manually:
```bash
roscore &
python3 main.py
```
- The **Tkinter GUI** will open, allowing you to:  
  - Adjust **sensor intervals**  
  - Manually **start/stop collection**  
  - View **current status of sensors**  

### **3. Check Sensor Data**
After running, your **collected data** will be available in:
- **Local CSV Storage**: `sensor_data/`
- **InfluxDB Storage** (if enabled)  

---

## **Program Details**

### **1. main.py** (Core Data Collection Manager)  
**Location:** `/main.py`  
- Initializes **ROS Node**.  
- Starts the **GUI and sensor interfaces**.  
- Manages data collection scheduling and **external processes** (e.g., `disdrometer.py`).  
- Uses **multi-threading** for concurrent execution.  

### **2. start_switch.py** (Sensor Launch GUI)  
**Location:** `/start_switch.py`  
- Provides a **Tkinter GUI** to start/stop ROS, sensors, and the main system.  
- Verifies **device connections** before launching.  
- Uses `gnome-terminal` to start processes.  

### **3. collection.py** (Data Collection Manager)  
**Location:** `/collection.py`  
- Manages `Camera`, `LiDAR`, `Radar`, and `Disdrometer` objects.  
- Implements **force-save logic** and **rain-based data collection filters**.  

### **4. camera.py** (Camera Data Logger)  
**Location:** `/camera.py`  
- Subscribes to `/usb_cam/image_raw`.  
- Captures and saves images as `.jpg` and **logs timestamps** in CSV.  
- **Uploads metadata to InfluxDB**.  

### **5. lidar.py** (LiDAR Data Logger)  
**Location:** `/lidar.py`  
- Subscribes to `/scan`.  
- Extracts **distance and intensity data**, stores it in CSV, and uploads to InfluxDB.  

### **6. radar.py** (Radar Data Logger)  
**Location:** `/radar.py`  
- Subscribes to `/ti_mmwave/radar_scan_pcl_0`.  
- Processes `PointCloud2` data, logs it in CSV, and uploads to InfluxDB.  

### **7. disdrometer.py** (Weather-Based Control)  
**Location:** `/disdrometer.py`  
- Reads from a **serial sensor** (Disdrometer).  
- Determines **rain status** and **logs precipitation data**.  
- **Controls data logging based on weather conditions**.  

### **8. gui.py** (Graphical Interface)  
**Location:** `/gui.py`  
- **Tkinter-based GUI** for system control.  
- Allows **manual start/stop** of data collection.  
- Displays **real-time status updates** of each sensor.  

---

## **Data Storage**
| **Sensor**      | **Storage Format**  | **Location** |
|----------------|-------------------|--------------|
| **Radar**      | CSV + InfluxDB    | `sensor_data/radar/` |
| **LiDAR**      | CSV + InfluxDB    | `sensor_data/lidar/` |
| **Camera**     | JPG + CSV + InfluxDB | `sensor_data/camera/` |
| **Disdrometer** | CSV + InfluxDB    | `sensor_data/disdrometer/` |

---

## **Disdrometer Filtering**
This project integrates a **disdrometer sensor** with **Radar, LiDAR, and Camera** data collection to **dynamically filter sensor readings** based on weather conditions. The system ensures **accurate data logging** by preventing unnecessary recordings during **heavy rain, snow, or poor visibility conditions**.



### 📌 **Overview**
- **Real-time disdrometer readings** control whether sensor data is logged.
- **Adjustable thresholds** allow fine-tuning of rain detection parameters.
- **Force-save intervals** ensure data is stored periodically, even in bad weather.
- **InfluxDB integration** allows efficient time-series data storage.



### 🛠 **How the Disdrometer Works**
The **disdrometer** continuously monitors:
- 🌧 **Total Precipitation (mm/h)**
- 👀 **Visibility (m/h)**
- 🌨 **Precipitation Type** (Rain, Drizzle, Snow, Hail, etc.)

These values determine whether sensor data should be **collected, paused, or saved**.



### 🔍 **Filtering Criteria**
| **Parameter**           | **Threshold**                 | **Effect on Data Collection** |
|------------------------|-----------------------------|------------------------------|
| **Total Precipitation** | **> 1.0 mm/h**              | 🚫 Stops logging to avoid noise in LiDAR/Radar |
| **Visibility**         | **< 2000 m**                | 🚫 Stops data collection if visibility is too low |
| **Precipitation Type** | **Snow/Hail/Heavy Rain**    | 🚫 Stops all sensors except Disdrometer |
| **Force-Save Interval** | **Reaches time limit**      | ✅ Saves data even if rain is detected |



### 📊 **Impact on Sensor Data Logging**
| **Sensor**  | **Filtered by Rain?** | **Condition** |
|------------|---------------------|------------------------|
| **LiDAR**  | ✅ Yes | Stops when **no rain** is detected |
| **Radar**  | ✅ Yes | Stops when **no rain** is detected |
| **Camera** | ✅ Yes | Stops when **no rain** is detected |
| **Disdrometer** | ❌ No | Always records rain data |



### ⚡ **Force-Save Mechanism**
- Ensures **data is saved periodically**, regardless of rain conditions.
- Prevents **gaps in time-series data**.
- Helps in **long-term sensor fusion analysis**.



### 📌 **Example Filtering Workflow**
1️⃣ **Rain Detected (2.5 mm/h, Visibility: 1800m) → Data Collection Stops**  
2️⃣ **Light Drizzle (0.5 mm/h, Visibility: 2500m) → Data Collection Continues**  
3️⃣ **No Rain Detected for 10 Minutes → System Resumes Full Logging**  
4️⃣ **Force-Save Triggered → Data Stored Regardless of Conditions**  



### 🚀 **Why Use Rain-Based Filtering?**
✅ **Reduces sensor noise** in **LiDAR & Radar**  
✅ **Saves storage space** by avoiding bad weather data  
✅ **Ensures clear images** for camera-based AI models  
✅ **Improves sensor fusion accuracy** for robotics & autonomous systems  




---
## **Analysis Programs**
- [Schedular Program](/readme_files/SCHEDULER_README.md)  
- [Radar Processing Program](/readme_files/Radar_Processing_README.md)  
- [Camera Processing Program](/readme_files/Camera_Processing_README.md)
- [Lidar Processing Program](/readme_files/Lidar_Processing_README.md)

---


## **License**
This project is open-source under the **MIT License**. 🚀

