# **Big Data Collection System (ROS, Radar, LiDAR, Camera, Disdrometer)**

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
- [InfluxDB Integration](#influxdb-integration)  
- [Troubleshooting](#troubleshooting)  
- [Future Enhancements](#future-enhancements)  
- [License](#license)  

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
your_project/
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
└── docs/                     <- Additional documentation and resources
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
| **Script**         | **Description** |
|--------------------|---------------|
| **main.py**        | Central controller that manages data collection. Runs the ROS loop, handles the GUI, and starts/stops sensors. |
| **start_switch.py**| Tkinter-based interface to start ROS, launch sensors, and start `main.py`. |
| **collection.py**  | Manages `Camera`, `LiDAR`, `Radar`, and `Disdrometer` objects, handles force-save and rain-based saving logic. |
| **camera.py**      | Captures images from a camera and saves them as `.jpg`, logs timestamps to CSV, and uploads to InfluxDB. |
| **lidar.py**       | Reads LiDAR range and intensity data from `/scan`, saves to CSV, and uploads to InfluxDB. |
| **radar.py**       | Logs radar data (`PointCloud2`) from `/ti_mmwave/radar_scan_pcl_0`, stores in CSV, and uploads to InfluxDB. |
| **disdrometer.py** | Monitors precipitation, logs rain/no-rain status, and saves visibility, precipitation data to CSV and InfluxDB. |
| **gui.py**         | Tkinter-based graphical control panel for sensor monitoring and interval adjustments. |

---

## **License**
This project is open-source under the **MIT License**. 🚀
