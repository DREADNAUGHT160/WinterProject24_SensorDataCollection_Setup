# **Automated Python Task Scheduler**

![Python](https://img.shields.io/badge/Python-3.8+-yellow) ![Task Scheduler](https://img.shields.io/badge/Scheduler-Automated-blue) 

This script automates the execution of multiple Python programs at a scheduled time. It is designed to run specific programs at the **50th minute of every hour** and ensures that tasks are executed sequentially with predefined delays.

---

## **Features**
✅ **Runs five different programs in a scheduled manner.**  
✅ **Executes at the 50th minute of every hour.**  
✅ **Waits between executions to prevent overloading.**  
✅ **Error handling included to prevent crashes.**  
✅ **Ensures proper sequencing of tasks.**  

---

## **File Structure**
```
your_project/
├── scheduler.py                    <- Main scheduler script (this program)
├── camera/
│   ├── camera_analytics.py         <- Camera data processing script
├── radar/
│   ├── Radaralgorithm16.py         <- Radar clustering & visualization
│   ├── Radaralgorithm23.py         <- Radar data filtering & InfluxDB upload
├── lidar/
│   ├── upload_lidar_data_script_to_influxdb.py  <- LiDAR data upload script
└── ...
```

---

## **Installation & Dependencies**
### **1️⃣ Prerequisites**
- **Python 3.8+**
- Ensure all **dependent scripts exist** at the correct file paths.
- Set proper **execution permissions** for the script.

### **2️⃣ Install Required Libraries**
Run the following command to install dependencies:
```bash
pip install subprocess datetime time
```

---

## **How It Works**
1. The script **monitors the system clock**.
2. When the **current time reaches the 50th minute of an hour**, it executes the tasks sequentially:
   - **Program 1:** `camera_analytics.py`
   - **Program 2:** `Radaralgorithm16.py`
   - **Program 3:** `upload_lidar_data_script_to_influxdb.py`
   - **Program 4:** Another instance of the **LiDAR program**.
   - **Program 5:** `Radaralgorithm23.py`
3. Each program runs **10 minutes apart** to avoid execution overlap.

---

## **How to Run the Scheduler**
### **1️⃣ Start the Scheduler**
Run the following command to **start the scheduling process**:
```bash
python scheduler.py
```
- This script will **keep running continuously** in the background.  
- It will **trigger tasks every hour at the 50th minute**.  

### **2️⃣ Stop the Scheduler**
To manually stop the scheduler, **press** `CTRL + C` in the terminal.

### **3️⃣ Run the Scheduler in the Background (Linux)**
If you want to **run this script continuously** even after closing the terminal, use:
```bash
nohup python scheduler.py &
```

---

## **Troubleshooting**
### **1️⃣ Programs not running**
- Verify that **all script paths** in `scheduler.py` are correct.
- Check if the scripts have **execution permissions** (`chmod +x script.py`).

### **2️⃣ The scheduler is not triggering at the 50th minute**
- Make sure your system clock is accurate.
- Restart the script using `python scheduler.py`.

### **3️⃣ High CPU usage**
- This script uses `time.sleep(1)` to avoid high CPU utilization.
- If CPU usage is still high, check if other programs are consuming resources.

---

## **Future Enhancements**
📌 **Implement logging** to record execution status.  
📌 **Add multi-threading** for parallel execution of tasks.  
📌 **Integrate email notifications** for execution status.  
📌 **Support custom scheduling intervals**.  

---

## **License**
This project is open-source under the **MIT License**. Feel free to contribute!

---

### **Need Help?**
For questions or contributions, open an **issue** in the GitHub repository! 🚀
