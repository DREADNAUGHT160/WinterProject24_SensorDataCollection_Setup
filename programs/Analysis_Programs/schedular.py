import time
import subprocess
from datetime import datetime

def run_program_1():
    print("Running Program 1...")
    try:
        subprocess.call(["python3", "/home/carissma/new_ros-workspace/src/Scheduling/camera/camera_alaytics.py"])
    except Exception as e:
        print(f"Error running Program 1: {e}")

def run_program_2():
    print("Running Program 2...")
    try:
        subprocess.call(["python3", "/home/carissma/new_ros-workspace/src/Scheduling/radar/Radaralgorithm16.py"])
    except Exception as e:
        print(f"Error running Program 2: {e}")

def run_program_3():
    print("Running Program 3...")
    try:
        subprocess.call(["python3", "/home/carissma/new_ros-workspace/src/Scheduling/lidar/upload_lidar_data_script_to_influxdb.py"])
    except Exception as e:
        print(f"Error running Program 3: {e}")


def run_program_4():
    print("Running Program 4...")
    try:
        subprocess.call(["python3", "lidar_program/upload_lidar_data_script_to_influxdb.py"])
    except Exception as e:
        print(f"Error running Program 4: {e}")

def run_program_5():
    print("Running Program 5...")
    try:
        subprocess.call(["python3", "/home/carissma/new_ros-workspace/src/Scheduling/radar/Radaralgorithm23.py"])
    except Exception as e:
        print(f"Error running Program 5: {e}")



def main():
    print("Scheduler started. Waiting for the 50th minute of the hour...")

    while True:
        now = datetime.now()
        
        # Check if it's the 59th minute of any hour
        if now.minute == 50 and now.second == 0:
            print("Starting scheduled tasks at the 59th minute of the hour.")
            run_program_1()
            time.sleep(10 * 60)  # Wait 10 minutes
            run_program_2()
            time.sleep(10 * 60)  # Wait another 10 minutes
            run_program_3()
            time.sleep(10 * 60)
            run_program_4()
            time.sleep(10 * 60)
            run_program_5()
        else:
            # Sleep for 1 second to avoid high CPU usage
            time.sleep(1)

if __name__ == "__main__":
    main()
