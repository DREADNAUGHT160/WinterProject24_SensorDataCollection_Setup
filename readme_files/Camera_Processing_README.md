# **Image Quality Analysis and AprilTag Detection**

![Python](https://img.shields.io/badge/Python-3.8+-yellow) ![OpenCV](https://img.shields.io/badge/OpenCV-Enabled-blue) ![InfluxDB](https://img.shields.io/badge/InfluxDB-Enabled-green)

This program performs **image quality analysis** by calculating various metrics (sharpness, contrast, colorfulness, entropy, brightness, and noise level) and detects **AprilTags** in images. The results are saved in a CSV file and also uploaded to an **InfluxDB database** for further analysis.

---

## **Features**
✅ **Loads images from a camera sensor directory**  
✅ **Enhances and sharpens images using CLAHE and Gaussian filters**  
✅ **Computes multiple image quality metrics:**  
   - **Sharpness**
   - **Contrast**
   - **Colorfulness**
   - **Entropy**
   - **Brightness**
   - **Noise Level**  
✅ **Detects AprilTags (tag36h11 family) in images**  
✅ **Classifies images into "Day" or "Night" based on timestamp**  
✅ **Saves annotated images with detected tags and metrics**  
✅ **Stores results in CSV and uploads data to InfluxDB**  

---

## **File Structure**
```
your_project/
├── image_processing.py         <- Main script (this program)
├── sensor_data/
│   ├── camera/                <- Directory containing input images
│   └── ...
├── output/
│   ├── output_images/         <- Directory for annotated images
│   ├── image_metrics.csv      <- Output CSV storing image analysis results
│   ├── processed_images.txt   <- Tracks processed images to avoid duplicates
└── ...
```

---

## **Installation & Dependencies**
### **1️⃣ Prerequisites**
- **Python 3.8+**
- **InfluxDB setup (optional, for cloud storage of image metrics)**

### **2️⃣ Install Required Libraries**
Run the following command to install dependencies:
```bash
pip install opencv-python numpy pupil-apriltags scikit-image influxdb-client
```

---

## **How It Works**
1. **Loads images** from the `/sensor_data/camera/` folder.
2. **Enhances images** using **CLAHE (Contrast Limited Adaptive Histogram Equalization)**.
3. **Computes multiple metrics** including sharpness, contrast, and brightness.
4. **Detects AprilTags** in the image and counts them.
5. **Classifies images as "Day" or "Night"** based on the timestamp.
6. **Saves annotated images** with detected tags and metrics.
7. **Stores results in a CSV file** (`image_metrics.csv`).
8. **Uploads data to InfluxDB** for advanced analysis.

---

## **Data Processing Metrics**
| **Metric**      | **Description** |
|----------------|----------------|
| **Sharpness**  | Measures image clarity using Laplacian variance. |
| **Contrast**   | Computes the difference between the max and min pixel intensity. |
| **Colorfulness** | Evaluates the vibrancy of colors in an image. |
| **Entropy**    | Shannon entropy, indicating the level of information in an image. |
| **Brightness** | Average pixel intensity (used for night/day classification). |
| **Noise Level** | Computed using Fourier Transform-based power spectral density. |
| **Total Tags Detected** | Number of AprilTags detected in the image. |

---

## **How to Run the Program**
### **1️⃣ Run the Python Script**
```bash
python image_processing.py
```
### **2️⃣ Outputs Generated**
- **Annotated images** are saved in `/output/output_images/`  
- **Image metrics CSV** is saved as `/output/image_metrics.csv`  
- **Processed images are tracked** in `/output/processed_images.txt`  
- **Data is uploaded to InfluxDB** for cloud-based storage

---

## **🔄 InfluxDB Integration**
This program uploads image quality metrics to **InfluxDB** for visualization and storage.

### **InfluxDB Setup**
1. Run InfluxDB (if using Docker):
   ```bash
   docker run -d -p 8086:8086 --name influxdb influxdb
   ```
2. Create a bucket: `image_metrics`
3. Update `INFLUXDB_URL`, `INFLUXDB_TOKEN`, `INFLUXDB_ORG`, and `INFLUXDB_BUCKET` in the script.

---

## **Troubleshooting**
### **1️⃣ Image not loading**
- Ensure the images exist in `/sensor_data/camera/`
- Check file format (supports `.jpg`, `.png`, `.jpeg`)

### **2️⃣ No AprilTags detected**
- Ensure the AprilTags are visible and clear
- Increase image resolution if detection is failing

### **3️⃣ InfluxDB data not saving**
- Verify that InfluxDB is running (`docker ps`)
- Check token and organization settings

---

## **Future Enhancements**
📌 **Automated Image Quality Filtering**: Discard low-quality images before processing.  
📌 **Real-Time Processing**: Stream camera feed and analyze frames live.  
📌 **More AprilTag Families**: Extend support beyond `tag36h11`.  
📌 **Cloud Upload**: Save images in AWS S3 or Google Cloud.  

---

## **License**
This project is open-source under the **MIT License**. You are free to modify and distribute it.

---

### **Need Help?**
For questions or contributions, open an **issue** in the GitHub repository! 🚀
