# image-inspector
This repository is about inspecting images and their instrinsics and extrinsics.
## RGB Image
Standard Resolutions are VGA, HD720p, HD1080p and HD2K.

## Depth Image
For unit8 type grayscaled depth images, they are 8 bit (0-255) depth images having a range of 0 m to 0.255 m. You can export them as jpg or png files. You can directly visualize color depth plots. 

For unit16 type grayscaled depth images, they are 16 bit (0-65535) depth images having a range of 0 m to 65.535 m [[reference]](https://support.stereolabs.com/hc/en-us/articles/5365701074967-Why-is-the-depth-map-so-dark). You can export them as png files. You have to normalize them to 8 bit (0-255) to visualize color depth plots. 

For float32 type grayscaled depth images, they are 32 bit (1.4×10^−45-3.4×10^38) depth images. Most depth cameras do not have a depth measurement range beyond several hundred meters. So, a float32 image can comfortably represent depth values, commonly from 0 m up to several kilometers (e.g., 3,400,000 meters), if needed. You have to normalize them to 8 bit (0-255) to visualize color depth plots. You can't export them as png files, you have to export as .exr (OpenEXR) or .tiff format (you can use GIMP to visualize them).

## Depth Image Intrinsics and Distortions
Depth images have associated **camera info**. The **camera info** provides essential calibration and metadata about the camera that captured the depth image. This information is critical for interpreting depth values correctly and for projecting depth pixels into 3D space.

   <div align="center">
     <img src="media/depth_map.png" width="400">
   </div>

For Depth Image Info, you can see the smallest and largest value of the depth map. Here is an example:
```
ros2 topic echo /zed/zed_node/depth/depth_info
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: zed_left_camera_optical_frame
min_depth: 0.6266739368438721
max_depth: 7.9091668128967285
```

---

### Why Does Camera Info Matter for Depth Images?
Depth images typically provide per-pixel depth values, where each pixel represents the distance to a surface in the scene. To use this data effectively (e.g., to compute 3D coordinates), you need the following information, often provided in the **camera info** message:

1. **Camera Intrinsics**:
   - **Intrinsic Matrix (K)**: Maps pixel coordinates to 3D camera coordinates.
     - Format:
       ```
       K = [fx,  0, cx,
             0, fy, cy,
             0,  0,  1]
       ```
       Where:
       - `fx` and `fy` are the focal lengths in pixels.
       - `cx` and `cy` are the optical center coordinates (principal point).

2. **Projection Matrix (P)**:
   - Used to project 3D points into the 2D image plane.
   - Includes additional parameters for stereo cameras.

3. **Distortion Model and Coefficients**:
   - If the depth image is raw (not rectified), these parameters describe the lens distortions.
   - Rectified depth images typically have zero distortion coefficients.

4. **Resolution**:
   - The height and width of the depth image, which must match the dimensions of the camera info.

5. **Frame ID**:
   - Indicates the coordinate frame associated with the depth image (e.g., `left_camera` or `depth_camera`).

---

### How to Use Camera Info with Depth Images
1. **Project Depth to 3D**:
   - Using the intrinsic matrix \( K \), you can compute the 3D position of any pixel \((u, v)\) with depth \( D \):

   <div align="center">
     <img src="media/depth_calculation.png" width="400">
   </div>
   
     Where X, Y, Z are the 3D coordinates in the camera frame.

2. **Stereo Depth**:
   - For stereo cameras, the projection matrix \( P \) also includes the baseline (distance between the two cameras), which is critical for converting disparity to depth.

3. **Point Cloud Generation**:
   - Depth images and camera info are combined to generate point clouds in the camera's coordinate frame.

---

### Example of Camera Info for Depth Images
Here’s how a typical `CameraInfo` message might look for a depth camera:

```python
camera_info = CameraInfo()
camera_info.header.stamp = <timestamp>
camera_info.header.frame_id = "depth_camera"
camera_info.height = 720
camera_info.width = 1280
camera_info.distortion_model = "plumb_bob"  # Or "none" for rectified images
camera_info.d = [-0.1, 0.01, 0.0, 0.0, 0.0]  # Distortion coefficients
camera_info.k = [700.0, 0.0, 640.0, 0.0, 700.0, 360.0, 0.0, 0.0, 1.0]  # Intrinsic matrix
camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
camera_info.p = [700.0, 0.0, 640.0, 0.0, 0.0, 700.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix
```

---

### How to Attach Camera Info to Depth Images in ROS 2
When publishing depth images, you can also publish the corresponding `CameraInfo` on a separate topic. For example:

- Depth Image Topic: `/camera/depth/image_raw`
- Camera Info Topic: `/camera/depth/camera_info`

The subscriber can then synchronize the two topics using tools like `message_filters` in ROS.

---

### How to Check If Your Depth Images Have Camera Info
1. **Inspect the Camera Info Topic**:
   Use `ros2 topic echo /camera/depth/camera_info` to see if the camera info is published.

2. **Check the Data**:
   Look for parameters like `K`, `P`, and `distortion_model` in the output.

---

## RGB Image Intrinsics and Distortions
### **Distortion Models**
1. **Plumb Bob (Standard Radial-Tangential)**
   - Commonly used in most camera calibration tools (e.g., OpenCV).
   - Supports radial and tangential distortion.
   - **Expected Values**: **5 coefficients**:
     - `k1`, `k2` (radial distortion coefficients).
     - `p1`, `p2` (tangential distortion coefficients).
     - `k3` (higher-order radial distortion).

   Example:
   ```python
   camera_info.d = [k1, k2, p1, p2, k3]
   ```

2. **Equidistant (Fisheye Model)**
   - Often used for fisheye or wide-angle cameras.
   - **Expected Values**: **4 coefficients**:
     - `k1`, `k2` (radial distortion coefficients).
     - `k3`, `k4` (higher-order radial distortion).

   Example:
   ```python
   camera_info.d = [k1, k2, k3, k4]
   ```

3. **Omnidirectional**
   - Used for omnidirectional or 360-degree cameras.
   - Can support a variable number of coefficients depending on the calibration.

---

### **Rectified vs. Raw Images**
1. **Rectified Images**
   - The distortion has already been corrected.
   - **`camera_info.d` for rectified images**:
     ```python
     camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
     ```
   - Even if your model allows more coefficients, all are set to **0** since the rectification removes distortions.

2. **Raw Images**
   - The distortion is present and needs to be corrected using the coefficients.
   - **`camera_info.d` for raw images**:
     - The number of values will match the distortion model (typically 5 for "plumb_bob").

   Example:
   ```python
   camera_info.d = [-0.174526006, 0.0277379993, 0.0000997691, -0.000323628, 0.0]
   ```

---

### **Key Difference**
- **Rectified**: `camera_info.d` values are all zeros (no distortion present).
- **Raw**: `camera_info.d` contains non-zero coefficients that describe the lens distortion.

## Intrinsics Matrix Derivation

The intrinsic parameters and camera matrices are typically obtained from the **camera calibration process**. Here's how each value is derived or determined:

---

### 1. Intrinsic Matrix (`camera_info.k`)
The intrinsic matrix defines how a 3D point in the camera's coordinate system is projected onto the 2D image plane.

<div align="center">
  <img src="media/intrinsic_matrix.png" width="200">
</div>

- **`fx`, `fy` (Focal Lengths)**:
  - These are the scaling factors for pixels in the x and y directions, measured in pixels.
  - These are derived from the physical focal length of the lens (in mm) and the pixel size of the camera sensor.

- **`cx`, `cy` (Principal Point)**:
  - These represent the optical center of the image in pixel coordinates. Ideally, this is at the center of the image, but due to imperfections, it might be slightly offset.
  - These values are computed during calibration.

- **How Your Values Are Incorporated**:
  ```
  camera_info.k = [1388.96728515625, 0.0, 954.34521484375, 
                   0.0, 1388.96728515625, 531.0472412109375, 
                   0.0, 0.0, 1.0]
  ```

---

### **2. Distortion Coefficients (`camera_info.d`)**
Distortion coefficients correct for lens distortions, such as:
- **Radial distortion** (barrel or pincushion distortion).
- **Tangential distortion** (caused by misalignment of the lens).

Distortion coefficients typically follow the model:

<div align="center">
  <img src="media/distortion_coefficient.png" width="200">
</div>

- `k_1, k_2, k_3`: Radial distortion coefficients.
- `p_1, p_2`: Tangential distortion coefficients.

In your case:
- Distortion coefficients are all **zero** (`[0. 0. 0. ...]`), indicating rectified images where distortions have already been removed.

---

### **3. Rectification Matrix (`camera_info.r`)**
The rectification matrix aligns the images from stereo cameras so that their epipolar lines become parallel. For rectified cameras, this is often an identity matrix:

<div align="center">
  <img src="media/rectification_matrix.png" width="150">
</div>

This is consistent with the values:
```
camera_info.r = [1.0, 0.0, 0.0, 
                 0.0, 1.0, 0.0, 
                 0.0, 0.0, 1.0]
```

---

### **4. Projection Matrix (`camera_info.p`)**
The projection matrix maps 3D points to 2D image coordinates, incorporating the intrinsic matrix and additional parameters like stereo baseline for the right camera.

For a stereo camera:

<div align="center">
  <img src="media/stereo_projection_matrix.png" width="300">
</div>

- **Baseline (B)**:
  - The distance between the left and right cameras.
  - The value `-fx * B` is present only in the right camera's projection matrix.

For zed cameras:
- Left Camera:
  ```
  camera_info.p = [1388.96728515625, 0.0, 954.34521484375, 0.0, 
                   0.0, 1388.96728515625, 531.0472412109375, 0.0, 
                   0.0, 0.0, 1.0, 0.0]
  ```

- Right Camera:
  ```
  camera_info.p = [1388.96728515625, 0.0, 954.34521484375, -B, 
                   0.0, 1388.96728515625, 531.0472412109375, 0.0, 
                   0.0, 0.0, 1.0, 0.0]
  ```
  (Here, `-B` would be the negative baseline multiplied by `fx`).

---

### **5. Image Size**
Image resolution is derived directly from the camera or dataset:
- **Image Width**: 1920
- **Image Height**: 1080

This information is explicitly used to define the image size.

---

### **How Are These Values Obtained?**
The values are typically obtained through a **camera calibration process**, such as:

1. **Calibration with a Checkerboard**:
   - A calibration tool (e.g., OpenCV, MATLAB, or ROS `camera_calibration` package) uses a checkerboard or a known calibration pattern.
   - Images of the pattern are captured from the camera and used to compute intrinsics, distortion coefficients, and extrinsics.

   Example in OpenCV:
   ```python
   ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, imageSize, None, None)
   ```

2. **Camera Calibration Tools**:
   - Tools like the ROS `camera_calibration` package or the ZED SDK provide automated calibration procedures.
   - They output the intrinsic parameters directly.

3. **Stereo Calibration**:
   - If you're using stereo cameras, a stereo calibration process computes the rectification matrix (`R`) and projection matrix (`P`) in addition to individual camera intrinsics.

---

### Generate camera info parameters for ROS
To use these intrinsics in ROS, encode them into the `CameraInfo` message directly, as shown in the code.

Example:
```python
from sensor_msgs.msg import CameraInfo

camera_info = CameraInfo()
camera_info.height = 1080
camera_info.width = 1920
camera_info.distortion_model = "plumb_bob"
camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_info.k = [1388.96728515625, 0.0, 954.34521484375,
                 0.0, 1388.96728515625, 531.0472412109375,
                 0.0, 0.0, 1.0]
camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera_info.p = [1388.96728515625, 0.0, 954.34521484375, 0.0,
                 0.0, 1388.96728515625, 531.0472412109375, 0.0,
                 0.0, 0.0, 1.0, 0.0]
```

In the context of stereo cameras, the **baseline** refers to the physical distance between the optical centers of the left and right cameras. This baseline is critical for calculating depth from stereo images because it directly influences the disparity between corresponding points in the left and right images.

### **What Does `-B` Mean?**
In the **projection matrix** for the right camera, `-B` appears as part of the translation term. Specifically:

<div align="center">
  <img src="media/right_camera_projection_matrix.png" width="300">
</div>

Here:
- \( f_x \): Focal length in the x-direction (in pixels).
- \( c_x, c_y \): Principal point coordinates.
- \( B \): Baseline distance between the left and right cameras (in meters).

---

### **Why `-B` is Multiplied by \( f_x \)?**
The term \( -f_x \cdot B \) arises because the projection matrix incorporates both the camera's intrinsics and its relative position in space. The translation \( -f_x \cdot B \) shifts the x-coordinate in the right camera's view to account for the offset caused by the baseline.

- \( B \) is measured in **meters**, but the focal length (\( f_x \)) is in **pixels**. The multiplication \( -f_x \cdot B \) ensures the translation is represented in **pixels**, aligning with the other terms in the projection matrix.

---

### How Does This Help?
This term allows stereo matching algorithms to compute the disparity between the left and right images, which is the key to estimating depth. 

Disparity (\( d \)) is defined as:
\[
d = x_{\text{left}} - x_{\text{right}}
\]

Using the disparity, the depth (\( Z \)) of a point can be calculated as:
\[
Z = \frac{f_x \cdot B}{d}
\]

- When \( -f_x \cdot B \) is correctly encoded in the projection matrix of the right camera, it simplifies stereo computations.

---

### **Example: Computing `-B`**
1. Suppose:
   - **Baseline** (\( B \)): \( 0.1 \, \text{m} \) (10 cm between the left and right cameras).
   - **Focal Length** (\( f_x \)): \( 1388.967 \, \text{pixels} \).

2. The translation term for the right camera becomes:
   \[
   -f_x \cdot B = -1388.967 \cdot 0.1 = -138.897 \, \text{pixels}
   \]

3. The right camera's projection matrix would then be:

<div align="center">
  <img src="media/projection_matrix.png" width="400">
</div>

---

Example Value in ROS1:
```
arghya@arghya-Pulse-GL66-12UEK:~/hydra_ws/src/hydra$ rostopic echo /tesse/left_cam/camera_info
header: 
  seq: 642
  stamp: 
    secs: 46
    nsecs: 404879999
  frame_id: "left_cam"
height: 480
width: 720
distortion_model: "radial-tangential"
D: [0.0, 0.0, 0.0, 0.0]
K: [415.69219381653056, 0.0, 360.0, 0.0, 415.69219381653056, 240.0, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [415.69219381653056, 0.0, 360.0, 0.0, 0.0, 415.69219381653056, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

### **Summary**
The term \( -B \) in the projection matrix represents the baseline offset, translated into pixel units using the focal length \( f_x \). This ensures the stereo cameras are correctly modeled for depth estimation. Without this term, depth computations from stereo images would not be possible.


