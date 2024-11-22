# image-inspector
This repository is about inspecting images and their instrinsics and extrinsics.

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
