### 1. **Pinhole Camera Model**
- **Description**: A simplified model that assumes a single point of projection, where light rays pass through a pinhole to form an image on a flat surface (image plane).
- **Characteristics**:
  - No lens distortion.
  - Linear mapping between 3D world points and 2D image points.
  - Governed by the camera's intrinsic matrix \(K\).
- **Equation**:
  \[
  \mathbf{p} = K \cdot [R|t] \cdot \mathbf{P}
  \]
  Where:
  - \( \mathbf{P} \) is the 3D point.
  - \( \mathbf{p} \) is the 2D point on the image plane.
  - \(K\) is the intrinsic matrix.
  - \([R|t]\) is the extrinsic transformation.
