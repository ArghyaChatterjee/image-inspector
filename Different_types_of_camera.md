There are several camera models used in computer vision, each designed to simulate or approximate the way a physical camera captures images. These models differ in how they handle projection, distortion, and field of view. Here are some commonly used camera models:

---

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

---

### 2. **Thin Lens Model**
- **Description**: Extends the pinhole model to include lens effects, particularly depth of field and focus.
- **Characteristics**:
  - Models light refraction through a lens.
  - Incorporates aperture size and focal length.
  - Typically used in optical simulations and rendering.

---

### 3. **Fisheye Camera Model**
- **Description**: Designed for wide-angle lenses with a field of view greater than 90°.
- **Characteristics**:
  - Non-linear projection to accommodate extreme distortion.
  - Various sub-models:
    - **Equidistant**: \(r = f \cdot \theta\)
    - **Equisolid Angle**: \(r = 2f \cdot \sin(\theta / 2)\)
    - **Stereographic**: \(r = 2f \cdot \tan(\theta / 2)\)
    - **Orthographic**: \(r = f \cdot \sin(\theta)\)
  - Often used in 360° imaging and panoramic applications.

---

### 4. **Radial Distortion Model**
- **Description**: Extends the pinhole model to account for lens distortion, particularly barrel and pincushion distortion.
- **Characteristics**:
  - Adds radial and tangential distortion terms.
  - Radial distortion:
    \[
    r_{\text{distorted}} = r_{\text{undistorted}} (1 + k_1 r^2 + k_2 r^4 + k_3 r^6 + \ldots)
    \]
  - Tangential distortion:
    \[
    x_{\text{distorted}} = x_{\text{undistorted}} + [2p_1 xy + p_2 (r^2 + 2x^2)]
    \]
    \[
    y_{\text{distorted}} = y_{\text{undistorted}} + [p_1 (r^2 + 2y^2) + 2p_2 xy]
    \]
  - Parameters \(k_i, p_i\) are determined through calibration.

---

### 5. **Spherical Camera Model**
- **Description**: Models the camera as capturing the entire surrounding scene (360° field of view) onto a spherical surface.
- **Characteristics**:
  - Used in omnidirectional cameras and virtual reality.
  - Projection involves mapping 3D points onto a unit sphere.

---

### 6. **Affine Camera Model**
- **Description**: A simplified linear approximation of the pinhole model, often used in scenarios where the field of view is narrow.
- **Characteristics**:
  - Assumes weak perspective projection.
  - Used in scenarios where depth variation is minimal.

---

### 7. **Catadioptric Camera Model**
- **Description**: Combines lenses and mirrors to achieve a wide field of view.
- **Characteristics**:
  - Often used in robotics and surveillance.
  - Requires specialized calibration due to the combination of reflective and refractive optics.

---

### 8. **Orthographic Camera Model**
- **Description**: Assumes parallel projection rather than perspective projection.
- **Characteristics**:
  - Ignores depth differences, projecting 3D points orthogonally onto the image plane.
  - Simplifies computations in specific applications, like technical drawings or certain rendering tasks.

---

### 9. **Stereo Camera Model**
- **Description**: Represents a setup with two or more cameras for depth estimation.
- **Characteristics**:
  - Models relative positions and orientations between cameras.
  - Often used in 3D reconstruction and disparity mapping.

---

### 10. **Rolling Shutter Camera Model**
- **Description**: Models the sequential capture of rows of the image sensor, as opposed to capturing the entire frame at once (global shutter).
- **Characteristics**:
  - Captures motion-induced distortions like "jello effect."
  - Requires time-dependent correction in dynamic scenes.

---

### 11. **Anamorphic Camera Model**
- **Description**: Models lenses that produce images with non-uniform scaling, commonly used in cinematography.
- **Characteristics**:
  - Introduces deliberate distortion to achieve specific artistic effects (e.g., wider aspect ratios).

---

### 12. **Extended Unified Camera Model (EUCM)**
- **Description**: A generalization that unifies perspective and fisheye cameras.
- **Characteristics**:
  - Includes parameters to transition between perspective and wide-angle projection.
  - Useful for systems with diverse camera types.

---

### Summary Table

| **Model**                | **Field of View** | **Distortion**       | **Use Case**                          |
|--------------------------|-------------------|----------------------|---------------------------------------|
| Pinhole                  | Narrow-Medium    | None                 | General computer vision               |
| Thin Lens                | Medium           | Optical effects      | Optical simulations, rendering        |
| Fisheye                  | Wide (>90°)      | Extreme distortion   | Panoramic, VR, robotics               |
| Radial Distortion        | Medium           | Barrel/Pincushion    | Lens calibration                      |
| Spherical                | 360°             | Spherical projection | Omnidirectional imaging               |
| Affine                   | Narrow           | Linear approximation | Simplified 3D-to-2D mapping           |
| Catadioptric             | Wide/360°        | Mirror distortion    | Robotics, surveillance                |
| Orthographic             | Narrow           | None                 | Rendering, technical drawing          |
| Stereo                   | Narrow-Medium    | Pair calibration     | Depth estimation, 3D reconstruction   |
| Rolling Shutter          | Medium           | Motion artifacts     | Video, dynamic scenes                 |
| Anamorphic               | Medium           | Non-uniform scaling  | Cinematography                        |
| EUCM                     | Wide/Medium      | Generalized          | Unified camera systems                |

Each model serves specific applications, and selecting the right one depends on the camera type, lens properties, and the intended application.
