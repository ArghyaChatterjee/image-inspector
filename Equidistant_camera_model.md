The equidistant distortion model typically does not use "distortion coefficients" in the same sense as polynomial radial distortion models. Instead, it is based on a direct angular mapping relationship between the field angle \(\theta\) (the angle between the incoming ray and the optical axis) and the image radius \(r\) (the distance from the optical center to the projected point on the image plane). The basic formula is:

\[
r = f \cdot \theta
\]

Where:
- \(r\) is the radial distance of a point on the image plane.
- \(f\) is the focal length (often referred to as an "effective focal length" for the model).
- \(\theta\) is the angle of incidence of the incoming ray with respect to the optical axis.

### Distortion Coefficients in Practical Applications
In real-world implementations, the equidistant model may also include additional terms to account for slight deviations from the ideal equidistant mapping. These deviations are modeled as higher-order terms in \(\theta\). The generalized mapping can be expressed as:

\[
r = f \cdot (\theta + k_1 \theta^3 + k_2 \theta^5 + \ldots)
\]

Where:
- \(k_1, k_2, \ldots\) are the distortion coefficients.
- These coefficients refine the relationship between \(\theta\) and \(r\) to better fit the physical lens behavior.

### Key Points
- **Number of Coefficients:** The number of distortion coefficients \(k_i\) depends on the desired accuracy of the distortion model. In practice, one or two terms (\(k_1\), \(k_2\)) are often sufficient for typical fisheye lenses.
- **Lens-Specific Calibration:** The distortion coefficients and focal length \(f\) are determined for each specific lens-camera setup through a calibration process using tools like OpenCV's fisheye calibration functions.
- **No Radial Symmetry Assumption:** Unlike traditional polynomial radial distortion models, the equidistant model assumes an angular relationship, so the "coefficients" adjust the angular projection rather than a simple radial distance.

### In Summary
While the equidistant model itself doesn't strictly require "distortion coefficients" in its pure form, when used in practical settings to correct fisheye images, additional terms (\(k_1, k_2, \ldots\)) may be included to improve accuracy. These coefficients are typically determined experimentally through calibration with known patterns (e.g., checkerboards).
