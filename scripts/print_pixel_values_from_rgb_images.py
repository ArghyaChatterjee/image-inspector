import cv2
import numpy as np

# Path to the RGB image
rgb_image_path = "/home/arghya/rgb_image.png"

# Load the RGB image
rgb_image = cv2.imread(rgb_image_path, cv2.IMREAD_COLOR)  # Loads the image in BGR format

# Check if the image was loaded correctly
if rgb_image is None:
    print("Failed to load image.")
else:
    # Convert the BGR image to RGB format for easier interpretation
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

    # Print RGB values for the entire image (might be large)
    print("RGB values for the entire image:")
    print(rgb_image)  # This will print the full 3D array of RGB values

    # Print specific pixel values
    print(f"RGB value at (100, 100): {rgb_image[100, 100]}")
    print(f"RGB value at (200, 150): {rgb_image[200, 150]}")

    # Calculate and print RGB channel statistics
    r_channel = rgb_image[:, :, 0]  # Red channel
    g_channel = rgb_image[:, :, 1]  # Green channel
    b_channel = rgb_image[:, :, 2]  # Blue channel

    print("\nStatistics for each channel:")
    print(f"Red channel - Min: {np.min(r_channel)}, Max: {np.max(r_channel)}, Mean: {np.mean(r_channel):.3f}")
    print(f"Green channel - Min: {np.min(g_channel)}, Max: {np.max(g_channel)}, Mean: {np.mean(g_channel):.3f}")
    print(f"Blue channel - Min: {np.min(b_channel)}, Max: {np.max(b_channel)}, Mean: {np.mean(b_channel):.3f}")
