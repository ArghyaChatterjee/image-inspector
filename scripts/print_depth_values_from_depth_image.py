import cv2
import numpy as np

# Path to the saved 16-bit depth image
depth_image_path = "/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241025_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080_test_6/depth/depth_000000.png"
# depth_image_path = "/home/arghya/kitti_depth/data_depth_selection/test_depth_completion_anonymous/velodyne_raw/0000000000.png"
# depth_image_path = "/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/Iphone_recording/2024-11-14--13-36-52/depth/0.png"

# Load the depth image as a 16-bit grayscale image
depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

# Check if the image was loaded correctly
if depth_image is None:
    print("Failed to load image.")
else:
    # Ask the user for the preferred unit interpretation
    unit = input("Is the depth image in meters (m) or millimeters (mm)? Enter 'm' for meters or 'mm' for millimeters: ").strip().lower()

    # Use the user's input for unit label only
    if unit == 'm':
        unit_label = "m"
    elif unit == 'mm':
        unit_label = "mm"
    else:
        print("Invalid unit input. Defaulting to millimeters.")
        unit_label = "mm"

    # Print depth values based on chosen unit
    print(f"Depth values (interpreted in {unit_label}):")
    print(depth_image)  # Print entire array (could be large)

    # Print specific pixel values
    print(f"Depth at (100, 100): {depth_image[100, 100]:.3f} {unit_label}")
    print(f"Depth at (200, 150): {depth_image[200, 150]:.3f} {unit_label}")

    # Print depth statistics
    print(f"Minimum depth: {np.min(depth_image):.3f} {unit_label}")
    print(f"Maximum depth: {np.max(depth_image):.3f} {unit_label}")
    print(f"Mean depth: {np.mean(depth_image):.3f} {unit_label}")
