#!/usr/bin/env python3
"""Test AprilTag detection with multiple dictionaries."""

import cv2
import sys

def test_all_dictionaries(image_path):
    """Try detecting AprilTags with all available dictionaries."""
    
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Failed to load image: {image_path}")
        return
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(f"Testing image: {image_path}")
    print(f"Image shape: {img.shape}")
    print()
    
    # All ArUco/AprilTag dictionaries
    dictionaries = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
        'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
        'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        'DICT_APRILTAG_16h5': cv2.aruco.DICT_APRILTAG_16h5,
        'DICT_APRILTAG_25h9': cv2.aruco.DICT_APRILTAG_25h9,
        'DICT_APRILTAG_36h10': cv2.aruco.DICT_APRILTAG_36h10,
        'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
    }
    
    detections_found = False
    
    # Try each dictionary
    for dict_name, dict_id in dictionaries.items():
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            parameters = cv2.aruco.DetectorParameters()
            
            # Use default parameters (they work well)
            
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                print(f"✓ {dict_name}: Found {len(ids)} tags")
                print(f"  Tag IDs: {sorted(ids.flatten().tolist())}")
                detections_found = True
        except Exception as e:
            print(f"✗ {dict_name}: Error - {e}")
    
    if not detections_found:
        print("\n⚠ No tags detected with any dictionary!")
        print("  Possible reasons:")
        print("  - Image doesn't contain AprilTags")
        print("  - Tags are too small or blurry")
        print("  - Lighting/contrast issues")
        print("  - Tags are occluded or damaged")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_detection.py <image_path>")
        sys.exit(1)
    
    test_all_dictionaries(sys.argv[1])
