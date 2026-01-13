#!/usr/bin/env python3
"""
Phase 3: Clean Multi-View Reconstruction Pipeline with Known Layout.

COORDINATE CONVENTION:
    - All 2D observations: Undistorted pixels (via cv2.undistortPoints with P=K)
    - All 3D coordinates: Millimeters (mm)
    - PnP calls: distCoeffs=None (distortion pre-removed)

This implementation uses known tag layout (layout_4tags.json) to:
1. Initialize structure with all 16 tag corners at known 3D positions
2. Register cameras via PnP with undistorted pixel observations
3. Run global bundle adjustment in L-frame
4. Validate reconstruction quality (<1px reprojection error)

Usage:
    python phase3_clean_pipeline.py \\
        --images calib/test/DSC_*.TIF \\
        --calib calib/1_10/camera_intrinsics.json \\
        --layout calib/fixtures/layout_4tags.json \\
        --output runs/phase3_clean
"""

import argparse
import sys
import json
import numpy as np
import cv2
from pathlib import Path
from glob import glob
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import logging

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import calibration_loader
from incremental_sfm import IncrementalSfM, Camera, Point3D
from bundle_adjustment import bundle_adjust_global
from reconstruction_qa import run_full_qa, print_qa_report, QAStatus
from image_quality_filter import apply_quality_gate_filter

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


def detect_apriltags(
    image_path: str,
    tag_family: str = "tag36h11"
) -> List[Dict]:
    """Detect AprilTags in image using OpenCV's ArUco detector.
    
    Args:
        image_path: Path to image file
        tag_family: AprilTag family (tag36h11 supported)
        
    Returns:
        List of detection dicts with tag_id, corners (4x2 distorted pixels), center
    """
    # Load image
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        logger.error(f"Failed to load image: {image_path}")
        return []
    
    # Initialize ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    
    # Detect markers
    corners, ids, _ = detector.detectMarkers(image)
    
    if ids is None or len(ids) == 0:
        return []
    
    # Convert to list of dicts
    detections = []
    for i, tag_id in enumerate(ids.flatten()):
        corner_pts = corners[i][0]  # Shape: (4, 2)
        center = corner_pts.mean(axis=0)
        
        detections.append({
            "tag_id": int(tag_id),
            "corners": corner_pts.astype(np.float32),  # (4, 2) distorted pixels
            "center": center.astype(np.float32)
        })
    
    return detections


def load_tag_layout(layout_path: Path) -> Tuple[Dict[int, np.ndarray], float]:
    """Load known tag layout with 3D positions.
    
    Args:
        layout_path: Path to layout JSON file
        
    Returns:
        tag_corners_3d_world: Dict mapping tag_id → (4, 3) corner positions in world frame (mm)
        tag_size_mm: Tag coded size in millimeters
    """
    with open(layout_path, 'r') as f:
        layout = json.load(f)
    
    tag_size_mm = layout["tag_size_mm"]
    half_size = tag_size_mm / 2.0
    
    # AprilTag corners in tag-local frame (origin at tag center)
    # Coordinate system: X right, Y down, Z out of page
    tag_corners_local = np.array([
        [-half_size, -half_size, 0],  # Top-left
        [ half_size, -half_size, 0],  # Top-right
        [ half_size,  half_size, 0],  # Bottom-right
        [-half_size,  half_size, 0],  # Bottom-left
    ], dtype=np.float32)
    
    # Transform corners to world frame using tag centers
    tag_corners_3d_world = {}
    for tag_id_str, center_2d in layout["centers_mm"].items():
        tag_id = int(tag_id_str)
        center_3d = np.array(center_2d + [0.0], dtype=np.float32)  # Add Z=0
        
        # All tags are coplanar (Z=0), so just translate corners
        corners_world = tag_corners_local + center_3d
        tag_corners_3d_world[tag_id] = corners_world
    
    logger.info(f"  Loaded layout: {len(tag_corners_3d_world)} tags, tag size {tag_size_mm:.2f}mm")
    
    return tag_corners_3d_world, tag_size_mm


def undistort_to_pixels(
    points_2d_distorted: np.ndarray,
    K: np.ndarray,
    D: np.ndarray
) -> np.ndarray:
    """Convert distorted pixels to undistorted pixels.
    
    COORDINATE CONVENTION:
        Input: Distorted pixel coordinates
        Output: Undistorted pixel coordinates (P=K keeps output as pixels)
    
    Args:
        points_2d_distorted: (N, 2) distorted pixel coordinates
        K: (3, 3) camera intrinsic matrix
        D: Distortion coefficients
        
    Returns:
        points_2d_undist_px: (N, 2) undistorted pixel coordinates
    """
    points_2d_undist_px = cv2.undistortPoints(
        points_2d_distorted.reshape(-1, 1, 2),
        cameraMatrix=K,
        distCoeffs=D,
        P=K  # CRITICAL: Keeps output as pixels (not normalized coords)
    ).reshape(-1, 2)
    
    return points_2d_undist_px


def build_feature_tracks(
    detections_all: Dict[str, List[Dict]]
) -> Dict[int, List[Tuple[str, int]]]:
    """Build feature tracks from AprilTag detections.
    
    Track ID encoding: track_id = tag_id * 4 + corner_idx
    
    Args:
        detections_all: Dict mapping image_id → list of detections
        
    Returns:
        feature_tracks: Dict mapping track_id → [(image_id, feature_idx), ...]
    """
    feature_tracks = {}
    
    for img_id, detections in detections_all.items():
        for det_idx, det in enumerate(detections):
            tag_id = det["tag_id"]
            
            # Each tag has 4 corners (track IDs: tag_id*4, tag_id*4+1, tag_id*4+2, tag_id*4+3)
            for corner_idx in range(4):
                track_id = tag_id * 4 + corner_idx
                
                if track_id not in feature_tracks:
                    feature_tracks[track_id] = []
                
                # Feature index: det_idx * 4 + corner_idx
                feature_idx = det_idx * 4 + corner_idx
                feature_tracks[track_id].append((img_id, feature_idx))
    
    return feature_tracks


def initialize_with_known_layout(
    detections_all: Dict[str, List[Dict]],
    tag_corners_3d_world: Dict[int, np.ndarray],
    K: np.ndarray,
    D: np.ndarray,
    img_id1: str,
    img_id2: str
) -> Tuple[IncrementalSfM, Dict]:
    """Initialize SfM structure using known tag layout.
    
    APPROACH:
        1. Solve PnP for both views using all visible tag corners
        2. Transform known 3D positions from world frame → camera 1 frame
        3. Create IncrementalSfM with all corners at known positions
        4. No triangulation needed (ground truth geometry)
    
    Args:
        detections_all: Dict mapping image_id → list of detections
        tag_corners_3d_world: Dict mapping tag_id → (4, 3) corners in world frame (mm)
        K: (3, 3) camera intrinsic matrix
        D: Distortion coefficients
        img_id1: First image ID
        img_id2: Second image ID
        
    Returns:
        sfm: Initialized IncrementalSfM object
        metadata: Dict with initialization statistics
    """
    logger.info(f"  Initializing with known layout: {img_id1} and {img_id2}")
    
    # Step 1: Collect 3D-2D correspondences for view 1
    points_3d_1_world = []
    points_2d_1_distorted = []
    tag_ids_1 = []
    corner_ids_1 = []
    
    for det in detections_all[img_id1]:
        tag_id = det["tag_id"]
        if tag_id not in tag_corners_3d_world:
            continue
        
        corners_3d = tag_corners_3d_world[tag_id]
        corners_2d = det["corners"]
        
        for corner_idx in range(4):
            points_3d_1_world.append(corners_3d[corner_idx])
            points_2d_1_distorted.append(corners_2d[corner_idx])
            tag_ids_1.append(tag_id)
            corner_ids_1.append(corner_idx)
    
    points_3d_1_world = np.array(points_3d_1_world, dtype=np.float32)
    points_2d_1_distorted = np.array(points_2d_1_distorted, dtype=np.float32)
    
    # Solve PnP for view 1
    success1, rvec1, tvec1 = cv2.solvePnP(
        objectPoints=points_3d_1_world,
        imagePoints=points_2d_1_distorted,
        cameraMatrix=K,
        distCoeffs=D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success1:
        raise ValueError(f"PnP failed for view 1: {img_id1}")
    
    R1, _ = cv2.Rodrigues(rvec1)
    
    # Step 2: Collect 3D-2D correspondences for view 2
    points_3d_2_world = []
    points_2d_2_distorted = []
    tag_ids_2 = []
    corner_ids_2 = []
    
    for det in detections_all[img_id2]:
        tag_id = det["tag_id"]
        if tag_id not in tag_corners_3d_world:
            continue
        
        corners_3d = tag_corners_3d_world[tag_id]
        corners_2d = det["corners"]
        
        for corner_idx in range(4):
            points_3d_2_world.append(corners_3d[corner_idx])
            points_2d_2_distorted.append(corners_2d[corner_idx])
            tag_ids_2.append(tag_id)
            corner_ids_2.append(corner_idx)
    
    points_3d_2_world = np.array(points_3d_2_world, dtype=np.float32)
    points_2d_2_distorted = np.array(points_2d_2_distorted, dtype=np.float32)
    
    # Solve PnP for view 2
    success2, rvec2, tvec2 = cv2.solvePnP(
        objectPoints=points_3d_2_world,
        imagePoints=points_2d_2_distorted,
        cameraMatrix=K,
        distCoeffs=D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success2:
        raise ValueError(f"PnP failed for view 2: {img_id2}")
    
    R2, _ = cv2.Rodrigues(rvec2)
    
    # Validate PnP with reprojection
    points_reproj1, _ = cv2.projectPoints(points_3d_1_world, rvec1, tvec1, K, D)
    points_reproj1 = points_reproj1.reshape(-1, 2)
    reproj_error1 = np.mean(np.linalg.norm(points_2d_1_distorted - points_reproj1, axis=1))
    
    points_reproj2, _ = cv2.projectPoints(points_3d_2_world, rvec2, tvec2, K, D)
    points_reproj2 = points_reproj2.reshape(-1, 2)
    reproj_error2 = np.mean(np.linalg.norm(points_2d_2_distorted - points_reproj2, axis=1))
    
    logger.info(f"     PnP view 1: {len(points_3d_1_world)} points, reproj error {reproj_error1:.3f}px")
    logger.info(f"     PnP view 2: {len(points_3d_2_world)} points, reproj error {reproj_error2:.3f}px")
    logger.info(f"     Baseline: {np.linalg.norm(tvec2 - tvec1):.2f}mm")
    
    if reproj_error1 > 5.0 or reproj_error2 > 5.0:
        raise ValueError(f"PnP reprojection error too high: view1={reproj_error1:.2f}px, view2={reproj_error2:.2f}px")
    
    # Step 3: Transform 3D points from world frame to camera 1 frame
    # Camera 1 becomes the L-frame origin
    # World-to-camera transformation: X_cam = R @ X_world + t
    points_3d_1_cam1 = (R1 @ points_3d_1_world.T + tvec1).T
    
    # Compute relative pose for camera 2
    R_relative = R2 @ R1.T
    t_relative = (tvec2 - R2 @ R1.T @ tvec1).reshape(3, 1)
    
    # Step 4: Undistort 2D observations to pixels
    points_2d_1_undist_px = undistort_to_pixels(points_2d_1_distorted, K, D)
    points_2d_2_undist_px = undistort_to_pixels(points_2d_2_distorted, K, D)
    
    # Step 5: Create IncrementalSfM and populate with known structure
    sfm = IncrementalSfM(K=K, min_ray_angle_deg=5.0)
    
    # Add cameras
    sfm.cameras[img_id1] = Camera(
        image_id=img_id1,
        R=np.eye(3),
        t=np.zeros((3, 1)),
        K=K,
        registered=True
    )
    
    sfm.cameras[img_id2] = Camera(
        image_id=img_id2,
        R=R_relative,
        t=t_relative,
        K=K,
        registered=True
    )
    
    # Add 3D points with track IDs
    for i in range(len(points_3d_1_cam1)):
        tag_id = tag_ids_1[i]
        corner_id = corner_ids_1[i]
        track_id = tag_id * 4 + corner_id
        
        # Check if this point is also visible in view 2
        # Find matching observation in view 2
        obs_2d = {img_id1: points_2d_1_undist_px[i]}
        
        for j in range(len(tag_ids_2)):
            if tag_ids_2[j] == tag_id and corner_ids_2[j] == corner_id:
                obs_2d[img_id2] = points_2d_2_undist_px[j]
                break
        
        if len(obs_2d) >= 2:  # Only add if visible in both views
            sfm.points_3d[track_id] = Point3D(
                point_id=track_id,
                xyz=points_3d_1_cam1[i],
                observations=obs_2d
            )
    
    # Build feature tracks
    for point_id, point in sfm.points_3d.items():
        track = []
        for img_id in point.observations.keys():
            # Feature index doesn't matter for known layout, use point_id
            track.append((img_id, point_id))
        sfm.feature_tracks[point_id] = track
    
    logger.info(f"  [OK] Initialization complete: {len(sfm.points_3d)} points, 2 cameras")
    
    metadata = {
        "method": "known_layout_pnp",
        "img_id1": img_id1,
        "img_id2": img_id2,
        "n_points": len(sfm.points_3d),
        "baseline_mm": float(np.linalg.norm(tvec2 - tvec1)),
        "reproj_error_px": {"view1": float(reproj_error1), "view2": float(reproj_error2)}
    }
    
    return sfm, metadata


def run_phase3_clean_pipeline(
    image_paths: List[Path],
    calib_path: Path,
    layout_path: Path,
    output_dir: Path,
    verbose: bool = True
) -> Tuple[IncrementalSfM, Dict]:
    """Run Phase 3 clean reconstruction pipeline with known layout.
    
    Args:
        image_paths: List of image file paths
        calib_path: Path to camera calibration JSON
        layout_path: Path to tag layout JSON
        output_dir: Output directory for results
        verbose: Enable verbose logging
        
    Returns:
        sfm: Final IncrementalSfM structure
        metadata: Dict with pipeline statistics
    """
    if verbose:
        logger.setLevel(logging.INFO)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    logger.info("=" * 70)
    logger.info("PHASE 3: CLEAN MULTI-VIEW RECONSTRUCTION (KNOWN LAYOUT)")
    logger.info("=" * 70)
    logger.info(f"Images: {len(image_paths)}")
    logger.info(f"Calibration: {calib_path}")
    logger.info(f"Layout: {layout_path}")
    logger.info(f"Output: {output_dir}")
    
    # Step 1: Load camera calibration
    logger.info(f"\n[1/6] Loading camera calibration...")
    K, D, image_size, calib_metadata = calibration_loader.load_calibration(str(calib_path))
    logger.info(f"  Camera matrix K: fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, cx={K[0,2]:.1f}, cy={K[1,2]:.1f}")
    logger.info(f"  Image size: {image_size[0]} x {image_size[1]}")
    
    # Step 2: Load tag layout
    logger.info(f"\n[2/6] Loading tag layout...")
    tag_corners_3d_world, tag_size_mm = load_tag_layout(layout_path)
    
    # Step 3: Detect AprilTags in all images
    logger.info(f"\n[3/6] Detecting AprilTags...")
    detections_all = {}
    for img_path in image_paths:
        img_id = img_path.stem
        detections = detect_apriltags(str(img_path))
        
        if len(detections) > 0:
            detections_all[img_id] = detections
            logger.info(f"  {img_id}: {len(detections)} tags detected")
        else:
            logger.warning(f"  {img_id}: No tags detected (skipping)")
    
    if len(detections_all) < 2:
        raise ValueError(f"Need at least 2 images with detections, got {len(detections_all)}")
    
    # Step 4: Build feature tracks
    logger.info(f"\n[4/6] Building feature tracks...")
    feature_tracks = build_feature_tracks(detections_all)
    feature_tracks_filtered = {
        track_id: track for track_id, track in feature_tracks.items()
        if len(track) >= 2
    }
    logger.info(f"  Total tracks: {len(feature_tracks)}")
    logger.info(f"  Tracks with >= 2 views: {len(feature_tracks_filtered)}")
    
    # Step 5: Initialize SfM with known layout
    logger.info(f"\n[5/6] Initializing SfM with known layout...")
    image_ids = sorted(detections_all.keys())
    img_id1, img_id2 = image_ids[0], image_ids[1]
    
    sfm, init_metadata = initialize_with_known_layout(
        detections_all=detections_all,
        tag_corners_3d_world=tag_corners_3d_world,
        K=K,
        D=D,
        img_id1=img_id1,
        img_id2=img_id2
    )
    
    # Step 6: Register remaining cameras
    logger.info(f"\n[6/6] Registering additional cameras...")
    n_registered = 2  # Already have 2 cameras
    
    for img_id in image_ids[2:]:
        if img_id in sfm.cameras:
            continue
        
        # Get 2D-3D correspondences for this image
        points_2d_distorted_all = []
        correspondences = []  # List of (point_id, feature_idx)
        
        for det in detections_all[img_id]:
            tag_id = det["tag_id"]
            if tag_id not in tag_corners_3d_world:
                continue
            
            for corner_idx in range(4):
                track_id = tag_id * 4 + corner_idx
                
                if track_id in sfm.points_3d:
                    feature_idx = len(points_2d_distorted_all)
                    points_2d_distorted_all.append(det["corners"][corner_idx])
                    correspondences.append((track_id, feature_idx))
        
        if len(correspondences) < 4:
            logger.warning(f"  [SKIP] {img_id}: Insufficient correspondences ({len(correspondences)} < 4)")
            continue
        
        points_2d_distorted_all = np.array(points_2d_distorted_all, dtype=np.float32)
        
        # Undistort 2D points
        points_2d_undist_px = undistort_to_pixels(points_2d_distorted_all, K, D)
        
        # Register camera via PnP
        success, error_msg = sfm.register_camera(
            image_id=img_id,
            points_2d=points_2d_undist_px,
            correspondences=correspondences
        )
        
        if success:
            n_registered += 1
            logger.info(f"  [OK] {img_id}: Registered ({len(correspondences)} correspondences)")
        else:
            logger.warning(f"  [SKIP] {img_id}: {error_msg}")
    
    logger.info(f"\n  Total cameras registered: {n_registered}/{len(image_ids)}")
    
    # Step 7: Global bundle adjustment
    logger.info(f"\n[7/6] Global bundle adjustment...")
    
    try:
        sfm_refined, ba_info = bundle_adjust_global(
            sfm=sfm,
            fix_first_camera=True,
            loss_function='huber',
            loss_scale=1.0,
            verbose=1
        )
        
        logger.info(f"  [OK] Bundle adjustment converged")
        sfm = sfm_refined
        
    except ValueError as e:
        logger.error(f"  [FAIL] Bundle adjustment failed: {e}")
        logger.info(f"  Continuing with pre-BA structure...")
    
    # Step 8: Quality gate - Remove worst 10% images
    logger.info(f"\n[8/6] Quality gate - Filtering worst images...")
    
    sfm_filtered, filter_report = apply_quality_gate_filter(
        sfm=sfm,
        percentile=10.0,
        criterion='mean',
        verbose=True
    )
    
    # Re-run bundle adjustment if images were removed
    if filter_report['status'] == 'success':
        logger.info(f"\n  Re-running bundle adjustment after image filtering...")
        
        try:
            sfm_filtered_refined, ba_info2 = bundle_adjust_global(
                sfm=sfm_filtered,
                fix_first_camera=True,
                loss_function='huber',
                loss_scale=1.0,
                verbose=1
            )
            
            logger.info(f"  [OK] Post-filter bundle adjustment converged")
            sfm = sfm_filtered_refined
            
        except ValueError as e:
            logger.error(f"  [FAIL] Post-filter bundle adjustment failed: {e}")
            sfm = sfm_filtered
    else:
        logger.info(f"  Quality gate skipped: {filter_report.get('reason', 'unknown')}")
    
    # Step 9: Quality assurance
    logger.info(f"\n[9/6] Quality assurance validation...")
    
    qa_report = run_full_qa(
        sfm=sfm,
        apriltag_corners_3d=tag_corners_3d_world,
        expected_tag_edge_mm=tag_size_mm,
        bridge_point_ids=None  # Will check all points
    )
    
    print_qa_report(qa_report)
    
    # Export results
    logger.info(f"\nExporting results to {output_dir}...")
    
    # Export L-frame structure
    sfm.export_to_json(output_dir / "refpoints_L.json")
    logger.info(f"  Exported: refpoints_L.json")
    
    # Export metadata
    metadata = {
        "pipeline_version": "phase3_clean_v1",
        "timestamp": datetime.now().isoformat(),
        "images": {
            "total": len(image_paths),
            "with_detections": len(detections_all),
            "registered": n_registered,
            "after_quality_gate": len([c for c in sfm.cameras.values() if c.registered])
        },
        "structure": {
            "n_cameras": len(sfm.cameras),
            "n_points_3d": len(sfm.points_3d),
            "n_tracks": len(feature_tracks_filtered)
        },
        "initialization": init_metadata,
        "quality_gate": filter_report,
        "qa_status": qa_report.overall_status.value,
        "qa_hard_failures": qa_report.hard_failures,
        "qa_warnings": qa_report.warnings,
        "cameras": [],  # For GUI Quality Gate tab
        "points": []    # For GUI Quality Gate tab
    }
    
    # Add camera data for GUI
    for cam in sfm.cameras.values():
        if cam.registered:
            metadata["cameras"].append({
                "id": cam.image_id,
                "image_name": cam.image_id,
                "K": cam.K.tolist(),
                "dist_coeffs": None,  # Already undistorted
                "R": cam.R.tolist(),
                "t": cam.t.tolist()
            })
    
    # Add point data for GUI
    for pt in sfm.points_3d.values():
        metadata["points"].append({
            "id": pt.point_id,
            "xyz": pt.xyz.tolist(),
            "observations": {
                img_id: obs.tolist() for img_id, obs in pt.observations.items()
            }
        })
    
    # Add reprojection stats if available
    reproj_check = next((c for c in qa_report.checks if c.name == "Reprojection Errors"), None)
    if reproj_check and reproj_check.details:
        metadata["qa_summary"] = {
            "reprojection_mean_px": reproj_check.details.get("mean_error_px"),
            "reprojection_max_px": reproj_check.details.get("max_error_px")
        }
    else:
        metadata["qa_summary"] = {
            "reprojection_mean_px": None,
            "reprojection_max_px": None
        }
    
    with open(output_dir / "metadata.json", 'w') as f:
        json.dump(metadata, f, indent=2)
    logger.info(f"  Exported: metadata.json")
    
    # Final status
    logger.info("\n" + "=" * 70)
    if qa_report.overall_status == QAStatus.PASS:
        logger.info("PHASE 3 COMPLETE - QUALITY GATES PASSED")
    elif qa_report.overall_status == QAStatus.WARN:
        logger.warning("PHASE 3 COMPLETE - QUALITY WARNINGS")
    else:
        logger.error("PHASE 3 FAILED - QUALITY GATES NOT MET")
    logger.info("=" * 70)
    
    return sfm, metadata


def main():
    parser = argparse.ArgumentParser(
        description="Phase 3: Clean multi-view reconstruction with known layout",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument(
        '--images',
        type=str,
        required=True,
        help='Glob pattern for input images (e.g., "calib/test/DSC_*.TIF")'
    )
    
    parser.add_argument(
        '--calib',
        type=Path,
        required=True,
        help='Path to camera calibration JSON'
    )
    
    parser.add_argument(
        '--layout',
        type=Path,
        required=True,
        help='Path to tag layout JSON'
    )
    
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('runs/phase3_clean'),
        help='Output directory (default: runs/phase3_clean)'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )
    
    args = parser.parse_args()
    
    # Resolve image paths
    image_paths = sorted([Path(p) for p in glob(args.images)])
    
    if len(image_paths) == 0:
        logger.error(f"No images found matching: {args.images}")
        sys.exit(1)
    
    # Run pipeline
    try:
        sfm, metadata = run_phase3_clean_pipeline(
            image_paths=image_paths,
            calib_path=args.calib,
            layout_path=args.layout,
            output_dir=args.output,
            verbose=args.verbose
        )
        
        # Exit with appropriate code based on QA status
        if metadata["qa_status"] == "fail":
            sys.exit(1)
        elif metadata["qa_status"] == "warn":
            sys.exit(0)  # Warnings are acceptable
        else:
            sys.exit(0)
            
    except Exception as e:
        logger.error(f"Pipeline failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
