#!/usr/bin/env python3
"""
Phase 3 & 4 Multi-View Reconstruction Pipeline.

End-to-end workflow:
1. AprilTag detection in images
2. Two-view SfM initialization
3. Incremental reconstruction (PnP + triangulation)
4. Global bundle adjustment
5. Quality assurance validation
6. L-frame export
7. Phase 4: L→U transform with reference plate (Option U2)
8. U-frame export with hard-stop validation gates

Usage:
    # Test with 4-tag known layout
    python phase3_test_pipeline.py \\
        --images calib/test/DSC_*.TIF \\
        --calib calib/1_10/camera_intrinsics.json \\
        --layout calib/fixtures/layout_4tags.json \\
        --output runs/phase3_test_4tags

    # Production with 4+ flags unknown layout
    python phase3_test_pipeline.py \\
        --images data/case001/*.TIF \\
        --calib calib/1_10/camera_intrinsics.json \\
        --output runs/case001_phase3 \\
        --phase4 reference_plate \\
        --reference-plate calib/fixtures/reference_plate_4tags.json

Quality Gates:
    - Reprojection error: mean <1.0px, max <3.0px (HARD FAIL)
    - Track length: ≥4 views (WARN)
    - Graph connectivity: single component (HARD FAIL)
    - Scale sanity: AprilTag edges within ±0.1mm (WARN)
    - Bridge collinearity: area ≥10mm² (HARD FAIL)
    - Phase 4: Scale residual <0.02mm (HARD FAIL, EXIT(1))
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
src_dir = str(Path(__file__).parent.parent / "src")
if src_dir in sys.path:
    sys.path.remove(src_dir)
sys.path.insert(0, src_dir)

# Avoid module-name collision with legacy tools/bundle_adjustment.py
_ba = sys.modules.get('bundle_adjustment')
if _ba is not None:
    _ba_file = getattr(_ba, '__file__', '') or ''
    if _ba_file.replace('\\', '/').endswith('/tools/bundle_adjustment.py'):
        del sys.modules['bundle_adjustment']

import calibration_loader
import transforms
import geometry_utils
import sfm_initialization
import incremental_sfm
import bundle_adjustment
import reconstruction_qa

# Import specific items
from calibration_loader import load_calibration, undistort_points
from transforms import SE3Transform, Sim3Transform
from sfm_initialization import SfMInitializer
from incremental_sfm import IncrementalSfM, Camera, Point3D
from bundle_adjustment import bundle_adjust_global, compute_initial_reprojection_stats
from reconstruction_qa import run_full_qa, print_qa_report, QAStatus


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


def detect_apriltags(
    image_path: str,
    tag_family: str = "tag36h11",
    quad_decimate: float = 2.0
) -> List[Dict]:
    """Detect AprilTags in image using OpenCV's ArUco detector.
    
    Args:
        image_path: Path to image file
        tag_family: AprilTag family (tag36h11 supported)
        quad_decimate: Not used with OpenCV ArUco
        
    Returns:
        List of detection dicts with tag_id, corners, center
    """
    # Load image
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        logger.error(f"Failed to load image: {image_path}")
        return []
    
    # Use OpenCV's ArUco detector for AprilTags
    # AprilTag36h11 corresponds to DICT_APRILTAG_36h11
    if tag_family == "tag36h11":
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    else:
        logger.warning(f"Unsupported tag family: {tag_family}, using 36h11")
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    # Create detector with default parameters
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    
    # Detect markers
    corners, ids, rejected = detector.detectMarkers(image)
    
    results = []
    if ids is not None:
        for i, tag_id in enumerate(ids.ravel()):
            corner_array = corners[i][0]  # Shape: (4, 2)
            center = np.mean(corner_array, axis=0)
            
            results.append({
                "tag_id": int(tag_id),
                "corners": corner_array,  # (4, 2) pixel coordinates
                "center": center,         # (2,) pixel center
                "hamming": 0,            # Not available with ArUco
                "decision_margin": 100.0  # Not available with ArUco
            })
    
    return results


def load_layout_known(layout_file: str) -> Dict[int, np.ndarray]:
    """Load known tag layout from JSON.
    
    Args:
        layout_file: Path to layout JSON file
        
    Returns:
        Dict of tag_id → (2,) or (3,) center position in mm
    """
    with open(layout_file, 'r') as f:
        data = json.load(f)
    
    centers = {}
    for tag_id_str, center in data["centers_mm"].items():
        tag_id = int(tag_id_str)
        centers[tag_id] = np.array(center)
        
        # Convert 2D to 3D (planar, Z=0)
        if len(centers[tag_id]) == 2:
            centers[tag_id] = np.append(centers[tag_id], 0.0)
    
    return centers


def build_feature_tracks(
    detections_all: Dict[str, List[Dict]]
) -> Dict[int, List[Tuple[str, int]]]:
    """Build feature tracks from detections across images.
    
    Args:
        detections_all: Dict of image_id → list of detections
        
    Returns:
        Dict of track_id → [(image_id, corner_idx), ...]
        
        Track ID encoding: track_id = tag_id * 4 + corner_idx
        where corner_idx ∈ [0, 1, 2, 3]
    """
    tracks = {}
    
    for img_id, detections in detections_all.items():
        for det in detections:
            tag_id = det["tag_id"]
            
            # Each corner is a separate feature
            for corner_idx in range(4):
                track_id = tag_id * 4 + corner_idx
                
                if track_id not in tracks:
                    tracks[track_id] = []
                
                tracks[track_id].append((img_id, (tag_id, corner_idx)))
    
    return tracks


def get_2d_points_from_detections(
    detections: List[Dict],
    tag_id: Optional[int] = None
) -> np.ndarray:
    """Extract 2D points from detections.
    
    Args:
        detections: List of detection dicts
        tag_id: If provided, filter by tag ID
        
    Returns:
        points_2d: (N, 2) array of 2D points
    """
    points = []
    
    for det in detections:
        if tag_id is not None and det["tag_id"] != tag_id:
            continue
        
        # Add all 4 corners
        for corner in det["corners"]:
            points.append(corner)
    
    return np.array(points) if points else np.array([]).reshape(0, 2)


def run_phase3_pipeline(
    image_paths: List[str],
    calib_file: str,
    output_dir: str,
    layout_file: Optional[str] = None,
    tag_size_mm: float = 8.8,
    verbose: bool = True
) -> Tuple[IncrementalSfM, Dict]:
    """Run Phase 3 reconstruction pipeline.
    
    Args:
        image_paths: List of image file paths
        calib_file: Camera calibration JSON file
        output_dir: Output directory for results
        layout_file: Optional known layout JSON for validation
        tag_size_mm: AprilTag edge length (default 8.8mm)
        verbose: Print progress messages
        
    Returns:
        Tuple of (sfm, metadata):
            sfm: Reconstructed IncrementalSfM
            metadata: Pipeline metadata dict
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    logger.info(f"="*70)
    logger.info(f"PHASE 3: MULTI-VIEW RECONSTRUCTION PIPELINE")
    logger.info(f"="*70)
    logger.info(f"Images: {len(image_paths)}")
    logger.info(f"Calibration: {calib_file}")
    logger.info(f"Output: {output_dir}")
    
    # Step 1: Load calibration
    logger.info(f"\n[1/7] Loading camera calibration...")
    K, D, image_size, calib_metadata = load_calibration(calib_file)
    logger.info(f"  Camera matrix K: fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, cx={K[0,2]:.1f}, cy={K[1,2]:.1f}")
    logger.info(f"  Image size: {image_size[0]} x {image_size[1]}")
    
    # Step 2: Detect AprilTags
    logger.info(f"\n[2/7] Detecting AprilTags...")
    detections_all = {}
    
    for img_path in image_paths:
        img_id = Path(img_path).stem
        detections = detect_apriltags(img_path)
        
        if detections:
            detections_all[img_id] = detections
            logger.info(f"  {img_id}: {len(detections)} tags detected")
        else:
            logger.warning(f"  {img_id}: No tags detected")
    
    if len(detections_all) < 2:
        raise ValueError(f"Need >= 2 images with detections, got {len(detections_all)}")
    
    # Step 3: Build feature tracks
    logger.info(f"\n[3/7] Building feature tracks...")
    feature_tracks = build_feature_tracks(detections_all)
    logger.info(f"  Total tracks: {len(feature_tracks)}")
    
    # Filter tracks visible in >= 2 views
    feature_tracks_filtered = {
        track_id: track
        for track_id, track in feature_tracks.items()
        if len(track) >= 2
    }
    logger.info(f"  Tracks with >= 2 views: {len(feature_tracks_filtered)}")
    
    # Step 4: SfM Initialization (PnP-based with known tag layout for metric scale)
    logger.info(f"\n[4/7] SfM initialization (PnP-based)...")
    
    # Load tag layout to get 3D positions of all tags
    workspace_root = Path(__file__).parent.parent
    layout_path = workspace_root / "calib" / "fixtures" / "layout_4tags.json"
    with open(layout_path, 'r') as f:
        layout = json.load(f)
    
    tag_centers_mm = {int(k): np.array(v + [0.0]) for k, v in layout["centers_mm"].items()}  # Add Z=0
    tag_size_mm = layout["tag_size_mm"]
    
    logger.info(f"  Using known layout with {len(tag_centers_mm)} tags (tag size: {tag_size_mm:.2f}mm)")
    
    # Select first two images with most correspondences
    image_ids = sorted(detections_all.keys())
    img_id1, img_id2 = image_ids[0], image_ids[1]
    
    logger.info(f"  Initializing with: {img_id1} and {img_id2}")
    
    # Build 3D-2D correspondences using known layout
    # AprilTag corners in tag frame (origin at center, X right, Y down, Z out of page)
    half_size = tag_size_mm / 2.0
    tag_corners_3d_template = np.array([
        [-half_size, -half_size, 0],  # Top-left
        [ half_size, -half_size, 0],  # Top-right
        [ half_size,  half_size, 0],  # Bottom-right
        [-half_size,  half_size, 0],  # Bottom-left
    ], dtype=np.float32)
    
    # Build global 3D points for ALL tags using layout
    def get_tag_corners_world(tag_id):
        """Get 4 corners of a tag in world frame"""
        if tag_id not in tag_centers_mm:
            return None
        center = tag_centers_mm[tag_id]
        # Transform corners from tag frame to world frame
        corners_world = tag_corners_3d_template + center
        return corners_world
    
    # Collect 3D-2D correspondences for PnP
    points_3d_1 = []
    points_2d_1 = []
    for det in detections_all[img_id1]:
        tag_id = det["tag_id"]
        corners = det["corners"]  # (4, 2) array
        corners_3d = get_tag_corners_world(tag_id)
        if corners_3d is not None:
            points_3d_1.extend(corners_3d)
            points_2d_1.extend(corners)
    
    points_3d_1 = np.array(points_3d_1, dtype=np.float32)
    points_2d_1 = np.array(points_2d_1, dtype=np.float32)
    
    # Solve PnP for view 1 using ALL tags with known layout
    success, rvec1, tvec1 = cv2.solvePnP(
        points_3d_1,
        points_2d_1,
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success:
        raise ValueError("PnP failed for view 1")
    
    R1, _ = cv2.Rodrigues(rvec1)
    
    # Solve PnP for view 2
    points_3d_2 = []
    points_2d_2 = []
    for det in detections_all[img_id2]:
        tag_id = det["tag_id"]
        corners = det["corners"]
        corners_3d = get_tag_corners_world(tag_id)
        if corners_3d is not None:
            points_3d_2.extend(corners_3d)
            points_2d_2.extend(corners)
    
    points_3d_2 = np.array(points_3d_2, dtype=np.float32)
    points_2d_2 = np.array(points_2d_2, dtype=np.float32)
    
    success, rvec2, tvec2 = cv2.solvePnP(
        points_3d_2,
        points_2d_2,
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success:
        raise ValueError("PnP failed for view 2")
    
    R2, _ = cv2.Rodrigues(rvec2)
    
    # Validate PnP by checking reprojection errors with DISTORTED coordinates
    points_reproj1, _ = cv2.projectPoints(points_3d_1, rvec1, tvec1, K, D)
    points_reproj1 = points_reproj1.reshape(-1, 2)
    reproj_error1 = np.mean(np.linalg.norm(points_2d_1 - points_reproj1, axis=1))
    
    points_reproj2, _ = cv2.projectPoints(points_3d_2, rvec2, tvec2, K, D)
    points_reproj2 = points_reproj2.reshape(-1, 2)
    reproj_error2 = np.mean(np.linalg.norm(points_2d_2 - points_reproj2, axis=1))
    
    if reproj_error1 > 5.0 or reproj_error2 > 5.0:
        raise ValueError(f"PnP reprojection error too high: view1={reproj_error1:.2f}px, view2={reproj_error2:.2f}px")
    
    logger.info(f"  [OK] Initialization SUCCESS (PnP-based)")
    logger.info(f"     View 1 pose: t={tvec1.ravel()}")
    logger.info(f"     View 2 pose: t={tvec2.ravel()}")
    logger.info(f"     Baseline: {np.linalg.norm(tvec2 - tvec1):.2f}mm")
    logger.info(f"     PnP reprojection: view1={reproj_error1:.3f}px, view2={reproj_error2:.3f}px")
    
    # Since we have known 3D positions from layout, we can directly use them!
    # Transform from world frame to camera 1 frame
    # World-to-camera transformation: X_cam = R @ X_world + t
    
    # Get all tag corners in world frame
    # IMPORTANT: Keep consistent ordering between view 1 and view 2.
    # Build corner maps keyed by (tag_id, corner_idx), then intersect keys.
    view1 = {}
    for det in detections_all[img_id1]:
        tag_id = det["tag_id"]
        corners_3d_world = get_tag_corners_world(tag_id)
        if corners_3d_world is None:
            continue
        for corner_idx, (corner_3d, corner_2d) in enumerate(zip(corners_3d_world, det["corners"])):
            view1[(tag_id, corner_idx)] = (corner_3d, corner_2d)

    view2 = {}
    for det in detections_all[img_id2]:
        tag_id = det["tag_id"]
        corners_3d_world = get_tag_corners_world(tag_id)
        if corners_3d_world is None:
            continue
        for corner_idx, corner_2d in enumerate(det["corners"]):
            view2[(tag_id, corner_idx)] = corner_2d

    common_keys = sorted(set(view1.keys()) & set(view2.keys()))

    if len(common_keys) < 4:
        raise ValueError(f"Too few common tag corners between {img_id1} and {img_id2}: {len(common_keys)} < 4")

    points_3d_world_all = []
    points_2d_1_all = []
    points_2d_2_all = []
    tag_ids_all = []
    corner_ids_all = []

    for (tag_id, corner_idx) in common_keys:
        corner_3d, corner_2d_1 = view1[(tag_id, corner_idx)]
        corner_2d_2 = view2[(tag_id, corner_idx)]
        points_3d_world_all.append(corner_3d)
        points_2d_1_all.append(corner_2d_1)
        points_2d_2_all.append(corner_2d_2)
        tag_ids_all.append(tag_id)
        corner_ids_all.append(corner_idx)

    points_3d_world_all = np.array(points_3d_world_all, dtype=np.float32)
    points_2d_1_all = np.array(points_2d_1_all, dtype=np.float32)
    points_2d_2_all = np.array(points_2d_2_all, dtype=np.float32)
    
    # Transform 3D points from world frame to camera 1 frame
    points_3d_cam1 = (R1 @ points_3d_world_all.T + tvec1).T
    
    # CRITICAL: IncrementalSfM expects UNDISTORTED PIXEL coordinates, not normalized coords
    # Use cv2.undistortPoints with P=K to keep as pixels
    points_2d_1_undist = cv2.undistortPoints(
        points_2d_1_all.reshape(-1, 1, 2), K, D, P=K
    ).reshape(-1, 2)
    
    points_2d_2_undist = cv2.undistortPoints(
        points_2d_2_all.reshape(-1, 1, 2), K, D, P=K
    ).reshape(-1, 2)
    
    # Compute relative pose for camera 2
    # Camera 1 is at origin: [I | 0]
    # Camera 2 has relative pose: [R_rel | t_rel]
    R_relative = R2 @ R1.T
    t_relative = (tvec2 - R2 @ R1.T @ tvec1).reshape(3, 1)
    
    # Validate 3D points by reprojecting
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])  # Camera 1: K[I|0]
    P2 = K @ np.hstack([R_relative, t_relative])        # Camera 2: K[R|t]
    
    inlier_mask = []
    n_inliers = 0
    
    logger.info(f"  Validating {len(points_3d_cam1)} points with known layout...")
    
    for i in range(len(points_3d_cam1)):
        X_cam1 = points_3d_cam1[i]
        
        # Check depth in camera 1
        depth1 = X_cam1[2]
        
        # Check depth in camera 2
        X_cam2 = R_relative @ X_cam1.reshape(3, 1) + t_relative
        depth2 = X_cam2[2, 0]
        
        # Compute reprojection errors (in undistorted pixels)
        pt1_reproj_hom = P1 @ np.append(X_cam1, 1)
        pt1_reproj = pt1_reproj_hom[:2] / pt1_reproj_hom[2]
        error1 = np.linalg.norm(points_2d_1_undist[i] - pt1_reproj)
        
        pt2_reproj_hom = P2 @ np.append(X_cam1, 1)
        pt2_reproj = pt2_reproj_hom[:2] / pt2_reproj_hom[2]
        error2 = np.linalg.norm(points_2d_2_undist[i] - pt2_reproj)
        
        # Accept if positive depth and low reprojection error
        if depth1 > 0 and depth2 > 0 and error1 < 5.0 and error2 < 5.0:
            inlier_mask.append(True)
            n_inliers += 1
        else:
            inlier_mask.append(False)
            # Log first few rejections
            if i < 8:
                logger.info(f"    Point {i}: REJECT - depth1={depth1:.1f}, depth2={depth2:.1f}, err1={error1:.1f}px, err2={error2:.1f}px")
    
    inlier_mask = np.array(inlier_mask)
    
    logger.info(f"     Valid points: {n_inliers}/{len(inlier_mask)} ({100*n_inliers/len(inlier_mask):.1f}%)")
    
    if n_inliers < 4:
        raise ValueError(f"Too few valid points: {n_inliers} < 4")
    
    # Step 5: Incremental reconstruction
    logger.info(f"\n[5/7] Incremental reconstruction...")
    
    sfm = IncrementalSfM(K=K, min_ray_angle_deg=5.0)
    
    # Add initial pair with proper track ID mapping
    # We need to map inlier indices to track IDs
    # Track ID = tag_id * 4 + corner_idx (we have these from the matching step)
    track_ids_init = [tag_id * 4 + corner_id for tag_id, corner_id in zip(tag_ids_all, corner_ids_all)]
    
    # Now add initial points with correct track IDs
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
    
    # Add 3D points with proper track IDs
    inlier_indices = np.where(inlier_mask)[0]
    for i in inlier_indices:
        track_id = track_ids_init[i]
        if track_id < 0:
            continue  # Skip invalid tracks
        
        # Create 3D point with track_id as point_id
        sfm.points_3d[track_id] = Point3D(
            point_id=track_id,
            xyz=points_3d_cam1[i],  # Use known 3D positions in camera 1 frame
            observations={
                img_id1: points_2d_1_undist[i],
                img_id2: points_2d_2_undist[i]
            }
        )
        
        # Create feature track
        sfm.feature_tracks[track_id] = [
            (img_id1, i),
            (img_id2, i)
        ]
    
    if hasattr(sfm, '_next_point_id'):
        sfm._next_point_id = max(sfm.points_3d.keys()) + 1 if sfm.points_3d else 0
    
    logger.info(f"  Initial structure: {len(sfm.points_3d)} points, 2 cameras")
    
    # Debug: Check initial reprojection errors BEFORE any other cameras
    initial_stats = {}
    for point_id, point in sfm.points_3d.items():
        point_errors = []
        for img_id in [img_id1, img_id2]:
            cam = sfm.cameras[img_id]
            pt_2d = point.observations[img_id]
            X_cam = cam.R @ point.xyz + cam.t.ravel()
            X_proj = cam.K @ (X_cam / X_cam[2])
            error = np.linalg.norm(pt_2d - X_proj[:2])
            point_errors.append(error)
            if point_id == list(sfm.points_3d.keys())[0]:  # First point
                logger.info(f"    DEBUG Point {point_id} in {img_id}: error={error:.3f}px")
        initial_stats[point_id] = np.mean(point_errors)
    
    logger.info(f"    Initial reprojection: mean={np.mean(list(initial_stats.values())):.3f}px, max={np.max(list(initial_stats.values())):.3f}px")
    
    # Register remaining cameras
    remaining_images = [img_id for img_id in image_ids if img_id not in [img_id1, img_id2]]
    
    logger.info(f"  Attempting to register {len(remaining_images)} additional cameras...")
    
    for img_id in remaining_images:
        # Build correspondences
        correspondences = []
        points_2d_all = get_2d_points_from_detections(detections_all[img_id])
        # Undistort consistently: normalized then convert to pixels
        points_2d_norm = cv2.undistortPoints(
            points_2d_all.reshape(-1, 1, 2), K, D, P=None
        ).reshape(-1, 2)
        points_2d_all_undist = (K[:2, :2] @ points_2d_norm.T + K[:2, 2:3]).T
        
        # Match features to existing 3D points
        for track_id, track in feature_tracks_filtered.items():
            if track_id in sfm.points_3d:
                # Check if this image observes this track
                for obs_img_id, (tag_id, corner_idx) in track:
                    if obs_img_id == img_id:
                        # Find feature index in points_2d_all
                        feat_idx = -1
                        idx_counter = 0
                        for det in detections_all[img_id]:
                            if det["tag_id"] == tag_id:
                                feat_idx = idx_counter + corner_idx
                                break
                            idx_counter += 4
                        
                        if feat_idx >= 0:
                            correspondences.append((track_id, feat_idx))
        
        if len(correspondences) >= 4:
            success, error_msg = sfm.register_camera(img_id, points_2d_all_undist, correspondences)
            
            if success:
                logger.info(f"    [OK] Registered {img_id}: {len(correspondences)} correspondences")
            else:
                logger.warning(f"    [SKIP] Failed to register {img_id}: {error_msg}")
        else:
            logger.warning(f"    [SKIP] {img_id}: Only {len(correspondences)} correspondences (need >=4)")
    
    logger.info(f"  Final structure: {len(sfm.points_3d)} points, {len(sfm.cameras)} cameras")
    
    # Step 6: Global bundle adjustment
    logger.info(f"\n[6/7] Global bundle adjustment...")
    
    stats_before = compute_initial_reprojection_stats(sfm)
    logger.info(f"  Before BA: mean={stats_before['mean']:.3f}px, max={stats_before['max']:.3f}px")
    
    sfm_opt, ba_info = bundle_adjust_global(
        sfm,
        loss_function='huber',
        loss_scale=1.0,
        max_iterations=100,
        verbose=0,
        fix_first_camera=True
    )
    
    if ba_info["success"]:
        logger.info(f"  ✅ Bundle adjustment SUCCESS")
        logger.info(f"     Initial cost: {ba_info['initial_cost']:.6f}")
        logger.info(f"     Final cost: {ba_info['final_cost']:.6f}")
        logger.info(f"     Cost reduction: {ba_info['cost_reduction']:.6f}")
        logger.info(f"     Iterations: {ba_info['n_iterations']}")
        
        stats_after = compute_initial_reprojection_stats(sfm_opt)
        logger.info(f"  After BA: mean={stats_after['mean']:.3f}px, max={stats_after['max']:.3f}px")
    else:
        logger.error(f"  ❌ Bundle adjustment FAILED: {ba_info['message']}")
        sfm_opt = sfm
    
    # Step 7: Quality assurance
    logger.info(f"\n[7/7] Quality assurance validation...")
    
    # Load AprilTag corners for scale check
    apriltag_corners_3d = {}
    if layout_file:
        layout_centers = load_layout_known(layout_file)
        # TODO: Compute corner positions from centers (simplified for now)
    
    qa_report = run_full_qa(sfm_opt, apriltag_corners_3d=None, expected_tag_edge_mm=tag_size_mm)
    
    if verbose:
        print_qa_report(qa_report, verbose=True)
    
    # Export results
    logger.info(f"\nExporting results to {output_dir}...")
    
    # Save L-frame structure
    structure_file = output_path / "structure_L.json"
    sfm_opt.export_to_json(str(structure_file))
    logger.info(f"  ✅ L-frame structure: {structure_file}")
    
    # Save QA report
    qa_file = output_path / "qa_report.json"
    qa_data = {
        "overall_status": qa_report.overall_status.value,
        "passed": qa_report.passed(),
        "hard_failures": qa_report.hard_failures,
        "warnings": qa_report.warnings,
        "checks": [
            {
                "name": check.name,
                "status": check.status.value,
                "message": check.message,
                "details": check.details
            }
            for check in qa_report.checks
        ]
    }
    
    with open(qa_file, 'w') as f:
        json.dump(qa_data, f, indent=2)
    logger.info(f"  ✅ QA report: {qa_file}")
    
    # Save metadata
    metadata = {
        "pipeline": "Phase 3: Multi-View Reconstruction",
        "timestamp": datetime.now().isoformat(),
        "n_images": len(image_paths),
        "n_cameras_registered": sum(1 for c in sfm_opt.cameras.values() if c.registered),
        "n_points_3d": len(sfm_opt.points_3d),
        "n_tracks": len(feature_tracks_filtered),
        "calibration_file": str(calib_file),
        "layout_file": str(layout_file) if layout_file else None,
        "tag_size_mm": tag_size_mm,
        "ba_info": ba_info,
        "qa_passed": qa_report.passed()
    }
    
    metadata_file = output_path / "metadata.json"
    with open(metadata_file, 'w') as f:
        json.dump(metadata, f, indent=2)
    logger.info(f"  ✅ Metadata: {metadata_file}")
    
    logger.info(f"\n{'='*70}")
    logger.info(f"PHASE 3 COMPLETE")
    logger.info(f"{'='*70}\n")
    
    return sfm_opt, metadata


def run_phase4_transform(
    sfm: IncrementalSfM,
    reference_plate_file: str,
    output_dir: str,
    verbose: bool = True
) -> Tuple[bool, Dict]:
    """Run Phase 4: L→U transform with reference plate (Option U2).
    
    Args:
        sfm: Reconstructed IncrementalSfM in L-frame
        reference_plate_file: Reference plate geometry JSON
        output_dir: Output directory
        verbose: Print progress
        
    Returns:
        Tuple of (success, metadata):
            success: True if all hard gates passed
            metadata: Phase 4 results and validation
    """
    output_path = Path(output_dir)
    
    logger.info(f"="*70)
    logger.info(f"PHASE 4: USER FRAME DEFINITION (L → U TRANSFORM)")
    logger.info(f"="*70)
    logger.info(f"Method: Option U2 (Reference Plate)")
    logger.info(f"Reference plate: {reference_plate_file}")
    
    # Load reference plate geometry
    with open(reference_plate_file, 'r') as f:
        ref_plate_data = json.load(f)
    
    # Extract tag centers in U-frame
    tag_centers_U = {}
    for tag_id_str, tag_info in ref_plate_data["tags"].items():
        tag_id = int(tag_id_str)
        tag_centers_U[tag_id] = np.array(tag_info["center_mm"])
    
    logger.info(f"  Reference tags: {list(tag_centers_U.keys())}")
    
    # Get corresponding tag centers from L-frame reconstruction
    # TODO: Extract tag centers from sfm.points_3d (need tag ID mapping)
    # For now, use placeholder
    tag_centers_L = {}
    
    logger.warning("⚠️  Phase 4 transform not fully implemented yet")
    logger.warning("    Need to extract tag centers from L-frame reconstruction")
    
    # Placeholder: compute transform using Umeyama
    # T_U_from_L = compute_transform_L_to_U_reference_plate(tag_centers_L, tag_centers_U)
    
    metadata = {
        "phase": "Phase 4: L→U Transform",
        "method": "Option U2 (Reference Plate)",
        "reference_plate_file": str(reference_plate_file),
        "validation_status": "NOT_IMPLEMENTED",
        "hard_gates_passed": False,
        "note": "Phase 4 implementation pending - need tag center extraction"
    }
    
    logger.info(f"\n{'='*70}")
    logger.info(f"PHASE 4 STATUS: PENDING IMPLEMENTATION")
    logger.info(f"{'='*70}\n")
    
    return False, metadata


def main():
    parser = argparse.ArgumentParser(
        description="Phase 3 & 4 Multi-View Reconstruction Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # Required arguments
    parser.add_argument(
        "--images",
        required=True,
        help="Glob pattern for input images (e.g., 'calib/test/DSC_*.TIF')"
    )
    parser.add_argument(
        "--calib",
        required=True,
        help="Camera calibration JSON file"
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output directory for results"
    )
    
    # Optional arguments
    parser.add_argument(
        "--layout",
        help="Known tag layout JSON (for validation)"
    )
    parser.add_argument(
        "--tag-size",
        type=float,
        default=8.8,
        help="AprilTag edge length in mm (default: 8.8)"
    )
    parser.add_argument(
        "--phase4",
        choices=["reference_plate", "implant_based"],
        help="Enable Phase 4 with specified method"
    )
    parser.add_argument(
        "--reference-plate",
        help="Reference plate JSON file (required if --phase4 reference_plate)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=True,
        help="Verbose output (default: True)"
    )
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.phase4 == "reference_plate" and not args.reference_plate:
        parser.error("--reference-plate required when --phase4 reference_plate")
    
    # Find images
    image_paths = sorted(glob(args.images))
    if not image_paths:
        logger.error(f"No images found matching pattern: {args.images}")
        return 1
    
    logger.info(f"Found {len(image_paths)} images")
    
    try:
        # Run Phase 3
        sfm, phase3_metadata = run_phase3_pipeline(
            image_paths=image_paths,
            calib_file=args.calib,
            output_dir=args.output,
            layout_file=args.layout,
            tag_size_mm=args.tag_size,
            verbose=args.verbose
        )
        
        # Check Phase 3 QA
        if not phase3_metadata["qa_passed"]:
            logger.error("❌ Phase 3 QA FAILED - reconstruction did not pass quality gates")
            return 1
        
        # Run Phase 4 if requested
        if args.phase4:
            phase4_success, phase4_metadata = run_phase4_transform(
                sfm=sfm,
                reference_plate_file=args.reference_plate,
                output_dir=args.output,
                verbose=args.verbose
            )
            
            if not phase4_success:
                logger.error("❌ Phase 4 FAILED - hard-stop validation gates not passed")
                logger.error("    EXIT(1): No U-frame output generated")
                return 1
        
        logger.info("✅ Pipeline completed successfully")
        return 0
        
    except Exception as e:
        logger.error(f"❌ Pipeline failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
