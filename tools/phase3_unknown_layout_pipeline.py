#!/usr/bin/env python3
"""
Phase 3 Multi-View Reconstruction Pipeline - UNKNOWN LAYOUT.

For 4 flags without known layout. Triangulates 3D positions from observations.

Usage:
    python phase3_unknown_layout_pipeline.py \\
        --images calib/test2/DSC_*.TIF \\
        --calib calib/test2/camera_intrinsics.json \\
        --output runs/test2_unknown_layout \\
        --tag-size 8.8

Process:
    1. Detect AprilTags (front face corners)
    2. PnP initialization with metric scale (one tag as world origin)
    3. Incremental registration + triangulation
    4. Global bundle adjustment
    5. Quality gates validation
    
Quality Gates:
    - Mean reprojection <1.0px (HARD FAIL)
    - Max reprojection <3.0px (HARD FAIL)
    - Track length >=4 views (WARN)
    - Scale sanity: AprilTag edges within ±0.1mm (WARN)
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
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

import calibration_loader
import incremental_sfm
import bundle_adjustment
import reconstruction_qa

from calibration_loader import load_calibration, undistort_points
from incremental_sfm import IncrementalSfM, Camera, Point3D
from bundle_adjustment import bundle_adjust_global
from reconstruction_qa import run_full_qa, print_qa_report, QAStatus

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s'
)
logger = logging.getLogger(__name__)


def detect_apriltags(image_path: str, tag_family: str = "tag36h11") -> List[Dict]:
    """Detect AprilTags in an image using OpenCV ArUco detector.
    
    Args:
        image_path: Path to image file
        tag_family: AprilTag family (default: tag36h11)
        
    Returns:
        List of detection dicts with tag_id, corners, center
    """
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        logger.error(f"Failed to load image: {image_path}")
        return []
    
    # AprilTag36h11 corresponds to DICT_APRILTAG_36h11
    if tag_family == "tag36h11":
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    else:
        logger.warning(f"Unsupported tag family: {tag_family}, using 36h11")
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    
    corners, ids, rejected = detector.detectMarkers(image)
    
    results = []
    if ids is not None:
        for i, tag_id in enumerate(ids.ravel()):
            corner_array = corners[i][0]  # Shape: (4, 2)
            center = np.mean(corner_array, axis=0)
            
            results.append({
                "tag_id": int(tag_id),
                "corners": corner_array,  # (4, 2) distorted pixels: TL, TR, BR, BL
                "center": center
            })
    
    return results


def build_feature_tracks(detections_all: Dict[str, List[Dict]]) -> Dict[int, List[Tuple[str, int]]]:
    """Build feature tracks from tag corner detections.
    
    Args:
        detections_all: Dict mapping image_id → list of detections
        
    Returns:
        Dict mapping track_id → list of (image_id, feature_idx) observations
        Track ID = tag_id * 10 + corner_idx (0-3)
        feature_idx is the index into the flattened corner array for that image
    """
    tracks = {}
    
    for img_id, detections in detections_all.items():
        feature_idx = 0
        for det in detections:
            tag_id = det["tag_id"]
            # Each tag has 4 corners
            for corner_idx in range(4):
                track_id = tag_id * 10 + corner_idx
                if track_id not in tracks:
                    tracks[track_id] = []
                tracks[track_id].append((img_id, feature_idx))
                feature_idx += 1
    
    return tracks


def get_tag_edge_length_from_corners(corners_3d: np.ndarray) -> float:
    """Compute mean edge length from 4 corners.
    
    Args:
        corners_3d: (4, 3) corner positions
        
    Returns:
        Mean edge length (mm)
    """
    edges = [
        np.linalg.norm(corners_3d[1] - corners_3d[0]),  # TL→TR
        np.linalg.norm(corners_3d[2] - corners_3d[1]),  # TR→BR
        np.linalg.norm(corners_3d[3] - corners_3d[2]),  # BR→BL
        np.linalg.norm(corners_3d[0] - corners_3d[3]),  # BL→TL
    ]
    return np.mean(edges)


def run_unknown_layout_pipeline(
    image_paths: List[str],
    calib_file: str,
    output_dir: str,
    tag_size_mm: float = 8.8,
    verbose: bool = True
) -> Tuple[IncrementalSfM, Dict]:
    """Run Phase 3 reconstruction with UNKNOWN layout.
    
    Args:
        image_paths: List of image file paths
        calib_file: Camera calibration JSON
        output_dir: Output directory
        tag_size_mm: Known AprilTag edge length (mm)
        verbose: Print progress
        
    Returns:
        Tuple of (sfm, metadata)
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    logger.info("="*70)
    logger.info("PHASE 3: UNKNOWN LAYOUT RECONSTRUCTION")
    logger.info("="*70)
    logger.info(f"Images: {len(image_paths)}")
    logger.info(f"Tag size: {tag_size_mm} mm")
    logger.info(f"Output: {output_dir}")
    
    # Step 1: Load calibration
    logger.info(f"\n[1/5] Loading calibration...")
    K, D, image_size, calib_metadata = load_calibration(calib_file)
    logger.info(f"  K: fx={K[0,0]:.1f}, fy={K[1,1]:.1f}")
    logger.info(f"  Image size: {image_size}")
    
    # Step 2: Detect AprilTags
    logger.info(f"\n[2/5] Detecting AprilTags...")
    detections_all = {}
    tag_ids_seen = set()
    
    for img_path in image_paths:
        img_id = Path(img_path).stem
        detections = detect_apriltags(img_path)
        
        if detections:
            detections_all[img_id] = detections
            for det in detections:
                tag_ids_seen.add(det["tag_id"])
            logger.info(f"  {img_id}: {len(detections)} tags")
        else:
            logger.warning(f"  {img_id}: No tags")
    
    logger.info(f"  Total unique tags: {sorted(tag_ids_seen)}")
    
    if len(detections_all) < 2:
        raise ValueError(f"Need >=2 images with detections, got {len(detections_all)}")
    
    # Step 3: Build feature tracks
    logger.info(f"\n[3/5] Building feature tracks...")
    feature_tracks = build_feature_tracks(detections_all)
    tracks_filtered = {
        tid: track for tid, track in feature_tracks.items()
        if len(track) >= 2
    }
    logger.info(f"  Tracks with >=2 views: {len(tracks_filtered)}")
    
    # Step 4: PnP initialization with metric scale
    logger.info(f"\n[4/5] PnP initialization (metric scale)...")
    
    # Strategy: Use one tag as "world origin" to establish metric coordinate system
    # Select image with most tags for robust initialization
    image_ids = sorted(detections_all.keys())
    best_init_img = None
    max_tags = 0
    origin_tag_id = None
    
    for img_id in image_ids:
        n_tags = len(detections_all[img_id])
        if n_tags > max_tags:
            max_tags = n_tags
            best_init_img = img_id
            # Use lowest tag ID as origin (e.g., tag 100)
            origin_tag_id = min(det["tag_id"] for det in detections_all[img_id])
    
    if best_init_img is None or max_tags == 0:
        raise ValueError("No suitable initialization image found")
    
    logger.info(f"  Init image: {best_init_img} ({max_tags} tags)")
    logger.info(f"  World origin: Tag {origin_tag_id} (center at 0,0,0, Z=0 plane)")
    
    # Define 3D coordinates for a tag in its local frame
    # Tag coordinate system: center at origin, corners at ±half_edge
    # Corner order: TL(0), TR(1), BR(2), BL(3) in CCW from top-left
    half_edge = tag_size_mm / 2.0
    tag_corners_3d_local = np.array([
        [-half_edge, +half_edge, 0.0],  # TL (top-left)
        [+half_edge, +half_edge, 0.0],  # TR (top-right)
        [+half_edge, -half_edge, 0.0],  # BR (bottom-right)
        [-half_edge, -half_edge, 0.0],  # BL (bottom-left)
    ], dtype=np.float64)
    
    # STRATEGY: Use PnP for ALL visible tags to get their 3D positions
    # This provides non-coplanar initial structure (16 points across multiple tags)
    
    # Step 1: Get camera pose from origin tag
    origin_det = None
    for det in detections_all[best_init_img]:
        if det["tag_id"] == origin_tag_id:
            origin_det = det
            break
    
    if origin_det is None:
        raise ValueError(f"Origin tag {origin_tag_id} not found in init image")
    
    # PnP to solve camera pose relative to origin tag
    corners_2d_origin = origin_det["corners"]  # (4, 2)
    
    success, rvec_cam, tvec_cam = cv2.solvePnP(
        tag_corners_3d_local,  # (4, 3) object points in tag frame
        corners_2d_origin,  # (4, 2) image points
        K, D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if not success:
        raise ValueError(f"PnP failed for origin tag")
    
    R_cam_to_origin, _ = cv2.Rodrigues(rvec_cam)
    t_cam_to_origin = tvec_cam.reshape(3, 1)
    
    # Camera pose in world frame (where origin tag is at identity)
    R1 = R_cam_to_origin
    t1 = t_cam_to_origin
    
    logger.info(f"  [OK] Camera pose from origin tag:")
    logger.info(f"     Position: [{t1[0,0]:.2f}, {t1[1,0]:.2f}, {t1[2,0]:.2f}] mm")
    logger.info(f"     Distance: {np.linalg.norm(t1):.2f} mm")
    
    # Step 2: For each OTHER tag, compute its pose in world frame
    # World frame = origin tag frame (tag 100 at identity)
    tag_poses_world = {origin_tag_id: (np.eye(3), np.zeros((3, 1)))}  # Origin at identity
    
    # Compute camera center in world frame for reference
    C_cam_world = -R_cam_to_origin.T @ t_cam_to_origin
    
    for det in detections_all[best_init_img]:
        tag_id = det["tag_id"]
        if tag_id == origin_tag_id:
            continue
        
        # PnP to get this tag's pose relative to camera
        corners_2d = det["corners"]
        success, rvec_tag, tvec_tag = cv2.solvePnP(
            tag_corners_3d_local,
            corners_2d,
            K, D,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success:
            logger.warning(f"     Skipping tag {tag_id}: PnP failed")
            continue
        
        R_tag_to_cam, _ = cv2.Rodrigues(rvec_tag)
        t_tag_to_cam = tvec_tag.reshape(3, 1)
        
        # Chain transformations: tag -> camera -> world
        # Point in tag frame: p_tag
        # Point in camera frame: p_cam = R_tag_to_cam @ p_tag + t_tag_to_cam
        # Point in world frame: p_world = R_cam_to_origin.T @ (p_cam - t_cam_to_origin)
        #                              = R_cam_to_origin.T @ (R_tag_to_cam @ p_tag + t_tag_to_cam - t_cam_to_origin)
        #                              = R_cam_to_origin.T @ R_tag_to_cam @ p_tag + R_cam_to_origin.T @ (t_tag_to_cam - t_cam_to_origin)
        # So: R_tag_to_world = R_cam_to_origin.T @ R_tag_to_cam
        #     t_tag_to_world = R_cam_to_origin.T @ (t_tag_to_cam - t_cam_to_origin)
        
        R_tag_world = R_cam_to_origin.T @ R_tag_to_cam
        t_tag_world = R_cam_to_origin.T @ (t_tag_to_cam - t_cam_to_origin)
        
        tag_poses_world[tag_id] = (R_tag_world, t_tag_world)
        
        logger.info(f"     Tag {tag_id} position: [{t_tag_world[0,0]:.2f}, {t_tag_world[1,0]:.2f}, {t_tag_world[2,0]:.2f}] mm")
    
    logger.info(f"  Initialized {len(tag_poses_world)} tags in world frame")
    
    # Step 5: Initialize IncrementalSfM with first camera and ALL visible tags
    logger.info(f"\n[5/5] Incremental registration + bundle adjustment...")
    
    sfm = IncrementalSfM(K=K, min_ray_angle_deg=5.0)
    
    # Add first camera
    sfm.cameras[best_init_img] = Camera(
        image_id=best_init_img,
        R=R1,
        t=t1,
        K=K,
        registered=True
    )
    
    # Build all 2D observations (undistorted) for triangulation
    all_points_2d = {}
    for img_id, detections in detections_all.items():
        points_list = []
        for det in detections:
            for corner_idx in range(4):
                points_list.append(det["corners"][corner_idx])
        
        if points_list:
            points_distorted = np.array(points_list)
            points_undist = cv2.undistortPoints(
                points_distorted.reshape(-1, 1, 2), K, D, P=K
            ).reshape(-1, 2)
            all_points_2d[img_id] = points_undist
    
    # Add ALL tag corners as initial 3D points
    # Transform corners from each tag's local frame to world frame
    n_points_added = 0
    for tag_id, (R_tag, t_tag) in tag_poses_world.items():
        # Find feature indices for this tag in best_init_img
        tag_feature_start = None
        feature_idx = 0
        for det in detections_all[best_init_img]:
            if det["tag_id"] == tag_id:
                tag_feature_start = feature_idx
                break
            feature_idx += 4
        
        if tag_feature_start is None:
            continue
        
        # Transform tag corners to world frame
        for corner_idx in range(4):
            track_id = tag_id * 10 + corner_idx
            
            # Corner in tag's local frame
            pt_tag_local = tag_corners_3d_local[corner_idx]
            
            # Transform to world frame: pt_world = R_tag @ pt_local + t_tag
            pt_world = R_tag @ pt_tag_local + t_tag.ravel()
            
            # Add to SfM structure
            feat_idx = tag_feature_start + corner_idx
            sfm.points_3d[track_id] = Point3D(
                point_id=track_id,
                xyz=pt_world,
                observations={
                    best_init_img: all_points_2d[best_init_img][feat_idx]
                }
            )
            n_points_added += 1
    
    logger.info(f"  Initial structure: {n_points_added} points ({len(tag_poses_world)} tags), 1 camera")
    
    # Verify initial structure by reprojecting
    reproj_errors = []
    for point_id, point in sfm.points_3d.items():
        for img_id, pt_2d in point.observations.items():
            cam = sfm.cameras[img_id]
            X_cam = cam.R @ point.xyz + cam.t.ravel()
            X_proj = cam.K @ X_cam
            pt_proj = X_proj[:2] / X_proj[2]
            error = np.linalg.norm(pt_2d - pt_proj)
            reproj_errors.append(error)
    
    mean_reproj_init = np.mean(reproj_errors) if reproj_errors else 0.0
    max_reproj_init = np.max(reproj_errors) if reproj_errors else 0.0
    logger.info(f"  Verification: mean reproj {mean_reproj_init:.3f}px, max {max_reproj_init:.3f}px")
    
    # Incremental registration with triangulation
    logger.info(f"  Starting incremental registration...")
    
    image_ids = sorted(detections_all.keys())
    remaining_images = [iid for iid in image_ids if iid != best_init_img]
    
    for img_id in remaining_images:
        # Build correspondences for PnP
        points_2d_undist_list = []
        correspondences = []
        
        point_idx = 0
        for det in detections_all[img_id]:
            tag_id = det["tag_id"]
            for corner_idx in range(4):
                track_id = tag_id * 10 + corner_idx
                if track_id in sfm.points_3d:
                    correspondences.append((track_id, point_idx))
                point_idx += 1
        
        if len(correspondences) < 4:
            logger.warning(f"  [SKIP] {img_id}: Only {len(correspondences)} correspondences (need >=4)")
            continue
        
        # Register camera with PnP
        success, error_msg = sfm.register_camera(img_id, all_points_2d[img_id], correspondences)
        
        if not success:
            logger.warning(f"  [SKIP] {img_id}: {error_msg} ({len(correspondences)} corresp)")
            continue
        
        # Triangulate new points now that we have more cameras
        new_points = sfm.triangulate_new_points(tracks_filtered, all_points_2d)
        
        logger.info(f"  [OK] {img_id}: {len(correspondences)} corresp → +{new_points} pts (total {len(sfm.points_3d)})")
    
    logger.info(f"  Final: {len(sfm.points_3d)} points, {len(sfm.cameras)} cameras")
    
    # Bundle adjustment
    logger.info(f"  Starting bundle adjustment...")
    
    sfm_optimized, ba_result = bundle_adjust_global(sfm, fix_first_camera=True, verbose=False)
    
    if ba_result["success"]:
        # Compute reprojection errors after BA
        all_errors = []
        max_error_info = None
        max_error_val = 0.0
        
        for point in sfm_optimized.points_3d.values():
            for img_id, pt_2d in point.observations.items():
                if img_id in sfm_optimized.cameras:
                    cam = sfm_optimized.cameras[img_id]
                    X_cam = cam.R @ point.xyz + cam.t.ravel()
                    X_proj = cam.K @ X_cam
                    pt_proj = X_proj[:2] / X_proj[2]
                    error = np.linalg.norm(pt_2d - pt_proj)
                    all_errors.append(error)
                    
                    if error > max_error_val:
                        max_error_val = error
                        max_error_info = (point.point_id, img_id, pt_2d, pt_proj, error)
        
        mean_error = np.mean(all_errors) if all_errors else 0.0
        max_error = np.max(all_errors) if all_errors else 0.0
        
        logger.info(f"  [OK] BA converged")
        logger.info(f"     Mean reprojection: {mean_error:.3f}px")
        logger.info(f"     Max reprojection: {max_error:.3f}px")
        
        if max_error_info:
            pid, iid, obs, proj, err = max_error_info
            logger.info(f"     Max error point: {pid} in {iid}")
            logger.info(f"       Observed: [{obs[0]:.2f}, {obs[1]:.2f}]")
            logger.info(f"       Projected: [{proj[0]:.2f}, {proj[1]:.2f}]")
        
        # Add to ba_result for later use
        ba_result["mean_error_px"] = mean_error
        ba_result["max_error_px"] = max_error
    else:
        logger.warning(f"  [WARN] BA did not fully converge")
    
    # Quality gates
    logger.info(f"\n[QA] Quality validation...")
    
    qa_result = run_full_qa(
        sfm_optimized,
        expected_tag_edge_mm=tag_size_mm
    )
    
    # Don't print QA report to avoid Unicode issues in Windows console
    # print_qa_report(qa_result)
    
    qa_passed = qa_result.passed()
    logger.info(f"  QA Status: {qa_result.overall_status.value}")
    logger.info(f"  Passed: {qa_passed}")
    if qa_result.hard_failures:
        logger.info(f"  Hard failures: {', '.join(qa_result.hard_failures)}")
    if qa_result.warnings:
        logger.info(f"  Warnings: {', '.join(qa_result.warnings)}")
    
    # Save outputs
    logger.info(f"\n[SAVE] Writing outputs to {output_dir}...")
    
    sfm_optimized.export_to_json(str(output_path / "refpoints_L.json"))
    
    # Also save as structure_L.json for compatibility
    sfm_optimized.export_to_json(str(output_path / "structure_L.json"))
    
    with open(output_path / "qa_report.json", "w") as f:
        qa_dict = {
            "overall_status": qa_result.overall_status.value,
            "passed": qa_result.passed(),
            "hard_failures": qa_result.hard_failures,
            "warnings": qa_result.warnings,
            "checks": [
                {
                    "name": check.name,
                    "status": check.status.value,
                    "message": check.message,
                    "details": check.details
                }
                for check in qa_result.checks
            ]
        }
        json.dump(qa_dict, f, indent=2, default=str)
    
    metadata = {
        "timestamp": datetime.now().isoformat(),
        "images_total": len(image_paths),
        "images_registered": len(sfm.cameras),
        "points_3d": len(sfm.points_3d),
        "tag_size_mm": tag_size_mm,
        "qa_passed": qa_passed,
        "ba_result": ba_result
    }
    
    with open(output_path / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    
    logger.info(f"✅ Pipeline complete")
    
    return sfm_optimized, metadata


def main():
    parser = argparse.ArgumentParser(description="Phase 3 Unknown Layout Pipeline")
    parser.add_argument("--images", required=True, help="Image glob pattern")
    parser.add_argument("--calib", required=True, help="Calibration JSON")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--tag-size", type=float, default=8.8, help="Tag edge length (mm)")
    parser.add_argument("--verbose", action="store_true", default=True)
    
    args = parser.parse_args()
    
    image_paths = sorted(glob(args.images))
    if not image_paths:
        logger.error(f"No images found: {args.images}")
        return 1
    
    logger.info(f"Found {len(image_paths)} images")
    
    try:
        sfm, metadata = run_unknown_layout_pipeline(
            image_paths=image_paths,
            calib_file=args.calib,
            output_dir=args.output,
            tag_size_mm=args.tag_size,
            verbose=args.verbose
        )
        
        if not metadata["qa_passed"]:
            logger.error("❌ QA FAILED")
            return 1
        
        logger.info("✅ SUCCESS")
        return 0
        
    except Exception as e:
        logger.error(f"❌ Pipeline failed: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
