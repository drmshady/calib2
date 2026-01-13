"""
Image quality filtering for reconstruction based on reprojection errors.

Provides tools to identify and remove low-quality images that contribute
high reprojection errors, improving overall reconstruction quality.

Functions:
    compute_per_image_errors: Calculate reprojection error statistics per image
    filter_worst_images: Identify worst N% of images by error
    remove_images_from_reconstruction: Remove specified images from SfM structure
"""

import numpy as np
from typing import Dict, List, Tuple, Set, Optional, Sequence
import logging

from incremental_sfm import IncrementalSfM, Camera, Point3D

logger = logging.getLogger(__name__)


def compute_per_image_errors(sfm: IncrementalSfM) -> Dict[str, Dict]:
    """Compute reprojection error statistics for each image.
    
    Args:
        sfm: IncrementalSfM reconstruction
        
    Returns:
        Dict mapping image_id to error statistics:
            {
                'mean_error_px': float,
                'max_error_px': float,
                'median_error_px': float,
                'n_observations': int,
                'errors': List[float]  # Individual errors
            }
    """
    image_errors = {}
    
    for img_id, camera in sfm.cameras.items():
        if not camera.registered:
            continue
        
        errors = []
        
        # Collect errors for all points visible in this image
        for point_id, point in sfm.points_3d.items():
            if img_id not in point.observations:
                continue
            
            # Project 3D point to image
            X_world = point.xyz
            X_cam = camera.R @ X_world + camera.t.ravel()
            
            if X_cam[2] <= 0:  # Behind camera
                continue
            
            # Project to pixel coordinates (undistorted)
            # Correct formula: K @ X_cam, then normalize by Z
            X_proj = camera.K @ X_cam
            pt_2d_proj = X_proj[:2] / X_proj[2]
            
            # Observed point
            pt_2d_obs = point.observations[img_id]
            
            # Compute error
            error = np.linalg.norm(pt_2d_proj - pt_2d_obs)
            errors.append(error)
        
        if len(errors) > 0:
            image_errors[img_id] = {
                'mean_error_px': float(np.mean(errors)),
                'max_error_px': float(np.max(errors)),
                'median_error_px': float(np.median(errors)),
                'n_observations': len(errors),
                'errors': errors
            }
    
    return image_errors


def filter_worst_images(
    image_errors: Dict[str, Dict],
    percentile: float = 10.0,
    criterion: str = 'mean'
) -> Tuple[List[str], List[str]]:
    """Identify worst images by reprojection error.
    
    Args:
        image_errors: Per-image error statistics from compute_per_image_errors()
        percentile: Percentage of worst images to filter (default 10%)
        criterion: Error metric to use ('mean', 'max', 'median')
        
    Returns:
        Tuple of (images_to_remove, images_to_keep):
            images_to_remove: List of image IDs with worst errors
            images_to_keep: List of image IDs with acceptable errors
    """
    if len(image_errors) == 0:
        return [], []
    
    # Sort images by error criterion
    metric_key = f'{criterion}_error_px'
    sorted_images = sorted(
        image_errors.items(),
        key=lambda x: x[1][metric_key],
        reverse=True  # Worst first
    )
    
    # Calculate how many to remove
    n_images = len(sorted_images)
    n_to_remove = max(1, int(np.ceil(n_images * percentile / 100.0)))
    
    # Don't remove too many (keep at least 50% or minimum 5 images)
    min_to_keep = max(5, int(n_images * 0.5))
    n_to_remove = min(n_to_remove, n_images - min_to_keep)
    
    if n_to_remove <= 0:
        return [], [img_id for img_id, _ in sorted_images]
    
    images_to_remove = [img_id for img_id, _ in sorted_images[:n_to_remove]]
    images_to_keep = [img_id for img_id, _ in sorted_images[n_to_remove:]]
    
    return images_to_remove, images_to_keep


def remove_images_from_reconstruction(
    sfm: IncrementalSfM,
    images_to_remove: List[str]
) -> Tuple[IncrementalSfM, Dict]:
    """Remove specified images from reconstruction.
    
    Removes cameras and their observations, but keeps 3D points that are
    still visible in remaining images. Points visible only in removed
    images are deleted.
    
    Args:
        sfm: IncrementalSfM reconstruction
        images_to_remove: List of image IDs to remove
        
    Returns:
        Tuple of (filtered_sfm, statistics):
            filtered_sfm: New IncrementalSfM with images removed
            statistics: Dict with removal statistics
    """
    from copy import deepcopy
    
    # Create new SfM structure
    sfm_filtered = IncrementalSfM(K=sfm.K, min_ray_angle_deg=sfm.min_ray_angle_deg)
    
    images_to_remove_set = set(images_to_remove)
    n_cameras_before = len([c for c in sfm.cameras.values() if c.registered])
    n_points_before = len(sfm.points_3d)
    
    # Copy cameras (excluding removed ones)
    for img_id, camera in sfm.cameras.items():
        if img_id not in images_to_remove_set:
            sfm_filtered.cameras[img_id] = deepcopy(camera)
    
    # Copy points and filter observations
    points_removed = []
    
    for point_id, point in sfm.points_3d.items():
        # Filter observations to keep only from remaining cameras
        filtered_obs = {
            img_id: obs for img_id, obs in point.observations.items()
            if img_id not in images_to_remove_set
        }
        
        # Keep point only if visible in at least 2 remaining images
        if len(filtered_obs) >= 2:
            point_filtered = Point3D(
                point_id=point.point_id,
                xyz=point.xyz.copy(),
                observations=filtered_obs,
                color=point.color,
                error=point.error
            )
            sfm_filtered.points_3d[point_id] = point_filtered
        else:
            points_removed.append(point_id)
    
    # Copy feature tracks (excluding removed images)
    for track_id, track in sfm.feature_tracks.items():
        filtered_track = [
            (img_id, feat_idx) for img_id, feat_idx in track
            if img_id not in images_to_remove_set
        ]
        if len(filtered_track) >= 2:
            sfm_filtered.feature_tracks[track_id] = filtered_track
    
    n_cameras_after = len([c for c in sfm_filtered.cameras.values() if c.registered])
    n_points_after = len(sfm_filtered.points_3d)
    
    statistics = {
        'n_images_removed': len(images_to_remove),
        'n_cameras_before': n_cameras_before,
        'n_cameras_after': n_cameras_after,
        'n_points_before': n_points_before,
        'n_points_after': n_points_after,
        'n_points_removed': len(points_removed),
        'removed_images': images_to_remove,
        'removed_points': points_removed
    }
    
    logger.info(f"  Removed {len(images_to_remove)} images:")
    for img_id in images_to_remove:
        logger.info(f"    - {img_id}")
    logger.info(f"  Cameras: {n_cameras_before} → {n_cameras_after}")
    logger.info(f"  3D points: {n_points_before} → {n_points_after}")
    
    return sfm_filtered, statistics


def apply_quality_gate_filter(
    sfm: IncrementalSfM,
    remove_image_ids: Optional[Sequence[str]] = None,
    percentile: float = 10.0,
    criterion: str = 'mean',
    verbose: bool = True
) -> Tuple[IncrementalSfM, Dict]:
    """Apply quality gate to remove worst images and improve reconstruction.
    
    This is a convenience function that:
    1. Computes per-image reprojection errors
    2. Identifies worst N% of images
    3. Removes them from reconstruction
    
    Args:
        sfm: IncrementalSfM reconstruction
        percentile: Percentage of worst images to remove (default 10%)
        criterion: Error metric to use ('mean', 'max', 'median')
        verbose: Enable logging
        
    Returns:
        Tuple of (filtered_sfm, report):
            filtered_sfm: New IncrementalSfM with worst images removed
            report: Dict with filtering statistics and before/after metrics
    """
    if verbose:
        if remove_image_ids is not None:
            logger.info(f"\nApplying quality gate filter (removing {len(list(remove_image_ids))} manually selected images)...")
        else:
            logger.info(f"\nApplying quality gate filter (removing worst {percentile}% by {criterion} error)...")
    
    # Step 1: Compute per-image errors
    image_errors = compute_per_image_errors(sfm)
    
    if len(image_errors) == 0:
        logger.warning("  No image errors computed - skipping filter")
        return sfm, {'status': 'skipped', 'reason': 'no_errors'}
    
    # Log before statistics
    errors_before = [stats['mean_error_px'] for stats in image_errors.values()]
    mean_before = np.mean(errors_before)
    max_before = np.max(errors_before)
    
    if verbose:
        logger.info(f"  Before filter: mean={mean_before:.3f}px, max={max_before:.3f}px ({len(image_errors)} images)")
    
    # Step 2: Choose images to remove
    if remove_image_ids is not None:
        # Preserve input order but drop unknown IDs
        remove_set = {str(x) for x in remove_image_ids}
        known_images = set(image_errors.keys())
        images_to_remove = [img_id for img_id in image_errors.keys() if img_id in remove_set]

        unknown = sorted(remove_set - known_images)
        if unknown and verbose:
            logger.warning(f"  Ignoring {len(unknown)} unknown image ids: {unknown[:5]}{'...' if len(unknown) > 5 else ''}")

        images_to_keep = [img_id for img_id in image_errors.keys() if img_id not in set(images_to_remove)]
    else:
        images_to_remove, images_to_keep = filter_worst_images(
            image_errors=image_errors,
            percentile=percentile,
            criterion=criterion
        )
    
    if len(images_to_remove) == 0:
        logger.info("  No images to remove (already minimal set)")
        return sfm, {
            'status': 'skipped',
            'reason': 'minimal_set' if remove_image_ids is None else 'empty_selection',
        }
    
    # Log selected images
    if verbose:
        header = "Worst" if remove_image_ids is None else "Selected"
        logger.info(f"\n  {header} {len(images_to_remove)} images:")
        for img_id in images_to_remove:
            stats = image_errors[img_id]
            logger.info(f"    {img_id}: mean={stats['mean_error_px']:.3f}px, "
                       f"max={stats['max_error_px']:.3f}px ({stats['n_observations']} obs)")
    
    # Step 3: Remove images
    sfm_filtered, filter_stats = remove_images_from_reconstruction(sfm, images_to_remove)
    
    # Compute after statistics
    image_errors_after = compute_per_image_errors(sfm_filtered)
    if len(image_errors_after) > 0:
        errors_after = [stats['mean_error_px'] for stats in image_errors_after.values()]
        mean_after = np.mean(errors_after)
        max_after = np.max(errors_after)
        
        if verbose:
            logger.info(f"\n  After filter: mean={mean_after:.3f}px, max={max_after:.3f}px ({len(image_errors_after)} images)")
            logger.info(f"  Improvement: Δmean={mean_before - mean_after:.3f}px, Δmax={max_before - max_after:.3f}px")
    else:
        mean_after = None
        max_after = None
    
    # Compile report
    report = {
        'status': 'success',
        'filter_config': {
            'mode': 'manual' if remove_image_ids is not None else 'percentile',
            'manual_n': len(images_to_remove) if remove_image_ids is not None else None,
            'percentile': percentile,
            'criterion': criterion
        },
        'before': {
            'n_images': len(image_errors),
            'mean_error_px': float(mean_before),
            'max_error_px': float(max_before)
        },
        'after': {
            'n_images': len(image_errors_after),
            'mean_error_px': float(mean_after) if mean_after is not None else None,
            'max_error_px': float(max_after) if max_after is not None else None
        },
        'removed': filter_stats
    }
    
    return sfm_filtered, report
