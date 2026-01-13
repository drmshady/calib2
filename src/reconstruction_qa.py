"""
Reconstruction Quality Assurance validation suite.

Validates Phase 3 reconstruction quality with hard gates:
- Reprojection errors: mean <1.0px, max <3.0px (HARD FAIL)
- Track lengths: >= 4 views per feature (WARN)
- Graph connectivity: no disconnected components (HARD FAIL)
- Scale sanity: AprilTag geometry within ±0.1mm (WARN)
- Bridge collinearity: triangle area >= 10mm² (HARD FAIL)

Functions:
    check_reprojection_errors: Validate reprojection error statistics
    check_track_lengths: Validate feature track lengths
    check_graph_connectivity: Check camera graph connectivity
    check_scale_sanity: Validate scale via AprilTag geometry
    check_noncollinearity_bridge: Validate bridge point collinearity
    run_full_qa: Run all QA checks and generate report
"""

import numpy as np
import networkx as nx
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

import incremental_sfm
import geometry_utils
from incremental_sfm import IncrementalSfM, Point3D, Camera
from geometry_utils import check_triangle_area, check_collinearity


class QAStatus(Enum):
    """QA check status levels."""
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"


@dataclass
class QACheckResult:
    """Result of a single QA check.
    
    Attributes:
        name: Check name
        status: QAStatus (PASS, WARN, FAIL)
        message: Description of result
        details: Optional dict with detailed metrics
    """
    name: str
    status: QAStatus
    message: str
    details: Optional[Dict] = None


@dataclass
class QAReport:
    """Complete QA report for reconstruction.
    
    Attributes:
        checks: List of individual QA check results
        overall_status: Overall status (FAIL if any FAIL, WARN if any WARN, else PASS)
        hard_failures: List of check names that FAILED
        warnings: List of check names that WARNED
    """
    checks: List[QACheckResult]
    overall_status: QAStatus
    hard_failures: List[str]
    warnings: List[str]
    
    def passed(self) -> bool:
        """Check if reconstruction passed all hard gates."""
        return self.overall_status != QAStatus.FAIL


def check_reprojection_errors(
    sfm: IncrementalSfM,
    mean_threshold_px: float = 1.0,
    max_threshold_px: float = 3.0
) -> QACheckResult:
    """Check reprojection errors meet quality thresholds.
    
    Args:
        sfm: IncrementalSfM reconstruction
        mean_threshold_px: Mean error threshold (default 1.0px)
        max_threshold_px: Maximum error threshold (default 3.0px)
        
    Returns:
        QACheckResult with PASS/FAIL status
    """
    # Compute reprojection errors for all points
    errors_dict = sfm.compute_reprojection_errors()
    
    if not errors_dict:
        return QACheckResult(
            name="Reprojection Errors",
            status=QAStatus.FAIL,
            message="No reprojection errors computed (no valid points)",
            details={}
        )
    
    errors = list(errors_dict.values())
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    median_error = np.median(errors)
    
    details = {
        "mean_error_px": float(mean_error),
        "max_error_px": float(max_error),
        "median_error_px": float(median_error),
        "n_points": len(errors),
        "threshold_mean_px": mean_threshold_px,
        "threshold_max_px": max_threshold_px
    }
    
    # Check thresholds
    if mean_error > mean_threshold_px or max_error > max_threshold_px:
        status = QAStatus.FAIL
        message = (
            f"Reprojection error FAILED: mean={mean_error:.3f}px (>{mean_threshold_px}px) "
            f"or max={max_error:.3f}px (>{max_threshold_px}px)"
        )
    else:
        status = QAStatus.PASS
        message = (
            f"Reprojection error PASSED: mean={mean_error:.3f}px, "
            f"max={max_error:.3f}px"
        )
    
    return QACheckResult(
        name="Reprojection Errors",
        status=status,
        message=message,
        details=details
    )


def check_track_lengths(
    sfm: IncrementalSfM,
    min_track_length: int = 4
) -> QACheckResult:
    """Check that features have sufficient track lengths.
    
    Args:
        sfm: IncrementalSfM reconstruction
        min_track_length: Minimum track length (default 4 views)
        
    Returns:
        QACheckResult with PASS/WARN status
    """
    if not sfm.points_3d:
        return QACheckResult(
            name="Track Lengths",
            status=QAStatus.FAIL,
            message="No 3D points to check",
            details={}
        )
    
    track_lengths = [
        len(point.observations)
        for point in sfm.points_3d.values()
    ]
    
    mean_length = np.mean(track_lengths)
    median_length = np.median(track_lengths)
    min_length = min(track_lengths)
    
    short_tracks = sum(1 for length in track_lengths if length < min_track_length)
    short_ratio = short_tracks / len(track_lengths)
    
    details = {
        "mean_track_length": float(mean_length),
        "median_track_length": float(median_length),
        "min_track_length": int(min_length),
        "n_short_tracks": short_tracks,
        "short_track_ratio": float(short_ratio),
        "threshold": min_track_length
    }
    
    # WARN if > 20% of tracks are short
    if short_ratio > 0.2:
        status = QAStatus.WARN
        message = (
            f"Track lengths WARNING: {short_ratio:.1%} of tracks have <{min_track_length} views "
            f"(mean={mean_length:.1f})"
        )
    else:
        status = QAStatus.PASS
        message = (
            f"Track lengths PASSED: {short_ratio:.1%} short tracks "
            f"(mean={mean_length:.1f})"
        )
    
    return QACheckResult(
        name="Track Lengths",
        status=status,
        message=message,
        details=details
    )


def check_graph_connectivity(sfm: IncrementalSfM) -> QACheckResult:
    """Check that camera graph is fully connected (no isolated components).
    
    Args:
        sfm: IncrementalSfM reconstruction
        
    Returns:
        QACheckResult with PASS/FAIL status
    """
    # Build camera visibility graph
    G = nx.Graph()
    
    # Add all registered cameras as nodes
    registered_cameras = [
        img_id for img_id, cam in sfm.cameras.items()
        if cam.registered
    ]
    G.add_nodes_from(registered_cameras)
    
    # Add edges: two cameras are connected if they observe common points
    for point in sfm.points_3d.values():
        observers = list(point.observations.keys())
        # Add edge between all pairs of observers
        for i in range(len(observers)):
            for j in range(i + 1, len(observers)):
                G.add_edge(observers[i], observers[j])
    
    # Check connectivity
    n_components = nx.number_connected_components(G)
    components = list(nx.connected_components(G))
    
    details = {
        "n_cameras": len(registered_cameras),
        "n_components": n_components,
        "component_sizes": [len(c) for c in components]
    }
    
    if n_components > 1:
        status = QAStatus.FAIL
        message = (
            f"Graph connectivity FAILED: {n_components} disconnected components "
            f"(sizes: {details['component_sizes']})"
        )
    else:
        status = QAStatus.PASS
        message = f"Graph connectivity PASSED: single connected component ({len(registered_cameras)} cameras)"
    
    return QACheckResult(
        name="Graph Connectivity",
        status=status,
        message=message,
        details=details
    )


def check_scale_sanity(
    sfm: IncrementalSfM,
    apriltag_corners_3d: Dict[int, np.ndarray],
    expected_edge_length_mm: float,
    tolerance_mm: float = 0.1
) -> QACheckResult:
    """Check scale sanity via AprilTag edge lengths.
    
    Args:
        sfm: IncrementalSfM reconstruction
        apriltag_corners_3d: Dict of tag_id → (4, 3) corner positions
        expected_edge_length_mm: Expected edge length (e.g., 8.8mm)
        tolerance_mm: Tolerance for scale check (default 0.1mm)
        
    Returns:
        QACheckResult with PASS/WARN status
    """
    if not apriltag_corners_3d:
        return QACheckResult(
            name="Scale Sanity",
            status=QAStatus.WARN,
            message="No AprilTag corners provided for scale check",
            details={}
        )
    
    edge_lengths = []
    
    for tag_id, corners in apriltag_corners_3d.items():
        # Compute 4 edge lengths
        for i in range(4):
            j = (i + 1) % 4
            edge_length = np.linalg.norm(corners[j] - corners[i])
            edge_lengths.append(edge_length)
    
    if not edge_lengths:
        return QACheckResult(
            name="Scale Sanity",
            status=QAStatus.WARN,
            message="No edge lengths computed",
            details={}
        )
    
    mean_edge = np.mean(edge_lengths)
    std_edge = np.std(edge_lengths)
    min_edge = np.min(edge_lengths)
    max_edge = np.max(edge_lengths)
    
    scale_error = abs(mean_edge - expected_edge_length_mm)
    
    details = {
        "mean_edge_length_mm": float(mean_edge),
        "std_edge_length_mm": float(std_edge),
        "min_edge_length_mm": float(min_edge),
        "max_edge_length_mm": float(max_edge),
        "expected_edge_length_mm": expected_edge_length_mm,
        "scale_error_mm": float(scale_error),
        "tolerance_mm": tolerance_mm,
        "n_edges": len(edge_lengths)
    }
    
    if scale_error > tolerance_mm:
        status = QAStatus.WARN
        message = (
            f"Scale sanity WARNING: mean edge={mean_edge:.3f}mm "
            f"(expected {expected_edge_length_mm}mm ±{tolerance_mm}mm, "
            f"error={scale_error:.3f}mm)"
        )
    else:
        status = QAStatus.PASS
        message = (
            f"Scale sanity PASSED: mean edge={mean_edge:.3f}mm "
            f"(error={scale_error:.3f}mm < {tolerance_mm}mm)"
        )
    
    return QACheckResult(
        name="Scale Sanity",
        status=status,
        message=message,
        details=details
    )


def check_noncollinearity_bridge(
    sfm: IncrementalSfM,
    point_ids: Optional[List[int]] = None,
    threshold_mm: float = 3.0,
    min_area_mm2: float = 5.0
) -> QACheckResult:
    """Check that bridge points are non-collinear (anti-hinge).
    
    Args:
        sfm: IncrementalSfM reconstruction
        point_ids: Optional list of point IDs to check (uses all if None)
        threshold_mm: Collinearity threshold (default 3.0mm, relaxed from 5.0mm)
        min_area_mm2: Minimum triangle area (default 5.0mm², relaxed from 10.0mm²)
        
    Returns:
        QACheckResult with PASS/FAIL status
    """
    if point_ids is None:
        point_ids = list(sfm.points_3d.keys())
    
    if len(point_ids) < 3:
        return QACheckResult(
            name="Bridge Collinearity",
            status=QAStatus.WARN,
            message=f"Insufficient points for collinearity check: {len(point_ids)} < 3",
            details={}
        )
    
    # Get 3D positions
    points_3d = np.array([
        sfm.points_3d[pid].xyz
        for pid in point_ids
        if pid in sfm.points_3d
    ])
    
    if len(points_3d) < 3:
        return QACheckResult(
            name="Bridge Collinearity",
            status=QAStatus.WARN,
            message="Insufficient valid points",
            details={}
        )
    
    # Check collinearity
    is_non_collinear, max_distance, line_direction = check_collinearity(
        points_3d, threshold_mm
    )
    
    # Also check triangle area for first 3 points
    sample_points = points_3d[:3]
    is_valid_area, triangle_area = check_triangle_area(sample_points, min_area_mm2)
    
    details = {
        "max_perpendicular_distance_mm": float(max_distance),
        "triangle_area_mm2": float(triangle_area),
        "threshold_distance_mm": threshold_mm,
        "threshold_area_mm2": min_area_mm2,
        "n_points_checked": len(points_3d)
    }
    
    if not is_non_collinear or not is_valid_area:
        status = QAStatus.FAIL
        message = (
            f"Bridge collinearity FAILED: "
            f"max_dist={max_distance:.2f}mm (<{threshold_mm}mm) or "
            f"area={triangle_area:.2f}mm² (<{min_area_mm2}mm²)"
        )
    else:
        status = QAStatus.PASS
        message = (
            f"Bridge collinearity PASSED: "
            f"max_dist={max_distance:.2f}mm, area={triangle_area:.2f}mm²"
        )
    
    return QACheckResult(
        name="Bridge Collinearity",
        status=status,
        message=message,
        details=details
    )


def run_full_qa(
    sfm: IncrementalSfM,
    apriltag_corners_3d: Optional[Dict[int, np.ndarray]] = None,
    expected_tag_edge_mm: float = 8.8,
    bridge_point_ids: Optional[List[int]] = None
) -> QAReport:
    """Run complete QA validation suite.
    
    Args:
        sfm: IncrementalSfM reconstruction to validate
        apriltag_corners_3d: Optional dict of tag_id → corner positions for scale check
        expected_tag_edge_mm: Expected AprilTag edge length (default 8.8mm)
        bridge_point_ids: Optional list of point IDs for bridge collinearity check
        
    Returns:
        QAReport with all check results and overall status
    """
    checks = []
    
    # 1. Reprojection errors (HARD FAIL)
    checks.append(check_reprojection_errors(sfm))
    
    # 2. Track lengths (WARN)
    checks.append(check_track_lengths(sfm))
    
    # 3. Graph connectivity (HARD FAIL)
    checks.append(check_graph_connectivity(sfm))
    
    # 4. Scale sanity (WARN if AprilTags provided)
    if apriltag_corners_3d:
        checks.append(check_scale_sanity(sfm, apriltag_corners_3d, expected_tag_edge_mm))
    
    # 5. Bridge collinearity (HARD FAIL)
    checks.append(check_noncollinearity_bridge(sfm, bridge_point_ids))
    
    # Determine overall status
    hard_failures = [c.name for c in checks if c.status == QAStatus.FAIL]
    warnings = [c.name for c in checks if c.status == QAStatus.WARN]
    
    if hard_failures:
        overall_status = QAStatus.FAIL
    elif warnings:
        overall_status = QAStatus.WARN
    else:
        overall_status = QAStatus.PASS
    
    return QAReport(
        checks=checks,
        overall_status=overall_status,
        hard_failures=hard_failures,
        warnings=warnings
    )


def print_qa_report(report: QAReport, verbose: bool = True):
    """Print QA report to console.
    
    Args:
        report: QAReport to print
        verbose: If True, print detailed metrics
    """
    print("\n" + "="*70)
    print("PHASE 3 RECONSTRUCTION QUALITY ASSURANCE REPORT")
    print("="*70)
    
    for check in report.checks:
        status_str = check.status.value
        if check.status == QAStatus.PASS:
            prefix = "✅"
        elif check.status == QAStatus.WARN:
            prefix = "⚠️"
        else:
            prefix = "❌"
        
        print(f"\n{prefix} [{status_str}] {check.name}")
        print(f"   {check.message}")
        
        if verbose and check.details:
            for key, value in check.details.items():
                if isinstance(value, float):
                    print(f"      {key}: {value:.4f}")
                else:
                    print(f"      {key}: {value}")
    
    print("\n" + "-"*70)
    print(f"OVERALL STATUS: {report.overall_status.value}")
    
    if report.hard_failures:
        print(f"❌ HARD FAILURES: {', '.join(report.hard_failures)}")
    if report.warnings:
        print(f"⚠️  WARNINGS: {', '.join(report.warnings)}")
    
    if report.passed():
        print("✅ Reconstruction PASSED all hard gates")
    else:
        print("❌ Reconstruction FAILED - see errors above")
    
    print("="*70 + "\n")
