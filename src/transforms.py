"""
Transform classes for coordinate frame conversions.

Implements SE(3) rigid transforms and Sim(3) similarity transforms for
dental photogrammetry pipeline. All coordinates are in millimeters (mm).

Classes:
    SE3Transform: Rigid transform (rotation + translation)
    Sim3Transform: Similarity transform (scale + rotation + translation)

Conventions:
    - Column vectors: p_dst = T @ p_src
    - 4x4 homogeneous matrices
    - OpenCV camera convention: p_cam = R @ p_world + t
    - Frame notation: T_dst_from_src
"""

import numpy as np
import json
from pathlib import Path
from typing import Dict, Tuple, Optional
from datetime import datetime


class SE3Transform:
    """SE(3) rigid transform: rotation + translation (6 DOF).
    
    Preserves distances and angles. No scale change.
    
    Attributes:
        R (np.ndarray): 3x3 rotation matrix (SO(3))
        t (np.ndarray): 3x1 translation vector
        source_frame (str): Source coordinate frame identifier
        target_frame (str): Target coordinate frame identifier
    """
    
    def __init__(
        self,
        R: np.ndarray,
        t: np.ndarray,
        source_frame: str = "unknown",
        target_frame: str = "unknown",
        validate: bool = True
    ):
        """Initialize SE3 transform.
        
        Args:
            R: 3x3 rotation matrix
            t: 3x1 or (3,) translation vector in mm
            source_frame: Source frame identifier (e.g., "L", "U", "I")
            target_frame: Target frame identifier
            validate: If True, validate rotation matrix properties
            
        Raises:
            ValueError: If R is not a valid rotation matrix
        """
        self.R = np.array(R, dtype=np.float64).reshape(3, 3)
        self.t = np.array(t, dtype=np.float64).reshape(3, 1)
        self.source_frame = source_frame
        self.target_frame = target_frame
        
        if validate:
            self._validate()
    
    def _validate(self):
        """Validate rotation matrix properties.
        
        Raises:
            ValueError: If det(R) != 1 or R is not orthonormal
        """
        det = np.linalg.det(self.R)
        if abs(det - 1.0) > 1e-6:
            raise ValueError(
                f"Invalid rotation matrix: det(R) = {det:.9f}, expected 1.0"
            )
        
        # Check orthonormality: R^T @ R should be identity
        identity_check = self.R.T @ self.R
        error = np.max(np.abs(identity_check - np.eye(3)))
        if error > 1e-6:
            raise ValueError(
                f"Rotation matrix not orthonormal: ||R^T @ R - I|| = {error:.2e}"
            )
    
    def apply(self, points: np.ndarray) -> np.ndarray:
        """Apply transform to points.
        
        Args:
            points: Nx3 array of points in source frame (mm)
            
        Returns:
            Nx3 array of points in target frame (mm)
        """
        points = np.array(points, dtype=np.float64)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        
        # p_dst = R @ p_src + t
        return (self.R @ points.T).T + self.t.T
    
    def inverse(self) -> 'SE3Transform':
        """Compute inverse transform.
        
        Returns:
            SE3Transform representing the inverse
        """
        R_inv = self.R.T
        t_inv = -R_inv @ self.t
        
        return SE3Transform(
            R=R_inv,
            t=t_inv,
            source_frame=self.target_frame,
            target_frame=self.source_frame,
            validate=False  # Already validated
        )
    
    def compose(self, other: 'SE3Transform') -> 'SE3Transform':
        """Compose with another transform: T_c_from_a = T_c_from_b @ T_b_from_a.
        
        Args:
            other: Transform to compose (applied first)
            
        Returns:
            Composed SE3Transform
            
        Raises:
            ValueError: If frames don't align
        """
        if self.source_frame != other.target_frame:
            raise ValueError(
                f"Frame mismatch: cannot compose {self.target_frame}_from_{self.source_frame} "
                f"with {other.target_frame}_from_{other.source_frame}"
            )
        
        R_composed = self.R @ other.R
        t_composed = self.R @ other.t + self.t
        
        return SE3Transform(
            R=R_composed,
            t=t_composed,
            source_frame=other.source_frame,
            target_frame=self.target_frame
        )
    
    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 homogeneous matrix.
        
        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.R
        T[:3, 3:4] = self.t
        return T
    
    @classmethod
    def from_matrix(cls, T: np.ndarray, source_frame: str = "unknown", 
                   target_frame: str = "unknown") -> 'SE3Transform':
        """Create SE3Transform from 4x4 matrix.
        
        Args:
            T: 4x4 homogeneous transformation matrix
            source_frame: Source frame identifier
            target_frame: Target frame identifier
            
        Returns:
            SE3Transform instance
        """
        T = np.array(T, dtype=np.float64)
        return cls(
            R=T[:3, :3],
            t=T[:3, 3],
            source_frame=source_frame,
            target_frame=target_frame
        )
    
    def save(self, filepath: str, **metadata):
        """Save transform to JSON file.
        
        Args:
            filepath: Output JSON file path
            **metadata: Additional metadata to include (e.g., rmse_mm, n_points)
        """
        data = {
            "transform_type": "SE3",
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "R": self.R.tolist(),
            "t": self.t.flatten().tolist(),
            "scale": 1.0,
            "timestamp": datetime.now().isoformat(),
            **metadata
        }
        
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    @classmethod
    def load(cls, filepath: str) -> 'SE3Transform':
        """Load transform from JSON file.
        
        Args:
            filepath: Input JSON file path
            
        Returns:
            SE3Transform instance
            
        Raises:
            ValueError: If file format is invalid
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        if data.get("transform_type") != "SE3":
            raise ValueError(f"Expected SE3 transform, got {data.get('transform_type')}")
        
        return cls(
            R=np.array(data["R"]),
            t=np.array(data["t"]),
            source_frame=data.get("source_frame", "unknown"),
            target_frame=data.get("target_frame", "unknown")
        )


class Sim3Transform:
    """Sim(3) similarity transform: scale + rotation + translation (7 DOF).
    
    Preserves angles but not distances. Uniform scale factor.
    
    Attributes:
        s (float): Uniform scale factor
        R (np.ndarray): 3x3 rotation matrix (SO(3))
        t (np.ndarray): 3x1 translation vector
        source_frame (str): Source coordinate frame identifier
        target_frame (str): Target coordinate frame identifier
    """
    
    def __init__(
        self,
        s: float,
        R: np.ndarray,
        t: np.ndarray,
        source_frame: str = "unknown",
        target_frame: str = "unknown",
        validate: bool = True
    ):
        """Initialize Sim3 transform.
        
        Args:
            s: Uniform scale factor (positive)
            R: 3x3 rotation matrix
            t: 3x1 or (3,) translation vector in mm
            source_frame: Source frame identifier
            target_frame: Target frame identifier
            validate: If True, validate rotation matrix
            
        Raises:
            ValueError: If scale is non-positive or R is invalid
        """
        if s <= 0:
            raise ValueError(f"Scale must be positive, got {s}")
        
        self.s = float(s)
        self.R = np.array(R, dtype=np.float64).reshape(3, 3)
        self.t = np.array(t, dtype=np.float64).reshape(3, 1)
        self.source_frame = source_frame
        self.target_frame = target_frame
        
        if validate:
            self._validate_rotation()
    
    def _validate_rotation(self):
        """Validate rotation matrix (same as SE3)."""
        det = np.linalg.det(self.R)
        if abs(det - 1.0) > 1e-6:
            raise ValueError(f"Invalid rotation matrix: det(R) = {det:.9f}")
        
        identity_check = self.R.T @ self.R
        error = np.max(np.abs(identity_check - np.eye(3)))
        if error > 1e-6:
            raise ValueError(f"Rotation matrix not orthonormal: error = {error:.2e}")
    
    def apply(self, points: np.ndarray) -> np.ndarray:
        """Apply transform to points.
        
        Args:
            points: Nx3 array of points in source frame (mm)
            
        Returns:
            Nx3 array of points in target frame (mm)
        """
        points = np.array(points, dtype=np.float64)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        
        # p_dst = s * R @ p_src + t
        return (self.s * self.R @ points.T).T + self.t.T
    
    def inverse(self) -> 'Sim3Transform':
        """Compute inverse transform.
        
        Returns:
            Sim3Transform representing the inverse
        """
        s_inv = 1.0 / self.s
        R_inv = self.R.T
        t_inv = -s_inv * R_inv @ self.t
        
        return Sim3Transform(
            s=s_inv,
            R=R_inv,
            t=t_inv,
            source_frame=self.target_frame,
            target_frame=self.source_frame,
            validate=False
        )
    
    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 homogeneous matrix with scale absorbed in rotation.
        
        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.s * self.R
        T[:3, 3:4] = self.t
        return T
    
    def save(self, filepath: str, **metadata):
        """Save transform to JSON file.
        
        Args:
            filepath: Output JSON file path
            **metadata: Additional metadata
        """
        data = {
            "transform_type": "Sim3",
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "scale": float(self.s),
            "R": self.R.tolist(),
            "t": self.t.flatten().tolist(),
            "timestamp": datetime.now().isoformat(),
            **metadata
        }
        
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    @classmethod
    def load(cls, filepath: str) -> 'Sim3Transform':
        """Load transform from JSON file.
        
        Args:
            filepath: Input JSON file path
            
        Returns:
            Sim3Transform instance
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        if data.get("transform_type") != "Sim3":
            raise ValueError(f"Expected Sim3 transform, got {data.get('transform_type')}")
        
        return cls(
            s=data["scale"],
            R=np.array(data["R"]),
            t=np.array(data["t"]),
            source_frame=data.get("source_frame", "unknown"),
            target_frame=data.get("target_frame", "unknown")
        )


def umeyama(
    src_points: np.ndarray,
    dst_points: np.ndarray,
    estimate_scale: bool = True
) -> Tuple[float, np.ndarray, np.ndarray, float]:
    """Compute optimal similarity transform using Umeyama's algorithm.
    
    Finds optimal scale, rotation, and translation that minimizes:
        sum ||dst_points[i] - (s * R @ src_points[i] + t)||^2
    
    Args:
        src_points: Nx3 array of source points (mm)
        dst_points: Nx3 array of destination points (mm)
        estimate_scale: If True, compute Sim(3); if False, compute SE(3)
        
    Returns:
        Tuple of (scale, R, t, rmse):
            scale: Uniform scale factor (1.0 if estimate_scale=False)
            R: 3x3 rotation matrix
            t: 3x1 translation vector (mm)
            rmse: Root mean square error in mm
            
    Raises:
        ValueError: If point counts don't match or < 3 points
        
    Reference:
        Umeyama, S. (1991). "Least-squares estimation of transformation
        parameters between two point patterns." IEEE TPAMI.
    """
    src = np.array(src_points, dtype=np.float64)
    dst = np.array(dst_points, dtype=np.float64)
    
    if src.shape != dst.shape:
        raise ValueError(f"Shape mismatch: src {src.shape} vs dst {dst.shape}")
    
    if src.shape[0] < 3:
        raise ValueError(f"Need at least 3 points, got {src.shape[0]}")
    
    if src.shape[1] != 3:
        src = src.reshape(-1, 3)
        dst = dst.reshape(-1, 3)
    
    n = src.shape[0]
    
    # Compute centroids
    src_mean = src.mean(axis=0, keepdims=True)
    dst_mean = dst.mean(axis=0, keepdims=True)
    
    # Center point clouds
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    
    # Compute covariance matrix
    H = src_centered.T @ dst_centered / n
    
    # SVD: H = U @ S @ Vt
    U, S, Vt = np.linalg.svd(H)
    
    # Compute rotation (handle reflection case)
    R = Vt.T @ U.T
    
    # Ensure proper rotation (det(R) = +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Compute scale
    if estimate_scale:
        src_var = np.sum(src_centered ** 2) / n
        scale = np.sum(S) / src_var if src_var > 1e-10 else 1.0
    else:
        scale = 1.0
    
    # Compute translation
    t = dst_mean.T - scale * R @ src_mean.T
    
    # Compute RMSE
    src_transformed = (scale * R @ src.T).T + t.T
    residuals = dst - src_transformed
    rmse = np.sqrt(np.mean(np.sum(residuals ** 2, axis=1)))
    
    return scale, R, t, rmse


def compute_alignment(
    src_points: Dict[str, np.ndarray],
    dst_points: Dict[str, np.ndarray],
    estimate_scale: bool = True,
    rmse_warn_threshold: float = 0.1,
    rmse_fail_threshold: float = 0.5
) -> Tuple[Sim3Transform, float, Dict[str, float]]:
    """Compute optimal transform between corresponding point sets.
    
    Args:
        src_points: Dict mapping point IDs to 3D positions in source frame (mm)
        dst_points: Dict mapping point IDs to 3D positions in dest frame (mm)
        estimate_scale: If True, use Sim(3); if False, use SE(3)
        rmse_warn_threshold: Warning threshold in mm
        rmse_fail_threshold: Failure threshold in mm
        
    Returns:
        Tuple of (transform, rmse, residuals):
            transform: Sim3Transform or SE3Transform
            rmse: Overall RMSE in mm
            residuals: Dict mapping point IDs to residual distances in mm
            
    Raises:
        ValueError: If no common points or RMSE exceeds fail threshold
    """
    # Find common point IDs
    common_ids = set(src_points.keys()) & set(dst_points.keys())
    if not common_ids:
        raise ValueError("No common points between source and destination")
    
    # Build matched arrays
    common_ids = sorted(common_ids)
    src_array = np.array([src_points[pid] for pid in common_ids])
    dst_array = np.array([dst_points[pid] for pid in common_ids])
    
    # Compute transform
    scale, R, t, rmse = umeyama(src_array, dst_array, estimate_scale)
    
    # Validate RMSE
    if rmse > rmse_fail_threshold:
        raise ValueError(
            f"Alignment RMSE {rmse:.3f} mm exceeds failure threshold "
            f"{rmse_fail_threshold:.3f} mm"
        )
    
    if rmse > rmse_warn_threshold:
        print(
            f"WARNING: Alignment RMSE {rmse:.3f} mm exceeds warning threshold "
            f"{rmse_warn_threshold:.3f} mm"
        )
    
    # Compute per-point residuals
    src_transformed = (scale * R @ src_array.T).T + t.T
    residuals_array = np.linalg.norm(dst_array - src_transformed, axis=1)
    residuals = dict(zip(common_ids, residuals_array))
    
    # Create appropriate transform object
    if estimate_scale:
        transform = Sim3Transform(s=scale, R=R, t=t)
    else:
        if abs(scale - 1.0) > 0.001:
            print(f"WARNING: SE(3) requested but scale = {scale:.6f} (expected 1.0)")
        transform = SE3Transform(R=R, t=t)
    
    return transform, rmse, residuals
