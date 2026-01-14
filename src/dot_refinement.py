"""Dot-assisted refinement for Phase 3 reconstructions.

Uses the AOX flag cap model (AprilTag corners + dot constellations) to:
  1) Estimate a per-tag rigid transform from cap-frame -> world (L-frame)
	 by aligning model AprilTag corners to reconstructed AprilTag corners.
  2) Project model dot centers into each image using current camera poses.
  3) Detect dots in a small ROI around the projection (skip if not visible).
  4) Triangulate dot 3D points and add them as additional observations.

All 2D observations stored in the SfM structure are UNDISTORTED pixels.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import json
import math

import cv2
import numpy as np

from incremental_sfm import IncrementalSfM, Point3D
from geometry_utils import (
	triangulate_opencv,
	check_cheirality,
	compute_camera_center,
	compute_reprojection_error,
)


@dataclass(frozen=True)
class CapDot:
	face: str
	index: int
	center_cap_mm: np.ndarray  # (3,)
	diameter_mm: float


@dataclass(frozen=True)
class CapModel:
	tag_corners_cap_mm: np.ndarray  # (4,3)
	dots: List[CapDot]


def _rigid_transform_kabsch(src_pts: np.ndarray, dst_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
	"""Compute R,t that best map src -> dst.

	Returns:
		R: (3,3)
		t: (3,)
	"""
	src = np.asarray(src_pts, dtype=np.float64)
	dst = np.asarray(dst_pts, dtype=np.float64)

	if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
		raise ValueError(f"Expected Nx3 matching shapes, got {src.shape} and {dst.shape}")

	src_centroid = src.mean(axis=0)
	dst_centroid = dst.mean(axis=0)

	src_centered = src - src_centroid
	dst_centered = dst - dst_centroid

	H = src_centered.T @ dst_centered
	U, _, Vt = np.linalg.svd(H)
	R = Vt.T @ U.T

	# Enforce proper rotation (det=+1)
	if np.linalg.det(R) < 0:
		Vt[-1, :] *= -1
		R = Vt.T @ U.T

	t = dst_centroid - R @ src_centroid
	return R, t


def load_cap_model(cap_model_path: str) -> CapModel:
	"""Load cap model JSON (AOX flag v2) into a lightweight structure."""
	path = Path(cap_model_path)
	with path.open("r", encoding="utf-8") as f:
		data = json.load(f)

	features = data.get("features", {})

	april = features.get("april_tag", {})
	tag_corners = np.asarray(april.get("corner_points_mm", []), dtype=np.float64)
	if tag_corners.shape != (4, 3):
		raise ValueError(f"cap model: expected 4x3 corner_points_mm, got {tag_corners.shape}")

	dots: List[CapDot] = []
	faces = features.get("faces", [])
	for face_entry in faces:
		if face_entry.get("type") != "dot_constellation":
			continue

		face_name = str(face_entry.get("face"))
		centers = np.asarray(face_entry.get("centers_mm", []), dtype=np.float64)
		diameters = face_entry.get("dot_diameters_mm")

		if centers.ndim != 2 or centers.shape[1] != 3 or centers.shape[0] == 0:
			continue

		if diameters is None:
			default_d = float(face_entry.get("dot_diameter_mm", 1.0))
			diam_arr = np.full((centers.shape[0],), default_d, dtype=np.float64)
		else:
			diam_arr = np.asarray(diameters, dtype=np.float64)
			if diam_arr.shape[0] != centers.shape[0]:
				raise ValueError(
					f"cap model: dot_diameters_mm length {diam_arr.shape[0]} != centers {centers.shape[0]} for face {face_name}"
				)

		for i, (c, d) in enumerate(zip(centers, diam_arr)):
			dots.append(CapDot(face=face_name, index=i, center_cap_mm=c, diameter_mm=float(d)))

	if not dots:
		raise ValueError("cap model: no dot constellations found")

	return CapModel(tag_corners_cap_mm=tag_corners, dots=dots)


def _dot_point_id(tag_id: int, face: str, dot_index: int) -> int:
	# Large offset to avoid collisions with tag-corner IDs (tag_id*10+corner).
	face_code_map = {"top": 1, "left": 2, "right": 3}
	face_code = face_code_map.get(face, 9)
	return 1_000_000 + int(tag_id) * 1_000 + face_code * 100 + int(dot_index)


def _undistort_pixel_to_undist_pixels(pt_xy: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
	pt = np.asarray(pt_xy, dtype=np.float64).reshape(1, 1, 2)
	und = cv2.undistortPoints(pt, K, D, P=K).reshape(2)
	return und.astype(np.float64)


def _detect_dot_in_roi(
	image_gray: np.ndarray,
	predicted_uv: Tuple[float, float],
	roi_half: int,
	max_center_dist_px: float,
	prefer_large: bool,
) -> Optional[np.ndarray]:
	"""Detect a dot center near predicted location in a grayscale image.

	Returns:
		(2,) distorted pixel coords if found; else None.
	"""
	h, w = image_gray.shape[:2]
	u0, v0 = float(predicted_uv[0]), float(predicted_uv[1])

	if not (math.isfinite(u0) and math.isfinite(v0)):
		return None

	if u0 < -roi_half or v0 < -roi_half or u0 > w - 1 + roi_half or v0 > h - 1 + roi_half:
		return None

	x1 = max(0, int(round(u0)) - roi_half)
	y1 = max(0, int(round(v0)) - roi_half)
	x2 = min(w, int(round(u0)) + roi_half + 1)
	y2 = min(h, int(round(v0)) + roi_half + 1)

	roi = image_gray[y1:y2, x1:x2]
	if roi.size == 0:
		return None

	# Basic preprocessing: blur + Otsu threshold (dots are typically dark)
	roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
	_, bw = cv2.threshold(roi_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

	# Remove tiny noise
	bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)

	contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if not contours:
		return None

	best = None
	best_score = float("inf")

	for cnt in contours:
		area = float(cv2.contourArea(cnt))
		if area < 6.0:
			continue

		perim = float(cv2.arcLength(cnt, True))
		if perim <= 1e-6:
			continue

		circularity = 4.0 * math.pi * area / (perim * perim)
		if circularity < 0.25:
			continue

		M = cv2.moments(cnt)
		m00 = float(M.get("m00", 0.0))
		if abs(m00) < 1e-6:
			continue

		cx = float(M["m10"] / m00) + x1
		cy = float(M["m01"] / m00) + y1

		dist = math.hypot(cx - u0, cy - v0)
		if dist > max_center_dist_px:
			continue

		# Score: prioritize closeness; secondarily prefer larger blob for anchor dots.
		size_bonus = -0.2 * math.log(max(area, 1.0)) if prefer_large else 0.0
		score = dist + size_bonus

		if score < best_score:
			best_score = score
			best = np.array([cx, cy], dtype=np.float64)

	return best


def add_dots_as_points(
	sfm: IncrementalSfM,
	cap_model: CapModel,
	detections_all: Dict[str, List[Dict]],
	image_paths: List[str],
	K: np.ndarray,
	D: np.ndarray,
	roi_half: int = 30,
	max_center_dist_px: float = 14.0,
	min_views: int = 2,
	max_reproj_px: float = 3.0,
	verbose: bool = True,
) -> Dict:
	"""Detect dots across images, triangulate, and add them to the SfM structure.

	Notes:
		- Does NOT run bundle adjustment. Caller can do that afterwards.
		- Uses existing camera poses in `sfm`.

	Returns:
		stats dict.
	"""
	img_path_by_id = {Path(p).stem: str(p) for p in image_paths}

	# Precompute per-tag cap->world transforms from reconstructed tag corners.
	cap_to_world: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
	tag_ids = {pid // 10 for pid in sfm.points_3d.keys() if pid < 1_000_000}

	for tag_id in sorted(tag_ids):
		corner_ids = [tag_id * 10 + i for i in range(4)]
		if not all(cid in sfm.points_3d for cid in corner_ids):
			continue
		world_corners = np.stack([sfm.points_3d[cid].xyz for cid in corner_ids], axis=0)
		try:
			R, t = _rigid_transform_kabsch(cap_model.tag_corners_cap_mm, world_corners)
		except Exception:
			continue
		cap_to_world[tag_id] = (R, t)

	if verbose:
		print(f"[dots] cap->world transforms: {len(cap_to_world)} tags")

	# Collect dot observations per dot-point-id.
	dot_obs: Dict[int, Dict[str, np.ndarray]] = {}
	dot_obs_count = 0

	for img_id, detections in detections_all.items():
		cam = sfm.cameras.get(img_id)
		if cam is None or not cam.registered:
			continue

		img_path = img_path_by_id.get(img_id)
		if not img_path:
			continue

		img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
		if img_gray is None:
			continue

		rvec, _ = cv2.Rodrigues(cam.R)
		tvec = cam.t

		for det in detections:
			tag_id = int(det.get("tag_id"))
			if tag_id not in cap_to_world:
				continue

			Rcw, tcw = cap_to_world[tag_id]

			for dot in cap_model.dots:
				Xw = (Rcw @ dot.center_cap_mm.reshape(3, 1) + tcw.reshape(3, 1)).reshape(3)

				# Project to distorted pixel coords for ROI search.
				proj, _ = cv2.projectPoints(
					Xw.reshape(1, 1, 3),
					rvec,
					tvec,
					K,
					D,
				)
				uv_pred = proj.reshape(2)

				uv_det = _detect_dot_in_roi(
					img_gray,
					(float(uv_pred[0]), float(uv_pred[1])),
					roi_half=roi_half,
					max_center_dist_px=max_center_dist_px,
					prefer_large=(dot.diameter_mm >= 1.2),
				)

				if uv_det is None:
					continue

				uv_und = _undistort_pixel_to_undist_pixels(uv_det, K, D)

				pid = _dot_point_id(tag_id, dot.face, dot.index)
				dot_obs.setdefault(pid, {})[img_id] = uv_und
				dot_obs_count += 1

	# Filter by min views
	dot_obs_filtered = {pid: obs for pid, obs in dot_obs.items() if len(obs) >= min_views}

	if verbose:
		print(
			f"[dots] observations: {dot_obs_count} (raw), "
			f"{len(dot_obs_filtered)} tracks with >= {min_views} views"
		)

	added_points = 0
	rejected_cheirality = 0
	rejected_reproj = 0

	for pid, obs in dot_obs_filtered.items():
		if pid in sfm.points_3d:
			continue

		views = [
			(img_id, pt2d)
			for img_id, pt2d in obs.items()
			if img_id in sfm.cameras and sfm.cameras[img_id].registered
		]
		if len(views) < 2:
			continue

		# Pick two views with largest baseline first for stable triangulation.
		cam_centers = {
			img_id: compute_camera_center(sfm.cameras[img_id].R, sfm.cameras[img_id].t)
			for img_id, _ in views
		}
		best_pair = None
		best_dist = -1.0
		for i in range(len(views)):
			for j in range(i + 1, len(views)):
				ci = cam_centers[views[i][0]]
				cj = cam_centers[views[j][0]]
				d = float(np.linalg.norm(ci - cj))
				if d > best_dist:
					best_dist = d
					best_pair = (views[i], views[j])

		if best_pair is None:
			continue

		ordered_views = [best_pair[0], best_pair[1]] + [v for v in views if v not in best_pair]

		pts2d = [pt for _, pt in ordered_views]
		Ps = [
			sfm.K @ np.hstack([sfm.cameras[img_id].R, sfm.cameras[img_id].t])
			for img_id, _ in ordered_views
		]

		try:
			X = triangulate_opencv(pts2d, Ps)
		except Exception:
			continue

		# Cheirality in all used views
		all_valid = True
		for img_id, _ in ordered_views:
			cam = sfm.cameras[img_id]
			valid_mask = check_cheirality(X, cam.R, cam.t, min_depth=0.1)
			if not bool(np.asarray(valid_mask).ravel()[0]):
				all_valid = False
				break

		if not all_valid:
			rejected_cheirality += 1
			continue

		# Reprojection quality check
		reproj_errors = []
		for img_id, pt2d in ordered_views:
			cam = sfm.cameras[img_id]
			err = float(
				compute_reprojection_error(X, pt2d, sfm.K, cam.R, cam.t, return_per_point=True)[0]
			)
			reproj_errors.append(err)

		if float(np.mean(reproj_errors)) > max_reproj_px:
			rejected_reproj += 1
			continue

		sfm.points_3d[pid] = Point3D(
			point_id=pid,
			xyz=np.asarray(X, dtype=np.float64),
			observations={img_id: pt2d for img_id, pt2d in ordered_views},
		)
		added_points += 1

	return {
		"cap_tags_with_pose": len(cap_to_world),
		"dot_tracks_raw": len(dot_obs),
		"dot_tracks_used": len(dot_obs_filtered),
		"dot_points_added": added_points,
		"dot_rejected_cheirality": rejected_cheirality,
		"dot_rejected_reproj": rejected_reproj,
	}