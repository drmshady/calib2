# Coordinate Frames and Transform Conventions

**Version:** 1.0  
**Last Updated:** 2026-01-13  
**Status:** Phase 0 Deliverable

## Overview

This document defines the three coordinate frames used throughout the dental photogrammetry pipeline and establishes conventions for transforms, units, and naming.

---

## Global Unit Standard

**ALL coordinates, distances, and geometric outputs MUST be in millimeters (mm).**

- **Prohibited:** Use of meters (m) anywhere in the pipeline
- **Enforcement:** `config.yaml` must include `force_mm_conversion: true`
- **Rationale:** Dental precision requires sub-millimeter accuracy; millimeters prevent floating-point errors and unit confusion

---

## The Three Frames

### L-Frame (Local / Solver Frame)

**Definition:** The coordinate system output by bundle adjustment and triangulation.

**Origin:** Arbitrary, determined by the optimization solver (typically near first camera or centroid of scene).

**Purpose:**
- Internal optimization workspace
- Direct output from Structure-from-Motion (SfM)
- Diagnostic analysis

**Key Properties:**
- **True solver output:** Triangulated 3D points from 2D observations + optimized camera poses
- **Not CAD-aligned:** L-frame geometry does NOT match CAD model coordinates
- **Metric scale:** Determined by AprilTag known geometry (8.8mm edge)

**Files:**
- `runs/<dataset>/refpoints_L.json` — Triangulated 3D points in L-frame
- `runs/<dataset>/cameras_L.json` — Camera poses in L-frame

---

### U-Frame (User Frame)

**Definition:** A canonical, repeatable reference frame for reporting and comparison.

**Origin:** Defined by the reference plate (AprilTag IDs 1-4).

**Purpose:**
- Consistent coordinate system across capture sessions
- Bench validation and repeatability testing
- Export format for downstream analysis
- "V-STARS WINTRANS equivalent" — standardized reporting frame

**Key Properties:**
- **Deterministic:** Defined by known reference plate geometry
- **Alignment:** Transform L → U computed via Umeyama algorithm (Sim3 or SE3)
- **Corner-based:** Uses AprilTag corner positions (IDs 1-4) for alignment

**Files:**
- `calib/fixtures/reference_plate_4tags.json` — Known U-frame geometry
- `runs/<dataset>/T_U_from_L.json` — Transform from L to U
- `runs/<dataset>/refpoints_U.json` — Points transformed to U-frame

---

### I-Frame (IOS / exocad Frame)

**Definition:** The coordinate system of the intraoral scanner (IOS) mesh.

**Origin:** Defined by the IOS device manufacturer or exocad scan-body fit.

**Purpose:**
- CAD/CAM integration
- Prosthetic design alignment
- Digital workflow compatibility

**Key Properties:**
- **External definition:** Determined by IOS software, not photogrammetry
- **Transform required:** U ↔ I alignment via common landmarks or scan bodies
- **Phase 6 deliverable:** Not implemented in Phase 0

**Files:**
- `runs/<dataset>/T_I_from_U.json` — Transform from U to I (Phase 6)
- `runs/<dataset>/arch_ios.stl` — IOS mesh in I-frame

---

## Transform Conventions

### Notation

**General form:** `T_dst_from_src`

- `dst` = destination frame
- `src` = source frame
- **Meaning:** Transform that converts points from `src` coordinates to `dst` coordinates

**Examples:**
- `T_U_from_L` — Transforms points from L-frame to U-frame
- `T_I_from_U` — Transforms points from U-frame to I-frame
- `T_cam_from_world` — Camera extrinsic transform (world → camera)

### Matrix Convention

**Column vectors:** `p_dst = T_dst_from_src @ p_src`

**Homogeneous 4×4 matrices:**
```
T = [ R  t ]
    [ 0  1 ]
```
Where:
- `R` — 3×3 rotation matrix (SO(3))
- `t` — 3×1 translation vector
- Bottom row is `[0, 0, 0, 1]`

**Camera convention (OpenCV standard):**
```
p_cam = R @ p_world + t
```
Where `R` and `t` are from camera extrinsic parameters.

### Transform Types

**SE(3) — Special Euclidean Group (Rigid transforms)**
- 6 degrees of freedom (3 rotation + 3 translation)
- Preserves distances and angles
- `det(R) = +1` (proper rotation)
- No scale change

**Sim(3) — Similarity transforms**
- 7 degrees of freedom (3 rotation + 3 translation + 1 scale)
- Preserves angles but NOT distances
- Uniform scale factor `s`
- Used when absolute scale varies between runs

**When to use:**
- **SE(3):** Single-run processing, known metric scale, bench validation
- **Sim(3):** Multi-run alignment, scale ambiguity >0.05%

---

## Corner ID Format

**AprilTag corners:** `<tag_id>_<corner_label>`

**Corner labels (standard order):**
- `TL` — Top-Left
- `TR` — Top-Right
- `BR` — Bottom-Right
- `BL` — Bottom-Left

**Examples:**
- `1_TL` — Tag ID 1, top-left corner
- `4_BR` — Tag ID 4, bottom-right corner

**Ordering:** Follows OpenCV AprilTag detector output (clockwise from top-left when tag is upright).

---

## Reference Plate Specification

**Reserved IDs:** 1, 2, 3, 4 (implant caps use IDs 5+)

**Purpose:** Define U-frame origin and orientation

**Geometry:** Known 3D positions of all 16 corners (4 tags × 4 corners) in U-frame

**File:** `calib/fixtures/reference_plate_4tags.json`

**Alignment algorithm:** Umeyama (closed-form optimal Sim3 or SE3)

---

## Validation Gates

### Transform Validation
- `det(R) ≈ 1.0` (proper rotation, tolerance ±1e-6)
- `||R^T @ R - I|| < 1e-6` (orthonormality)
- Scale ≈ 1.0 for SE(3) transforms (tolerance ±0.001)

### Alignment Quality (L → U)
- **Warning threshold:** RMSE > 0.1 mm
- **Failure threshold:** RMSE > 0.5 mm
- Log per-corner residuals for diagnostics

### Triangulation Quality
- **Outlier rejection:** Reprojection error > 5px
- **Minimum views:** 2 views per 3D point
- **DLT initialization:** Linear solution, then Levenberg-Marquardt refinement

---

## File Formats

### Transform JSON

```json
{
  "transform_type": "SE3",
  "source_frame": "L",
  "target_frame": "U",
  "R": [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]],
  "t": [tx, ty, tz],
  "scale": 1.0,
  "rmse_mm": 0.042,
  "n_points": 16,
  "timestamp": "2026-01-13T10:30:00Z"
}
```

### Reference Points JSON

```json
{
  "frame": "L",
  "units": "mm",
  "points": {
    "1_TL": [x1, y1, z1],
    "1_TR": [x2, y2, z2],
    "4_BL": [x16, y16, z16]
  },
  "metadata": {
    "triangulation_method": "DLT+LM",
    "mean_reprojection_error_px": 0.3,
    "n_views_per_point": {"1_TL": 8, "1_TR": 7, ...}
  }
}
```

---

## Implementation Rules

1. **Always declare frame:** Every JSON file must include `"frame": "L"` or `"U"` or `"I"`
2. **Always declare units:** Every JSON file must include `"units": "mm"`
3. **Fail-fast validation:** Raise `ValueError` if `det(R) ≠ 1` or RMSE exceeds threshold
4. **Log residuals:** Per-point alignment errors for debugging
5. **Prefer refpoints_L.json:** Use triangulated points, not CAD model coordinates

---

## Examples

### Example 1: Load and apply transform

```python
from src.transforms import SE3Transform

# Load transform
T_U_from_L = SE3Transform.load("runs/dataset_01/T_U_from_L.json")

# Apply to points
points_L = np.array([[10.0, 20.0, 30.0], [40.0, 50.0, 60.0]])  # Nx3 in mm
points_U = T_U_from_L.apply(points_L)
```

### Example 2: Compute U-frame alignment

```python
from tools.define_user_frame import define_user_frame

# Compute transform from reference plate
T_U_from_L, rmse = define_user_frame(
    refpoints_L_path="runs/dataset_01/refpoints_L.json",
    reference_plate_path="calib/fixtures/reference_plate_4tags.json"
)

# Validate
if rmse > 0.5:
    raise ValueError(f"Alignment RMSE {rmse:.3f} mm exceeds 0.5 mm threshold")
```

---

## Phase Dependencies

- **Phase 0:** Define frames, implement transforms, triangulation
- **Phase 1:** Camera calibration (operates in camera frame, outputs intrinsics)
- **Phase 2-4:** Marker detection, BA → generates L-frame outputs
- **Phase 5:** Bench validation in U-frame (repeatability testing)
- **Phase 6:** I-frame integration (IOS mesh alignment)

---

## References

- Umeyama, S. (1991). "Least-squares estimation of transformation parameters between two point patterns." IEEE TPAMI.
- OpenCV camera calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
- Hartley & Zisserman (2004). "Multiple View Geometry in Computer Vision." (DLT triangulation)
