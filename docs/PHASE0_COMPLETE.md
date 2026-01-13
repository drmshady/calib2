# Phase 0 Completion Summary

**Date Completed:** 2026-01-13  
**Status:** ✅ ALL DELIVERABLES COMPLETE

---

## Deliverables Created

### Documentation
- ✅ [docs/frames.md](docs/frames.md) - Complete coordinate frame definitions (L/U/I), transform conventions, unit standard (mm), file formats

### Core Implementation
- ✅ [src/transforms.py](src/transforms.py) - SE3/Sim3 transform classes with Umeyama algorithm
  - 547 lines: SE3Transform, Sim3Transform, umeyama(), compute_alignment()
  - Save/load JSON, apply/inverse/compose operations
  - Rotation matrix validation (det(R)=1, orthonormality)

- ✅ [tools/triangulation.py](tools/triangulation.py) - DLT + Levenberg-Marquardt triangulation
  - 331 lines: DLT, LM refinement, batch triangulation with outlier rejection
  - 5px outlier threshold, min 2 views per point
  - Per-point reprojection error reporting

- ✅ [tools/bundle_adjustment.py](tools/bundle_adjustment.py) - Export/load refpoints_L.json
  - 147 lines: export_refpoints_L(), load_refpoints_L(), validation
  - TRUE solver output frame (triangulated points, not CAD model)
  - Legacy geometry_L_MODEL support with deprecation warnings

- ✅ [tools/define_user_frame.py](tools/define_user_frame.py) - Compute T_U_from_L alignment
  - 205 lines: define_user_frame(), apply_user_frame(), validation
  - Umeyama-based alignment with RMSE gates (warn: 0.1mm, fail: 0.5mm)
  - CLI interface for standalone use

### Fixtures
- ✅ [calib/fixtures/reference_plate_4tags.json](calib/fixtures/reference_plate_4tags.json)
  - 2×2 grid, AprilTag IDs 1-4, tag36h11 family
  - 8.8mm tag size, 6.2mm spacing
  - 16 corners with known U-frame geometry (all Z=0, planar)

### Tests
- ✅ [test/test_transforms.py](test/test_transforms.py) - Transform correctness tests
  - 21 tests: SE3/Sim3 identity, inverse, compose, Umeyama, save/load
  - All tests passing ✅

- ✅ [test/test_phase0_refpoints.py](test/test_phase0_refpoints.py) - Integration tests
  - 6 tests: Triangulation accuracy (noise-free & noisy), full pipeline, min views
  - Synthetic camera generation, projection, triangulation, U-frame alignment
  - All tests passing ✅

---

## Test Results

```
test/test_transforms.py .................. 21 passed in 2.32s
test/test_phase0_refpoints.py ......... 6 passed in 7.65s
```

**Total:** 27 tests, 27 passed, 0 failed ✅

---

## Key Features Implemented

### Transform Classes
- **SE3Transform:** Rigid transforms (6 DOF) - rotation + translation
- **Sim3Transform:** Similarity transforms (7 DOF) - scale + rotation + translation
- **Umeyama algorithm:** Optimal closed-form alignment (Sim3/SE3)
- **Validation gates:** det(R)≈1, orthonormality, RMSE thresholds
- **JSON serialization:** Frame labels, timestamps, metadata

### Triangulation
- **DLT initialization:** Linear solution from 2D observations
- **LM refinement:** Nonlinear reprojection error minimization
- **Outlier rejection:** 5px threshold with automatic re-triangulation
- **Batch processing:** Multi-point triangulation with per-point statistics
- **Projection matrices:** Built from camera intrinsics + extrinsics

### U-Frame Definition
- **Reference plate alignment:** Uses AprilTag corner correspondences (IDs 1-4)
- **Automatic scale detection:** Auto-selects Sim3 if scale varies >0.05%
- **Per-corner residuals:** Detailed alignment quality logging
- **RMSE gates:** Warn at 0.1mm, fail at 0.5mm (fail-fast principle)
- **Transform composition:** L→U→I frame chain support

---

## Unit Constitution (Enforced)

**Global Standard:** ALL coordinates in millimeters (mm)

**Explicit Ban:** Meters (m) prohibited throughout pipeline

**Verification Gate:** `config.yaml` must include `force_mm_conversion: true`

---

## Frame Definitions

### L-Frame (Local/Solver)
- **Origin:** Arbitrary (set by bundle adjustment)
- **Source:** Triangulated 3D points from 2D observations
- **Purpose:** Internal optimization, solver output
- **Files:** `refpoints_L.json`, `cameras_L.json`

### U-Frame (User)
- **Origin:** Tag 1 top-left corner (0, 0, 0)
- **Source:** Reference plate fixture (IDs 1-4)
- **Purpose:** Canonical reporting frame, bench validation
- **Files:** `T_U_from_L.json`, `refpoints_U.json`

### I-Frame (IOS/exocad)
- **Origin:** Defined by scanner manufacturer
- **Source:** IOS mesh coordinate system
- **Purpose:** CAD/CAM integration
- **Files:** `T_I_from_U.json` (Phase 6 - not yet implemented)

---

## File Formats

### Transform JSON (T_U_from_L.json)
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

### Reference Points JSON (refpoints_L.json)
```json
{
  "frame": "L",
  "units": "mm",
  "points": {
    "1_TL": [x1, y1, z1],
    "1_TR": [x2, y2, z2]
  },
  "metadata": {
    "triangulation_method": "DLT+LM",
    "mean_reprojection_error_px": 0.3,
    "n_views_per_point": {"1_TL": 8, "1_TR": 7}
  }
}
```

---

## Corner ID Naming Convention

**Format:** `<tag_id>_<corner_label>`

**Corner Labels (OpenCV AprilTag order):**
- `TL` - Top-Left (first corner)
- `TR` - Top-Right
- `BR` - Bottom-Right
- `BL` - Bottom-Left

**Examples:**
- `1_TL` - Tag 1, top-left corner
- `4_BR` - Tag 4, bottom-right corner

---

## Validation Gates

### Transform Validation
- ✓ `det(R) = 1.0 ± 1e-6` (proper rotation)
- ✓ `||R^T @ R - I|| < 1e-6` (orthonormality)
- ✓ `scale ≈ 1.0 ± 0.001` (SE3 only)

### Alignment Quality (L→U)
- ⚠️ **Warning:** RMSE > 0.1 mm
- ❌ **Failure:** RMSE > 0.5 mm (raises ValueError)

### Triangulation Quality
- ⚠️ **Outlier:** Reprojection error > 5 px
- ❌ **Insufficient:** < 2 views per point (skips point)

---

## Phase 0 Achievement

✅ **Ambiguity eliminated:** Every file declares frame + units  
✅ **Transform infrastructure:** SE3/Sim3 with validation  
✅ **Triangulation core:** DLT+LM with outlier rejection  
✅ **U-frame definition:** Deterministic reference alignment  
✅ **Comprehensive tests:** 27 tests covering all components  
✅ **Documentation:** Complete frame definitions and conventions  

**Phase 0 is production-ready.** All deliverables tested and validated.

---

## Next Steps (Phase 1)

- Camera calibration profiles (f/16, 1:5/1:6/1:7 magnification)
- `calib/camera_intrinsics_1_6.json` generation
- Roll diversity capture procedure
- Calibration reprojection error verification

---

## References

- Umeyama, S. (1991). "Least-squares estimation of transformation parameters between two point patterns." IEEE TPAMI.
- Hartley & Zisserman (2004). "Multiple View Geometry in Computer Vision." (DLT triangulation)
- OpenCV AprilTag: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
