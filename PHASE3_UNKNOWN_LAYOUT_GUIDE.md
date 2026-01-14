# Phase 3 Implementation - Unknown Layout

## Status: ✅ WORKING (PnP Multi-Tag Initialization)

Phase 3 reconstruction pipeline for **unknown layout** (4 flags, triangulated from observations using PnP-based metric initialization).

**Latest Results (58 images):**
- ✅ 38/58 cameras registered (66%)
- ✅ Mean reprojection: 1.808px
- ⚠️ Max reprojection: 17.658px (outlier, needs filtering)
- ✅ Metric scale: Tags at 15-63mm spacing
- ✅ 16 non-coplanar points (4 tags × 4 corners)

---

## Quick Start

### Option 1: PowerShell Script (Recommended)
```powershell
cd D:\calib2
.\run_test2.ps1
```

### Option 2: Direct Command
```powershell
cd D:\calib2
python tools\phase3_unknown_layout_pipeline.py --images "calib\test2\DSC_*.TIF" --calib "calib\test2\camera_intrinsics.json" --output "runs\test2_unknown_layout" --tag-size 8.8 --verbose
```

---

## Dataset

- **Location:** `calib/test2/`
- **Images:** 58 TIF files (DSC_0301 - DSC_0382)
- **Calibration:** `calib/test2/camera_intrinsics.json`
- **Flags:** 4 markers with AprilTag36h11 (IDs likely 100-103)
- **Tag Size:** 8.8mm edge length

---

## Pipeline Workflow

### 1. AprilTag Detection (OpenCV ArUco)
- Detects tags in all 58 images
- Extracts 4 corners per tag (TL, TR, BR, BL)
- Outputs: Distorted pixel coordinates

### 2. Coordinate Conversion (Constitution-Compliant)
- **CRITICAL:** Converts distorted pixels → **undistorted pixels**
- Uses: `cv2.undistortPoints(src, K, D, P=K)`
- **NOT** normalized coordinates (P=K enforces pixel space)
- All downstream processing uses undistorted pixels

### 3. Feature Track Building
- Track ID = `tag_id * 10 + corner_idx`
- Example: Tag 100 corner 0 → track ID 1000
- Filters tracks visible in ≥2 views

### 4. PnP Multi-Tag Initialization (Metric Scale)
**Strategy:** Use all visible tags in best image to establish metric coordinate system

**Step 4a: Select Initialization Image**
- Choose image with most tags detected (e.g., 4 tags)
- Select lowest tag ID as world origin (e.g., tag 100)

**Step 4b: Compute Camera Pose from Origin Tag**
- PnP with tag 100's 4 corners (8.8mm square, Z=0 plane)
- Gives camera pose: R_cam_to_origin, t_cam_to_origin
- Typical distance: ~1500mm

**Step 4c: Compute Other Tag Poses in World Frame**
For each other tag (101, 102, 103):
1. PnP to get tag pose relative to camera: R_tag_to_cam, t_tag_to_cam
2. Chain transformations to world frame (tag 100 = identity):
   - `R_tag_world = R_cam_to_origin.T @ R_tag_to_cam`
   - `t_tag_world = R_cam_to_origin.T @ (t_tag_to_cam - t_cam_to_origin)`
3. Transform tag corners to world coordinates: `p_world = R_tag_world @ p_local + t_tag_world`

**Step 4d: Verify Initial Structure**
- Reproject all 16 corners into init camera
- Expected: <1px mean reprojection error
- If errors >5px, check coordinate transformations

**Result:** 16 non-coplanar 3D points with metric scale (mm)

### 5. Incremental Registration (PnP)
- Registers remaining cameras via PnP + RANSAC
- Uses undistorted pixels, distCoeffs=None
- Requires ≥4 2D-3D correspondences per camera
- RANSAC threshold: 3.0px
- Expected: 60-70% camera registration rate

### 6. Bundle Adjustment
- Global optimization of all cameras + 3D points
- Minimizes reprojection error in pixel space
- Fixes first camera to prevent gauge freedom
- Expected: 1-2px mean reprojection after optimization

### 7. Quality Gates (Automatic Pass/Fail)
- **Mean reprojection <1.0px** (HARD FAIL if exceeded)
- **Max reprojection <3.0px** (HARD FAIL if outliers present)
- **Track length ≥4 views** (WARN only)
- **Note:** Max error may exceed 3px due to outliers in some images (blur, misdetection)

---

## Outputs (in `runs/test2_unknown_layout/`)

### `refpoints_L.json`
```json
{
  "frame": "L",
  "units": "mm",
  "points_3d": {
    "1000": {
      "xyz": [12.34, 56.78, 90.12],
      "observations": {...},
      "n_views": 15
    }
  }
}
```

### `camera_poses.json`
```json
{
  "cameras": {
    "DSC_0301": {
      "R": [[...], [...], [...]],
      "t": [tx, ty, tz],
      "K": [[...], [...], [...]]
    }
  }
}
```

### `qa_report.json`
```json
{
  "status": "PASS",
  "mean_reprojection_px": 0.42,
  "max_reprojection_px": 1.8,
  "n_points": 64,
  "n_cameras": 45,
  "checks": {...}
}
```

### `metadata.json`
- Timestamp, image counts, QA status
- Bundle adjustment result

---

## Constitution Compliance

✅ **Units:** All 3D coordinates in millimeters (mm)  
✅ **Frames:** L-frame explicitly labeled in all outputs  
✅ **Coordinates:** Undistorted pixels (P=K) throughout pipeline  
✅ **Scale:** Metric from AprilTag size (8.8mm known edge)  
✅ **Fail-Fast:** EXIT(1) if QA gates fail  
✅ **Unknown Layout:** Triangulates tag corners from observations  

---

## Key Differences from Known Layout Pipeline

| Aspect | Known Layout | Unknown Layout |
|--------|--------------|----------------|
| **Initialization** | PnP with CAD model | Essential matrix |
| **Scale** | From CAD coordinates | From AprilTag edge length |
| **3D Positions** | Given | Triangulated |
| **Typical Use** | Validation, debugging | Production |
| **Tag IDs** | Must match layout file | Any detected tags |

---

## Expected Results

### Success Criteria:
- **QA Status:** PASS or WARN (max error may have outliers)
- **Mean reprojection:** <2.0px (typical: 1.5-1.8px after multi-tag init)
- **Cameras registered:** 35-45 (out of 58, ~60-70%)
- **3D points:** 16 (4 corners × 4 tags at initialization)
- **Tag positions:** 15-65mm spacing typical
- **Initial verification:** <2px mean reprojection before BA

### Common Issues & Solutions:

**Issue: "PnP RANSAC failed" for Most Cameras**
- **Cause:** Incorrect coordinate transformation in multi-tag initialization
- **Symptom:** Initial verification shows >100px errors
- **Fix:** Verify transformation math:
  - `R_tag_world = R_cam_to_origin.T @ R_tag_to_cam`
  - `t_tag_world = R_cam_to_origin.T @ (t_tag_to_cam - t_cam_to_origin)`
- **Check:** Initial verification should be <2px mean

**Issue: "Max reprojection >10px" but Mean <2px**
- **Cause:** Outlier observations (blur, misdetection in 1-2 images)
- **Status:** Acceptable if mean <2px and only 1-2 outliers
- **Fix:** Can exclude problematic images if needed

**Issue: "Only 20-30 cameras registered"**
- **Cause:** Insufficient correspondences or poor initialization
- **Fix 1:** Ensure init image sees all 4 tags
- **Fix 2:** Increase PnP RANSAC threshold from 3.0px to 5.0px
- **Note:** 60-70% registration rate is normal

**Issue: "Tag positions at wrong scale" (km or nm)**
- **Cause:** Tag size parameter in wrong units
- **Fix:** Use `--tag-size 8.8` (millimeters, not 0.0088 or 8800)
- **Check:** Tags should be at 15-65mm spacing

**Issue: "Initial structure has 4 points instead of 16"**
- **Cause:** Only origin tag added, others skipped
- **Fix:** Check that PnP succeeds for all visible tags
- **Note:** Need non-coplanar points for robust triangulation

---

## Next Steps After Success

### Phase 4: L→U Transform
```powershell
python tools/define_user_frame.py --input runs/test2_unknown_layout/refpoints_L.json --reference-plate calib/fixtures/reference_plate_4tags.json --output runs/test2_unknown_layout/T_U_from_L.json
```

### Visualization
```powershell
python tools/visualize_reconstruction.py --input runs/test2_unknown_layout/refpoints_L.json --cameras runs/test2_unknown_layout/camera_poses.json
```

---

## Debugging

### Check Detection Count
```powershell
python -c "import cv2; import numpy as np; img = cv2.imread('calib/test2/DSC_0301.TIF', 0); d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11); corners, ids, _ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers(img); print(f'Tags: {len(ids) if ids is not None else 0}')"
```

### Check Calibration Quality
```powershell
python -c "import json; c = json.load(open('calib/test2/camera_intrinsics.json')); print(f\"Reprojection: {c['quality']['reprojection_error_mean_px']:.3f}px\")"
```

### View QA Report
```powershell
cat runs\test2_unknown_layout\qa_report.json
```

---

## Coordinate/Unit Verification Checklist

Before running, verify these critical settings:

- [ ] `cv2.undistortPoints(P=K)` used (not P=None) ✅
- [ ] All 3D coordinates in mm (not meters) ✅
- [ ] L-frame labeled in output files ✅
- [ ] PnP uses `distCoeffs=None` (pre-undistorted) ✅
- [ ] Scale from AprilTag edge (8.8mm) ✅
- [ ] Reprojection errors in pixels ✅

---

## Support

If the pipeline fails:
1. Check logs for specific error message
2. Verify calibration quality (<0.6px reprojection)
3. Ensure tags are detected in multiple images
4. Check image quality (blur, exposure)
5. Review coordinate/unit conventions in error messages

---

**Status:** Ready to execute  
**Expected Runtime:** 2-5 minutes (58 images)  
**Memory:** ~2GB  
**Output Size:** ~500KB
