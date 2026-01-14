# Phase 3 & 4 Quick Start Guide

## 🚀 Running the Pipeline

### Test with 4 Known Tags (Validation)

```bash
# Basic test with 4-tag layout
python tools/phase3_test_pipeline.py \
    --images "calib/test/DSC_*.TIF" \
    --calib "calib/1_10/camera_intrinsics.json" \
    --output "runs/phase3_test_4tags" \
    --layout "calib/fixtures/layout_4tags.json" \
    --tag-size 7.0
```

### Production with Unknown Layout (Recommended)

```bash
# Unknown layout reconstruction - L-frame output is final
python tools/phase3_unknown_layout_pipeline.py \
    --images "calib/test2/DSC_*.TIF" \
    --calib "calib/test2/camera_intrinsics.json" \
    --output "runs/unknown_reconstruction" \
    --tag-size 8.8

# Output: structure_L.json with metric scale (mm)
# NO --phase4 FLAG - L-frame is your final deliverable
```

### Known Layout with Phase 4

```bash
# Only use Phase 4 when you have physically measured reference geometry
python tools/phase3_test_pipeline.py \
    --images "data/case001/*.TIF" \
    --calib "calib/1_10/camera_intrinsics.json" \
    --output "runs/case001_reconstruction" \
    --layout "calib/fixtures/layout_4tags.json" \
    --phase4 reference_plate \
    --reference-plate "calib/fixtures/reference_plate_4tags.json" \
    --tag-size 7.0
```

## 📋 Command-Line Arguments

### Required
- `--images` : Glob pattern for input images (e.g., "path/to/*.TIF")
- `--calib` : Camera calibration JSON file
- `--output` : Output directory for results

### Optional
- `--layout` : Known tag layout JSON (for validation)
- `--tag-size` : AprilTag edge length in mm (default: 8.8)
- `--phase4` : Enable Phase 4 (`reference_plate` or `implant_based`)
- `--reference-plate` : Reference plate JSON (required if using reference_plate method)
- `--verbose` : Verbose output (default: True)

## 📂 Output Structure

```
runs/your_output_dir/
├── structure_L.json       # L-frame 3D reconstruction
│   ├── cameras            # Camera poses (R, t, K)
│   ├── points_3d          # 3D point positions with observations
│   └── summary            # Statistics (n_cameras, n_points, track lengths)
│
├── qa_report.json         # Quality assurance validation
│   ├── overall_status     # PASS/WARN/FAIL
│   ├── checks             # Individual check results
│   ├── hard_failures      # List of failed gates
│   └── warnings           # List of warning gates
│
└── metadata.json          # Pipeline metadata
    ├── n_images
    ├── n_cameras_registered
    ├── n_points_3d
    ├── ba_info            # Bundle adjustment stats
    └── qa_passed          # Boolean
```

## ✅ Success Indicators

### Console Output (Normal)
```
======================================================================
PHASE 3: MULTI-VIEW RECONSTRUCTION PIPELINE
======================================================================
Images: 9
Calibration: calib/1_10/camera_intrinsics.json
Output: runs/phase3_test_4tags

[1/7] Loading camera calibration...
  Camera matrix K: fx=24568.1, fy=24612.2, cx=3208.5, cy=2146.3
  Image size: 6000 x 4000

[2/7] Detecting AprilTags...
  DSC_0281: 4 tags detected
  DSC_0282: 4 tags detected
  ... (continues)

[4/7] SfM initialization (two-view)...
  ✅ Initialization SUCCESS
     Inliers: 16/16
     Mean ray angle: 12.5°
     Reprojection error: 0.45px

[6/7] Global bundle adjustment...
  ✅ Bundle adjustment SUCCESS
     Initial cost: 2.345
     Final cost: 0.123
     Cost reduction: 2.222
  After BA: mean=0.85px, max=1.98px

[7/7] Quality assurance validation...
✅ [PASS] Reprojection Errors
✅ [PASS] Track Lengths
✅ [PASS] Graph Connectivity
✅ [PASS] Bridge Collinearity

OVERALL STATUS: PASS
✅ Reconstruction PASSED all hard gates

======================================================================
PHASE 3 COMPLETE
======================================================================
```

### QA Report (qa_report.json)
```json
{
  "overall_status": "PASS",
  "passed": true,
  "hard_failures": [],
  "warnings": [],
  "checks": [
    {
      "name": "Reprojection Errors",
      "status": "PASS",
      "message": "Reprojection error PASSED: mean=0.85px, max=1.98px",
      "details": {
        "mean_error_px": 0.85,
        "max_error_px": 1.98,
        "n_points": 16
      }
    }
  ]
}
```

## ❌ Common Issues & Solutions

### Issue: No tags detected
```
[WARNING] DSC_0281: No tags detected
```
**Causes:**
- Wrong tag family (expecting tag36h11)
- Tag size too small in image
- Image quality poor (blur, lighting)
- Tags outside field of view

**Solutions:**
- Verify tags are tag36h11 family
- Check tag size parameter (--tag-size)
- Improve image quality
- Ensure tags visible in images

### Issue: Initialization failed
```
SfM initialization failed: Insufficient inliers: 25.0% < 50.0%
```
**Causes:**
- Too few matching points between views
- Poor camera baseline (too parallel)
- Incorrect calibration

**Solutions:**
- Use images with more parallax
- Check camera calibration quality
- Try different image pair for initialization

### Issue: Bundle adjustment failed
```
Bundle adjustment FAILED: Jacobian is rank deficient
```
**Causes:**
- Degenerate geometry (cameras in line)
- Too few observations
- Bad initial estimates

**Solutions:**
- Ensure varied camera positions
- Add more images/views
- Check SfM initialization quality

### Issue: QA hard gate failure
```
❌ [FAIL] Reprojection Errors
Reprojection error FAILED: mean=1.85px (>1.0px)
```
**Causes:**
- Poor calibration
- Scene motion during capture
- Incorrect intrinsics

**Solutions:**
- Re-run camera calibration
- Ensure static scene
- Verify calibration file matches camera/magnification

## 🔍 Debugging Tips

### 1. Check Detection First
Test detection on single image:
```python
import cv2
import numpy as np

# Load image
img = cv2.imread("calib/test/DSC_0281.TIF", cv2.IMREAD_GRAYSCALE)

# Setup detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(aruco_dict)

# Detect
corners, ids, _ = detector.detectMarkers(img)
print(f"Detected {len(ids) if ids is not None else 0} tags")
```

### 2. Validate Calibration
```python
from src.calibration_loader import load_calibration

K, D, image_size, metadata = load_calibration("calib/1_10/camera_intrinsics.json")
print(f"fx={K[0,0]:.1f}, fy={K[1,1]:.1f}")
print(f"Calibration quality: {metadata.get('mean_reprojection_error', 'N/A')}")
```

### 3. Check Module Imports
```bash
cd "d:/new trial"
python -c "from src.sfm_initialization import *; print('✅ Imports OK')"
```

### 4. View Structure JSON
```python
import json

with open("runs/phase3_test_4tags/structure_L.json") as f:
    data = json.load(f)

print(f"Cameras: {len(data['cameras'])}")
print(f"3D Points: {len(data['points_3d'])}")
print(f"Summary: {data['summary']}")
```

## 📊 Performance Optimization

### Large Image Sets (>20 images)
- Use subset for initialization (first 5-10 images)
- Incremental adding of remaining views
- Consider downsampling images for faster detection

### High-Resolution Images (>10 MP)
- Detection takes longer (~2-3 sec/image)
- Consider resizing for detection only
- Keep full resolution for final reconstruction

### Many Tags (>10)
- More feature tracks = longer BA optimization
- Consider robust loss functions
- May need more iterations for convergence

## 🎯 Quality Targets

| Metric | Target | Excellent | Acceptable | Poor |
|--------|--------|-----------|------------|------|
| Mean Reproj Error | <1.0px | <0.5px | 0.5-1.0px | >1.0px |
| Max Reproj Error | <3.0px | <2.0px | 2.0-3.0px | >3.0px |
| Track Length | ≥4 views | ≥6 views | 4-5 views | <4 views |
| Mean Ray Angle | ≥5° | ≥10° | 5-10° | <5° |
| Triangle Area | ≥10mm² | ≥20mm² | 10-20mm² | <10mm² |

## 📝 Workflow Summary

```
Input Images → Detection → Feature Tracks → SfM Init → Incremental → BA → QA → Export
     ↓             ↓            ↓              ↓           ↓         ↓    ↓      ↓
  TIF/JPG      OpenCV       Track Map      2 views     PnP+Tri   Opt  Gates  JSON
               ArUco                       Essential              Huber
                                           Matrix                 Loss
```

---

**For detailed implementation:** See [PHASE3_4_IMPLEMENTATION_SUMMARY.md](PHASE3_4_IMPLEMENTATION_SUMMARY.md)  
**For project overview:** See [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)  
**For validation results:** Check `runs/your_output_dir/qa_report.json`
