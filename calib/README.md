# Camera Calibration Guide

**Equipment:** Nikon D5600 + 50mm lens @ f/16  
**Board:** ChArUco 9×6 (AprilTag36h11 markers)  
**Last Updated:** 2026-01-13

## Overview

Camera calibration determines the intrinsic parameters (focal length, principal point, distortion coefficients) required for accurate 3D reconstruction. This guide uses a ChArUco board (hybrid checkerboard + AprilTag markers) for robust corner detection and automatic magnification computation.

---

## ChArUco Board Specifications

- **Grid:** 9×6 squares (40 corners, 24 AprilTag markers)
- **Square size:** 10.0 mm
- **Marker size:** 7.0 mm (AprilTag36h11, IDs 0-23)
- **Print resolution:** 600 DPI
- **Board file:** [calib/fixtures/charuco_board_9x6.json](fixtures/charuco_board_9x6.json)

### Board Fabrication

1. **Print board at 600 DPI** on matte cardstock or photo paper
2. **Verify dimensions** with calipers: squares should measure 10.0 ± 0.1 mm
3. **Mount to rigid substrate:**
   - Aluminum plate (preferred for flatness)
   - Acrylic sheet (3mm+ thickness)
   - Foam board (budget option)
4. **Ensure flatness:** ±0.1 mm tolerance across board surface
5. **Avoid lamination:** Can introduce glare and reduce marker contrast

---

## Capture Procedure

### Camera Settings

- **Aperture:** f/16 (fixed for all calibration images)
- **ISO:** 400-800 (adjust for lighting)
- **Shutter speed:** Auto or manual (ensure sharp images, no motion blur)
- **Focus:** Manual focus on board surface
- **Format:** RAW + JPEG (use JPEG for calibration)
- **White balance:** Auto or daylight

### Lighting Setup

- **Diffuse lighting** (softbox, window light, or overcast sky)
- **Avoid direct sunlight** (creates harsh shadows)
- **No glare on markers** (check that AprilTags remain dark)
- **Even illumination** (no hot spots or dark corners)

### Image Capture Requirements

**Minimum:** 20 images  
**Recommended:** 30-40 images  
**Roll diversity:** 8+ unique orientations (required)

### Capture Protocol

1. **Position board** on flat surface or wall-mounted
2. **Fill frame** with board (~70-90% of image height)
3. **Keep board in focus** across entire frame
4. **Maintain working distance:** 150-300mm from board
5. **Capture at varied angles:**
   - Roll angles: 0°, 22.5°, 45°, 67.5°, 90°, 112.5°, 135°, 157.5°
   - Slight pitch/yaw variation (±15°)
6. **Avoid extreme angles** (>30° from perpendicular degrades accuracy)

### Roll Diversity Requirement

The calibration tool bins images into 8 angle buckets (0-180° divided into 22.5° bins). You must capture images spanning at least 8 unique bins.

**Quick method:** Capture 2-3 images at each of these board orientations:
- Horizontal (0°)
- Diagonal lower-right (22.5°)
- Diagonal (45°)
- Diagonal upper-right (67.5°)
- Vertical (90°)
- Diagonal upper-left (112.5°)
- Diagonal (135°)
- Diagonal lower-left (157.5°)

---

## Running Calibration

### Step 1: Organize Images

Create a folder with calibration images:

```bash
mkdir -p calib/capture_session_01
# Copy images to this folder
```

### Step 2: Run Calibration Tool

```bash
python tools/camera_calibration.py calib/capture_session_01 calib/camera_intrinsics.json
```

**Options:**
- `--min-images N` — Minimum valid images required (default: 20)
- `--min-corners N` — Minimum corners per image (default: 30)
- `--min-angles N` — Minimum unique roll angles (default: 8)
- `--squares-x N` — Board width in squares (default: 9)
- `--squares-y N` — Board height in squares (default: 6)
- `--square-size MM` — Square size in mm (default: 10.0)
- `--marker-size MM` — Marker size in mm (default: 7.0)

### Step 3: Validate Calibration

```bash
python src/calibration_loader.py calib/camera_intrinsics.json
```

This runs quality checks:
- ✓ Reprojection error <0.5 px
- ✓ Principal point near image center
- ✓ Aspect ratio ≈ 1.0
- ✓ Round-trip undistort→redistort <1e-3 px

---

## Quality Gates

### Reprojection Error

**Target:** <0.3 px (excellent)  
**Warning:** 0.3-0.5 px (acceptable)  
**Fail:** >0.5 px (recapture required)

**If error is high:**
- Check board flatness (most common issue)
- Verify print dimensions match specification
- Ensure images are in focus
- Check for motion blur

### Roll Diversity

**Target:** 8+ unique angle bins  
**Warning:** 6-7 bins (marginal)  
**Fail:** <6 bins (recapture with more angles)

### Corner Detection

**Per-image minimum:** 30 corners (out of 40 total)

**If detection fails:**
- Improve lighting (reduce glare)
- Move closer to board (increase marker size in pixels)
- Ensure AprilTags have high contrast (dark markers, white background)
- Check focus (markers must be sharp)

---

## Magnification Computation

The calibration tool **automatically computes magnification** from detected ChArUco corners. Magnification is stored in the calibration file as `magnification_px_per_mm`.

**Target range:** 35-50 px/mm (for 10mm features ≈ 350-500 px)

**Note:** Magnification varies with working distance. The calibration file stores the median magnification observed across all calibration images.

---

## When to Recalibrate

### Required Recalibration (intrinsics change)

- **Lens swap** (different focal length)
- **Significant magnification change** (>20% difference in working distance)
- **Focus distance change** (macro vs. normal range)
- **Lens disassembly/repair**

### No Recalibration Needed (intrinsics unchanged)

- **Aperture change** (f/16 → f/11, etc.) — distortion coefficients remain valid
- **Minor working distance variation** (±20% from calibration distance)
- **Camera body swap** (same lens)

### Verification Calibration (recommended)

Run a quick validation every 3-6 months to check for:
- Lens creep (if zoom lens)
- Mechanical drift
- Confirmation of calibration stability

---

## Calibration File Format

Output: `calib/camera_intrinsics.json`

```json
{
  "calibration_type": "charuco",
  "camera_model": "Nikon D5600",
  "lens": "50mm",
  "aperture": "f/16",
  "timestamp": "2026-01-13T10:30:00Z",
  "image_size": {
    "width": 6000,
    "height": 4000
  },
  "intrinsics": {
    "fx": 5234.56,
    "fy": 5238.91,
    "cx": 3012.34,
    "cy": 2005.67,
    "K": [[5234.56, 0, 3012.34], [0, 5238.91, 2005.67], [0, 0, 1]]
  },
  "distortion": {
    "k1": -0.123,
    "k2": 0.045,
    "p1": 0.001,
    "p2": -0.002,
    "k3": -0.012,
    "coefficients": [-0.123, 0.045, 0.001, -0.002, -0.012]
  },
  "magnification": {
    "median_px_per_mm": 42.3,
    "min_px_per_mm": 38.1,
    "max_px_per_mm": 46.7
  },
  "quality": {
    "reprojection_error_px": 0.28,
    "n_images_used": 35,
    "n_unique_roll_angles": 8
  }
}
```

---

## Troubleshooting

### Problem: No corners detected in images

**Solutions:**
- Increase board size in frame (fill 70-90% of image height)
- Improve lighting (avoid shadows and glare)
- Check AprilTag contrast (should be very dark markers on white background)
- Verify board is printed at correct size (measure with calipers)

### Problem: Insufficient roll diversity

**Solutions:**
- Capture more images at different board rotations
- Use a turntable or rotate board in 20-30° increments
- Check detected angles with `--verbose` flag (if added)

### Problem: High reprojection error (>0.5 px)

**Solutions:**
- **Check board flatness** — Most common cause of high error
- Re-print board if curled or warped
- Mount board to rigid substrate (aluminum preferred)
- Verify dimensions: squares should measure exactly 10.0 mm
- Ensure images are sharp (no motion blur, good focus)

### Problem: Principal point far from image center

**Solutions:**
- Usually indicates lens misalignment or decentering
- If offset >200 px, consider lens repair/replacement
- For small offsets (<100 px), calibration is still valid

---

## Best Practices

1. **Capture more images than minimum** — 30-40 images provides better coverage
2. **Vary distance slightly** — Helps model distortion across different magnifications
3. **Keep board planar** — Non-flat boards introduce systematic errors
4. **Avoid reflections** — Matte board surface, diffuse lighting
5. **Check each image** — Verify corners/markers detected before proceeding
6. **Save calibration sessions** — Date-stamped folders for tracking
7. **Document working distance** — Note approximate camera-to-board distance for future reference

---

## Integration with Phase 0

Calibration integrates with Phase 0 transforms via:

1. **Load calibration:**
   ```python
   from src.calibration_loader import load_calibration, undistort_points
   K, D, image_size, metadata = load_calibration('calib/camera_intrinsics.json')
   ```

2. **Undistort 2D observations before triangulation:**
   ```python
   points_distorted = np.array([[x1, y1], [x2, y2], ...])  # From image
   points_normalized = undistort_points(points_distorted, K, D)
   ```

3. **Build projection matrices for triangulation:**
   ```python
   from tools.triangulation import build_projection_matrix
   P = build_projection_matrix(K, R, t)  # For each camera pose
   ```

**Critical:** Always undistort points **before** triangulation, not after 3D reconstruction.

---

## Files in calib/

- `camera_intrinsics.json` - Current camera calibration (ChArUco)
- `fixtures/reference_plate_4tags.json` - Known U-frame geometry (AprilTag IDs 1-4)
- `fixtures/charuco_board_9x6.json` - ChArUco board specification

## References

- OpenCV ChArUco calibration: https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html
- AprilTag36h11 dictionary: https://april.eecs.umich.edu/software/apriltag.html
- Camera calibration theory: Zhang (2000), "A Flexible New Technique for Camera Calibration"

---

## Capture Procedure (Legacy)

1. Use checkerboard or AprilTag calibration board
2. Capture 20-30 images with roll diversity
3. Run `tools/camera_calibration.py`
4. Verify reprojection error <0.5px

## Verification

- Calibration reprojection mean consistent across sessions
- Principal point within 100px of image center
- Round-trip accuracy (undistort → redistort) <1e-6 px
