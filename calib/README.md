# Camera Calibration

## Standard Capture Preset

- **Magnification:** 1:6 (default)
- **Aperture:** f/11 (default)
- **Target:** 10mm edge ≈ 380-480 px

## Calibration Rule

Recalibrate when magnification/focus distance changes (1:5 vs 1:6 vs 1:7).  
Changing f-stop alone does **not** require recalibration.

## Files

- `camera_intrinsics_1_6.json` - Intrinsics for 1:6 magnification
- `fixtures/reference_plate_4tags.json` - Known U-frame geometry (AprilTag IDs 1-4)

## Capture Procedure

1. Use checkerboard or AprilTag calibration board
2. Capture 20-30 images with roll diversity
3. Run `tools/camera_calibration.py`
4. Verify reprojection error <0.5px

## Verification

- Calibration reprojection mean consistent across sessions
- Principal point within 100px of image center
- Round-trip accuracy (undistort → redistort) <1e-6 px
