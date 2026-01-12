# Processing Tools

Scripts for camera calibration, detection, reconstruction, and analysis.

## Camera Calibration
- **`camera_calibration.py`** - Generate camera intrinsics from checkerboard/AprilTag images

## Phase 0: Coordinate Frames
- **`triangulation.py`** - DLT + LM triangulation from 2D observations
- **`define_user_frame.py`** - Compute T_U_from_L via reference plate alignment
- **`analysis_gui.py`** - GUI for frame definition and visualization

## Phase 2: Marker Analysis
- **`analyze_model.py`** - Comprehensive cap model analysis
- **`validate_model.py`** - Schema and constraint validation
- **`test_symmetry.py`** - Verify 7-dot constellation asymmetry

## Phase 3: Reconstruction Pipeline
- **`phase3_test_pipeline.py`** - End-to-end driver (detection → SfM → BA → QA → exports)
- **`bundle_adjustment.py`** - Global optimization with robust loss

## Usage Examples

```bash
# Camera calibration
python tools/camera_calibration.py --images calib_images/ --output calib/camera_intrinsics_1_6.json

# Full reconstruction pipeline
python tools/phase3_test_pipeline.py --dataset runs/case_001 --calib calib/camera_intrinsics_1_6.json

# Define user frame
python tools/define_user_frame.py --input runs/case_001/refpoints_L.json --output runs/case_001/T_U_from_L.json
```
