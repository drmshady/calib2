# GUI Application - Phase 3 & 4 Reconstruction Tab

## 🎨 New Features Added

Added **Phase 3/4 Reconstruction** tab to the Camera Calibration GUI application with full integration of the multi-view reconstruction pipeline.

## 🚀 Launch GUI

```bash
cd "d:/new trial"
python tools/camera_calibration_gui.py
```

## 📋 Phase 3/4 Reconstruction Tab

### Input Fields

1. **Input Images** 
   - Browse to select image folder or enter glob pattern
   - Examples: `calib/test/*.TIF`, `data/case001/*.jpg`
   - Supports TIF, JPG, PNG formats

2. **Calibration File**
   - Select camera intrinsics JSON file
   - Example: `calib/1_10/camera_intrinsics.json`

3. **Output Directory**
   - Select folder for reconstruction results
   - Will create: `structure_L.json`, `qa_report.json`, `metadata.json`

4. **Layout File (Optional)**
   - For validation with known tag positions
   - Example: `calib/fixtures/layout_4tags.json`

### Parameters

- **AprilTag Edge Length (mm)**: Default 8.8mm
  - Set to 7.0mm for 4-tag test layout
  - Adjust based on your physical tags

### Phase 4 Options

- **☑ Enable Phase 4 (L→U Transform)**
  - Checkbox to enable user frame transformation
  
- **Method Selection:**
  - ⚪ **Reference Plate (Option U2)** - Metrology-grade (recommended)
  - ⚪ **Implant-based (Option U1)** - Uses implant geometry

- **Reference Plate File**: Required for Option U2
  - Example: `calib/fixtures/reference_plate_4tags.json`

### Controls

- **Run Reconstruction** button
  - Starts the full pipeline
  - Shows real-time progress in log window
  - Disables during execution (prevents multiple runs)

- **Progress Bar**
  - Animated during reconstruction
  - Stops when complete or on error

- **Reconstruction Log**
  - Real-time output from pipeline
  - Shows detection, SfM, BA, and QA results
  - Scrollable for long outputs

## 📸 GUI Layout

```
┌─────────────────────────────────────────────────────────────┐
│  Camera Calibration & Validation Tool                       │
├─────────────────────────────────────────────────────────────┤
│  [Calibration] [Validation (PnP + BA)] [Phase 3/4 Recon]   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Phase 3 & 4: Multi-View Reconstruction                     │
│                                                              │
│  Input Images:       [calib/test/*.TIF        ] [Browse...] │
│  Calibration File:   [calib/1_10/camera...   ] [Browse...] │
│  Output Directory:   [runs/reconstruction     ] [Browse...] │
│  Layout File:        [optional                ] [Browse...] │
│                                                              │
│  ┌─ Parameters ─────────────────────────────────────────┐  │
│  │ AprilTag Edge Length (mm): [8.8]                     │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌─ Phase 4: User Frame Transform ─────────────────────┐  │
│  │ ☑ Enable Phase 4 (L→U Transform)                    │  │
│  │ Method: ⦿ Reference Plate  ○ Implant-based          │  │
│  │ Reference Plate: [calib/fixtures/...] [Browse...]   │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  [Progress Bar ================================>          ]  │
│                                                              │
│  ┌─ Reconstruction Log ─────────────────────────────────┐  │
│  │ [2/7] Detecting AprilTags...                         │  │
│  │   DSC_0281: 4 tags detected                          │  │
│  │   DSC_0282: 4 tags detected                          │  │
│  │ [4/7] SfM initialization (two-view)...               │  │
│  │   ✅ Initialization SUCCESS                          │  │
│  │      Inliers: 16/16                                  │  │
│  │      Mean ray angle: 12.5°                           │  │
│  │ ...                                                   │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│                  [ Run Reconstruction ]                     │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## ✅ Success Workflow

### 1. Test with 4-Tag Known Layout

**Inputs:**
- Images: `calib/test/DSC_*.TIF`
- Calibration: `calib/1_10/camera_intrinsics.json`
- Output: `runs/phase3_test_4tags`
- Layout: `calib/fixtures/layout_4tags.json`
- Tag Size: `7.0` mm
- Phase 4: ☐ Disabled (for initial test)

**Expected Output:**
```
======================================================================
PHASE 3: MULTI-VIEW RECONSTRUCTION PIPELINE
======================================================================
[1/7] Loading camera calibration...
  ✅ Loaded successfully
[2/7] Detecting AprilTags...
  DSC_0281: 4 tags detected
  DSC_0282: 4 tags detected
  ... (continues)
[4/7] SfM initialization (two-view)...
  ✅ Initialization SUCCESS
[6/7] Global bundle adjustment...
  ✅ Bundle adjustment SUCCESS
[7/7] Quality assurance validation...
  ✅ [PASS] Reprojection Errors
  ✅ [PASS] Track Lengths
  ✅ [PASS] Graph Connectivity
  ✅ [PASS] Bridge Collinearity

OVERALL STATUS: PASS
======================================================================
PHASE 3 COMPLETE
======================================================================

✅ Reconstruction completed successfully!
```

### 2. Production with Phase 4 Transform

**Inputs:**
- Images: `data/case001/*.TIF`
- Calibration: `calib/1_10/camera_intrinsics.json`
- Output: `runs/case001_reconstruction`
- Tag Size: `8.8` mm
- Phase 4: ☑ **Enabled**
- Method: ⦿ Reference Plate
- Reference Plate: `calib/fixtures/reference_plate_4tags.json`

**Expected Output:**
```
... (Phase 3 output)

======================================================================
PHASE 4: USER FRAME DEFINITION (L → U TRANSFORM)
======================================================================
Method: Option U2 (Reference Plate)
Reference plate: calib/fixtures/reference_plate_4tags.json
  Reference tags: [1, 2, 3, 4]
...
✅ Scale validation passed
✅ Transform computed successfully

======================================================================
PHASE 4 COMPLETE
======================================================================

✅ Reconstruction completed successfully!
```

## 🎯 Features

### Real-time Progress
- ✅ Live log output during reconstruction
- ✅ Animated progress bar
- ✅ Step-by-step pipeline status

### Error Handling
- ✅ Input validation before starting
- ✅ Clear error messages in log
- ✅ Graceful failure with traceback
- ✅ Button re-enabled after completion

### User Experience
- ✅ Browse dialogs for all file inputs
- ✅ Tooltips and help text
- ✅ Disabled controls during execution
- ✅ Success/error message boxes
- ✅ Scrollable log for long outputs

### Integration
- ✅ Seamlessly integrates with existing Calibration and Validation tabs
- ✅ Shares same window and styling
- ✅ Thread-safe execution (doesn't freeze UI)
- ✅ Clean separation of concerns

## 🔧 Technical Details

### Architecture
- **Main Thread**: GUI rendering and user interaction
- **Worker Thread**: Reconstruction pipeline execution
- **Log Redirector**: Captures stdout to GUI log widget

### Dependencies
All existing dependencies from `requirements.txt`:
- tkinter (built-in)
- threading (built-in)
- All Phase 3/4 modules automatically imported

### File Structure
```
tools/
├── camera_calibration_gui.py    # Main GUI (now with Phase 3/4 tab)
├── camera_calibration.py        # Calibration backend
├── apriltag_pnp_ba.py           # Validation backend
└── phase3_test_pipeline.py      # Reconstruction backend (NEW)
```

## 🐛 Troubleshooting

### GUI doesn't launch
```bash
# Check Python version
python --version  # Should be 3.8+

# Check tkinter installation
python -c "import tkinter; print('tkinter OK')"

# Install if missing (Ubuntu/Debian)
sudo apt-get install python3-tk
```

### Import errors in Reconstruction tab
- Ensure all Phase 3/4 modules are in `src/` directory
- Check `sys.path` includes project root
- Verify no circular imports

### "No images found" error
- Check glob pattern syntax
- Use forward slashes: `data/*.TIF` not `data\*.TIF`
- Verify image files exist in specified folder

### Reconstruction freezes GUI
- Should NOT happen (runs in separate thread)
- If it does, check for blocking operations
- Look for missing `self.root.update()` calls

## 📚 Related Documentation

- [PHASE3_4_IMPLEMENTATION_SUMMARY.md](../PHASE3_4_IMPLEMENTATION_SUMMARY.md) - Technical details
- [PHASE3_4_QUICK_START.md](../PHASE3_4_QUICK_START.md) - Command-line usage
- [PROJECT_OVERVIEW.md](../PROJECT_OVERVIEW.md) - Project architecture

---

**Status**: ✅ **FULLY INTEGRATED**  
**Testing**: Ready for validation with real data  
**Next**: Test with 4-tag known layout, then unknown layout reconstruction
