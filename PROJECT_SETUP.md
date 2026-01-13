# Project Setup Checklist

## Phase 0: Standards and Frames ✅ READY TO IMPLEMENT

### Required Files
- [ ] `src/transforms.py` - SE(3)/Sim(3) transformation classes
- [ ] `tools/triangulation.py` - DLT + LM triangulation
- [ ] `tools/define_user_frame.py` - Compute T_U_from_L
- [ ] `calib/fixtures/reference_plate_4tags.json` - Known U-frame geometry (Tag IDs 1-4)
- [ ] `test/test_transforms.py` - Transform correctness tests
- [ ] `test/test_phase0_refpoints.py` - Triangulation tests
- [ ] `docs/frames.md` - Frame definitions and conventions

### Dependencies
```bash
pip install numpy scipy opencv-python apriltag
```

---

## Phase 1: Camera Calibration ⏳ NEXT

### Required Files
- [ ] `tools/camera_calibration.py` - Calibration script
- [ ] `calib/camera_intrinsics_1_6.json` - Intrinsics for 1:6 magnification
- [ ] `calib/README.md` - Capture procedure (ALREADY CREATED ✅)

### Capture Requirements
- Magnification: 1:6 (10mm edge ≈ 380-480 px)
- Aperture: f/11
- 20-30 images with roll diversity
- Target: reprojection error <0.5px

---

## Phase 2: Marker Cap Design ✅ DIGITAL DESIGN COMPLETE

### Status
Digital design and tooling complete (2026-01-13). Physical manufacturing pending.

### Generated Files (Example Caps)
- ✅ `aox-photogrammetry-flags/schemas/flag_schema_v1.3.0.json`
- ✅ `aox-photogrammetry-flags/src/aox_flag_generator_gui_v2.py`
- ✅ `aox-photogrammetry-flags/src/defaults_aox_flag.json`
- ✅ Cap models (IDs 100-104) in `out_aox_flag_v2/models/`

### Tooling
- ✅ `src/model_loader_v1.py` - Load v1.3.0 models
- ✅ `tools/analyze_model.py` - Model analysis
- ✅ `tools/validate_model.py` - Schema validation
- ✅ `tools/test_symmetry.py` - Asymmetry verification

### Next Step
- [ ] Physical manufacturing (CNC + anodizing + laser engraving)

---

## Phase 3: Geometric Core & Reconstruction ✅ READY TO IMPLEMENT

### Required Files
- [ ] `src/calibration_loader.py` - Load intrinsics, undistort points
- [ ] `src/sfm_initialization.py` - Two-view SfM (Essential matrix)
- [ ] `src/incremental_sfm.py` - Incremental reconstruction (PnP + triangulation)
- [ ] `src/reconstruction_qa.py` - QA checks
- [ ] `src/geometry_utils.py` - Shared utilities
- [ ] `tools/phase3_test_pipeline.py` - End-to-end pipeline
- [ ] `tools/bundle_adjustment.py` - Global optimization

### Key Features
- Undistort points BEFORE triangulation
- DOP check: ray angles ≥5°
- Anti-hinge rule: bridge frames with ≥3 non-collinear points
- QA gates: reprojection <1.0px mean, track length ≥4 views

---

## Phase 4: User Frame Definition ⏳ AFTER PHASE 3

### Required Files
- [ ] `tools/define_user_cs.py` - Define U-frame from implants or reference plate
- [ ] Integration with `tools/phase3_test_pipeline.py` (--phase4-method flag)

---

## Phase 5: Bench Validation ⏳ AFTER PHASE 4

### Protocol
- 6 caps, 10× reseat
- Left / Right / Anterior + bridge frames per reseat
- Track repeatability (translation RMS ≤35 µm, angular RMS ≤0.20°)

---

## Phase 6: IOS Integration ⏳ FUTURE

### Required Files
- [ ] `tools/solve_T_I_from_U.py` - Align U ↔ I frames
- [ ] Export package creation

---

## Quick Start Order

1. **Install Python dependencies**
   ```bash
   pip install numpy scipy opencv-python apriltag matplotlib
   ```

2. **Implement Phase 0** (Standards and Frames)
   - Start with `src/transforms.py`
   - Add `tools/triangulation.py`
   - Create reference plate fixture JSON
   - Add tests

3. **Implement Phase 1** (Camera Calibration)
   - Write `tools/camera_calibration.py`
   - Capture calibration images
   - Generate `calib/camera_intrinsics_1_6.json`

4. **Implement Phase 3** (Core Pipeline)
   - `src/calibration_loader.py` → `src/sfm_initialization.py` → `src/incremental_sfm.py`
   - Add QA checks
   - Build end-to-end pipeline

5. **Test with synthetic data**
   - Create test dataset in `runs/synthetic_001/`
   - Validate reprojection errors
   - Check scale accuracy

6. **Physical capture** (requires manufactured caps from Phase 2)
   - Bench validation (6 caps, 10× reseat)
   - Clinical cases
