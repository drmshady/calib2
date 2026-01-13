# Project Overview: Dental Photogrammetry System

**Last Updated:** 2026-01-13  
**Version:** v0.1.0

## Executive Summary

This project implements a precision photogrammetry pipeline for All-on-X dental implant positioning with **<35 µm repeatability**. The system captures DSLR images of marker caps on implants and reconstructs 3D positions for CAD/CAM integration.

---

## Project Purpose

**Clinical Problem:** Accurate digital capture of full-arch implant positions for prosthetic design.

**Technical Solution:** Multi-view photogrammetry using AprilTag markers + dot refinement + bundle adjustment.

**Target Accuracy:** Sub-35 micron repeatability (0.035 mm) for clinical reliability.

---

## Core Technologies

- **Language:** Python 3.x
- **Computer Vision:** OpenCV, AprilTag detection
- **Optimization:** SciPy (Levenberg-Marquardt, bundle adjustment)
- **Geometry:** NumPy (SE(3)/Sim(3) transforms)
- **Hardware:** Nikon D5600 DSLR + 50mm f/16 lens

---

## Coordinate Frame Philosophy

The project uses three explicit coordinate frames to eliminate ambiguity:

| Frame | Name | Purpose | Origin |
|-------|------|---------|--------|
| **L** | Local/Solver | Bundle adjustment workspace | Arbitrary (solver determined) |
| **U** | User | Consistent reporting frame | Reference plate (Tags 1-4) |
| **I** | IOS/exocad | CAD integration | Intraoral scanner |

**Transform chain:** L → U → I

**Critical rule:** Every file must declare its frame (e.g., `refpoints_L.json`, `implants_U.json`)

---

## Unit Standard (Non-Negotiable)

**ALL coordinates MUST be in millimeters (mm).**

- ✅ Allowed: mm
- ❌ Forbidden: meters (m)
- **Enforcement:** `config.yaml` must include `force_mm_conversion: true`
- **Rationale:** Dental precision requires sub-millimeter accuracy

---

## Implementation Status

### ✅ Phase 0: Frames & Transforms (COMPLETE)
**Date:** 2026-01-13

**Deliverables:**
- `src/transforms.py` — SE(3)/Sim(3) transform classes (527 lines)
- `tools/triangulation.py` — DLT + LM triangulation
- `tools/define_user_frame.py` — L→U transform computation
- `tools/bundle_adjustment.py` — Optimization + L-frame export
- `docs/frames.md` — Frame definitions (296 lines)
- `test/test_transforms.py` — Transform validation tests
- `test/test_phase0_refpoints.py` — Triangulation tests
- `calib/fixtures/reference_plate_4tags.json` — U-frame geometry

**Key Features:**
- Umeyama algorithm for optimal Sim(3) alignment
- Deterministic U-frame from reference plate corners
- RMSE validation (warn: 0.1mm, fail: 0.5mm)
- OpenCV camera convention

---

### ✅ Phase 1: Camera Calibration (COMPLETE)
**Date:** 2026-01-13

**Deliverables:**
- `tools/camera_calibration.py` — ChArUco calibration tool (462 lines)
- `src/calibration_loader.py` — Load/undistort utilities (367 lines)
- `calib/README.md` — Capture procedure guide (335 lines)
- `calib/fixtures/charuco_board_9x6.json` — Board specification
- `test/test_calibration.py` — 14 unit tests (all passing)

**Key Features:**
- 9×6 ChArUco board (10mm squares, 7mm AprilTag36h11)
- Roll diversity requirement (8+ unique angles)
- Auto-magnification detection
- Reprojection error <0.5px gate
- Calibration at 1:6 magnification, f/16

**Status:** ⏳ Physical calibration capture pending (requires printed board)

---

### ⏳ Phase 2: Marker Cap Design (DIGITAL COMPLETE)
**Date:** 2026-01-13

**Deliverables (Digital):**
- `aox-photogrammetry-flags/schemas/flag_schema_v1.3.0.json` — JSON schema
- `aox-photogrammetry-flags/src/aox_flag_generator_gui_v2.py` — GUI generator
- `aox-photogrammetry-flags/out_aox_flag_v2/models/` — 5 example caps (IDs 100-104)
- `src/model_loader_v1.py` — Model loader
- `tools/analyze_model.py` — Model analysis
- `tools/validate_model.py` — Schema validation
- `docs/PHASE2_MANUFACTURING_SPEC.md` — Manufacturing handoff

**Cap Specification:**
- **Dimensions:** 10×10×10 mm cubic
- **Front face:** AprilTag36h11 (8.8mm edge)
- **Top/Left/Right faces:** 7-dot asymmetric constellations
- **Total features:** 25 points (4 corners + 21 dots)
- **Dots:** Ø 1.0mm (standard) + Ø 1.5mm (anchor)
- **Material:** Anodized aluminum + laser engraving

**Status:** ⏳ Physical manufacturing pending (CNC + anodizing + engraving)

---

### ⏳ Phase 2.1: Image Triage (NOT STARTED)
**Purpose:** Reject low-quality images before detection

**Planned Checks:**
- Blur detection (Laplacian variance <100 → reject)
- Saturation check (>10% pure white → reject)
- Minimum viable images (abort if <20 usable)

---

### ⏳ Phase 3: Geometric Core (NOT STARTED)
**Purpose:** Multi-view reconstruction with <1.0px reprojection

**Planned Implementation:**
- `src/sfm_initialization.py` — Two-view SfM (Essential matrix)
- `src/incremental_sfm.py` — Incremental PnP + triangulation
- `src/reconstruction_qa.py` — Quality gates
- `src/geometry_utils.py` — Shared utilities
- `tools/phase3_test_pipeline.py` — End-to-end pipeline

**Key Constraints:**
- Undistort points BEFORE triangulation
- DOP check: ray angles ≥5°
- Anti-hinge rule: ≥3 non-collinear points per bridge frame
- QA gates: reprojection <1.0px mean, track length ≥4 views

**Status:** Ready to implement (dependencies complete)

---

### ⏳ Phase 4: User Frame Definition (NOT STARTED)
**Purpose:** Transform L-frame results to consistent U-frame

**Dependencies:** Phase 3 complete

---

### ⏳ Phase 5: Bench Validation (NOT STARTED)
**Purpose:** Validate repeatability with physical test

**Protocol:**
- 6 caps, 10× reseat cycles
- Left/Right/Anterior + bridge frames per reseat
- Track repeatability metrics

**Target:** Translation RMS ≤35 µm, Angular RMS ≤0.20°

---

### ⏳ Phase 6: IOS Integration (NOT STARTED)
**Purpose:** Align photogrammetry with intraoral scan

**Planned:**
- `tools/solve_T_I_from_U.py` — U↔I alignment
- Ghost model visualization (implant constellation overlay)
- Export package creation

---

## Critical Design Rules

### 1. Explicit Frame Declaration
**Never manipulate points without knowing their frame.**

❌ Bad: `points.json`  
✅ Good: `refpoints_L.json`, `implants_U.json`

### 2. Rigid Body Constraint
**Implant constellations move as rigid bodies (SE(3)).**

- ✅ Allowed: Rotation + translation
- ✅ Conditionally allowed: Similarity (Sim(3)) for scale establishment only
- ❌ Forbidden: Shearing, non-linear warping (TPS/splines)

### 3. Fail-Fast Logic
**Do not proceed with invalid data.**

```python
if rmse_px > threshold_px:
    raise ValueError("Reprojection error exceeds threshold")
if abs(np.linalg.det(R) - 1.0) > eps:
    raise ValueError("Invalid rotation matrix (reflection/scaling detected)")
```

### 4. Quality Gates
**Every processing stage has validation checks.**

- Image triage: blur, saturation, count
- Detection: ≥3 tags per frame, ≥12 corners
- Reconstruction: reprojection <1.0px, track length ≥4
- Alignment: RMSE <0.5mm
- Bench: repeatability <35 µm

---

## Project File Structure

```
.
├── src/                    # Core modules
│   ├── transforms.py       # ✅ SE(3)/Sim(3) classes
│   ├── calibration_loader.py  # ✅ Undistortion
│   └── model_loader_v1.py  # ✅ Cap model loader
├── tools/                  # Processing scripts
│   ├── camera_calibration.py   # ✅ Calibration
│   ├── triangulation.py    # ✅ DLT + LM
│   ├── bundle_adjustment.py    # ✅ BA + export
│   └── define_user_frame.py    # ✅ L→U transform
├── test/                   # Unit tests
│   ├── test_transforms.py  # ✅ Transform validation
│   ├── test_calibration.py # ✅ 14 passing tests
│   └── test_phase0_refpoints.py  # ✅ Triangulation
├── calib/                  # Calibration data
│   ├── fixtures/           # ✅ Reference geometries
│   └── camera_intrinsics_*.json
├── docs/                   # Documentation
│   ├── frames.md           # ✅ Frame definitions
│   └── PHASE0_COMPLETE.md  # ✅ Phase 0 report
├── aox-photogrammetry-flags/   # Marker design system
│   ├── schemas/            # ✅ v1.3.0 schema
│   ├── src/                # ✅ GUI generator
│   └── out_aox_flag_v2/models/  # ✅ Example caps
├── runs/                   # Processing outputs
└── exports/                # Final deliverables
```

---

## Quick Start Guide

### 1. Install Dependencies
```bash
pip install -r requirements.txt
# numpy, scipy, opencv-python, apriltag, matplotlib
```

### 2. Review Documentation
- Start: [PROJECT_CONSTITUTION.md](PROJECT_CONSTITUTION.md) — Core principles
- Frames: [docs/frames.md](docs/frames.md) — Coordinate conventions
- Setup: [PROJECT_SETUP.md](PROJECT_SETUP.md) — Implementation checklist
- Roadmap: [ROADMAP.md](ROADMAP.md) — Detailed phase descriptions

### 3. Run Tests
```bash
# Phase 0 tests
python -m pytest test/test_transforms.py
python -m pytest test/test_phase0_refpoints.py

# Phase 1 tests
python -m pytest test/test_calibration.py
```

### 4. Next Steps (Phase 1 Physical Capture)
1. Print ChArUco board (600 DPI, 9×6 grid, 10mm squares)
2. Capture 20-30 calibration images with roll diversity
3. Run calibration tool:
   ```bash
   python tools/camera_calibration.py calib/capture_session_01 calib/camera_intrinsics_1_6.json
   ```

### 5. Future: Phase 3 Implementation
Once physical caps are manufactured:
1. Implement `src/sfm_initialization.py`
2. Implement `src/incremental_sfm.py`
3. Build end-to-end pipeline
4. Test with synthetic data

---

## Key Design Decisions

### Why ChArUco instead of checkerboard?
- More robust corner detection
- Works with partial occlusion
- Auto-identifies corners (no ambiguity)
- AprilTags provide redundancy

### Why AprilTag36h11?
- Good balance: 36 codes, 11-bit error correction
- Robust to blur and lighting variations
- Industry standard in robotics/photogrammetry

### Why millimeters everywhere?
- Dental implants: ~4-5mm diameter
- Target precision: 0.035mm (35 µm)
- Meters would require 6+ decimal places (error-prone)
- Millimeters keep values in 0.001-100 range (clean)

### Why three frames (L/U/I)?
- **L-frame:** Solver needs arbitrary workspace
- **U-frame:** Clinical users need consistent reference
- **I-frame:** CAD/CAM needs scanner alignment
- Explicit transforms eliminate coordinate confusion

---

## Validation Philosophy

### "Definition of Done" Checklist

Before marking any task complete:

- [ ] Reprojection error: mean RMSE <0.5px?
- [ ] Scale check: known distance within 20 µm?
- [ ] Planarity sanity: implant Z variance <5mm?
- [ ] Visual check: debug STL generated?
- [ ] Unit test: all assertions pass?
- [ ] Code review: follows naming conventions?

### Hard Stop Gates (EXIT(1) if fail)

**Image Triage:**
- Blur score <100
- Saturation >10%
- Usable images <20

**Solver:**
- `success == false`
- Mean RMSE >0.5px
- Scale residual >0.02mm

**Bench Validation:**
- Translation RMS >35 µm
- Angular RMS >0.20°

---

## Common Pitfalls to Avoid

1. **❌ Unit confusion:** Mixing mm and meters
2. **❌ Frame ambiguity:** Not labeling coordinate frame
3. **❌ Late distortion:** Triangulating before undistortion
4. **❌ Reflection:** det(R) = -1 (mirrored geometry)
5. **❌ Silent failures:** Proceeding with bad data
6. **❌ Collinear points:** Degenerate triangulation geometry

---

## Resources

- **Project Constitution:** [PROJECT_CONSTITUTION.md](PROJECT_CONSTITUTION.md)
- **Implementation Plan:** [PROJECT_SETUP.md](PROJECT_SETUP.md)
- **Detailed Roadmap:** [ROADMAP.md](ROADMAP.md)
- **Frame Definitions:** [docs/frames.md](docs/frames.md)
- **Calibration Guide:** [calib/README.md](calib/README.md)
- **Phase 0 Report:** [docs/PHASE0_COMPLETE.md](docs/PHASE0_COMPLETE.md)

---

## Contact & Maintenance

**Role:** Senior Computer Vision Engineer (Industrial Metrology)  
**Objective:** <35 µm repeatability for clinical reliability  
**Constraint:** Precision is paramount; aesthetics are secondary  
**Philosophy:** Fail-fast validation at every stage
