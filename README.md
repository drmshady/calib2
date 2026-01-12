# Dental Photogrammetry System (All-on-X)

**Version:** v0.1.0  
**Status:** Under Development  
**Last updated:** 2026-01-13

Full-arch implant photogrammetry system using DSLR capture and Windows processing pipeline.

## Project Structure

```
.
├── calib/                  # Camera calibration files and reference fixtures
│   ├── fixtures/           # Known geometry reference plates
│   └── camera_intrinsics_*.json
├── src/                    # Core source code modules
│   ├── transforms.py       # SE(3)/Sim(3) transformations
│   ├── calibration_loader.py
│   ├── sfm_initialization.py
│   ├── incremental_sfm.py
│   ├── reconstruction_qa.py
│   ├── geometry_utils.py
│   └── model_loader_v1.py
├── tools/                  # Processing scripts and utilities
│   ├── camera_calibration.py
│   ├── triangulation.py
│   ├── bundle_adjustment.py
│   ├── define_user_frame.py
│   ├── phase3_test_pipeline.py
│   └── analysis_gui.py
├── test/                   # Unit and integration tests
│   ├── test_transforms.py
│   └── test_phase0_refpoints.py
├── runs/                   # Output directories for processing runs
│   └── <dataset>/
├── docs/                   # Documentation
│   ├── frames.md
│   ├── PHASE2_MANUFACTURING_SPEC.md
│   └── qa_report_json.md
├── aox-photogrammetry-flags/  # Marker design system
│   ├── schemas/            # JSON schemas
│   ├── src/                # Flag generator GUI
│   ├── docs/               # Design documentation
│   └── out_aox_flag_v2/models/  # Generated cap models
├── exports/                # Case export packages
│   └── case_<id>/
├── ROADMAP.md              # Development roadmap
└── README.md               # This file
```

## Coordinate Frames

- **L (Local/Solver)**: Bundle adjustment output frame
- **U (User)**: Consistent reporting frame (reference plate aligned)
- **I (IOS/exocad)**: CAD integration frame

## Getting Started

1. **Phase 0-1**: Setup calibration (see [calib/README.md](calib/README.md))
2. **Phase 2**: Generate marker caps (see [aox-photogrammetry-flags/](aox-photogrammetry-flags/))
3. **Phase 3**: Run detection and reconstruction pipeline
4. **Phase 4-6**: Transform to user/IOS frames

See [ROADMAP.md](ROADMAP.md) for detailed phase descriptions and validation gates.
