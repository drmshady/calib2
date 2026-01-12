# Source Code Modules

Core photogrammetry pipeline components.

## Modules

### Phase 0: Transforms and Frames
- **`transforms.py`** - SE(3)/Sim(3) transformation classes (save/load/apply/inverse/compose)

### Phase 3: Multi-View Geometry
- **`calibration_loader.py`** - Load camera intrinsics, undistort points
- **`sfm_initialization.py`** - Two-view SfM initialization (Essential matrix + pose recovery)
- **`incremental_sfm.py`** - Incremental reconstruction (PnP + triangulation)
- **`reconstruction_qa.py`** - Quality assurance checks (reprojection, track length, connectivity)
- **`geometry_utils.py`** - Shared geometric utilities (triangulation, validation, rotation ops)

### Phase 2: Marker Models
- **`model_loader_v1.py`** - Load and parse v1.3.0 cap models

## Key Principles

1. **Lens distortion removed BEFORE triangulation** (not after)
2. **Anti-hinge rule**: Bridge frames must have ≥3 non-collinear points
3. **DOP check**: Ray angles ≥5° for stable triangulation
4. **OpenCV camera convention**: `p_cam = R @ p_world + t`
