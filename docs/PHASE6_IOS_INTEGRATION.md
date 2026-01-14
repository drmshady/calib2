# Phase 6: IOS Integration Pipeline

## Overview

Phase 6 integrates photogrammetry implant positions with intraoral scanner (IOS) data for CAD/CAM workflow. By aligning the photogrammetry U-frame with the IOS I-frame (exocad coordinate system), it enables accurate prosthetic design using both imaging modalities.

**Status:** ✅ **Complete** (25/25 tests passing)

**Quality Gates:**
- ✅ Alignment RMSE < 0.2mm (clinical grade)
- ✅ Minimum 3 implant correspondences
- ✅ All unit tests passing

---

## Architecture

### Data Flow

```
Phase 3 Output (refpoints_U.json)
         ↓
[Implant Extractor] → implants_U.json
         ↓
    IOS Data (implants_exocad.json)
         ↓
[Solve T_I_from_U] → T_I_from_U.json
         ↓                   ↓
[Generate Constellation] [Export Package]
         ↓                   ↓
U_Constellation.stl    Final Deliverable
```

### Coordinate Frames

- **U-frame:** User/photogrammetry frame (Phase 4 output, consistent reference)
- **I-frame:** IOS/exocad frame (intraoral scanner coordinate system)
- **Transform:** T_I_from_U = SE(3) rigid transformation (rotation + translation, no scale)

---

## Implementation

### 1. Implant Extraction (`src/implant_extractor.py`)

**Purpose:** Extract implant centroids and axis vectors from AprilTag marker cap corners.

**Algorithm:**
- Centroid = mean of 4 tag corners
- Axis = normal to tag plane (cross product of diagonals)
- Validation: Edge lengths ≈ 8.8mm, planarity check

**Usage:**
```bash
python src/implant_extractor.py \\
    runs/case001/refpoints_U.json \\
    runs/case001/implants_U.json \\
    8.8
```

**Output Format:** [`implant_schema.json`](../calib/fixtures/implant_schema.json)
```json
{
  "frame": "U",
  "units": "mm",
  "implants": {
    "1": {
      "centroid_mm": [x, y, z],
      "axis_vector": [nx, ny, nz],
      "marker_id": 100,
      "reprojection_error_px": 0.45
    }
  }
}
```

### 2. IOS Alignment (`tools/solve_T_I_from_U.py`)

**Purpose:** Compute SE(3) transformation from U-frame to I-frame using implant correspondences.

**Algorithm:**
- Match implants by marker_id between U-frame and I-frame
- Umeyama closed-form solution (reuses `src/transforms.py`)
- RMSE validation with 0.2mm threshold

**Usage:**
```bash
python tools/solve_T_I_from_U.py \\
    --implants-u runs/case001/implants_U.json \\
    --implants-i data/ios/implants_exocad.json \\
    --output runs/case001/T_I_from_U.json \\
    --rmse-threshold 0.2 \\
    --export-transformed
```

**Quality Gates:**
- ✓ RMSE < 0.2mm → Clinical grade
- ⚠ RMSE 0.2-0.5mm → Acceptable for most cases
- ✗ RMSE > 0.5mm → Review IOS scan quality / marker seating

**Output:**
```json
{
  "transform": "T_I_from_U",
  "R": [[...], [...], [...]],  // 3×3 rotation
  "t": [tx, ty, tz],            // 3×1 translation (mm)
  "metadata": {
    "rmse_mm": 0.15,
    "n_correspondences": 4,
    "matched_marker_ids": [100, 101, 102, 103]
  }
}
```

### 3. Constellation Visualization (`tools/generate_constellation.py`)

**Purpose:** Generate 3D STL mesh of implant positions for overlay on IOS arch mesh.

**Geometry:**
- Cylinder: 4mm diameter × 12mm height (implant body)
- Cone: 2mm base × 6mm height (axis direction indicator)
- Coordinate system: Matches input frame (U or I)

**Usage:**
```bash
python tools/generate_constellation.py \\
    --implants runs/case001/implants_U.json \\
    --output runs/case001/U_Constellation.stl \\
    --cylinder-radius 2.0 \\
    --cylinder-height 12.0
```

**CAD Workflow:**
1. Import `arch_ios.stl` (IOS mesh)
2. Import `U_Constellation.stl`
3. Visually verify implant positions
4. Use `implants_I.json` for prosthetic design

### 4. Export Package (`tools/export_ios_package.py`)

**Purpose:** Bundle all Phase 6 outputs into deliverable package for CAD/CAM workflow.

**Package Contents:**
```
case_YYYYMMDD_HHMMSS/
├── README.txt                  # Package documentation
├── T_I_from_U.json            # Transformation matrix
├── implants_U.json            # Photogrammetry implants
├── implants_I.json            # IOS-aligned implants
├── U_Constellation.stl        # Visualization mesh
├── arch_ios.stl               # IOS arch mesh (optional)
├── qa_report.json             # QA metrics
└── metadata.json              # Package metadata
```

**Usage:**
```bash
python tools/export_ios_package.py \\
    --run-dir runs/case001 \\
    --ios-mesh data/ios/arch.stl \\
    --output exports/case001_delivery \\
    --case-name patient_12345
```

---

## Testing

### Unit Tests

**Test Coverage:** 25 tests (100% passing)

```bash
pytest test/test_implant_extraction.py test/test_ios_alignment.py -v
```

**Test Suites:**

1. **`test_implant_extraction.py`** (10 tests)
   - Tag geometry: Centroid, axis, normal computation
   - Validation: Edge lengths, planarity
   - Pipeline: End-to-end extraction with mock data

2. **`test_ios_alignment.py`** (15 tests)
   - Loading: JSON parsing, schema validation
   - Matching: Correspondence by marker_id
   - Alignment: Identity, translation, rotation, rigid transform recovery
   - RMSE: Zero error, known displacement, mixed residuals
   - Application: Transform centroids and axes

### Test Fixtures

- [`test_implants_I.json`](../test/fixtures/mock_ios_data/test_implants_I.json) - Mock IOS data (4 implants)
- [`test_implants_U.json`](../test/fixtures/mock_ios_data/test_implants_U.json) - Mock U-frame data

---

## End-to-End Workflow

### Prerequisites

1. **Phase 3 Complete:**
   - `refpoints_U.json` (triangulated tag corners in U-frame)
   - Mean reprojection error < 1.0px

2. **IOS Data Available:**
   - `implants_exocad.json` (scan body positions from exocad)
   - `arch_ios.stl` (intraoral scan mesh)

### Workflow Steps

```bash
# 1. Extract implants from photogrammetry
python src/implant_extractor.py \\
    runs/case001/refpoints_U.json \\
    runs/case001/implants_U.json \\
    8.8

# 2. Solve T_I_from_U alignment
python tools/solve_T_I_from_U.py \\
    --implants-u runs/case001/implants_U.json \\
    --implants-i data/ios/implants_exocad.json \\
    --output runs/case001/T_I_from_U.json \\
    --rmse-threshold 0.2 \\
    --export-transformed

# 3. Generate constellation STL
python tools/generate_constellation.py \\
    --implants runs/case001/implants_U.json \\
    --output runs/case001/U_Constellation.stl

# 4. Export final package
python tools/export_ios_package.py \\
    --run-dir runs/case001 \\
    --ios-mesh data/ios/arch.stl \\
    --output exports/case001_delivery \\
    --case-name patient_12345
```

### Quality Validation

**Check RMSE:**
```bash
jq '.metadata.rmse_mm' runs/case001/T_I_from_U.json
# Expected: < 0.2mm for clinical applications
```

**Check Correspondences:**
```bash
jq '.metadata.n_correspondences' runs/case001/T_I_from_U.json
# Expected: ≥ 3 implants
```

**Inspect QA Report:**
```bash
cat exports/case001_delivery/qa_report.json | jq '.alignment'
```

---

## Integration with Phase 3

### Input from Phase 3

Phase 6 requires Phase 3 unknown layout output:
- `refpoints_U.json` - Tag corners in U-frame (mm)
- `qa_report.json` - Quality metrics
- `metadata.json` - Run information

### Phase 3 + Phase 6 Pipeline

If Phase 4 was enabled (known layout with reference plate):
```
Phase 3: L-frame → refpoints_L.json
         ↓
Phase 4: L→U transform → refpoints_U.json
         ↓
Phase 6: U→I alignment → Final deliverable
```

If Phase 4 not applicable (unknown layout):
```
Phase 3: U-frame (direct) → refpoints_U.json
         ↓
Phase 6: U→I alignment → Final deliverable
```

**Note:** For unknown layout, L-frame ≈ U-frame (minimal transform).

---

## Dependencies

```
numpy>=1.24.0,<2.3
scipy>=1.10.0
opencv-contrib-python>=4.8.0
matplotlib>=3.7.0
networkx>=3.0
numpy-stl>=2.16.0  # Phase 6 requirement
```

Install Phase 6 dependencies:
```bash
pip install numpy-stl>=2.16.0
```

---

## Troubleshooting

### High RMSE (>0.5mm)

**Possible Causes:**
- Poor IOS scan quality (motion artifacts, occlusion)
- Incorrect scan body seating during IOS capture
- Wrong marker_id correspondence between datasets

**Solutions:**
1. Re-scan with better scan body seating
2. Verify `marker_id` matches in both JSON files
3. Use `--rmse-threshold 0.5` if clinically acceptable
4. Check per-implant residuals in `qa_report.json`

### Insufficient Correspondences (<3)

**Causes:**
- Missing `marker_id` field in JSON
- Implant IDs don't match between U and I frames

**Solutions:**
1. Ensure both JSON files have `marker_id` field
2. Manually add `marker_id` to exocad export
3. Use at least 3 implants for stable alignment

### STL Mesh Issues

**Problems:**
- Constellation not visible in CAD software
- Incorrect scale or orientation

**Solutions:**
1. Verify units are mm in input JSON
2. Check frame field matches expectation (U or I)
3. Import both `arch_ios.stl` and `U_Constellation.stl`
4. Adjust `--cylinder-radius` if too small/large

---

## References

- [Implant Schema](../calib/fixtures/implant_schema.json) - JSON format specification
- [Transform API](../src/transforms.py) - SE(3)/Sim(3) classes
- [Phase 4 Implementation](./PHASE3_4_IMPLEMENTATION_SUMMARY.md) - L→U transform
- [Project Overview](../PROJECT_OVERVIEW.md) - System architecture

---

## Future Enhancements

1. **Automatic marker_id matching** - Use spatial proximity instead of explicit IDs
2. **IOS mesh registration** - Align arch mesh surfaces (ICP algorithm)
3. **Axis refinement** - Use IOS scan body geometry for better axis estimation
4. **Multi-modal visualization** - Overlay photogrammetry texture on IOS mesh
5. **CAD plugin** - Direct integration with exocad/3Shape
6. **Outlier detection** - Identify and flag problematic implants automatically

---

**Last Updated:** 2026-01-14  
**Status:** Production Ready  
**Tests:** 25/25 passing ✅
