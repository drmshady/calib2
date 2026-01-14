# Phase 6: IOS Integration - GUI Guide

## Overview

The Phase 6 tab in the Camera Calibration GUI provides a complete workflow for aligning photogrammetry-derived implant positions with IOS (Intraoral Scanner) scan body data for CAD/CAM integration.

## Workflow Steps

### Step 1: Extract Implants from Photogrammetry

**Purpose**: Extract implant centroids and axes from AprilTag corner positions in your photogrammetry reconstruction.

**Inputs**:
- **Input refpoints_U.json**: The reference points file from Phase 3 reconstruction (typically `runs/<run_name>/refpoints_U.json`)
- **Tag Size (mm)**: AprilTag edge length (default: 8.8mm)

**Output**:
- **Output implants_U.json**: Extracted implant data in U-frame (photogrammetry coordinate system)

**Action**: Click "Extract Implants"

**Notes**:
- If you don't have `refpoints_U.json`, you can create it from `structure_L.json`:
  ```python
  python -c "import json; s=json.load(open('runs/test2/structure_L.json')); 
  json.dump({'frame':'U','units':'mm','refpoints':{k:{'position_mm':v['position_mm'],'track_id':k} 
  for k,v in s['points_3d'].items()}}, open('runs/test2/refpoints_U.json','w'), indent=2)"
  ```

---

### Step 2: Convert IOS Scan Body Data

**Purpose**: Convert IOS scan body transformation matrices (from CloudCompare, exocad, etc.) to the standardized implant format.

**Inputs**:
- **IOS Scan Body JSON**: JSON file containing scan body 4×4 transformation matrices
  - Example format:
    ```json
    {
      "implants": [
        {
          "name": "SB01",
          "T_world_from_scanbody": [[1,0,0,x], [0,1,0,y], [0,0,1,z], [0,0,0,1]]
        }
      ]
    }
    ```
- **Marker IDs**: Comma-separated AprilTag IDs corresponding to each scan body (e.g., "100,101,102,103")

**Output**:
- **Output implants_I.json**: Converted implant data in I-frame (IOS coordinate system)

**Action**: Click "Convert IOS Data"

**Notes**:
- Marker IDs must match the physical stickers on your scan bodies
- The converter includes scan bodies with identity matrices (positioned at origin)

---

### Step 3: Compute Alignment (T_I_from_U)

**Purpose**: Compute the SE(3) rigid transformation that aligns the photogrammetry U-frame with the IOS I-frame.

**Inputs**:
- Uses `implants_U.json` and `implants_I.json` from Steps 1 and 2
- **RMSE Threshold (mm)**: Maximum acceptable alignment error (default: 5.0mm)
  - Clinical grade: < 0.2mm
  - Acceptable: < 5.0mm
  - Warning: > 5.0mm

**Output**:
- **Output T_I_from_U.json**: Transformation matrix with rotation, translation, and quality metrics

**Action**: Click "Compute Alignment"

**Quality Checks**:
- ✓ RMSE < 0.2mm: Excellent (clinical grade)
- ✓ RMSE < 5.0mm: Acceptable for CAD/CAM
- ✗ RMSE > 5.0mm: Review correspondence or data quality

**Common Issues**:
- High RMSE: Check marker ID assignments match physical labels
- Non-uniform residuals: One or more scan bodies may be misaligned
- Scale mismatch: Verify both datasets use same units (mm)

---

### Step 4: Generate Constellation STL Visualization

**Purpose**: Create 3D STL mesh visualizations of implant positions for visual inspection.

**Inputs**:
- Uses `implants_U.json` and `implants_I.json` from Steps 1 and 2

**Outputs**:
- **U-Constellation Output**: STL file showing photogrammetry implant positions
- **I-Constellation Output**: STL file showing IOS implant positions

**Action**: Click "Generate Both STLs"

**Visualization**:
- Each implant rendered as:
  - Cylinder: 4mm diameter × 12mm height (implant body)
  - Cone: 2mm base × 6mm height (axis direction indicator)
- Import both STLs into MeshLab/Blender to visually compare alignment
- After alignment, apply `T_I_from_U` to U-constellation to overlay with I-constellation

---

### Step 5: Export IOS Package

**Purpose**: Bundle all Phase 6 outputs into a complete deliverable package for CAD/CAM software.

**Inputs**:
- **Package Output Directory**: Where to save the complete package
- **Case Name**: Patient/case identifier (e.g., "Patient001")

**Package Contents**:
```
<output_dir>/
├── T_I_from_U.json              # Transformation matrix
├── implants_U.json              # Photogrammetry implants
├── implants_I.json              # IOS-aligned implants
├── U_Constellation.stl          # Visualization (U-frame)
├── I_Constellation.stl          # Visualization (I-frame)
├── qa_report.json               # Quality assessment
├── README.txt                   # Complete documentation
└── metadata.json                # Package metadata
```

**Action**: Click "Export Package"

**Deliverable Use**:
- Import into CAD/CAM software (exocad, 3Shape, etc.)
- Use `T_I_from_U` to transform photogrammetry scan to IOS coordinate system
- QA report provides alignment quality metrics for clinical validation

---

## Complete Workflow Example

### Scenario: Align 4 implants from test2 reconstruction with IOS data

1. **Photogrammetry** (Phase 3):
   - Images: `calib/test2/DSC_*.TIF`
   - Output: `runs/test2_multitag_full/structure_L.json`
   - Tags: 100, 101, 102, 103 (8.8mm)

2. **IOS Scan** (external):
   - Scan bodies aligned in CloudCompare
   - Export transformation history to JSON
   - Scan bodies labeled: SB01→100, SB02→101, SB03→102, SB04→103

3. **Phase 6 Workflow**:

   **Step 1**: Extract Implants
   - Input: `runs/test2_multitag_full/refpoints_U.json`
   - Output: `runs/test2_multitag_full/implants_U.json`
   - Tag Size: 8.8mm

   **Step 2**: Convert IOS Data
   - Input: `ios_data/scanbody_poses.json`
   - Output: `ios_data/implants_I.json`
   - Marker IDs: `100,101,102,103`

   **Step 3**: Compute Alignment
   - Output: `runs/test2_multitag_full/T_I_from_U.json`
   - RMSE Threshold: 5.0mm
   - Review: Check RMSE < 5.0mm, inspect per-implant residuals

   **Step 4**: Generate STLs
   - Output: `runs/test2_multitag_full/U_Constellation.stl`
   - Output: `ios_data/I_Constellation.stl`
   - Visual check: Import both in MeshLab, compare positions

   **Step 5**: Export Package
   - Output: `exports/test2_ios_package/`
   - Case Name: `Patient001_Test2`
   - Deliverable ready for CAD/CAM

---

## Troubleshooting

### High RMSE (> 5.0mm)

**Possible Causes**:
1. **Marker ID mismatch**: Physical label doesn't match data
   - Solution: Verify stickers on scan bodies match `marker_ids` parameter
2. **Different coordinate systems**: IOS and photogrammetry use different axes
   - Solution: Check if IOS data needs coordinate transform before conversion
3. **Scale mismatch**: Data not in same units
   - Solution: Verify both datasets in mm, check distances between implants
4. **Poor scan body seating**: Scan bodies moved between photogrammetry and IOS
   - Solution: Ensure same physical setup for both scans

**Diagnostic Steps**:
1. Generate constellation STLs and visually compare relative positions
2. Calculate pairwise distances between implants:
   ```python
   import numpy as np
   dist = np.linalg.norm(centroid_A - centroid_B)
   ```
3. Compare distances between U-frame and I-frame - should match if same setup
4. Check per-implant residuals in log - high variance indicates specific implant issue

### Missing refpoints_U.json

If Phase 3 didn't generate `refpoints_U.json`, create it from `structure_L.json`:

```bash
python -c "
import json
s = json.load(open('runs/test2/structure_L.json'))
refpoints = {
    'frame': 'U',
    'units': 'mm',
    'refpoints': {
        str(k): {
            'position_mm': v['position_mm'],
            'track_id': int(k)
        }
        for k, v in s['points_3d'].items()
    }
}
json.dump(refpoints, open('runs/test2/refpoints_U.json', 'w'), indent=2)
"
```

### IOS Data Format Issues

**CloudCompare Transformation History**:
- Matrices are typically **relative transformations**, not absolute positions
- You need either:
  - Final 3D coordinates of each scan body centroid, OR
  - Original scan body positions + transformations to compute final positions

**exocad Export**:
- Use "Export Implant Positions" feature
- Format should be world coordinates, not relative transforms

**Correct Format**:
```json
{
  "implants": [
    {
      "name": "SB01",
      "T_world_from_scanbody": [
        [R11, R12, R13, tx],
        [R21, R22, R23, ty],
        [R31, R32, R33, tz],
        [0,   0,   0,   1]
      ]
    }
  ]
}
```

Where:
- `R` = 3×3 rotation matrix (scan body orientation)
- `t` = [tx, ty, tz] = translation (scan body centroid position in mm)

---

## Quality Gates

### Alignment Quality Thresholds

| RMSE (mm) | Rating | Clinical Use | Action |
|-----------|--------|--------------|--------|
| < 0.2 | Excellent | All applications | Proceed |
| 0.2 - 1.0 | Good | Most applications | Proceed with caution |
| 1.0 - 5.0 | Acceptable | Non-critical cases | Review carefully |
| > 5.0 | Poor | Not recommended | Investigate issues |

### Per-Implant Residuals

- **Uniform residuals** (all similar): Likely global coordinate/scale issue
- **Non-uniform residuals** (one outlier): Specific implant misalignment
  - Check that implant's marker ID assignment
  - Verify scan body seating quality

---

## Integration with CAD/CAM Software

### exocad Workflow

1. Import IOS scan (`.stl` or native format)
2. Load scan body positions from IOS software
3. Import photogrammetry-derived implant positions:
   - Use `T_I_from_U.json` to transform `implants_U.json` to IOS frame
   - Or directly use `implants_I.json` (already aligned)
4. Design prosthetics with aligned implant positions
5. Export to manufacturing

### 3Shape Workflow

1. Open case with IOS scan and scan body alignment
2. Import external implant data:
   - Convert `implants_I.json` to 3Shape format
   - Apply positions to scan body objects
3. Verify alignment using constellation STL overlay
4. Proceed with prosthetic design

---

## Command-Line Alternative

For automation or scripting, Phase 6 tools can be run from command line:

```bash
# Step 1: Extract implants
python src/implant_extractor.py \
    runs/test2/refpoints_U.json \
    runs/test2/implants_U.json \
    8.8

# Step 2: Convert IOS data
python tools/convert_ios_scanbody.py \
    --input ios_data/scanbody_poses.json \
    --output ios_data/implants_I.json \
    --marker-ids 100 101 102 103 \
    --include-identity

# Step 3: Compute alignment
python tools/solve_T_I_from_U.py \
    --implants-u runs/test2/implants_U.json \
    --implants-i ios_data/implants_I.json \
    --output runs/test2/T_I_from_U.json \
    --rmse-threshold 5.0 \
    --export-transformed

# Step 4: Generate STLs
python tools/generate_constellation.py \
    --implants runs/test2/implants_U.json \
    --output runs/test2/U_Constellation.stl

python tools/generate_constellation.py \
    --implants ios_data/implants_I.json \
    --output ios_data/I_Constellation.stl

# Step 5: Export package
python tools/export_ios_package.py \
    --run-dir runs/test2 \
    --output exports/test2_package \
    --case-name Patient001
```

---

## See Also

- [PHASE6_IOS_INTEGRATION.md](PHASE6_IOS_INTEGRATION.md) - Complete Phase 6 technical documentation
- [GUI_QUALITY_GATE_GUIDE.md](GUI_QUALITY_GATE_GUIDE.md) - Quality gate workflow
- [PHASE3_4_QUICK_START.md](../PHASE3_4_QUICK_START.md) - Reconstruction workflow

---

**Version**: 1.0  
**Last Updated**: 2026-01-14
