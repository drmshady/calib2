# Processing Runs

Output directories for each dataset processing run.

## Directory Structure

Each run creates a subdirectory with the following structure:

```
runs/<dataset>/
├── images/                     # Input images
├── detections.json             # 2D observations (raw pixel coords)
├── observations_normalized.json # Undistorted/normalized observations
├── refpoints_L.json            # Triangulated L-frame structure
├── structure_L_incremental.json # Incremental SfM output
├── ba_summary.json             # Bundle adjustment report
├── qa_report.json              # Quality assurance metrics
├── scale_proxies_by_cap.json   # Per-cap scale diagnostics
├── T_U_from_L.json             # L → U transform
└── implants_U.json             # Final implant poses in U-frame
```

## Example Datasets

- **`bench_6caps/`** - Bench validation (10× reseat protocol)
- **`case_001/`** - Clinical case example

## QA Gates

See `qa_report.json` for validation against thresholds:
- Reprojection error mean <1.0px, max <3.0px
- Track length ≥4 views per implant feature
- Scale sanity check ≤0.1mm over 100mm
- Graph connectivity (no disconnected cameras)
