# Export Packages

Case export packages for CAD integration (IOS + exocad workflow).

## Package Structure

Each case export contains:

```
exports/case_<id>/
├── arch_ios.stl                # IOS scan mesh
├── implants_PG_in_I.json       # Photogrammetry implants in I-frame
├── implants_IOS_in_I.json      # exocad scan-body fit results
├── T_I_from_U.json             # U → I transform
├── qa_metrics.json             # Alignment residuals and QA
└── report.pdf                  # Full case report (transforms + residuals + QA)
```

## Workflow (Phase 6)

1. **Photogrammetry:** Capture → Process → `implants_U.json`
2. **IOS Scan:** Digital impression → `arch_ios.stl`
3. **Scan-body Fit:** exocad → `implants_I.json`
4. **Alignment:** Solve `T_I_from_U` using corresponding implant origins
5. **Export:** Package for design software

## Transform Chain

```
L (Solver) → U (User) → I (IOS/exocad)
```

- **L → U:** Reference plate alignment (Phase 4)
- **U → I:** Implant correspondence alignment (Phase 6)

## QA Checks

- Alignment residuals consistent with IOS scan-body fit quality
- Flag outlier implants (residual >200 µm)
- Cross-check scale between photogrammetry and IOS
