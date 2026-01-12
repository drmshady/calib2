# Documentation

Technical documentation for the photogrammetry system.

## Phase Documentation

- **`frames.md`** - Coordinate frame definitions (L, U, I) and conventions
- **`PHASE2_MANUFACTURING_SPEC.md`** - Manufacturing specification for marker caps
- **`qa_report_json.md`** - QA report JSON schema and field descriptions

## Key Concepts

### Coordinate Frames
- **L (Local/Solver)**: Bundle adjustment output (arbitrary but stable)
- **U (User)**: Reference plate aligned for repeatability
- **I (IOS/exocad)**: CAD system frame for design integration

### Transform Convention
- Column vectors: `p_dst = T_dst_from_src * p_src`
- SE(3): Rotation + translation (6 DOF)
- Sim(3): Scale + rotation + translation (7 DOF)

### Camera Convention
- OpenCV standard: `p_cam = R @ p_world + t`
- Normalized coordinates after undistortion (K=I, D=0)

## Validation Gates

### Per-Frame (Hard Fail)
- ≥3 tags visible (≥12 tag corners)
- Tag reprojection mean <2.0px, max <4.0px
- Dot inlier rate >70% after refinement

### Repeatability (U-frame, across reseats)
- Translation RMS per implant: ≤35 µm (pass), ≤25 µm (stretch)
- Translation max per implant: ≤90 µm
- Angular RMS per implant: ≤0.20°
- Angular max per implant: ≤0.50°
