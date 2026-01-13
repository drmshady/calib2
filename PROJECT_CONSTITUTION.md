# Project Constitution — Dental Photogrammetry (All-on-X)

**Purpose:** single source of truth for goals, rules, and progress.  
**Role:** Senior Computer Vision Engineer specializing in Industrial Metrology.  
**Objective:** Build a Python-based photogrammetry pipeline for All-on-X dental implants with **<35 µm repeatability**.  
**Constraint:** Precision is paramount. Aesthetics are secondary.

---

## 0) Global Unit Standard (Non‑Negotiable)

- **All internal processing, outputs, and JSON files use millimeters (mm).**
- **Explicit ban:** meters (m) anywhere in the pipeline is forbidden.
- **Config hard gate:** `config.yaml` must set `force_mm_conversion: true`.

Naming rule: variable names must imply units.
- Examples: `dist_mm`, `error_px`, `scale_mm_per_unit`, `residual_um`.

---

## I) The Three Immutable Laws of Coordinates

### Law 1 — Explicit Frame Declaration
Never manipulate a point cloud without knowing its frame.

- **L-Frame (Local / Solver):** raw solver output (arbitrary origin/axes).
- **U-Frame (User):** gravity-aligned, scaled, final delivery coordinates.
- **I-Frame (IOS):** intraoral scanner mesh frame (context; can be warped).

Rule: every artifact must declare frame in filename and/or JSON metadata.
- Examples: `refpoints_L.json`, `implants_U.json`, `T_U_from_L.json`, `T_I_from_U.json`.

### Law 2 — No Implicit Units
Units are not optional. Variables and JSON fields must encode or imply units.

- **Standard:** final outputs are **mm**.
- Internal exceptions are allowed only if explicitly labeled (e.g., `normalized` image coords).

### Law 3 — Rigid Transforms Only (Implants are rigid bodies)
Implant constellations must move as rigid bodies.

- **Permitted:** rotation + translation via **4×4 homogeneous transforms** (SE(3)).
- **Conditionally permitted:** similarity (Sim(3)) **only** when establishing metric scale from a known reference (e.g., scale bar / known tag geometry). After scale is established, treat implant constellation as SE(3).
- **Forbidden:** shearing, non-linear warping (TPS/splines) on implant points.

---

## II) Coding Standards for Metrology

### Vector math
- Use **NumPy** for all geometry.
- Prefer explicit shapes in docstrings and type hints.

Example:
- Bad: `def transform(points):`
- Good: `def transform(points: np.ndarray) -> np.ndarray:  # shape (N, 3)`

### Matrices
Use **4×4** homogeneous transformation matrices for spatial moves:

Format:
- `[[r11, r12, r13, tx],
   [r21, r22, r23, ty],
   [r31, r32, r33, tz],
   [  0,   0,   0,  1]]`

### Coordinate System Convention (Mandatory)
**Standard:** Use **undistorted pixels** throughout reconstruction pipeline.

**Definition:**
- Undistorted pixels = distortion removed, K still applied
- Generated via: `cv2.undistortPoints(src, K, D, P=K)`
- Output format: Pixel coordinates (not normalized)

**Three Forbidden Patterns:**

1. **Mixing normalized/pixel coords without explicit conversion:**
   ```python
   # ❌ FORBIDDEN: Returns normalized coords
   pts_norm = cv2.undistortPoints(src, K, D)
   
   # ✅ REQUIRED: Returns undistorted pixels
   pts_undist_px = cv2.undistortPoints(src, K, D, P=K)
   ```

2. **Passing distorted coords to triangulation/bundle adjustment:**
   ```python
   # ❌ FORBIDDEN: Raw detections have distortion
   triangulate(pts_distorted, P1, P2)
   
   # ✅ REQUIRED: Undistort first
   pts_undist = cv2.undistortPoints(pts_distorted, K, D, P=K)
   triangulate(pts_undist, P1, P2)
   ```

3. **Using meters instead of millimeters for 3D points:**
   ```python
   # ❌ FORBIDDEN: Scale ambiguity
   point_3d_m = np.array([0.030, 0.015, 1.200])
   
   # ✅ REQUIRED: Millimeters only
   point_3d_mm = np.array([30.0, 15.0, 1200.0])
   ```

**Required annotations in function signatures:**
```python
def register_camera(
    points_2d_undist_px: np.ndarray,  # (N, 2) undistorted pixels
    points_3d_mm: np.ndarray,         # (N, 3) millimeters
    K: np.ndarray                     # (3, 3) intrinsic matrix
) -> Tuple[np.ndarray, np.ndarray]:
    """Coordinate types explicitly documented."""
```

### Fail-fast logic
- If `rmse_px > threshold_px`: raise `ValueError` (do not proceed).
- If `abs(det(R) - 1.0) > eps`: raise error (prevent reflection/scaling).
- If solver returns `success: false`: **EXIT(1)**; no downstream exports.
- If coordinate type ambiguous: raise `TypeError` with clarification.

---

## III) Data Structure Protocol

### Implant JSON schema (required)
All implant outputs must follow:

```json
{
  "implant_id": "string",
  "centroid_mm": [0.0, 0.0, 0.0],
  "normal_vector": [0.0, 0.0, 1.0],
  "quality_score": 0.0
}
```

Constraints:
- `centroid_mm` is always mm.
- `normal_vector` must be unit length (within tolerance).
- `quality_score` in [0.0, 1.0].

### Flag vs Cap terminology
- **Flag:** the 2D printed marker (AprilTag).
- **Cap:** the physical mount/body attached to the implant.
- **Offset:** fixed calibration vector from flag center to implant connection. Treat as a constant calibration term.

---

## IV) Validation Checklist (“Definition of Done”)

Before marking any task complete, answer all:

- **Reprojection error:** is mean RMSE < 0.5 px?
- **Scale check:** does the scale bar / known distance match within 20 µm (0.02 mm)?
- **Planarity sanity:** do implant Z values make sense? (e.g., variance < 5 mm for a nominally flat arch)
- **Visual check:** did we generate a debug STL to visualize fit?

---

## V) Run Gates (Fail-Fast)

### Image triage (Phase 2.1 “Bouncer”)
- Blur (Laplacian variance): discard if < 100.
- Saturation: discard if >10% pure-white pixels.
- Abort run if usable images < 20.

### Solver hard stop (Phase 4)
- `success == true`
- mean RMSE < 0.5 px
- scale residual < 0.02 mm
- If any fail: **EXIT(1)** and do NOT generate `implants_U.json`.

---

## VI) Visual Verification (“Ghost Model”)

- Generate `U_Constellation.stl` (virtual scan bodies / implant constellation in U).
- Overlay onto `arch_ios.stl`.
- Visually confirm drift patterns, especially at distal implants.

---

## VII) Progress (Living)

### Current phase status (from roadmap)
- Phase 0 (Frames/Transforms): ✅ COMPLETE (2026-01-13)
- Phase 1 (Camera Calibration): ✅ COMPLETE (2026-01-13)
- Phase 2 (Marker design): ✅ Digital design complete (2026-01-13); ⏳ Physical manufacturing pending
- Phase 2.1 (Image triage): ⏳ NOT STARTED
- Phase 3 (Geometric core): ⏳ NOT STARTED (ready to implement)
- Phase 4 (L→U + hard-stop gates): ⏳ NOT STARTED
- Phase 5 (Bench validation): ⏳ NOT STARTED
- Phase 6 (IOS integration + Ghost Model): ⏳ NOT STARTED

### Decisions log (append-only)
- 2026-01-13: Units standardized to mm; added image triage + solver hard-stop + ghost model requirements.
- 2026-01-13: Phase 0 (Frames/Transforms) completed - all core transform classes and tests implemented.
- 2026-01-13: Phase 1 (Camera Calibration) completed - ChArUco-based calibration with roll diversity checks implemented.

### Next tasks (short list)
- Implement `config.yaml` + loader enforcing `force_mm_conversion: true`.
- Implement Phase 2.1 image triage tool in `tools/`.
- Implement solver hard-stop enforcement in pipeline code (EXIT(1) on fail).
- Implement `U_Constellation.stl` exporter.
