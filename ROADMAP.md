# Dental Photogrammetry Roadmap (All-on-X)

**Version:** v0.1.0  
**Status:** Under Development  
**Last updated:** 2026-01-13  
**Primary goal:** Full-arch implant photogrammetry (DSLR capture + Windows processing)  
**Marker strategy:** AprilTag36h11 (seed) + dots (refine) + BA  
**Frames:** L (Local) → U (User) → I (IOS/exocad)

## Key validation gates (current)
- Per frame: ≥3 tags visible (≥12 tag corners)
- Per implant: ≥4 views min (6–10 recommended)
- Capture sets: Left / Right / Anterior + bridge frames
- Independent scale sanity checks
- Bench gate: 6-cap reseat repeatability (10×), pass thresholds defined in Phase 5

---

## Phase 0 — Standards, naming, and "what frame am I in?"
**Goal:** eliminate ambiguity early; every file and result declares its coordinate system.
**Status:** ✅ COMPLETE (2026-01-13)

### Unit Constitution (global, non-negotiable)
Your coordinate frames are meaningless without consistent units.

**Global unit standard:** ALL internal processing, outputs, and JSONs must use **millimeters (mm)**.

**Explicit ban:** The use of **meters (m)** anywhere in the pipeline is forbidden.

**Verification (hard gate):** `config.yaml` must include `force_mm_conversion: true`.

### Define 3 frames (always explicit)
**L (Local / Solver frame)**
- Coordinate system BA outputs (true solver output from triangulation)
- Used internally for optimization and diagnostics
- **Critical:** L-frame geometry comes from triangulated 3D points, NOT CAD model

**U (User frame)**
- Consistent reporting/comparisons/bench validation/exports
- "V-STARS WINTRANS equivalent": transform L → U

**I (IOS / exocad frame)**
- Coordinate system of the IOS mesh and exocad scan-body fit
- Transform U ↔ I for CAD integration

### Core deliverables
- `docs/frames.md` (definitions + conventions)
- `src/transforms.py` (Sim3/SE3 transform class with save/load/apply/inverse/compose)
- `tools/triangulation.py` (DLT + LM triangulation from 2D observations)
- `tools/bundle_adjustment.py` exports `refpoints_L.json` (triangulated solver output)
- `tools/define_user_frame.py` (compute T_U_from_L via reference plate alignment)
- `calib/fixtures/reference_plate_4tags.json` (known U-frame geometry)
- GUI integration in `tools/analysis_gui.py` ("Define U Frame" button)
- `runs/<dataset>/refpoints_L.json` (true L-frame from triangulation)
- `runs/<dataset>/T_U_from_L.json` output format
- `test/test_phase0_refpoints.py` (triangulation correctness tests)
- `test/test_transforms.py` (SE3/Sim3 correctness tests)
- `runs/<dataset>/T_I_from_U.json` (Phase 6 — not yet implemented)

### Transform features
- **SE(3) rigid transforms:** Rotation + translation (6 DOF)
- **Sim(3) similarity transforms:** Scale + rotation + translation (7 DOF)
- **Deterministic U-frame:** Reference plate with AprilTag corners (IDs 1-4)
- **Corner ID format:** `tag_id + "_" + corner_label` (e.g., "1_TL", "2_BR")
- **Umeyama algorithm:** Optimal similarity transform from point correspondences
- **RMSE validation:** Alignment quality gates (warn: 0.1mm, fail: 0.5mm)
- **Scale detection:** Auto-detect if scale varies across runs (>0.05% triggers Sim3)
- **Triangulation:** DLT + Levenberg-Marquardt refinement (5px outlier threshold)
- **Camera convention:** OpenCV standard `p_cam = R @ p_world + t`

### Implementation details (Phase 0 Option B)
**Critical Fix:** Bundle adjustment now exports TRUE solver output frame:
- BA optimizes camera poses (rvecs, tvecs) but not 3D landmark positions
- Tag corners triangulated from 2D observations + BA-solved camera poses
- `refpoints_L.json` contains triangulated 3D points in true L-frame
- Legacy `geometry_L_MODEL` (CAD model) retained for backward compatibility with deprecation warning
- `define_user_frame.py` prefers `refpoints_L.json` over legacy model coordinates
- Validation: det(R)≈1, scale≈1 checks, per-corner residuals logged

### Notes
- Scale is metric from AprilTag geometry (7.0mm coded size + known layout)
- Reference plate tags reserved as IDs 1-4 (implant caps use 5+)
- Transform convention: column vectors, `p_dst = T_dst_from_src * p_src`
- Minimum 2 views per corner required for triangulation
- RMSE threshold: 5px for outlier rejection during triangulation

---

## Phase 1 — Camera calibration (ChArUco + roll diversity)
**Goal:** compute intrinsics reliably to handle distortion in DLT + triangulation.  
**Status:** ✅ COMPLETE (2026-01-13)

### Background
- Use ChArUco (checkerboard + AprilTags) for robust marker detection
- **8+ unique roll angles** in ≥20 cal images (avoids degeneracy)
- Capture 1:6 magnification images with **f/16** (replicates production settings)

### Calibration constraints
- Fixed focal length (50 mm prime)
- Fixed f-stop (f/16 in production)

---

## Coordinate System Architecture Rules
**Goal:** Prevent coordinate/unit mismatches that cause reconstruction failures.  
**Status:** ✅ ESTABLISHED (2026-01-13) — Lessons from Phase 3 debugging

### The Two "Undistorted" Conventions

**Problem:** The term "undistorted coordinates" is ambiguous and causes critical bugs.

**Convention 1: Normalized Coordinates** (Epipolar geometry textbooks)
- Function: `cv2.undistortPoints(src, K, D)` — default behavior
- Output: Dimensionless coordinates with K=I, D=0
- Example: (0.05, 0.03) for a point near the optical axis
- Use case: Essential matrix estimation, epipolar constraint verification

**Convention 2: Undistorted Pixels** (Practical reconstruction)
- Function: `cv2.undistortPoints(src, K, D, P=K)` — explicit P=K parameter
- Output: Pixel coordinates with distortion removed but K still applied
- Example: (3124.5, 2045.8) in pixel space
- Use case: Projection, reprojection error computation, bundle adjustment

### Project-Wide Standard (MANDATORY)

**Architecture Decision:** Use **undistorted pixels** throughout the reconstruction pipeline.

**Rationale:**
1. IncrementalSfM stores full K matrix and applies it in projection functions
2. Bundle adjustment optimizes in pixel space (intuitive error units)
3. Reprojection error thresholds (1px, 3px) are meaningful in pixels
4. Matches OpenCV's PnP solver expectations when distCoeffs=None

**Implementation:**
```python
# CORRECT: Convert distorted pixels → undistorted pixels at entry point
points_2d_undist_px = cv2.undistortPoints(
    points_2d_distorted.reshape(-1, 1, 2),
    cameraMatrix=K,
    distCoeffs=D,
    P=K  # ← CRITICAL: Keeps output as pixels
).reshape(-1, 2)

# FORBIDDEN: Using normalized coords without conversion
points_2d_norm = cv2.undistortPoints(points_2d_distorted, K, D)  # ❌
# This returns normalized coords, incompatible with pixel-space functions
```

### Coordinate Type Documentation Standard

All functions must explicitly annotate coordinate types in docstrings:

```python
def register_camera(
    points_2d_undist_px: np.ndarray,  # (N, 2) undistorted pixels
    points_3d_mm: np.ndarray,         # (N, 3) millimeters in L-frame
    K: np.ndarray                     # (3, 3) intrinsic matrix
) -> Tuple[np.ndarray, np.ndarray]:   # (rvec, tvec)
    """
    Register camera via PnP with UNDISTORTED pixel observations.
    
    COORDINATE CONVENTION:
    - Input 2D points: Undistorted pixels (distortion removed via cv2.undistortPoints with P=K)
    - Input 3D points: Millimeters in L-frame (world coordinates)
    - PnP call: cv2.solvePnPRansac(distCoeffs=None) since distortion already removed
    """
```

### Three Common Coordinate Errors (Lessons Learned)

**Error #1: Mixing Normalized and Pixel Coordinates**
- Symptom: Reprojection errors ~500-1000px, triangulation fails (0/16 points)
- Root cause: Undistorted with `P=None` (normalized) but used in pixel-space projection
- Solution: Always use `P=K` when undistorting for reconstruction pipeline

**Error #2: Scale Ambiguity (Essential vs PnP)**
- Symptom: Triangle area 0.00mm² (anti-hinge check fails), scale drift
- Root cause: Essential matrix gives relative pose without metric scale
- Solution: Use PnP-based initialization with known AprilTag size (7.0mm) for metric scale

**Error #3: World/Camera Frame Confusion**
- Symptom: Good initial reprojection (0.3px) degrades to 39px after bundle adjustment
- Root cause: Transforming 3D points between world/camera frames incorrectly
- Solution: Document frame for every coordinate array; validate with reprojection checks

### Validation Checklist (Before Committing Code)

- [ ] **Entry point:** All 2D detections converted to undistorted pixels via `cv2.undistortPoints(P=K)`
- [ ] **PnP calls:** Use `distCoeffs=None` (distortion already removed) or `distCoeffs=np.zeros(5)`
- [ ] **Projection:** Use full K matrix: `pt_2d = K @ (X_cam / X_cam[2])`
- [ ] **Triangulation:** Input undistorted pixels + projection matrices with K
- [ ] **Bundle adjustment:** Operates on undistorted pixels, outputs pixel-space residuals
- [ ] **Documentation:** Function signatures annotate coordinate type (e.g., `points_2d_undist_px`)
- [ ] **Unit tests:** Round-trip accuracy <0.01px for undistort → project → compare
- [ ] **3D units:** All 3D coordinates in millimeters (mm), never meters

### Module Boundary Contract

| Module | Input Coords | Internal Representation | Output Coords |
|--------|--------------|------------------------|---------------|
| **calibration_loader** | Distorted px | — | Undistorted px (via `P=K`) |
| **incremental_sfm** | Undistorted px | Stores undistorted px + K matrix | 3D points (mm) |
| **bundle_adjustment** | Undistorted px | Optimizes poses + points | Updated structure (mm) |
| **geometry_utils** | Undistorted px | Depends on P matrix | 3D points (mm) |
| **phase3_pipeline** | Distorted px (entry) | Converts to undistorted px | Final L-frame (mm) |

**Critical Rule:** Never pass coordinates across module boundaries without verifying coordinate type matches expectations.

### Known Layout vs Unknown Layout

**Known Layout (e.g., layout_4tags.json):**
- Use known 3D positions directly (no triangulation needed for initial structure)
- Transform from world frame → camera frame via PnP-solved poses
- Validates pipeline with ground truth geometry

**Unknown Layout (production flags):**
- Must triangulate 3D positions from multi-view observations
- Use undistorted pixels + projection matrices
- Validate with ray angle checks (≥5°) and reprojection errors (<5px)

### References
- **Phase 0 unit standard:** All 3D coordinates in millimeters (mm)
- **OpenCV camera convention:** `p_cam = R @ p_world + t`
- **Projection matrix format:** `P = K @ [R | t]` for world → image mapping

### Quality Gate Filtering (Automatic Image Removal)

**Purpose:** Automatically identify and remove low-quality images that contribute excessive reprojection errors, improving overall reconstruction quality.

**Implementation:** `src/image_quality_filter.py`

**Workflow:**
1. Compute per-image reprojection error statistics (mean, max, median)
2. Rank images by error criterion (default: mean error)
3. Remove worst N% of images (default: 10%)
4. Filter 3D point observations (keep points visible in ≥2 remaining images)
5. Re-run bundle adjustment with filtered structure
6. Validate improvement in QA metrics

**Safety Constraints:**
- Minimum images retained: max(5, 50% of total)
- Points must remain visible in ≥2 views after filtering
- Track connectivity validated post-removal

**Typical Results:**
- Before filtering: mean=3.4px, max=12.7px (19 images)
- After removing worst 10%: mean=2.5px, max=4.4px (17 images)
- Improvement: 29% mean error reduction, 65% max error reduction

**Configuration:**
```python
from image_quality_filter import apply_quality_gate_filter

sfm_filtered, report = apply_quality_gate_filter(
    sfm=sfm,
    percentile=10.0,      # Remove worst 10%
    criterion='mean',     # Options: 'mean', 'max', 'median'
    verbose=True
)
```

**Integration:** Automatically applied in Phase 3 pipeline after initial bundle adjustment, before final QA validation.

---
- Compute intrinsics once per magnification+focus pair (1:6 typical)
- Store calibration metadata with images: date, lens, magnification, f-stop
- Recalibrate when magnification/focus distance changes (1:5 vs 1:6 vs 1:7)
- Changing f-stop alone does **not** require recalibration

### Implementation highlights
- **Board specification:** 9×6 ChArUco grid, 10mm squares, 7mm AprilTag36h11 markers
- **Auto-magnification:** Computed from detected corners (median pixel distance / 10mm)
- **Roll diversity:** PCA-based angle binning (8 bins, 22.5° spacing, 0-180° range)
- **Quality gates:** Reprojection <0.5px (fail threshold), min 20 images, min 30 corners/image
- **Integration:** Undistort points BEFORE triangulation in Phase 3

### Deliverables
- ✅ `calib/fixtures/charuco_board_9x6.json` — Board specification
- ✅ `tools/camera_calibration.py` — Calibration tool (462 lines, CLI)
- ✅ `src/calibration_loader.py` — Load/undistort/QA utilities (367 lines)
- ✅ `calib/README.md` — Capture procedure, troubleshooting guide (269 lines)
- ✅ `test/test_calibration.py` — Unit tests (14 tests passing)

### Gate
- ✅ Tests pass (14/14)
- ✅ Round-trip accuracy <1e-3px
- ⏳ Physical calibration capture (requires printed ChArUco board)

---

## Phase 2 — Cap / flag marker system (ID + refinement)
**Goal:** deterministic seeding + high-precision refinement.  
**Status:** ⏳ NOT STARTED (reset 2026-01-13)

### Marker design (v1.3.0 standard)
- **Front face:** AprilTag36h11 (8.8mm edge, provides ID + initial pose + 4 corner features)
- **Top face:** 7-dot asymmetric constellation (asymmetry aids orientation)
- **Left face:** 7-dot asymmetric constellation
- **Right face:** 7-dot asymmetric constellation
- **Total features per cap:** 25 points (4 tag corners + 21 dots)

### Dot specifications
- **Standard dots:** Ø 1.0 mm (20 dots)
- **Anchor dot:** Ø 1.5 mm (1 per face, 7th dot in constellation)
- **Purpose:** High-precision refinement via project-and-search after AprilTag seeding
- **Material:** Laser-engraved on anodized aluminum (matte white background, dark marks)

### Cap specifications (baseline)
- **Dimensions:** 10 × 10 × 10 mm (cubic)
- **AprilTag edge:** 8.8 mm (fits within 10mm face with margin)
- **Mounting:** Temporary abutment (D-post), cemented
- **Tag IDs:** 100-104 (5 example caps generated)

### Deliverables (planned)
- **Schema:** `aox-photogrammetry-flags/schemas/flag_schema_v1.3.0.json`
- **GUI generator:** `aox-photogrammetry-flags/src/aox_flag_generator_gui_v2.py`
- **Default config:** `aox-photogrammetry-flags/src/defaults_aox_flag.json`
- **Example models:** 5 caps (Tag IDs 100-104) in `out_aox_flag_v2/models/`
  - `cap_AOX_FLAG_10x10x10_ID100.json`
  - `cap_AOX_FLAG_10x10x10_ID101.json`
  - `cap_AOX_FLAG_10x10x10_ID102.json`
  - `cap_AOX_FLAG_10x10x10_ID103.json`
  - `cap_AOX_FLAG_10x10x10_ID104.json`
- **Python tooling:**
  - `src/model_loader_v1.py` — Load and parse v1.3.0 cap models
  - `tools/analyze_model.py` — Comprehensive model analysis
  - `tools/validate_model.py` — Schema and constraint validation
  - `tools/test_symmetry.py` — Asymmetry verification
- **Documentation:**
  - `aox-photogrammetry-flags/docs/FLAG_JSON_SPECIFICATION.md`
  - `aox-photogrammetry-flags/docs/COORDINATE_SYSTEM.md`
  - `docs/PHASE2_MANUFACTURING_SPEC.md` (manufacturing handoff document)

### Gates
- JSON schema validation passes for all generated models
- 7-dot constellations maintain asymmetry (orientation-distinguishable)
- Anchor dots (Ø 1.5mm) clearly identifiable in each constellation
- AprilTag corners + 21 dots = 25 feature points per cap
- ⏳ **Physical manufacturing:** Pending CNC machining + anodizing + laser engraving

### Next steps
- **Phase 2B (Manufacturing):** Produce physical caps per manufacturing spec
- **Phase 3 (Detection pipeline):** AprilTag detection + dot refinement + Bundle Adjustment

---

## Phase 2.1 — Image triage (the “Bouncer”)
**Goal:** reject low-quality inputs before detection.

### Checks (per image)
- **Blur detection:** compute Laplacian variance. If score < 100 → discard image.
- **Exposure / saturation check:** if >10% of pixels are saturated (pure white) → discard image.

### Gate
- If usable images < 20 → abort run (do not proceed to detection/solver).

---

## Phase 3 — Geometric Core & Multi-View Geometry (L-Frame Solver)
**Goal:** Rigorous 3D reconstruction with <1.0px reprojection error.  
**Status:** ⏳ NOT STARTED (reset 2026-01-13)

### Revised Architecture (Lead Systems Architect)
**Critical principle:** Lens distortion must be removed BEFORE triangulation, not after.  
**Anti-hinge rule:** Bridge frames must have ≥3 non-collinear points (triangle area ≥10mm²).

### Phase 3 Pipeline
1) **Intrinsic Calibration Enforcement** — Load K, D; undistort 2D points to normalized coordinates
2) **Structure from Motion (SfM) Initialization** — Essential matrix + pose recovery (2-view)
3) **Incremental Reconstruction** — PnP + triangulation (multi-view)
4) **Global Bundle Adjustment** — Robust optimization in L-frame
5) **Quality Assurance** — Reprojection error, track length, graph connectivity, scale sanity

### Step 1: Intrinsic Calibration Enforcement
**Critical:** Undistort point coordinates (not images) before triangulation.

**Implementation:**
- `src/calibration_loader.py` — Load camera intrinsics, undistort points
- `load_calibration()` — Parse JSON to OpenCV K, D matrices
- `undistort_points()` — Map distorted pixels → normalized coordinates
- `CalibrationQA` class — Pre-flight checks (principal point, aspect ratio, distortion magnitude)

**QA Gates:**
- Calibration reprojection error <0.55px (checked during camera_calibration.py)
- Round-trip accuracy <1e-6 px (undistort → redistort)
- Principal point within 100px of image center (warning if exceeded)

**Workflow:**
```python
from src.calibration_loader import load_calibration, undistort_points
K, D, size = load_calibration('calib/intrinsics.json')
points_norm = undistort_points(points_raw, K, D)  # Nx2 normalized coords
```

### Step 2: SfM Initialization (Enhanced with DOP Check)
**Essential matrix estimation + pose recovery for initial two-view geometry.**

**Implementation:**
- `src/sfm_initialization.py` — Two-view SfM initialization (408 lines)
- `SfMInitializer` class — Essential matrix (RANSAC), pose recovery, triangulation
- `estimate_essential_matrix()` — Nister's 5-point with RANSAC (1.0px threshold)
- `recover_pose()` — Decompose E → R, t with cheirality check
- `triangulate_initial_points()` — DLT triangulation of initial 3D points
- `compute_ray_angles()` — Calculate ray angles for DOP validation
- `check_dop_quality()` — Metrology check: mean ray angle ≥5° required

**Anti-Hinge Enforcement:**
- `check_noncollinearity()` — Verify triangle area ≥10mm² (prevents collinear bridge frames)
- `check_cheirality()` — Points must be in front of both cameras

**DOP Check (Dilution of Precision):**
- Validates triangulation stability via ray angle computation
- Rejects pairs with mean ray angle <5° (unstable triangulation)
- Ensures geometric quality before initializing reconstruction

**Workflow:**
```python
from src.sfm_initialization import SfMInitializer
initializer = SfMInitializer(K, D, ransac_threshold=1.0)
result = initializer.initialize_from_pair(pts1_norm, pts2_norm)
# Optional: Validate DOP quality
passed, stats = initializer.check_dop_quality(pts1_norm, pts2_norm, R, t, min_ray_angle_deg=5.0)
result = initializer.initialize_from_pair(points1_norm, points2_norm)
# Returns: E, R, t, points_3d, inlier_mask
```

### Step 3: Incremental Reconstruction
**Incremental addition of views with PnP + triangulation.**

**Implementation:**
- `src/incremental_sfm.py` — Complete IncrementalSfM class (668 lines)
- `IncrementalSfM` class — Camera/point storage, PnP solver, triangulation engine
- `select_next_camera()` — Choose view with most 2D-3D correspondences
- `solve_pnp_ransac()` — Camera registration with normalized coordinates (K=I, D=0)
- `triangulate_new_points()` — Add new 3D points with ray angle check ≥5°
- `compute_ray_angle()` — DOP validation (Dilution of Precision check)
- `check_collinearity()` — Anti-hinge rule with PCA line fitting (residual <5mm = FAIL)
- `register_camera()` — Complete workflow: PnP → triangulation → validation
- `export_structure()` — Export L-frame to JSON

**Anti-Hinge Enforcement:**
- Bridge frames must have 3 non-collinear points (triangle area ≥10mm²)
- PCA line fitting: max perpendicular residual ≥5mm required
- Prevents arch warping from collinear control points

**DOP Check (Dilution of Precision):**
- Ray angle between cameras ≥5° for stable triangulation
- Filters unstable points before adding to reconstruction

**Workflow:**
```python
from src.incremental_sfm import IncrementalSfM
sfm = IncrementalSfM(K, D, min_ray_angle_deg=5.0)
sfm.initialize_from_two_views(img1, img2, rvec1, tvec1, rvec2, tvec2, pts_3d, tracks)
success = sfm.register_camera(next_image, observations)
sfm.export_structure("structure_L.json")
```

### Step 4: Global Bundle Adjustment
**Global optimization in L-frame with robust loss.**

**Implementation:**
- `tools/phase3_test_pipeline.py` — Global BA using `scipy.optimize.least_squares` (robust loss) + sparse Jacobian

**Note:**
- `tools/bundle_adjustment.py` can still be enhanced further, but Phase 3 global BA is available end-to-end via the test pipeline.

### Step 5: Quality Assurance
**Validation gates before exporting L-frame results.**

**Implementation:**
- `src/reconstruction_qa.py` — Comprehensive QA checks
- `check_reprojection_errors()` — Mean <1.0px, max <3.0px
- `check_track_lengths()` — ≥4 views per implant feature
- `check_graph_connectivity()` — No disconnected camera components
- `check_scale_sanity()` — AprilTag edge length within ±0.1mm
- `check_noncollinearity_bridge()` — Bridge points form non-degenerate triangle

**QA Gates (Hard Fail):**
- Reprojection mean >1.0px → HALT
- Reprojection max >3.0px → HALT
- Track length <4 views for implant features → WARN
- Graph disconnected → HALT
- Scale error >0.1mm → WARN
- Bridge collinearity (area <10mm²) → HALT (prevents arch warping)

### Shared Utilities
**Geometric operations used across all steps.**

**Implementation:**
- `src/geometry_utils.py` — Rotation conversions, projection, validation
- `triangulate_points_opencv()` — Fast triangulation using cv2.triangulatePoints
- `compute_reprojection_error()` — Project 3D → 2D, compute errors
- `check_cheirality()` — Validate points in front of cameras
- `check_noncollinearity()` — Anti-hinge rule (triangle area ≥10mm²)
- `validate_rotation_matrix()` — Check R^T R = I, det(R) = 1

### Industrial Photogrammetry QA Rules
- Per frame: ≥3 tags visible (≥12 tag corners)
- Per implant: ≥4 views min, 6–10 recommended
- Capture sets: Left / Right / Anterior + bridge frames
- Geometry diversity: "wider but not too wide", plus roll diversity
- Minimum pixels on target + off-axis warning
- Independent scale sanity checks (AprilTag geometry)
- Marker thickness accounted for if chasing <50 µm

### Deliverables (Planned)
- `src/calibration_loader.py` — Intrinsic calibration enforcement
- `src/sfm_initialization.py` — Two-view SfM initialization
- `src/incremental_sfm.py` — Incremental reconstruction (Step 3)
- `src/reconstruction_qa.py` — Quality assurance checks
- `src/geometry_utils.py` — Shared geometric utilities
- `tools/phase3_test_pipeline.py` — End-to-end driver (detection → SfM → BA → QA → exports)
- `runs/<dataset>/detections.json` — 2D observations
- `runs/<dataset>/observations_normalized.json` — Undistorted/normalized observations
- `runs/<dataset>/structure_L_incremental.json` — Incremental L-frame structure
- `runs/<dataset>/ba_summary.json` — Optimization report
- `runs/<dataset>/qa_report.json` — QA metrics
- `runs/<dataset>/scale_proxies_by_cap.json` — Per-cap scale diagnostics (mm per L-unit)
- `docs/qa_report_json.md` — `qa_report.json` fields + units (normalized vs pixel-equivalent)

### Gate
- Calibration loader validated (round-trip error <1e-6 px)
- SfM initialization working (essential matrix + pose recovery)
- QA checks implemented (reprojection, track length, connectivity, scale)
- Incremental SfM implemented
- Global BA converges reliably with reprojection <1.0px mean

---

## Phase 4 — Coordinate system definition (L → U)
**Goal:** meaningful, comparable, exportable results—without injecting error.

**Status:** ⏳ NOT STARTED (reset 2026-01-13)

### 4A) Define Local frame (automatic)
- Whatever BA outputs is **L**. Don't fight it during optimization.

### 4B) Define User frame U (deterministic rule)
Pick a repeatable definition that does not depend on "hand-placed" uncertain points.

**Option U1 (implant-based, no external jig)**
- Origin: centroid of all implant origins (or a designated implant origin)
- Z axis: best-fit average of implant axes (or normal of best-fit plane through implant origins)
- X axis: projection of vector from right-most cluster centroid → left-most cluster centroid (or anterior direction) onto XY plane
- Y = Z × X

**Option U2 (reference plate/jig-based, metrology-grade)**
- Use known rigid plate/board with precisely known points/targets
- Compute transform L → U via best-fit to known points
- Provides stable user frame + scale sanity check

**Principle:** points defining the user frame must be the most repeatable points available, or the transform becomes a dominant error source.

### Deliverables
- `runs/<dataset>/T_U_from_L.json`
- `runs/<dataset>/implants_U.json`
- `tools/define_user_cs.py`

### Gate
**Strict validation gates (hard stop):**
- **Convergence:** solver must return `success: true`.
- **Reprojection error:** mean RMSE must be < 0.55 px.
- **Scale residual:** the calculated distance of the scale bar must match the known distance within < 20 µm (0.02 mm).

**Action:** If any condition fails, the pipeline must `EXIT(1)` and must NOT generate `implants_U.json`.

---

## Phase 5 — Bench validation (6 flags, reseat dominates)
**Goal:** quantify the ceiling: seating repeatability + workflow repeatability.

### Protocol (10× reseat)
For each reseat:
- Seat all 6 caps with standardized protocol
- Capture: Left / Right / Anterior + bridge frames
- Run pipeline → produce L and U outputs + QA report

### Acceptance thresholds (baseline gates)
**Per-frame (hard fail)**
- ≥3 tags visible
- Tag reprojection mean < 2.0 px, max < 4.0 px
- Dot inlier rate > 70% after refinement

**Repeatability (U-frame, across reseats)**
- Translation RMS per implant: ≤ 35 µm (pass), stretch ≤ 25 µm
- Translation max per implant: ≤ 90 µm
- Angular RMS per implant: ≤ 0.20°
- Angular max per implant: ≤ 0.50°

**Scale sanity**
- ≤ 50 µm over 100 mm (0.05 mm/m) drift limit per run

### Deliverables
- `runs/bench_6caps/repeatability.json`
- `runs/bench_6caps/repeatability_report.html`
- Failure atlas notes: glare / occlusion / scale drift / fragmentation patterns

### Gate
- Pass repeatability thresholds before chasing <50 µm claims

---

## Phase 6 — IOS + exocad integration (U ↔ I alignment)
**Goal:** merge arch context from IOS with implant precision from PG.

### Steps
- IOS scan → `arch_ios.stl`
- exocad scan-body fit → `implants_I.json`
- PG output in U → `implants_U.json`
- Solve transform `T_I_from_U` using corresponding implant origins (optionally axis constraints)
- **Step 6.3: Visual verification (the “Ghost Model”)**
  - **Action:** generate `U_Constellation.stl` (virtual scan bodies / implant constellation in U).
  - **Procedure:** superimpose `U_Constellation.stl` onto `arch_ios.stl` in the viewer.
  - **Check:** visually confirm “drift” patterns (especially at distal implants).
- Export package:
  - `arch_ios.stl`
  - `implants_PG_in_I.json`
  - `T_I_from_U.json`
  - `report.pdf/html` (QA + transforms + residuals)

### Deliverables
- `tools/solve_T_I_from_U.py`
- `tools/export_u_constellation_stl.py` (generate `U_Constellation.stl`)
- `exports/case_<id>/...`

### Gate
- Alignment residuals consistent with expected IOS scan-body fit quality; flag outliers

---

## Phase 7 — Hardening + risk mitigation
**Glare/specular:** cross-polarization, saturation rejection in dot fitting  
**Seating repeatability:** protocol, torque control, detect outlier reseats  
**Scale drift:** independent scale checks each run; don't trust EXIF  
**Graph disconnect:** enforce bridge frames, view-graph QA  
**Marker thickness:** include `marker_thickness_mm`; do sensitivity sweep  

### Deliverables
- `qa_rules.json` (machine-checkable gates)
- Automated "fail-fast" warnings in reports
