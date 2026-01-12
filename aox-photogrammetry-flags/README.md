# AOX Photogrammetry Flags (Marker Caps)

Digital design system for multi-face marker caps with AprilTags and dot constellations.

## Marker Design v1.3.0

### Features per Cap (25 points total)
- **Front face:** AprilTag36h11 (8.8mm edge) + 4 corner features
- **Top face:** 7-dot asymmetric constellation
- **Left face:** 7-dot asymmetric constellation  
- **Right face:** 7-dot asymmetric constellation

### Dot Specifications
- **Standard dots:** Ø 1.0 mm (20 dots per cap)
- **Anchor dots:** Ø 1.5 mm (1 per face, 7th dot in constellation)
- **Material:** Laser-engraved on anodized aluminum

### Cap Specifications
- **Dimensions:** 10 × 10 × 10 mm (cubic baseline)
- **AprilTag edge:** 8.8 mm (fits within 10mm face with margin)
- **Mounting:** Temporary abutment (D-post), cemented
- **Tag IDs:** 100+ (example caps use IDs 100-104)

## Directory Structure

```
aox-photogrammetry-flags/
├── schemas/                    # JSON schemas
│   └── flag_schema_v1.3.0.json
├── src/                        # Generator tools
│   ├── aox_flag_generator_gui_v2.py
│   └── defaults_aox_flag.json
├── docs/                       # Design documentation
│   ├── FLAG_JSON_SPECIFICATION.md
│   └── COORDINATE_SYSTEM.md
└── out_aox_flag_v2/
    └── models/                 # Generated cap models
        ├── cap_AOX_FLAG_10x10x10_ID100.json
        ├── cap_AOX_FLAG_10x10x10_ID101.json
        ├── cap_AOX_FLAG_10x10x10_ID102.json
        ├── cap_AOX_FLAG_10x10x10_ID103.json
        └── cap_AOX_FLAG_10x10x10_ID104.json
```

## Usage

1. Run GUI: `python src/aox_flag_generator_gui_v2.py`
2. Configure cap parameters (dimensions, tag ID, dot size)
3. Generate model → saves JSON to `out_aox_flag_v2/models/`
4. Validate: Use `tools/validate_model.py` and `tools/analyze_model.py`
5. Manufacturing: Export to CNC/laser engraving (see `docs/PHASE2_MANUFACTURING_SPEC.md`)

## Validation Tools

```bash
# Validate JSON schema compliance
python tools/validate_model.py --model out_aox_flag_v2/models/cap_AOX_FLAG_10x10x10_ID100.json

# Analyze geometry and asymmetry
python tools/analyze_model.py --model out_aox_flag_v2/models/cap_AOX_FLAG_10x10x10_ID100.json

# Test 7-dot constellation asymmetry
python tools/test_symmetry.py --model out_aox_flag_v2/models/cap_AOX_FLAG_10x10x10_ID100.json
```

## Manufacturing Handoff

See `../docs/PHASE2_MANUFACTURING_SPEC.md` for complete manufacturing specifications.
