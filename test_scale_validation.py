"""Quick test of scale validation with current data"""
import json
import numpy as np
from pathlib import Path
import sys

# Add paths
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / 'tools'))

# Import the validation function
from phase3_test_pipeline import validate_scale_distances

# Test with current data
passed, errors = validate_scale_distances(
    'runs/phase 4/refpoints_U.json',
    'calib/fixtures/reference_plate_4tags.json',
    tolerance_mm=1.0
)

print(f"Validation passed: {passed}\n")
print("Detailed results:")
print("="*70)

for dist_name, result in errors.items():
    if isinstance(result, dict) and "error_mm" in result:
        status = "✓ PASS" if result["passed"] else "✗ FAIL"
        print(f"{status} {dist_name}:")
        print(f"  Expected: {result['expected_mm']:.4f}mm")
        print(f"  Computed: {result['computed_mm']:.4f}mm")
        print(f"  Error: {result['error_mm']:+.4f}mm")
        print(f"  Tolerance: ±1.0mm")
        print()
    elif isinstance(result, dict) and "relative_error" in result:
        status = "✓ PASS" if result["passed"] else "⚠ INFO"
        print(f"{status} {dist_name} (INFORMATIONAL):")
        print(f"  Expected ratio: {result['expected_ratio']:.6f}")
        print(f"  Computed ratio: {result['computed_ratio']:.6f}")
        print(f"  Relative error: {result['relative_error']*100:.2f}%")
        print(f"  {result['note']}")
        print()

print("="*70)
if passed:
    print("✅ ALL HARD GATES PASSED")
else:
    print("❌ HARD GATES FAILED - Distance errors exceed ±1.0mm tolerance")
