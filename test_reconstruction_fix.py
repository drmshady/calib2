#!/usr/bin/env python3
"""Quick test script to verify the PnP-based initialization fix."""

import sys
from pathlib import Path
from glob import glob

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from tools import phase3_test_pipeline

# Test parameters
image_pattern = "calib/test/DSC_*.TIF"
calib_file = "calib/1_10/camera_intrinsics.json"
output_dir = "runs/fix_test"
layout_file = "calib/fixtures/layout_4tags.json"
tag_size_mm = 7.0  # Correct size for test images!

# Get image paths
image_paths = sorted(glob(image_pattern))
print(f"Found {len(image_paths)} images")

if len(image_paths) == 0:
    print("ERROR: No images found!")
    sys.exit(1)

# Run pipeline
try:
    print("\nRunning Phase 3 pipeline with PnP-based initialization...")
    sfm, metadata = phase3_test_pipeline.run_phase3_pipeline(
        image_paths=image_paths,
        calib_file=calib_file,
        output_dir=output_dir,
        layout_file=layout_file,
        tag_size_mm=tag_size_mm,
        verbose=True
    )
    
    print("\n" + "="*70)
    print("SUCCESS! Reconstruction completed.")
    print("="*70)
    print(f"Cameras: {len(sfm.cameras)}")
    print(f"3D Points: {len(sfm.points_3d)}")
    print(f"Output: {output_dir}")
    
except Exception as e:
    print("\n" + "="*70)
    print("FAILED!")
    print("="*70)
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
