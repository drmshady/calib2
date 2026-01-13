#!/usr/bin/env python3
"""Test Phase 3 clean pipeline with known layout."""

import sys
from pathlib import Path
from glob import glob

# Add tools to path
sys.path.insert(0, str(Path(__file__).parent / "tools"))

import phase3_clean_pipeline

if __name__ == "__main__":
    # Setup paths
    workspace = Path(__file__).parent
    image_pattern = str(workspace / "calib" / "test" / "DSC_*.TIF")
    image_paths = sorted([Path(p) for p in glob(image_pattern)])
    
    calib_path = workspace / "calib" / "1_10" / "camera_intrinsics.json"
    layout_path = workspace / "calib" / "fixtures" / "layout_4tags.json"
    output_dir = workspace / "runs" / "phase3_clean_test"
    
    print(f"Found {len(image_paths)} images")
    print(f"Calibration: {calib_path}")
    print(f"Layout: {layout_path}")
    print(f"Output: {output_dir}")
    print()
    
    if not calib_path.exists():
        print(f"ERROR: Calibration file not found: {calib_path}")
        sys.exit(1)
    
    if not layout_path.exists():
        print(f"ERROR: Layout file not found: {layout_path}")
        sys.exit(1)
    
    if len(image_paths) == 0:
        print(f"ERROR: No images found: {image_pattern}")
        sys.exit(1)
    
    print("Running Phase 3 clean pipeline with known layout...\n")
    
    try:
        sfm, metadata = phase3_clean_pipeline.run_phase3_clean_pipeline(
            image_paths=image_paths,
            calib_path=calib_path,
            layout_path=layout_path,
            output_dir=output_dir,
            verbose=True
        )
        
        print("\n" + "=" * 70)
        print("PIPELINE COMPLETE")
        print("=" * 70)
        print(f"Cameras registered: {metadata['images']['registered']}/{metadata['images']['total']}")
        print(f"3D points: {metadata['structure']['n_points_3d']}")
        print(f"QA status: {metadata['qa_status'].upper()}")
        
        if metadata['qa_summary']['reprojection_mean_px'] is not None:
            print(f"Reprojection error: mean={metadata['qa_summary']['reprojection_mean_px']:.3f}px, "
                  f"max={metadata['qa_summary']['reprojection_max_px']:.3f}px")
        
    except Exception as e:
        print("\n" + "=" * 70)
        print("PIPELINE FAILED")
        print("=" * 70)
        print(f"Error: {e}")
        sys.exit(1)
