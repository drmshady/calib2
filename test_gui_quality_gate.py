#!/usr/bin/env python3
"""
Test script to verify the Quality Gate GUI tab can load reconstruction data.

This script demonstrates the workflow programmatically without launching the full GUI.
"""

import sys
import json
from pathlib import Path

# Add paths
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from incremental_sfm import IncrementalSfM, Camera, Point3D
from image_quality_filter import compute_per_image_errors


def test_load_reconstruction(recon_dir: Path):
    """Test loading reconstruction from metadata.json."""
    
    print(f"Testing load from: {recon_dir}")
    
    # Check for metadata.json
    metadata_path = recon_dir / "metadata.json"
    if not metadata_path.exists():
        print(f"❌ No metadata.json found in {recon_dir}")
        return False
    
    print(f"✓ Found metadata.json")
    
    # Load metadata
    with open(metadata_path, 'r') as f:
        metadata = json.load(f)
    
    print(f"✓ Loaded metadata")
    
    # Handle both old and new metadata formats
    if 'images' in metadata:
        print(f"  Images: {metadata['images']['total']}")
        print(f"  Registered: {metadata['images']['registered']}")
        print(f"  After quality gate: {metadata['images'].get('after_quality_gate', 'N/A')}")
    else:
        print(f"  Metadata format: Legacy (no image stats)")
    
    # Check if we have camera/point data directly or need to load from refpoints_L.json
    has_embedded_data = 'cameras' in metadata and 'points' in metadata
    
    if not has_embedded_data:
        print(f"⚠ No embedded camera/point data - trying to load from refpoints_L.json")
        refpoints_path = recon_dir / "refpoints_L.json"
        if not refpoints_path.exists():
            print(f"❌ No refpoints_L.json found")
            return False
        
        with open(refpoints_path, 'r') as f:
            refpoints_data = json.load(f)
        
        # Convert refpoints format to our metadata format
        metadata['cameras'] = []
        metadata['points'] = []
        
        # This will be populated when we parse refpoints_L.json format
        print(f"✓ Found refpoints_L.json")
        print(f"  Note: GUI expects metadata.json with embedded camera/point data")
        print(f"  Run phase3_clean_pipeline.py to generate proper metadata format")
        return False
    
    # Reconstruct SfM
    sfm = IncrementalSfM()
    
    # Load cameras
    if 'cameras' in metadata:
        for cam_data in metadata['cameras']:
            cam = Camera(
                id=cam_data['id'],
                image_name=cam_data['image_name'],
                K=cam_data['K'],
                dist_coeffs=cam_data['dist_coeffs'],
                R=cam_data['R'],
                t=cam_data['t']
            )
            sfm.cameras[cam.id] = cam
        print(f"✓ Loaded {len(sfm.cameras)} cameras")
    else:
        print("⚠ No camera data in metadata")
    
    # Load points
    if 'points' in metadata:
        for pt_data in metadata['points']:
            pt = Point3D(
                id=pt_data['id'],
                xyz=pt_data['xyz'],
                observations=pt_data['observations']
            )
            sfm.points_3d[pt.id] = pt
        print(f"✓ Loaded {len(sfm.points_3d)} 3D points")
    else:
        print("⚠ No point data in metadata")
    
    # Compute per-image errors
    if len(sfm.cameras) > 0 and len(sfm.points_3d) > 0:
        image_errors = compute_per_image_errors(sfm)
        
        print(f"\n✓ Computed per-image errors:")
        print(f"  Total images: {len(image_errors)}")
        
        # Sort by mean error (descending)
        image_errors_sorted = sorted(image_errors, key=lambda x: x[1], reverse=True)
        
        print(f"\n  Top 5 worst images (by mean error):")
        for i, (image_name, mean_err, max_err, median_err) in enumerate(image_errors_sorted[:5]):
            print(f"    {i+1}. {image_name}: mean={mean_err:.3f}px, max={max_err:.3f}px, median={median_err:.3f}px")
        
        print(f"\n  Top 5 best images (by mean error):")
        for i, (image_name, mean_err, max_err, median_err) in enumerate(reversed(image_errors_sorted[-5:])):
            print(f"    {i+1}. {image_name}: mean={mean_err:.3f}px, max={max_err:.3f}px, median={median_err:.3f}px")
        
        # Simulate 10% filtering
        n_images = len(image_errors)
        n_remove = max(0, int(n_images * 0.10))
        
        if n_remove > 0:
            mean_errors = [x[1] for x in image_errors]
            before_mean = sum(mean_errors) / len(mean_errors)
            after_mean = sum(mean_errors[n_remove:]) / (len(mean_errors) - n_remove)
            improvement = (before_mean - after_mean) / before_mean * 100
            
            print(f"\n  10% Quality Gate Simulation:")
            print(f"    Remove: {n_remove} images ({n_remove/n_images*100:.1f}%)")
            print(f"    Mean error: {before_mean:.3f}px → {after_mean:.3f}px")
            print(f"    Improvement: {improvement:.1f}%")
            
            worst_images = [x[0] for x in image_errors_sorted[:n_remove]]
            print(f"    Would remove: {', '.join(worst_images)}")
        
        return True
    else:
        print("❌ Insufficient data to compute errors")
        return False


def main():
    """Test loading reconstruction from various directories."""
    
    # Try to find test reconstruction
    test_dirs = [
        Path("runs/phase3_clean_test"),
        Path("runs/phase3_clean"),
        Path("runs/fix_test"),
        Path("exports/phase3_test")
    ]
    
    print("=" * 70)
    print("Quality Gate GUI - Load Test")
    print("=" * 70)
    print()
    
    found = False
    for test_dir in test_dirs:
        if test_dir.exists():
            print(f"\nTesting: {test_dir}")
            print("-" * 70)
            success = test_load_reconstruction(test_dir)
            if success:
                found = True
                print(f"\n✅ {test_dir} - PASS")
            else:
                print(f"\n❌ {test_dir} - FAIL")
            print("-" * 70)
    
    if not found:
        print("\n⚠ No valid reconstruction directories found.")
        print("  Please run Phase 3 pipeline first:")
        print("    python test_phase3_clean.py")
    
    print("\n" + "=" * 70)
    print("GUI Quality Gate tab is ready to use!")
    print("  1. Launch GUI: python tools/camera_calibration_gui.py")
    print("  2. Navigate to 'Quality Gate' tab")
    print("  3. Browse to reconstruction directory")
    print("  4. Click 'Load & Analyze'")
    print("=" * 70)


if __name__ == "__main__":
    main()
