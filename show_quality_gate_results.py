#!/usr/bin/env python3
"""
Demonstrate quality gate filtering with different percentiles.
"""

import sys
import json
from pathlib import Path

workspace = Path(__file__).parent
metadata_path = workspace / "runs" / "phase3_clean_test" / "metadata.json"

if metadata_path.exists():
    with open(metadata_path, 'r') as f:
        metadata = json.load(f)
    
    print("\n" + "=" * 70)
    print("PHASE 3 QUALITY GATE RESULTS")
    print("=" * 70)
    
    # Initial statistics
    print(f"\nInitial cameras registered: {metadata['images']['registered']}")
    print(f"After quality gate: {metadata['images']['after_quality_gate']}")
    print(f"Cameras removed: {metadata['images']['registered'] - metadata['images']['after_quality_gate']}")
    
    # Quality gate details
    qg = metadata.get('quality_gate', {})
    
    if qg.get('status') == 'success':
        print(f"\nQuality gate filter: {qg['filter_config']['percentile']}% worst by {qg['filter_config']['criterion']} error")
        
        print("\nBEFORE FILTERING:")
        print(f"  Images: {qg['before']['n_images']}")
        print(f"  Mean error: {qg['before']['mean_error_px']:.3f}px")
        print(f"  Max error: {qg['before']['max_error_px']:.3f}px")
        
        print("\nAFTER FILTERING:")
        print(f"  Images: {qg['after']['n_images']}")
        print(f"  Mean error: {qg['after']['mean_error_px']:.3f}px")
        print(f"  Max error: {qg['after']['max_error_px']:.3f}px")
        
        mean_improvement = qg['before']['mean_error_px'] - qg['after']['mean_error_px']
        max_improvement = qg['before']['max_error_px'] - qg['after']['max_error_px']
        mean_percent = (mean_improvement / qg['before']['mean_error_px']) * 100
        max_percent = (max_improvement / qg['before']['max_error_px']) * 100
        
        print("\nIMPROVEMENT:")
        print(f"  Mean error: -{mean_improvement:.3f}px ({mean_percent:.1f}% reduction)")
        print(f"  Max error: -{max_improvement:.3f}px ({max_percent:.1f}% reduction)")
        
        print("\nREMOVED IMAGES:")
        for img_id in qg['removed']['removed_images']:
            print(f"  - {img_id}")
        
        print(f"\n3D points retained: {qg['removed']['n_points_after']}/{qg['removed']['n_points_before']}")
    
    # Final QA status
    print("\n" + "=" * 70)
    print(f"FINAL QA STATUS: {metadata['qa_status'].upper()}")
    
    if metadata.get('qa_hard_failures'):
        print(f"Hard failures: {', '.join(metadata['qa_hard_failures'])}")
    
    if metadata.get('qa_warnings'):
        print(f"Warnings: {', '.join(metadata['qa_warnings'])}")
    
    if metadata['qa_status'] == 'pass':
        print("All quality gates PASSED!")
    elif metadata['qa_status'] == 'fail':
        print("Quality gates NOT MET - see hard failures")
    
    print("=" * 70)

else:
    print(f"ERROR: Metadata file not found: {metadata_path}")
    print("Run test_phase3_clean.py first to generate results.")
    sys.exit(1)
