#!/usr/bin/env python3
"""
Simple test script to run Phase 3 unknown layout pipeline.
"""

import sys
from pathlib import Path

# Add tools to path
tools_dir = str(Path(__file__).parent / "tools")
sys.path.insert(0, tools_dir)

import subprocess

def main():
    cmd = [
        sys.executable,
        "tools/phase3_unknown_layout_pipeline.py",
        "--images", "calib/test2/DSC_*.TIF",
        "--calib", "calib/test2/camera_intrinsics.json",
        "--output", "runs/test2_unknown_layout",
        "--tag-size", "8.8"
    ]
    
    print("Running Phase 3 Unknown Layout Pipeline...")
    print(" ".join(cmd))
    print()
    
    result = subprocess.run(cmd, cwd=Path(__file__).parent)
    return result.returncode

if __name__ == "__main__":
    sys.exit(main())
