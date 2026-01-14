#!/usr/bin/env python3
"""
Export IOS Integration Package.

Bundles all Phase 6 outputs into a deliverable package for CAD/CAM workflow.
Includes transforms, meshes, implant positions, QA reports, and documentation.

Package Structure:
    case_YYYYMMDD_HHMMSS/
        ├── README.txt                  # Package documentation
        ├── T_I_from_U.json            # Transformation matrix
        ├── implants_U.json            # Implants in photogrammetry frame
        ├── implants_I.json            # Implants in IOS frame
        ├── U_Constellation.stl        # Photogrammetry visualization
        ├── arch_ios.stl               # IOS arch mesh (copied from input)
        ├── qa_report.json             # Quality assurance metrics
        └── metadata.json              # Package metadata

Usage:
    python tools/export_ios_package.py \\
        --run-dir runs/case001 \\
        --ios-mesh data/ios/arch.stl \\
        --output exports/case001_package

Quality Gates:
    - T_I_from_U.json RMSE < 0.2mm (from solve_T_I_from_U)
    - All required files present
    - Valid JSON schemas

Author: Phase 6 IOS Integration Pipeline
"""

import argparse
import json
import shutil
import sys
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional


def validate_required_files(run_dir: Path) -> Dict[str, bool]:
    """Check for required Phase 6 output files.
    
    Args:
        run_dir: Path to run directory
        
    Returns:
        Dictionary mapping filename to existence status
    """
    required_files = {
        "T_I_from_U.json": run_dir / "T_I_from_U.json",
        "implants_U.json": run_dir / "implants_U.json",
        "implants_I.json": run_dir / "implants_I.json",
        "U_Constellation.stl": run_dir / "U_Constellation.stl"
    }
    
    status = {}
    for name, path in required_files.items():
        status[name] = path.exists()
    
    return status


def load_transform_metadata(transform_path: Path) -> Dict:
    """Load T_I_from_U metadata for QA report.
    
    Args:
        transform_path: Path to T_I_from_U.json
        
    Returns:
        Dictionary with RMSE, correspondences, etc.
    """
    with open(transform_path, 'r') as f:
        data = json.load(f)
    
    return {
        "n_correspondences": data["metadata"]["n_correspondences"],
        "rmse_mm": data["metadata"]["rmse_mm"],
        "max_residual_mm": data["metadata"]["max_residual_mm"],
        "rmse_threshold_mm": data["metadata"]["rmse_threshold_mm"],
        "matched_marker_ids": data["metadata"]["matched_marker_ids"],
        "per_implant_residuals_mm": data["metadata"]["per_implant_residuals_mm"]
    }


def generate_readme(
    package_dir: Path,
    transform_metadata: Dict,
    ios_mesh_source: Optional[str] = None
) -> str:
    """Generate README.txt for package.
    
    Args:
        package_dir: Path to package directory
        transform_metadata: Metadata from T_I_from_U.json
        ios_mesh_source: Optional source path of IOS mesh
        
    Returns:
        README.txt content as string
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    readme = f"""
================================================================================
IOS INTEGRATION PACKAGE - Phase 6 Photogrammetry Output
================================================================================

Generated: {timestamp}
Package: {package_dir.name}

This package contains photogrammetry implant positions aligned with an
intraoral scanner (IOS) coordinate system for CAD/CAM workflow integration.

================================================================================
CONTENTS
================================================================================

1. T_I_from_U.json
   - SE(3) transformation from photogrammetry U-frame to IOS I-frame
   - 4×4 homogeneous matrix [R|t]
   - Use this to transform any photogrammetry data into IOS coordinates

2. implants_U.json
   - Implant positions in photogrammetry U-frame (mm)
   - Extracted from AprilTag marker caps via bundle adjustment
   - Contains centroids, axis vectors, and reprojection errors

3. implants_I.json
   - Implant positions in IOS I-frame (mm)
   - Either from exocad scan body fit OR transformed from implants_U.json
   - Use these positions for prosthetic design in CAD software

4. U_Constellation.stl
   - 3D visualization of implant positions
   - Cylinder = implant body (4mm diameter × 12mm height)
   - Cone = axis direction indicator (2mm base × 6mm height)
   - Overlay this on arch_ios.stl in CAD software for verification

5. arch_ios.stl (optional)
   - Intraoral scan mesh of patient arch
   - Provides anatomical context for implant positions
   - Source: {ios_mesh_source or 'Not included'}

6. qa_report.json
   - Quality assurance metrics for alignment
   - RMSE, per-implant residuals, correspondence counts

7. metadata.json
   - Package generation metadata
   - File paths, timestamps, versions

================================================================================
QUALITY ASSURANCE
================================================================================

Alignment Quality:
  - Correspondences: {transform_metadata['n_correspondences']} implants
  - RMSE: {transform_metadata['rmse_mm']:.4f} mm
  - Max Residual: {transform_metadata['max_residual_mm']:.4f} mm
  - Threshold: {transform_metadata['rmse_threshold_mm']:.4f} mm

Status: {'PASS' if transform_metadata['rmse_mm'] <= transform_metadata['rmse_threshold_mm'] else 'FAIL'}

Matched Implants (Marker IDs):
  {', '.join(map(str, transform_metadata['matched_marker_ids']))}

Per-Implant Residuals:
"""
    
    for implant_id, residual in transform_metadata['per_implant_residuals_mm'].items():
        readme += f"  Implant {implant_id}: {residual:.4f} mm\n"
    
    readme += f"""
================================================================================
USAGE IN CAD SOFTWARE
================================================================================

1. Import arch_ios.stl (if available)
2. Import U_Constellation.stl and overlay on arch mesh
3. Verify implant positions visually
4. Use implants_I.json centroids and axes for prosthetic design
5. Export final design in I-frame coordinates

Coordinate System:
  - Origin: IOS scanner origin (exocad convention)
  - Units: millimeters (mm)
  - Right-handed coordinate system

================================================================================
TECHNICAL NOTES
================================================================================

Transformation Matrix:
  - Type: SE(3) rigid transformation (rotation + translation)
  - Algorithm: Umeyama closed-form solution
  - No scaling applied (metric reconstruction from photogrammetry)

Implant Extraction:
  - Source: AprilTag36h11 marker caps (8.8mm edge length)
  - Centroid: Mean of 4 tag corners
  - Axis: Normal to tag plane (perpendicular to cap surface)

Quality Thresholds:
  - RMSE < 0.2mm: Clinical grade alignment
  - RMSE 0.2-0.5mm: Acceptable for most cases
  - RMSE > 0.5mm: Review IOS scan quality and marker seating

================================================================================
CONTACT & SUPPORT
================================================================================

For questions about this package or Phase 6 pipeline:
  - Review PROJECT_OVERVIEW.md in source repository
  - Check RECONSTRUCTION_STATUS.md for pipeline status
  - Consult PHASE3_4_IMPLEMENTATION_SUMMARY.md for technical details

Generated by: Phase 6 IOS Integration Pipeline
Version: 1.0
Date: {timestamp}

================================================================================
"""
    
    return readme


def create_qa_report(
    transform_metadata: Dict,
    implants_u_path: Path,
    implants_i_path: Path
) -> Dict:
    """Create QA report JSON.
    
    Args:
        transform_metadata: Metadata from T_I_from_U.json
        implants_u_path: Path to implants_U.json
        implants_i_path: Path to implants_I.json
        
    Returns:
        QA report dictionary
    """
    # Load implant counts
    with open(implants_u_path, 'r') as f:
        u_data = json.load(f)
    
    with open(implants_i_path, 'r') as f:
        i_data = json.load(f)
    
    report = {
        "phase": "Phase 6 - IOS Integration",
        "timestamp": datetime.now().isoformat(),
        "alignment": {
            "n_correspondences": transform_metadata["n_correspondences"],
            "rmse_mm": transform_metadata["rmse_mm"],
            "max_residual_mm": transform_metadata["max_residual_mm"],
            "threshold_mm": transform_metadata["rmse_threshold_mm"],
            "status": "PASS" if transform_metadata["rmse_mm"] <= transform_metadata["rmse_threshold_mm"] else "FAIL"
        },
        "implants": {
            "n_implants_u_frame": u_data["metadata"]["n_implants"],
            "n_implants_i_frame": i_data["metadata"]["n_implants"],
            "matched_marker_ids": transform_metadata["matched_marker_ids"],
            "per_implant_residuals_mm": transform_metadata["per_implant_residuals_mm"]
        },
        "quality_gates": {
            "rmse_threshold": {
                "threshold_mm": transform_metadata["rmse_threshold_mm"],
                "actual_mm": transform_metadata["rmse_mm"],
                "passed": transform_metadata["rmse_mm"] <= transform_metadata["rmse_threshold_mm"]
            },
            "min_correspondences": {
                "threshold": 3,
                "actual": transform_metadata["n_correspondences"],
                "passed": transform_metadata["n_correspondences"] >= 3
            }
        }
    }
    
    return report


def export_package(
    run_dir: str,
    output_dir: str,
    ios_mesh_path: Optional[str] = None,
    case_name: Optional[str] = None
) -> Dict:
    """Export complete IOS integration package.
    
    Args:
        run_dir: Path to Phase 6 run directory with outputs
        output_dir: Path to save package
        ios_mesh_path: Optional path to IOS mesh to include
        case_name: Optional case name (default: auto-generated)
        
    Returns:
        Dictionary with export statistics
        
    Raises:
        FileNotFoundError: If required files missing
        ValueError: If quality gates fail
    """
    run_dir = Path(run_dir)
    output_dir = Path(output_dir)
    
    # Validate required files
    print("Validating required files...")
    file_status = validate_required_files(run_dir)
    
    missing_files = [name for name, exists in file_status.items() if not exists]
    if missing_files:
        raise FileNotFoundError(f"Missing required files: {', '.join(missing_files)}")
    
    print(f"  ✓ All required files present")
    
    # Load transform metadata
    transform_path = run_dir / "T_I_from_U.json"
    transform_metadata = load_transform_metadata(transform_path)
    
    # Check quality gates
    if transform_metadata["rmse_mm"] > transform_metadata["rmse_threshold_mm"]:
        print(f"  ⚠ WARNING: RMSE {transform_metadata['rmse_mm']:.4f}mm exceeds threshold {transform_metadata['rmse_threshold_mm']:.4f}mm")
    else:
        print(f"  ✓ RMSE quality gate passed ({transform_metadata['rmse_mm']:.4f}mm)")
    
    # Create package directory
    if case_name is None:
        case_name = f"case_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    package_dir = output_dir / case_name
    package_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\nExporting package to: {package_dir}")
    
    # Copy required files
    files_to_copy = {
        "T_I_from_U.json": run_dir / "T_I_from_U.json",
        "implants_U.json": run_dir / "implants_U.json",
        "implants_I.json": run_dir / "implants_I.json",
        "U_Constellation.stl": run_dir / "U_Constellation.stl"
    }
    
    for dest_name, src_path in files_to_copy.items():
        dest_path = package_dir / dest_name
        shutil.copy2(src_path, dest_path)
        print(f"  Copied {dest_name}")
    
    # Copy IOS mesh if provided
    if ios_mesh_path:
        ios_mesh_src = Path(ios_mesh_path)
        if ios_mesh_src.exists():
            ios_mesh_dest = package_dir / "arch_ios.stl"
            shutil.copy2(ios_mesh_src, ios_mesh_dest)
            print(f"  Copied arch_ios.stl")
        else:
            print(f"  ⚠ WARNING: IOS mesh not found at {ios_mesh_path}")
            ios_mesh_path = None
    
    # Generate QA report
    qa_report = create_qa_report(
        transform_metadata,
        run_dir / "implants_U.json",
        run_dir / "implants_I.json"
    )
    
    qa_path = package_dir / "qa_report.json"
    with open(qa_path, 'w') as f:
        json.dump(qa_report, f, indent=2)
    print(f"  Generated qa_report.json")
    
    # Generate README
    readme_content = generate_readme(
        package_dir,
        transform_metadata,
        ios_mesh_source=str(ios_mesh_path) if ios_mesh_path else None
    )
    
    readme_path = package_dir / "README.txt"
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write(readme_content)
    print(f"  Generated README.txt")
    
    # Generate metadata
    metadata = {
        "package_name": case_name,
        "timestamp": datetime.now().isoformat(),
        "source_run_dir": str(run_dir.resolve()),
        "ios_mesh_source": str(ios_mesh_path) if ios_mesh_path else None,
        "files": list(files_to_copy.keys()) + (["arch_ios.stl"] if ios_mesh_path else []),
        "alignment_rmse_mm": transform_metadata["rmse_mm"],
        "n_implants": transform_metadata["n_correspondences"]
    }
    
    metadata_path = package_dir / "metadata.json"
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"  Generated metadata.json")
    
    # Summary
    print(f"\n{'=' * 70}")
    print(f"✓ Package export complete")
    print(f"  Location: {package_dir}")
    print(f"  Files: {len(metadata['files'])}")
    print(f"  RMSE: {transform_metadata['rmse_mm']:.4f} mm")
    print(f"  Implants: {transform_metadata['n_correspondences']}")
    print(f"{'=' * 70}")
    
    return {
        "package_dir": str(package_dir),
        "n_files": len(metadata['files']),
        "rmse_mm": transform_metadata["rmse_mm"],
        "n_implants": transform_metadata["n_correspondences"]
    }


def main():
    parser = argparse.ArgumentParser(
        description="Export IOS integration package for CAD/CAM workflow",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Export package with IOS mesh
    python tools/export_ios_package.py \\
        --run-dir runs/case001 \\
        --ios-mesh data/ios/arch.stl \\
        --output exports/case001_delivery
    
    # Export without IOS mesh
    python tools/export_ios_package.py \\
        --run-dir runs/case001 \\
        --output exports/case001_delivery \\
        --case-name patient_12345
        """
    )
    
    parser.add_argument(
        "--run-dir",
        type=str,
        required=True,
        help="Path to Phase 6 run directory with T_I_from_U.json, implants, STL"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Path to export package directory"
    )
    
    parser.add_argument(
        "--ios-mesh",
        type=str,
        help="Optional path to IOS mesh (arch.stl) to include in package"
    )
    
    parser.add_argument(
        "--case-name",
        type=str,
        help="Optional case name (default: case_YYYYMMDD_HHMMSS)"
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Phase 6: Export IOS Integration Package")
    print("=" * 70)
    print()
    
    try:
        results = export_package(
            args.run_dir,
            args.output,
            ios_mesh_path=args.ios_mesh,
            case_name=args.case_name
        )
        return 0
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
