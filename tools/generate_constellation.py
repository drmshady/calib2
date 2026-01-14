#!/usr/bin/env python3
"""
Generate Implant Constellation STL Mesh.

Creates a 3D mesh visualization of implant positions with cylinders representing
implant bodies and cones representing axes. Output STL can be overlaid on IOS
arch mesh in CAD software for visual verification.

Geometry:
    - Cylinder: 4mm diameter, 12mm height (implant body)
    - Cone: 2mm base diameter, 6mm height (axis direction indicator)
    - Total height: 18mm per implant
    - Coordinate system: Matches input JSON frame (U or I)

Usage:
    python tools/generate_constellation.py \\
        --implants runs/case001/implants_U.json \\
        --output runs/case001/U_Constellation.stl \\
        --cylinder-radius 2.0 \\
        --cylinder-height 12.0

Dependencies:
    - numpy-stl (default mode: primitive cylinder+cone export)
    - numpy (geometry computation)
    - trimesh (optional: scanbody STL template placement)

Author: Phase 6 IOS Integration Pipeline
"""

import argparse
import json
import sys
from pathlib import Path
from typing import List, Tuple
import numpy as np
from stl import mesh


def _axis_from_string(axis_str: str) -> np.ndarray:
    axis_map = {
        'x': np.array([1.0, 0.0, 0.0]),
        'y': np.array([0.0, 1.0, 0.0]),
        'z': np.array([0.0, 0.0, 1.0]),
        '-x': np.array([-1.0, 0.0, 0.0]),
        '-y': np.array([0.0, -1.0, 0.0]),
        '-z': np.array([0.0, 0.0, -1.0]),
    }
    if axis_str not in axis_map:
        raise ValueError(f"Invalid axis '{axis_str}'. Expected one of {list(axis_map.keys())}")
    return axis_map[axis_str]


def _rotation_matrix_from_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return R such that R @ a == b (both 3D)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm == 0 or b_norm == 0:
        raise ValueError("Cannot rotate from/to zero-length vector")
    a = a / a_norm
    b = b / b_norm

    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        # Parallel or anti-parallel
        if c > 0.0:
            return np.eye(3)
        # 180 deg: choose any orthogonal axis
        ortho = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        v = np.cross(a, ortho)
        v = v / np.linalg.norm(v)
        # Rodrigues for 180 degrees: R = -I + 2 vv^T
        return -np.eye(3) + 2.0 * np.outer(v, v)

    s = float(np.linalg.norm(v))
    vx = np.array(
        [[0.0, -v[2], v[1]],
         [v[2], 0.0, -v[0]],
         [-v[1], v[0], 0.0]],
        dtype=float,
    )
    # Rodrigues' rotation formula
    R = np.eye(3) + vx + (vx @ vx) * ((1.0 - c) / (s * s))
    return R


def create_cylinder_mesh(
    center: np.ndarray,
    axis: np.ndarray,
    radius: float,
    height: float,
    n_segments: int = 16
) -> mesh.Mesh:
    """Create cylinder mesh aligned with axis.
    
    Args:
        center: (3,) cylinder center position in mm
        axis: (3,) unit axis vector (cylinder points along this direction)
        radius: Cylinder radius in mm
        height: Cylinder height in mm
        n_segments: Number of radial segments (higher = smoother)
        
    Returns:
        mesh.Mesh object with cylinder triangles
    """
    # Normalize axis
    axis = axis / np.linalg.norm(axis)
    
    # Create orthogonal basis (axis, u, v)
    # Choose u perpendicular to axis
    if abs(axis[2]) < 0.9:
        u = np.cross(axis, np.array([0, 0, 1]))
    else:
        u = np.cross(axis, np.array([1, 0, 0]))
    u = u / np.linalg.norm(u)
    
    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)
    
    # Generate cylinder vertices
    theta = np.linspace(0, 2 * np.pi, n_segments, endpoint=False)
    
    # Bottom circle (at center - height/2 * axis)
    bottom_center = center - (height / 2) * axis
    bottom_verts = []
    for t in theta:
        point = bottom_center + radius * (np.cos(t) * u + np.sin(t) * v)
        bottom_verts.append(point)
    bottom_verts = np.array(bottom_verts)
    
    # Top circle (at center + height/2 * axis)
    top_center = center + (height / 2) * axis
    top_verts = []
    for t in theta:
        point = top_center + radius * (np.cos(t) * u + np.sin(t) * v)
        top_verts.append(point)
    top_verts = np.array(top_verts)
    
    # Build triangles
    faces = []
    
    # Side faces (2 triangles per segment)
    for i in range(n_segments):
        i_next = (i + 1) % n_segments
        
        # Triangle 1: bottom[i], bottom[i+1], top[i]
        faces.append([bottom_verts[i], bottom_verts[i_next], top_verts[i]])
        
        # Triangle 2: top[i], bottom[i+1], top[i+1]
        faces.append([top_verts[i], bottom_verts[i_next], top_verts[i_next]])
    
    # Bottom cap (triangles radiating from center)
    for i in range(n_segments):
        i_next = (i + 1) % n_segments
        faces.append([bottom_center, bottom_verts[i_next], bottom_verts[i]])
    
    # Top cap (triangles radiating from center)
    for i in range(n_segments):
        i_next = (i + 1) % n_segments
        faces.append([top_center, top_verts[i], top_verts[i_next]])
    
    # Convert to mesh
    faces = np.array(faces)
    cylinder_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    
    for i, face in enumerate(faces):
        for j in range(3):
            cylinder_mesh.vectors[i][j] = face[j]
    
    return cylinder_mesh


def create_cone_mesh(
    tip: np.ndarray,
    axis: np.ndarray,
    base_radius: float,
    height: float,
    n_segments: int = 16
) -> mesh.Mesh:
    """Create cone mesh with tip at specified point.
    
    Args:
        tip: (3,) cone tip position in mm
        axis: (3,) unit axis vector (cone points along this direction)
        base_radius: Base radius in mm
        height: Cone height in mm
        n_segments: Number of radial segments
        
    Returns:
        mesh.Mesh object with cone triangles
    """
    # Normalize axis
    axis = axis / np.linalg.norm(axis)
    
    # Create orthogonal basis
    if abs(axis[2]) < 0.9:
        u = np.cross(axis, np.array([0, 0, 1]))
    else:
        u = np.cross(axis, np.array([1, 0, 0]))
    u = u / np.linalg.norm(u)
    
    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)
    
    # Base circle (at tip + height * axis)
    base_center = tip + height * axis
    theta = np.linspace(0, 2 * np.pi, n_segments, endpoint=False)
    
    base_verts = []
    for t in theta:
        point = base_center + base_radius * (np.cos(t) * u + np.sin(t) * v)
        base_verts.append(point)
    base_verts = np.array(base_verts)
    
    # Build triangles
    faces = []
    
    # Side faces (triangles from tip to base edge)
    for i in range(n_segments):
        i_next = (i + 1) % n_segments
        faces.append([tip, base_verts[i], base_verts[i_next]])
    
    # Base cap (triangles radiating from center)
    for i in range(n_segments):
        i_next = (i + 1) % n_segments
        faces.append([base_center, base_verts[i_next], base_verts[i]])
    
    # Convert to mesh
    faces = np.array(faces)
    cone_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    
    for i, face in enumerate(faces):
        for j in range(3):
            cone_mesh.vectors[i][j] = face[j]
    
    return cone_mesh


def create_implant_marker(
    centroid: np.ndarray,
    axis: np.ndarray,
    cylinder_radius: float = 2.0,
    cylinder_height: float = 12.0,
    cone_radius: float = 1.0,
    cone_height: float = 6.0
) -> mesh.Mesh:
    """Create combined cylinder + cone marker for one implant.
    
    Args:
        centroid: (3,) implant centroid in mm
        axis: (3,) unit axis vector
        cylinder_radius: Cylinder radius in mm
        cylinder_height: Cylinder height in mm
        cone_radius: Cone base radius in mm
        cone_height: Cone height in mm
        
    Returns:
        Combined mesh.Mesh object
    """
    # Cylinder centered at centroid
    cylinder = create_cylinder_mesh(
        centroid,
        axis,
        cylinder_radius,
        cylinder_height
    )
    
    # Cone tip at top of cylinder
    cone_tip = centroid + (cylinder_height / 2) * axis / np.linalg.norm(axis)
    cone = create_cone_mesh(
        cone_tip,
        axis,
        cone_radius,
        cone_height
    )
    
    # Combine meshes
    combined = mesh.Mesh(np.concatenate([cylinder.data, cone.data]))
    
    return combined


def generate_constellation(
    implants_path: str,
    output_path: str,
    cylinder_radius: float = 2.0,
    cylinder_height: float = 12.0,
    cone_radius: float = 1.0,
    cone_height: float = 6.0,
    scanbody_stl: str | None = None,
    scanbody_axis: str = 'z',
    scanbody_scale: float = 1.0,
    scanbody_recenter: str = 'none'
) -> dict:
    """Generate constellation STL from implants JSON.
    
    Args:
        implants_path: Path to implants JSON (U-frame or I-frame)
        output_path: Path to save STL file
        cylinder_radius: Implant body radius in mm
        cylinder_height: Implant body height in mm
        cone_radius: Axis indicator cone radius in mm
        cone_height: Axis indicator cone height in mm
        
    Returns:
        Dictionary with generation statistics
        
    Raises:
        FileNotFoundError: If implants_path doesn't exist
        ValueError: If JSON format is invalid
    """
    # Load implants
    with open(implants_path, 'r') as f:
        data = json.load(f)
    
    if "implants" not in data:
        raise ValueError("Missing 'implants' field in JSON")
    
    frame = data.get("frame", "U")
    units = data.get("units", "mm")
    
    if units != "mm":
        raise ValueError(f"Expected mm units, got {units}")
    
    print(f"Generating constellation for {len(data['implants'])} implants...")
    print(f"  Frame: {frame}")

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    if scanbody_stl:
        # Template scanbody mode (uses trimesh)
        try:
            import trimesh
        except Exception as e:
            raise RuntimeError("scanbody STL mode requires 'trimesh'. Install with: pip install trimesh") from e

        scanbody_path = Path(scanbody_stl)
        if not scanbody_path.exists():
            raise FileNotFoundError(f"Scanbody STL not found: {scanbody_path}")

        template = trimesh.load(str(scanbody_path))
        if isinstance(template, trimesh.Scene):
            template = trimesh.util.concatenate([g for g in template.geometry.values()])

        if scanbody_scale != 1.0:
            template = template.copy()
            template.apply_scale(float(scanbody_scale))

        recenter = (scanbody_recenter or 'none').lower()
        if recenter not in ('none', 'bbox', 'centroid'):
            raise ValueError("--scanbody-recenter must be one of: none, bbox, centroid")
        if recenter != 'none':
            template = template.copy()
            if recenter == 'bbox':
                offset = template.bounding_box.centroid
            else:
                offset = template.centroid
            template.apply_translation(-offset)

        a0 = _axis_from_string(scanbody_axis)

        placed = []
        for implant_id, implant in data["implants"].items():
            centroid = np.array(implant["centroid_mm"], dtype=float)
            axis = np.array(implant["axis_vector"], dtype=float)
            axis_norm = np.linalg.norm(axis)
            if axis_norm == 0:
                raise ValueError(f"Implant {implant_id} has zero axis_vector")
            axis = axis / axis_norm

            R = _rotation_matrix_from_vectors(a0, axis)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = centroid

            inst = template.copy()
            inst.apply_transform(T)
            placed.append(inst)
            print(f"  Placed scanbody for implant {implant_id} (marker {implant.get('marker_id', 'N/A')})")

        combined = trimesh.util.concatenate(placed)
        combined.export(output_path)

        bbox_min = combined.bounds[0]
        bbox_max = combined.bounds[1]
        bbox_size = bbox_max - bbox_min

        n_triangles = int(len(combined.faces))
        print(f"  Geometry: scanbody STL template '{scanbody_path.name}'")
    else:
        # Default primitive mode (numpy-stl)
        print(f"  Geometry: {cylinder_radius}mm × {cylinder_height}mm cylinder + {cone_radius}mm × {cone_height}mm cone")

        all_meshes = []
        for implant_id, implant in data["implants"].items():
            centroid = np.array(implant["centroid_mm"])
            axis = np.array(implant["axis_vector"])

            implant_mesh = create_implant_marker(
                centroid,
                axis,
                cylinder_radius,
                cylinder_height,
                cone_radius,
                cone_height
            )
            all_meshes.append(implant_mesh)
            print(f"  Created marker for implant {implant_id} (marker {implant.get('marker_id', 'N/A')})")

        combined_data = np.concatenate([m.data for m in all_meshes])
        constellation = mesh.Mesh(combined_data)
        constellation.save(output_path)

        vertices = constellation.vectors.reshape(-1, 3)
        bbox_min = vertices.min(axis=0)
        bbox_max = vertices.max(axis=0)
        bbox_size = bbox_max - bbox_min
        n_triangles = int(len(constellation.data))
    
    print(f"\nConstellation saved: {output_path}")
    print(f"  Triangles: {n_triangles}")
    print(f"  Bounding box: [{bbox_min[0]:.1f}, {bbox_min[1]:.1f}, {bbox_min[2]:.1f}] to [{bbox_max[0]:.1f}, {bbox_max[1]:.1f}, {bbox_max[2]:.1f}]")
    print(f"  Size: {bbox_size[0]:.1f} × {bbox_size[1]:.1f} × {bbox_size[2]:.1f} mm")
    
    return {
        "n_implants": len(data["implants"]),
        "n_triangles": n_triangles,
        "frame": frame,
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
        "output_path": output_path
    }


def main():
    parser = argparse.ArgumentParser(
        description="Generate implant constellation STL visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Generate U-frame constellation
    python tools/generate_constellation.py \\
        --implants runs/case001/implants_U.json \\
        --output runs/case001/U_Constellation.stl
    
    # Generate I-frame constellation with custom geometry
    python tools/generate_constellation.py \\
        --implants runs/case001/implants_I.json \\
        --output runs/case001/I_Constellation.stl \\
        --cylinder-radius 1.5 \\
        --cylinder-height 10.0
        """
    )
    
    parser.add_argument(
        "--implants",
        type=str,
        required=True,
        help="Path to implants JSON file (U-frame or I-frame)"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Path to save STL file"
    )
    
    parser.add_argument(
        "--scanbody-stl",
        type=str,
        default=None,
        help="Optional: path to a scanbody STL to place at each implant (uses trimesh). If set, primitive cylinder/cone options are ignored."
    )

    parser.add_argument(
        "--scanbody-axis",
        type=str,
        default='z',
        choices=['x', 'y', 'z', '-x', '-y', '-z'],
        help="Template axis direction in the scanbody STL that should align with implant axis_vector (default: z)"
    )

    parser.add_argument(
        "--scanbody-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to scanbody STL before placement (default: 1.0)"
    )

    parser.add_argument(
        "--scanbody-recenter",
        type=str,
        default='none',
        choices=['none', 'bbox', 'centroid'],
        help="Recenter scanbody STL before placement: none | bbox | centroid (default: none)"
    )

    parser.add_argument(
        "--cylinder-radius",
        type=float,
        default=2.0,
        help="Implant body cylinder radius in mm (default: 2.0)"
    )
    
    parser.add_argument(
        "--cylinder-height",
        type=float,
        default=12.0,
        help="Implant body cylinder height in mm (default: 12.0)"
    )
    
    parser.add_argument(
        "--cone-radius",
        type=float,
        default=1.0,
        help="Axis indicator cone base radius in mm (default: 1.0)"
    )
    
    parser.add_argument(
        "--cone-height",
        type=float,
        default=6.0,
        help="Axis indicator cone height in mm (default: 6.0)"
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Phase 6: Generate Implant Constellation STL")
    print("=" * 70)
    print()
    
    results = generate_constellation(
        args.implants,
        args.output,
        cylinder_radius=args.cylinder_radius,
        cylinder_height=args.cylinder_height,
        cone_radius=args.cone_radius,
        cone_height=args.cone_height,
        scanbody_stl=args.scanbody_stl,
        scanbody_axis=args.scanbody_axis,
        scanbody_scale=args.scanbody_scale,
        scanbody_recenter=args.scanbody_recenter
    )
    
    print(f"\n{'=' * 70}")
    print("[OK] Constellation generation complete")
    print(f"  {results['n_implants']} implants -> {results['n_triangles']} triangles")
    print(f"{'=' * 70}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
