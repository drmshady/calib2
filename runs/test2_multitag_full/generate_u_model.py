import numpy as np
import json
import trimesh
import copy
from pathlib import Path

# --- CONFIGURATION ---
JSON_PATH = 'implants_U.json'          # Photogrammetry implants (U-frame)
LIBRARY_PATH = 'scanbody.stl'          # Scan body STL in this folder
OUTPUT_PATH = 'U_Constellation.stl'    # Output STL

def create_constellation():
    # 1. Load Data
    try:
        with open(JSON_PATH, 'r') as f:
            data = json.load(f)

        # Resolve STL path (prefer configured name; otherwise auto-pick first .stl in folder)
        folder = Path(__file__).resolve().parent
        stl_path = folder / LIBRARY_PATH
        if not stl_path.exists():
            stls = sorted(folder.glob('*.stl')) + sorted(folder.glob('*.STL'))
            if len(stls) == 0:
                raise FileNotFoundError(f"No STL files found in {folder}")
            stl_path = stls[0]
            print(f"[INFO] Using STL: {stl_path.name}")

        base_mesh = trimesh.load(str(stl_path))
    except FileNotFoundError as e:
        print(f"Error: Missing file. {e}")
        return

    combined_mesh = trimesh.util.concatenate([])
    
    print(f"--- Generating U-Frame Model ---")
    
    # 2. Iterate through each implant in the JSON
    implants = data.get('implants', {})
    if not isinstance(implants, dict) or len(implants) == 0:
        print("Error: implants_U.json missing/empty 'implants' dict")
        return

    for _, implant in implants.items():
        tag_id = implant.get('marker_id', 'unknown')

        # Photogrammetry output is already in mm
        origin = np.array(implant['centroid_mm'], dtype=float)

        # Build an orthonormal basis using axis_vector as Z
        uz = np.array(implant['axis_vector'], dtype=float)
        uz_norm = np.linalg.norm(uz)
        if uz_norm == 0:
            raise ValueError(f"Implant {tag_id}: axis_vector has zero length")
        uz = uz / uz_norm

        # Choose a stable reference vector not parallel to uz
        ref = np.array([1.0, 0.0, 0.0]) if abs(uz[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        ux = np.cross(ref, uz)
        ux = ux / np.linalg.norm(ux)
        uy = np.cross(uz, ux)
        
        # 3. Construct Transformation Matrix (4x4)
        # [ Rx Ry Rz  Tx ]
        # [ Rx Ry Rz  Ty ]
        # [ Rx Ry Rz  Tz ]
        # [ 0  0  0   1  ]
        transform = np.eye(4)
        transform[0:3, 0] = ux
        transform[0:3, 1] = uy
        transform[0:3, 2] = uz
        transform[0:3, 3] = origin
        
        # 4. Clone and Move the Scan Body
        current_body = copy.deepcopy(base_mesh)
        current_body.apply_transform(transform)
        
        # Add to collection
        combined_mesh = trimesh.util.concatenate([combined_mesh, current_body])
        print(f"Placed Implant (Tag {tag_id}) at {origin}")

    # 5. Export
    combined_mesh.export(OUTPUT_PATH)
    print(f"-" * 30)
    print(f"Success! Model saved to: {OUTPUT_PATH}")

if __name__ == "__main__":
    create_constellation()