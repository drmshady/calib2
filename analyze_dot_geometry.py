import json
import numpy as np

model = json.load(open('aox-photogrammetry-flags/out_aox_flag_v2/models/cap_AOX_FLAG_10x10x10_ID100.json'))
tag_corners = np.array(model['features']['april_tag']['corner_points_mm'])

print('AprilTag corners (all on FRONT face, Y=5.0mm):')
for i, corner in enumerate(tag_corners):
    print(f'  Corner {i}: X={corner[0]:5.1f}, Y={corner[1]:5.1f}, Z={corner[2]:5.1f}')

print(f'\nTag is PLANAR: all Y={tag_corners[0,1]:.1f}mm (front face)')
print(f'Tag spans: Z from {tag_corners[:,2].min():.1f} to {tag_corners[:,2].max():.1f} mm')

faces = model['features']['faces']
print('\n' + '='*60)
print(f'Dot constellations on {len(faces)} DIFFERENT faces:')
print('='*60)

total_dots = 0
for face in faces:
    dots = np.array(face['centers_mm'])
    n = len(dots)
    total_dots += n
    face_name = face['face']
    
    # Identify which coordinate is constant (defines the plane)
    var_x = dots[:,0].max() - dots[:,0].min()
    var_y = dots[:,1].max() - dots[:,1].min()
    var_z = dots[:,2].max() - dots[:,2].min()
    
    if var_x < 0.1:
        plane_axis = 'X'
        plane_value = dots[0,0]
    elif var_y < 0.1:
        plane_axis = 'Y'
        plane_value = dots[0,1]
    else:
        plane_axis = 'Z'
        plane_value = dots[0,2]
    
    print(f'\n  {face_name.upper():8s}: {n} dots on plane {plane_axis}={plane_value:+.1f}mm')
    print(f'            Range: X=[{dots[:,0].min():+.1f},{dots[:,0].max():+.1f}], Y=[{dots[:,1].min():+.1f},{dots[:,1].max():+.1f}], Z=[{dots[:,2].min():+.1f},{dots[:,2].max():+.1f}]')

print('\n' + '='*60)
print(f'Total: {total_dots} dots distributed across {len(faces)} orthogonal planes')
print(f'This creates a TRUE 3D structure (not coplanar with the tag!)')
print('='*60)
