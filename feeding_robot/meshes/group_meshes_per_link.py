"""
Group individual solid STL files into per-link meshes for URDF.

Merges solid_XXX.stl files into:
  - base_link.stl (base platform + fixed electronics)
  - shoulder_link.stl (motor stack at shoulder)
  - lower_arm_link.stl (vertical arm segment)
  - upper_arm_link.stl (horizontal arm segment)
  - feeder_link.stl (end effector + sensors)

Uses trimesh to load and merge STL files.
"""

import os
import trimesh
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VISUAL_DIR = os.path.join(SCRIPT_DIR, "visual")
COLLISION_DIR = os.path.join(SCRIPT_DIR, "collision")

# Solid grouping based on spatial analysis of the CAD bounding boxes.
# From the conversion output:
#
# Robot orientation in CAD: Z is up, X is horizontal arm direction
#
# Base platform (Z < ~88mm):
#   solids 59,60,61,62,63 - base housing, mounting plate
#
# Shoulder motor stack (Z ~88-134mm, around X=-16 to -28):
#   solids 0,1,3,7,8,9,10,58 - main motors and bracket
#   solids 2,4,5,6 - mounting screws
#
# Lower arm - vertical segment (Z ~134-338mm, X near -16):
#   solids 11,20,21 - motor housings and brackets at joints
#   solids 14,15,18,19,12,13,16,17 - mounting screws
#   solids 23,24,25,26 - arm tube segments and connectors
#
# Upper arm - horizontal segment (Z ~337-363, X from -25 to ~158):
#   solid 22 - elbow bracket
#   solids 27,36 - motor housing
#   solids 28,29,30,31,32,33,34,35 - mounting screws (around X=-25 to 15)
#   solids 37,46,47 - motor and bracket (X~136-158)
#   solids 38,39,40,41,42,43,44,45 - mounting screws (around X=138-178)
#   solids 48,49,50,51,52 - arm tube segments (X~20 to ~183)
#
# Feeder / end effector (X > ~183):
#   solids 53,54,55,56,57 - feeder mechanism + sensor mount
#   solids 64,65,66 - feeder housing, fork, camera bracket

LINK_GROUPS = {
    'base_link': [59, 60, 61, 62, 63],
    'shoulder_link': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 58],
    'lower_arm_link': [11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 25, 26],
    'upper_arm_link': [22, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
                       37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52],
    'feeder_link': [53, 54, 55, 56, 57, 64, 65, 66],
}


def merge_stls(solid_indices, source_dir):
    """Load and merge multiple STL files into one mesh."""
    meshes = []
    for idx in solid_indices:
        path = os.path.join(source_dir, f"solid_{idx:03d}.stl")
        if os.path.exists(path):
            m = trimesh.load(path)
            if isinstance(m, trimesh.Trimesh) and len(m.faces) > 0:
                meshes.append(m)
            else:
                print(f"  Warning: solid_{idx:03d}.stl is empty or invalid")
        else:
            print(f"  Warning: solid_{idx:03d}.stl not found")

    if not meshes:
        return None

    combined = trimesh.util.concatenate(meshes)
    return combined


def compute_link_info(mesh, link_name):
    """Compute and print bounding box info for a link mesh."""
    bounds = mesh.bounds  # [[xmin,ymin,zmin],[xmax,ymax,zmax]]
    center = (bounds[0] + bounds[1]) / 2
    size = bounds[1] - bounds[0]
    print(f"  {link_name}:")
    print(f"    Bounds (mm): X[{bounds[0][0]:.1f}, {bounds[1][0]:.1f}] "
          f"Y[{bounds[0][1]:.1f}, {bounds[1][1]:.1f}] "
          f"Z[{bounds[0][2]:.1f}, {bounds[1][2]:.1f}]")
    print(f"    Center (mm): ({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f})")
    print(f"    Size (mm):   ({size[0]:.1f}, {size[1]:.1f}, {size[2]:.1f})")
    print(f"    Triangles:   {len(mesh.faces)}")
    return center, size


def main():
    print("=== Grouping solid meshes into per-link STL files ===\n")

    all_centers = {}

    for link_name, indices in LINK_GROUPS.items():
        print(f"\nProcessing {link_name} (solids: {indices})")

        # Visual mesh (high res)
        vis_mesh = merge_stls(indices, VISUAL_DIR)
        if vis_mesh is None:
            print(f"  ERROR: No valid meshes for {link_name}")
            continue

        center, size = compute_link_info(vis_mesh, link_name)
        all_centers[link_name] = center

        vis_path = os.path.join(VISUAL_DIR, f"{link_name}.stl")
        vis_mesh.export(vis_path)
        print(f"    Visual:    {vis_path} ({os.path.getsize(vis_path)/1024:.0f} KB)")

        # Collision mesh (low res)
        col_mesh = merge_stls(indices, COLLISION_DIR)
        if col_mesh is not None:
            # Simplify collision mesh
            if len(col_mesh.faces) > 2000:
                col_mesh = col_mesh.simplify_quadric_decimation(2000)
            col_path = os.path.join(COLLISION_DIR, f"{link_name}.stl")
            col_mesh.export(col_path)
            print(f"    Collision: {col_path} ({os.path.getsize(col_path)/1024:.0f} KB)")

    # Print URDF origin offsets
    # The URDF joint positions define where each link's frame is.
    # The mesh origin offset = -(joint_position_in_CAD_coords) in mm, then scale to m
    print("\n\n=== URDF Mesh Origin Offsets ===")
    print("These values go in the <origin> tag of each link's <visual>/<collision>")
    print("Format: <origin xyz='X Y Z' rpy='0 0 0'/> (in metres)")
    print()

    # Joint positions in CAD coordinates (mm):
    # base_y_joint: X=-25, Y=0, Z=88 (top of base)
    # lower_z_joint: X=-16, Y=0, Z=134 (shoulder)
    # upper_z_joint: X=-5, Y=0, Z=338 (elbow - top of vertical arm)
    # feeder_joint: X=158, Y=0, Z=351 (wrist)

    joint_positions_mm = {
        'base_link':      np.array([0.0, 0.0, 0.0]),      # world origin
        'shoulder_link':  np.array([-25.0, 0.0, 88.0]),    # base_y_joint
        'lower_arm_link': np.array([-16.0, 0.0, 134.0]),   # lower_z_joint (approx)
        'upper_arm_link': np.array([-5.0, 0.0, 338.0]),    # upper_z_joint
        'feeder_link':    np.array([158.0, 0.0, 351.0]),   # feeder_joint
    }

    for link_name in LINK_GROUPS:
        joint_pos = joint_positions_mm[link_name]
        # Mesh origin offset = -joint_position (to shift mesh so joint is at link frame origin)
        offset_m = -joint_pos / 1000.0
        print(f"  {link_name}:")
        print(f"    <origin xyz=\"{offset_m[0]:.4f} {offset_m[1]:.4f} {offset_m[2]:.4f}\" rpy=\"0 0 0\"/>")

    print("\n=== DONE ===")
    print("Per-link mesh files created. Update feeding_robot_core.xacro to use them.")


if __name__ == "__main__":
    main()
