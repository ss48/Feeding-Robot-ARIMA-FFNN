"""
Convert Auto Feeder SSE v222.step into individual STL mesh files for URDF.

Usage:
    python convert_step_to_stl.py
"""

import os
import sys

from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.StlAPI import StlAPI_Writer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VISUAL_DIR = os.path.join(SCRIPT_DIR, "visual")
COLLISION_DIR = os.path.join(SCRIPT_DIR, "collision")

os.makedirs(VISUAL_DIR, exist_ok=True)
os.makedirs(COLLISION_DIR, exist_ok=True)

# Find the STEP file
STEP_CANDIDATES = [
    os.path.join(SCRIPT_DIR, "..", "..", "Auto Feeder SSE v222.step"),
    os.path.join(SCRIPT_DIR, "..", "..", "..", "CAD_Unity Shabnam Added", "CAD models", "Auto Feeder SSE v222.step"),
    os.path.join(SCRIPT_DIR, "..", "..", "..", "feeder files on the D drive", "Auto Feeder SSE v222.step"),
]


def find_step_file():
    for p in STEP_CANDIDATES:
        rp = os.path.normpath(p)
        if os.path.exists(rp):
            return rp
    return None


def export_shape_to_stl(shape, filepath, linear_deflection=0.1, angular_deflection=0.5):
    """Export an OCC shape to binary STL file."""
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, angular_deflection, True)
    mesh.Perform()
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    writer.Write(shape, filepath)


def get_bounding_box(shape):
    """Get bounding box of a shape."""
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return {
        'xmin': xmin, 'ymin': ymin, 'zmin': zmin,
        'xmax': xmax, 'ymax': ymax, 'zmax': zmax,
        'dx': xmax - xmin, 'dy': ymax - ymin, 'dz': zmax - zmin,
        'cx': (xmin + xmax) / 2, 'cy': (ymin + ymax) / 2, 'cz': (zmin + zmax) / 2,
    }


def main():
    step_file = find_step_file()
    if step_file is None:
        print("ERROR: Cannot find 'Auto Feeder SSE v222.step'. Searched:")
        for p in STEP_CANDIDATES:
            print(f"  {os.path.normpath(p)}")
        sys.exit(1)

    print(f"Loading: {step_file}")
    print(f"Size: {os.path.getsize(step_file) / 1024 / 1024:.1f} MB")

    reader = STEPControl_Reader()
    status = reader.ReadFile(step_file)
    if status != IFSelect_RetDone:
        print(f"ERROR: Failed to read STEP file (status={status})")
        sys.exit(1)

    reader.TransferRoots()
    shape = reader.OneShape()
    print("STEP file loaded successfully.")

    # Assembly bounding box
    bbox = get_bounding_box(shape)
    print(f"\nAssembly bounding box (mm):")
    print(f"  X: {bbox['xmin']:.1f} to {bbox['xmax']:.1f} (width: {bbox['dx']:.1f})")
    print(f"  Y: {bbox['ymin']:.1f} to {bbox['ymax']:.1f} (depth: {bbox['dy']:.1f})")
    print(f"  Z: {bbox['zmin']:.1f} to {bbox['zmax']:.1f} (height: {bbox['dz']:.1f})")

    # Export full assembly
    print(f"\n--- Exporting full assembly ---")
    full_vis = os.path.join(VISUAL_DIR, "feeding_robot_full.stl")
    full_col = os.path.join(COLLISION_DIR, "feeding_robot_full.stl")
    export_shape_to_stl(shape, full_vis, 0.1, 0.3)
    export_shape_to_stl(shape, full_col, 1.0, 1.0)
    print(f"  Visual:    {os.path.getsize(full_vis)/1024:.0f} KB")
    print(f"  Collision: {os.path.getsize(full_col)/1024:.0f} KB")

    # Extract individual solids
    print(f"\n--- Extracting individual solids ---")
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    solids = []
    idx = 0
    while explorer.More():
        solid = TopoDS.Solid_s(explorer.Current())
        bb = get_bounding_box(solid)
        solids.append({
            'index': idx,
            'shape': solid,
            'bbox': bb,
            'volume_approx': bb['dx'] * bb['dy'] * bb['dz'],
        })
        idx += 1
        explorer.Next()

    print(f"Found {len(solids)} solids in assembly")

    # Sort by Z position (bottom to top)
    solids.sort(key=lambda s: s['bbox']['cz'])

    print(f"\n{'#':>3} {'CenterX':>8} {'CenterY':>8} {'CenterZ':>8} {'Width':>8} {'Depth':>8} {'Height':>8}")
    print("-" * 60)
    for s in solids:
        b = s['bbox']
        print(f"{s['index']:3d} {b['cx']:8.1f} {b['cy']:8.1f} {b['cz']:8.1f} {b['dx']:8.1f} {b['dy']:8.1f} {b['dz']:8.1f}")

    # Export each solid
    print(f"\n--- Exporting individual solids ---")
    for s in solids:
        name = f"solid_{s['index']:03d}"
        vis_path = os.path.join(VISUAL_DIR, f"{name}.stl")
        col_path = os.path.join(COLLISION_DIR, f"{name}.stl")
        export_shape_to_stl(s['shape'], vis_path, 0.1, 0.3)
        export_shape_to_stl(s['shape'], col_path, 1.0, 1.0)
        vis_size = os.path.getsize(vis_path) / 1024
        print(f"  {name}: center=({s['bbox']['cx']:.1f}, {s['bbox']['cy']:.1f}, {s['bbox']['cz']:.1f}) {vis_size:.0f}KB")

    print(f"\n=== CONVERSION COMPLETE ===")
    print(f"Visual meshes:    {VISUAL_DIR}")
    print(f"Collision meshes: {COLLISION_DIR}")
    print(f"\nIMPORTANT: STEP units are in millimetres.")
    print(f"In URDF xacro use: <mesh filename='...' scale='0.001 0.001 0.001'/>")


if __name__ == "__main__":
    main()
