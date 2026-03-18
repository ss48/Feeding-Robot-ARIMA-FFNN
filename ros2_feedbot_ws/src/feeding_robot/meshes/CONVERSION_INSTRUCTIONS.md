# STEP to STL Mesh Conversion Instructions

## Source Files
Located at: `CAD/V41 111123/STP/`

| STEP File | Target Link(s) | Notes |
|-----------|----------------|-------|
| Base.step | base_link, turntable_link | May need splitting into fixed base and rotating turntable |
| Arm.step | lower_arm_link, upper_arm_link | Split into lower and upper arm segments at elbow joint |
| Feeder.step | feeder_link | Fork/spoon end-effector mechanism |
| PCBHolder.step | base_link (decorative) | Electronics mount, attach to base |
| ReinforcedBrackets.step | joint brackets | Attach to appropriate joint links |

## Method 1: FreeCAD (Free, Recommended)

1. Install FreeCAD: https://www.freecad.org/
2. Open each `.step` file: File > Open
3. Select all bodies in the model tree
4. Export as STL: File > Export > Select `.stl` format

### For Visual Meshes (high quality)
- Export at full resolution
- Save to `meshes/visual/` directory
- Optionally export as `.dae` (Collada) to preserve colours

### For Collision Meshes (simplified)
- Use Mesh Design workbench
- Mesh > Mesh from shape (reduce to 1000-5000 faces)
- Save to `meshes/collision/` directory

### Splitting Arm.step
The Arm.step file (3.7MB) contains both arm segments. To split:
1. Open in FreeCAD
2. Identify the lower and upper arm bodies in the model tree
3. Select only the lower arm bodies > Export as `lower_arm.stl`
4. Select only the upper arm bodies > Export as `upper_arm.stl`

## Method 2: Blender (Free)

1. Install Blender: https://www.blender.org/
2. Install the STEP importer add-on (or use FreeCAD to convert STEP > STL first)
3. Import the STL
4. Simplify: Modifier > Decimate (for collision meshes)
5. Export as STL

## Important Notes

### Origin Alignment
- Each mesh's origin MUST align with its parent joint axis in the URDF
- If the STEP file origin doesn't match, adjust the `<origin>` tag in the xacro file
- Test in RViz with `display.launch.py` and adjust offsets iteratively

### Units
- STEP files are typically in millimetres
- URDF uses metres
- When exporting STL, ensure units are in metres (scale by 0.001 if needed)
- In FreeCAD: Edit > Preferences > Import-Export > check unit settings

### After Conversion
Place files as follows:
```
meshes/
  visual/
    base.stl
    turntable.stl
    lower_arm.stl
    upper_arm.stl
    feeder.stl
    pcb_holder.stl
    reinforced_brackets.stl
  collision/
    base.stl
    turntable.stl
    lower_arm.stl
    upper_arm.stl
    feeder.stl
```

Then launch with meshes enabled:
```bash
ros2 launch feeding_robot display.launch.py use_meshes:=true
```
