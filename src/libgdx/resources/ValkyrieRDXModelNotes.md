# Valkyrie Model Notes

For some reason, in Blender, the frames for importing & exporting are wrong.

To get things to work, the obj files were imported into Blender with:

- Clamp Size: 0.00
- Forward: Y Forward
- Up: Z Up

and exported to FBX with:

- Selected objects: Unchecked
- Scale: 0.01
- Apply Scalings: All Local
- Forward: -Z Forward
- Up: Y Up
- Apply Unit: Checked
- Use Space Transform: Unchecked
- Apply Transform: Unchecked
