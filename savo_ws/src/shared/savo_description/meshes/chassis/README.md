# Chassis meshes

Optional visual/collision meshes for the measured chassis. The current production description must not infer physical geometry from placeholder mesh files. Add only reviewed assets with correct units, origin, license, and matching locked geometry.

## Current Robot Savo assets

All four physical plates are 4 mm thick. The binary STL coordinates are in
millimetres and require a `0.001` scale when used from URDF.

| Asset | Raw STL bounds (mm) | Physical meaning |
| --- | --- | --- |
| `Base_Layer.STL` | 210 x 28 x 279.6 | 4 mm base plate plus integrated DC-motor mounts |
| `first_Layer.STL` | 210 x 4 x 279.6 | Full-size first plate |
| `second_Layer.STL` | 210 x 4 x 279.6 | Full-size second plate |
| `Third_Layer.STL` | 144 x 4 x 142 | Smaller RPLIDAR-only third plate |

The 28 mm base-mesh envelope must not be treated as plate thickness. Keep
primitive 4 mm plate collision geometry and model motor-mount collision
separately if required. CAD-to-ROS axis signs, mesh origins, and the asset
license still require confirmation before these files become production URDF
visuals.

Do not add generated build products here. Prefer lightweight visual meshes and conservative primitive collision geometry unless a mesh has been validated for real-time collision use.
