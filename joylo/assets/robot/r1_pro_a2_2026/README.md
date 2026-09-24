# Official Galaxea R1 Pro 2026 / A2

This is the **seven-joint-per-arm** R1 Pro model used by the comparison viewer.
It is separate from `../r1_pro/r1_pro.urdf`, whose arm chains contain six joints.
That older BRS asset is not overwritten.

Source: https://github.com/userguide-galaxea/URDF/tree/343902060f14622b6048d63b698423443eb4c26d/R1Pro/urdf_r1pro_g1z_2026

- Commit: `343902060f14622b6048d63b698423443eb4c26d`.
- `upstream.urdf`: unchanged official `urdf/r1pro_2026.urdf`.
- `meshes/`: 43 unchanged official mesh files.
- `r1_pro_a2_2026.urdf`: only the `package://r1pro_urdf/meshes/...` URIs
  are rewritten to `meshes/...` for loading without ROS. Geometry, joints,
  limits and inertial data are unchanged.
- All 44 downloaded files were checked against the upstream Git blob SHA-1.

This is the manufacturer's 2026 variant, not an assertion that it is the exact
sensor/gripper configuration used in the 2025 BEHAVIOR Challenge.

Official product documentation:
https://docs.galaxea-ai.com/Guide/R1Pro/hardware_introduction/R1Pro_Hardware_Introduction/

The upstream 2025 URDF has slightly different limits; the viewer uses this
2026 URDF consistently, without mixing 2025 geometry and 2026 limits.
