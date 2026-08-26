# Onero H1 / X1 robot model

This directory is copied verbatim from the BeaVR deployment model at:

```text
beavr-teleop/deploy/urdf/
```

Canonical BeaVR MuJoCo-loadable model:

```text
urdf/X1_mjcf.urdf
```

Simulator model with simple visual grippers added by this project:

```text
urdf/X1_mjcf_with_grippers.urdf
```

The derived model leaves every BeaVR link and joint unchanged. It adds a fixed
palm and two opposing prismatic fingers to each `Link_l7` / `Link_r7` wrist.
Each normalized gripper command drives both fingers from 0 to 22 mm.

The only difference between `X1_mjcf.urdf` and `X1_URDF_V1_2.urdf` is mesh
path syntax: the MuJoCo variant uses relative `../meshes/*.STL` paths instead
of ROS `package://X1_URDF_V1_2/meshes/*.STL` paths.

Model contents:

- mobile base, drive/steering wheels and support wheels
- lift column (`sj`)
- head yaw/pitch (`t01`, `t02`)
- right arm (`r1` through `r7`)
- left arm (`l1` through `l7`)
- 30 STL meshes, 30 canonical links and 29 canonical joints
- derived simulator model: 36 links and 35 joints after adding fake grippers

The LeRobot simulator maps Onero SDK ROS joint names `joint1-l..joint7-l` and
`joint1-r..joint7-r` to these URDF arm joint names. Do not replace this model
with the former hand-written approximate MJCF.
