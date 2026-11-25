# Onero H1 LeRobot Adapter

English | [中文](README_zh.md)

A LeRobot `Robot` / `Teleoperator` adapter for the Onero H1 / SwitchBot H1 ROS 2 SDK. Supports teleoperation, dataset recording/replay, ACT / pi0.5 training and policy rollout, with MuJoCo tools for simulation validation.

> **Safety: teleoperation, active recording, replay and rollout can move the robot.** Clear the workspace, keep the emergency stop accessible and stop competing control publishers. Start installation acceptance with read-only observations, not motion examples.

## Start here

| Goal | Guide |
|---|---|
| Installation, wiring, networking, activation and read-only acceptance | [Setup guide](docs/setup.md) (Chinese) |
| Teleoperation, recording, replay, visualization, training and rollout | [Usage guide](docs/usage.md) (Chinese) |
| Native PI0.5 rollout, loaded-weight verification and optional Kalman filtering | [PI0.5 inference guide](docs/native_pi05_inference.md) (Chinese) |
| H1 model files, provenance and upstream compatibility | [Robot model](urdf/README.md) |
| Field operations reference | [Feishu SOP](https://ihqz5dyhwa.feishu.cn/wiki/EC81wA9H0iuzOZkbM4bcu6b0nHd) (access required) |

## Installation paths

Recommended baseline: **Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12+, pinned LeRobot 0.6.1**.

- **Fresh x86_64 operator PC requiring communication and leader-arm setup:** use the [separate full-system installer](https://github.com/OpenWonderLabs/Onero_Lerobot_setup). It changes system services and device rules, not just Python packages. Review the setup guide first.
- **Source development with an existing SDK/communication environment:** run `bash scripts/setup.sh` for collection; GPU workstations can then run `bash scripts/setup_ubuntu24_native.sh`.
- **Jetson Orin:** use `bash scripts/setup_orin_native.sh`; do not mix it with the workstation CUDA installer.

The full-system installer generates `~/h1_env.sh`; activate with `source ~/h1_env.sh`. This repository's setup scripts do not generate that file: load ROS, the site communication environment and the venv manually as described in [setup](docs/setup.md).

After activation, start with read-only checks:

```bash
onero-h1-print-observation --no-cameras --count 3
onero-h1-print-observation --count 3
```

## Fixed 16-dimensional schema

Policy state and action use this interleaved order:

```text
left_joint_1..7.pos, left_gripper.pos, right_joint_1..7.pos, right_gripper.pos
```

Seven left joints, left gripper, seven right joints, right gripper. Lift, head, base and velocity channels are excluded from the scalar policy schema. There is no selectable native/interleaved mode.
`policy_feature_layout=interleaved` is accepted as a compatibility argument for existing SOP commands; other layout values are rejected.
Default cameras are `head`, `left`, `right`, exposed as separate image features. The adapter translates hardware names to the ROS interfaces.

Datasets, training and checkpoints must agree on feature order, cameras and preprocessing. Do not truncate or pad incompatible checkpoints.

## Command entry points

Prefer native `lerobot-teleoperate`, `lerobot-record`, `lerobot-replay`, `lerobot-dataset-viz`, `lerobot-train` and `lerobot-rollout`. Full examples are in the [usage guide](docs/usage.md).

Additional collection/debugging tools:

| Command / script | Purpose |
|---|---|
| `onero-h1-print-observation` | Read observations without automatically starting the leader arm |
| `onero-h1-pi05-rollout` | Native PI05 rollout with strict feature/feedback checks, actual loaded-weight verification and optional output Kalman filtering |
| `onero-h1-record-episode` / `scripts/record.sh` | Separate control and recording rates; active/passive collection |
| `onero-h1-replay-episode` / `scripts/replay.sh` | Real-robot replay with automatic teleop handshake; differs from native replay |
| `scripts/viz.sh` | Offline dataset visualization |
| `scripts/run_sim_train_rollout.sh` | MuJoCo → recording → short ACT training → simulation rollout |

`send_action=false` disables adapter action publication, but does not by itself guarantee that leader-arm startup or a handshake cannot affect robot state. See the usage guide for control ownership, recording shutdown and replay differences.

## Simulation and development

The default model is `urdf/urdf/H1_mjcf_with_grippers.urdf`, with assets in `urdf/meshes/`. Real-robot ROS operation does not directly load this URDF. Keep the complete source checkout for repository-level simulation tools; installing the Python package alone does not replace model assets.

Gitee `master` is the code baseline; GitHub is the synchronized publication remote. The separate system installer installs the adapter from GitHub by default. Check the source revision, installation source and virtual environment before development updates.
