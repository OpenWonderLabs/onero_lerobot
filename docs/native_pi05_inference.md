# 原生 LeRobot 0.6.1 PI0.5 推理

适用于本分支的 Onero 接口 0.2.5。模型、处理器、同步/RTC 推理引擎和 rollout
策略来自官方 LeRobot 0.6.1。插件不修改或复制 LeRobot 源码，也不依赖 `one_vla`。

## 本分支提供什么

- 保留已有的 16 维 interleaved 布局：左 7 关节、左夹爪、右 7 关节、右夹爪。
- 新增 `onero-h1-pi05-rollout`。它复用原生配置、context 和 strategy，在策略启动前
  逐个核验实际模型参数与 checkpoint；上游加载器吞掉错误并返回随机模型时，核验失败即退出。
- 新增可选输出 Kalman：`--robot.action_filter=kalman`。默认 `none`，不改变旧命令。
- 严格入口要求真实、完整且未过期的状态与图像，左右夹爪分别计时；不填充缺失图像。
- 严格入口在构建 context 前关闭 `auto_start_arm_node` 和 `auto_enter_teleop`，
  不自动启动主臂节点或发送遥操握手；已有采集和遥操命令保持原行为。
- 编译仍由原生 `--use_torch_compile` 控制，默认关闭。启用时仅在内存中选择
  `max-autotune-no-cudagraphs`，checkpoint 文件和训练统计保持原样。

普通 `lerobot-rollout` 与 `onero-h1-lerobot-rollout` 仍使用上游加载行为。
需要上述加载保障时，显式使用新增的 `onero-h1-pi05-rollout`。
当前严格入口接受本地完整 PI05、16 维 H1 绝对关节动作；不接受 PEFT 或相对动作模型。

## 安装和环境

按项目安装流程激活 Python 3.12 环境，安装官方版本和当前插件源码：

```bash
uv pip install 'lerobot[pi,core_scripts]==0.6.1'
uv pip install -e .
onero-h1-pi05-rollout --help
```

Torch/CUDA 使用对应设备已验证的版本。部署终端加载 ROS Jazzy，并按本机站点配置设置
RMW、ROS domain 和路由。机器人地址、模型路径与离线 tokenizer 路径保留在操作员配置中。

```bash
source /opt/ros/jazzy/setup.bash
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
export ONERO_POLICY_PATH=/absolute/path/to/pretrained_model
export ONERO_RENAME_MAP='{"observation.images.head":"observation.images.base_0_rgb","observation.images.left":"observation.images.left_wrist_0_rgb","observation.images.right":"observation.images.right_wrist_0_rgb"}'
```

若 checkpoint 的相机名已与机器人一致，使用 `ONERO_RENAME_MAP='{}'`。
保持配套的 preprocessor/postprocessor 和统计文件；不额外交换夹爪，不重新计算或替换统计。

## 先做真实观测无动作检查

下例使用原生同步引擎，运行 30 秒，不发布控制，也不在退出时回位：

```bash
onero-h1-pi05-rollout \
  --strategy.type=base \
  --policy.path="$ONERO_POLICY_PATH" \
  --robot.type=onero_h1 --robot.id=h1 \
  --robot.use_lift=false --robot.policy_feature_layout=interleaved \
  --robot.send_action=false --robot.use_cameras=true \
  --robot.fill_missing_images=false \
  --robot.shutdown_rclpy_on_disconnect=true \
  --robot.camera_shapes='{"head":[168,224,3],"left":[168,224,3],"right":[168,224,3]}' \
  --rename_map="$ONERO_RENAME_MAP" \
  --inference.type=sync --use_torch_compile=false \
  --device=cuda --fps=30 \
  --task="Flatten and fold the cloth precisely, then stack them properly." \
  --display_data=false --return_to_initial_position=false --duration=30
```

检查 `PI05 loaded-weight verification passed`、相机与状态就绪、实际推理输出和正常退出。
该检查验证模型加载与数据链路，任务质量需要现场另行验证。

## 开启输出 Kalman

无动作检查完成后，操作员在同一命令中将 `--robot.send_action` 改为 `true`，并增加：

```bash
--robot.action_filter=kalman
```

若要连续运行，再将 `--duration` 改为 `0`。启动前停止主臂或其他动作发布者。
每条位置指令在已有安全限位之前只滤波一次，滤波从实测反馈初始化；左右第 1 关节使用
最短圆周差。内部角度连续，最终输出仍遵守本接口已有的关节范围，不绕过限位。
被动模式不运行滤波，不推进滤波历史；断开重连会清理历史。

可调参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--robot.kalman_process_variance` | 0.5 | 位置/速度过程噪声 |
| `--robot.kalman_threshold` | 0.075 | 相邻目标变化超过该值时切换测量噪声 |
| `--robot.kalman_jump_variance` | 15.0 | 跳变时的测量噪声；普通步骤使用 7.5 |

严格入口自动按原生实际命令频率设置滤波步长，包括 `interpolation_multiplier`。
普通 `lerobot-rollout` 也可使用 `--robot.action_filter=kalman`，但需让
`--robot.action_filter_fps` 与实际命令频率一致；它不会因此获得严格权重核验。

## RTC、图像与编译的边界

Kalman 只处理输出流，不做额外动作块平均或 temporal smoothing，也不改变原生队列。
本例使用 `sync`，RTC 关闭。已有 RTC 工作流可继续显式选择
`--inference.type=rtc --inference.rtc.enabled=true`，插件不自动切换它。
`rtc.enabled=false` 的原生后台队列与 `one_vla` 的 async 调度不同，不能直接照搬
`chunk-size-threshold`、`inference-mode`、`remote-observation-similarity-filter` 等参数。

相机先解码为 RGB，再用 AREA 等比例缩小为 168×224，保持 HWC uint8。原生 PI05
预处理上下各补 28 行，得到 224×224。严格模式拒绝会拉伸或放大源图的尺寸配置。
这一原生路径原本就不会经过 async-gRPC 的方形特征 resize，不需要移植服务端特征视图。

启用 `--use_torch_compile=true` 时，首轮包含编译与原生 warmup。先在无动作模式验证，
分别记录冷启动与稳定推理耗时。适配器未改变去噪步数、BF16 精度或模型算法。

## 开发验证

```bash
uv sync --extra dev --extra camera
uv pip install 'lerobot[pi,core_scripts]==0.6.1'
uv run --no-sync pytest -q
uv run --no-sync ruff check src tests
```

测试覆盖真实的 0.6.1 图像准备/PI05 补边和 key-fixup API；加载失败、错位特征、非有限动作、
夹爪断流和圆周角边界均有回归。CPU 单测不证明 GPU 性能或机器人任务效果。

2026-11-25：587 step63490 实际权重触发 embedding 核验误报，尽管原生加载报告全部键成功。
诊断：原生 key-fixup 会从 `lm_head` 复制 embedding，完整字典后续键可覆盖该副本；
逐张量调用 hook 会丢失覆盖顺序。修复：先用完整键集解析原生映射，再流式比对实际参数；
保留非有限值、未知键及覆盖完整性检查，并增加前缀兼容和错误 embedding/head 的回归测试。
