# Phase 2 SO-101 海绵任务 eval 失败复盘 — Domain Gap 三层 mismatch

**日期**: 2026-06-02
**结论**: Phase 2 「直接 eval SFT'd Pi 0.5 模型在我们 sim 里跑 ≥30% 成功率」目标本身**不可能成立**。不是 plumbing 问题、不是 norm_stats 问题，是 **cross-domain transfer** 问题。

---

## 一、Phase 2 的原定假设（事后看是错的）

1. lerobot SFT 在 `aswinkumar99/LeRobot-SO101-task1-single-sponge-no-distractors-random-locations` 数据集上跑 10k step（已完成，落盘 `outputs/sft_pi05_sponge/checkpoints/010000/`）。
2. Phase 1.5 把 lerobot ckpt 桥接成 RLinf openpi 格式（`openpi_remapped/`，已完成、key audit 通过、norm_stats 真值替换 dummy）。
3. 加载到 RLinf 自定义 sim env `Isaac-Lift-Sponge-Bowl-SO101-Play-v0`，跑 100 episodes、要 ≥30% 成功率（picking sponge → placing in bowl）作为 Phase 3 PPO 入场券。

**假设错在哪**：以为「同样 SO-101 + 同样任务（捡海绵入碗）+ 视觉看上去差不多」就可以零样本迁移。**实际上模型从未见过 sim 域**。

---

## 二、实测现象（standalone eval driver）

`scripts/eval_pi05_sponge.py` 跑通后（绕过 Ray，单进程加 RLinf `get_model()`）：

- **1 episode, 250-600 steps**: success=0, xy_dist≈0.32-0.37m, max_lift≈0.022-0.025m → 即手臂离碗 30+cm，海绵从未离开桌面 2.5cm 以上。
- **DIAG 输出**: 模型每个 chunk（5 步 horizon）的动作内部一致（chunk-内连续），但 chunk-间剧烈震荡：
  - chunk 0: 想 `shoulder_lift=-45°`
  - chunk 1: 想 `shoulder_lift=-15°`（往上抬）
  - chunk 2: 想 `shoulder_lift=-63°`（往下扎）
  - chunk 3: 又 `-41°`...
- **PD 跟踪**: 正常。commanded vs post-step state 差 ~10° 是正常 PD 滞后，**不是 PD 问题**。
- **GUI 肉眼观察**: 手臂在桌面上方一片乱晃，从未稳定下扎到海绵附近。

→ 模型在 **开环输出从 dataset 学到的视觉 → 动作映射**，因为图像 OOD，每个 chunk 的视觉判断不一致，导致动作震荡。

---

## 三、Domain gap 三层 mismatch（按影响排序）

### 层 1: 视觉 mismatch（最大）

| 维度 | Dataset (aswinkumar99) | 我们的 sim |
|------|------------------------|-----------|
| Overhead 视角 | 真实房间，桌面+海绵+碗自然光，相机后方下视约 30° | Isaac Sim 默认地板，蓝色 cuboid 当海绵，bowl USD 是测试资产，相机位姿 6 个候选都不匹配 |
| 海绵 | 真海绵纹理（蓝+黄+白海绵纤维） | `CuboidCfg` 实心蓝色 `(0.20, 0.45, 0.85)` |
| 碗 | 真陶瓷碗 | `serving_bowl.usd` (Isaac Sim test data) |
| 光照 | 房间漫反射 | RTX path tracer + Isaac Sim 默认 dome light |
| 桌面 | 木纹/桌布 | 灰色 ground plane |
| Wrist 视角 | URDF gripper 视角真机 | sim wrist_cam offset 调过但底层视觉 domain 不同 |

→ Vision encoder（PaliGemma）输出的 image embedding 完全 OOD。LLM 没办法做出正确的 action 决策。

### 层 2: 机器人动力学 mismatch（中等）

- **URDF 标定**: dataset 的 `shoulder_lift` mean=-103°、min=-104°，**超过**我们 URDF 限位 ±100°。说明真机 SO-101 出厂标定的「零点」与 URDF 模型不一致，真机能转的范围比 URDF 声明的更大。
  - 修复尝试: sim reset 改成 `[-3.16, -99.5, 96.02, 79.35, -9.70, 3.27]`（clamp 到 URDF 限位内）— 部分有效，至少让 chunk-0 不再是 `URDF zero → dataset home` 的搬运浪费。
- **PD 增益**: sim `stiffness=1000, damping=50` 是手调的「能扛住重力」值，与真机伺服响应不一定一致。
- **关节速度限制**: sim 默认 actuator effort/velocity 限制 vs 真机控制器实际响应曲线 — 没对齐。

### 层 3: Train 域全无 sim 数据（必然结论）

- SFT data = 100% 真机遥操数据
- 0 sim demonstrations
- PaliGemma + Pi 0.5 expert 学到的 visual-motor coupling 是真机域 specific 的
- → 即便把视觉、相机、关节限位全调成 sim-friendly，**不在 sim 里见过任何一帧的 policy** 在 sim 里就是没希望直接达到 30% SR

---

## 四、已尝试 / 已排除的「假问题」

1. ❌ ~~norm_stats 错~~ — Phase 1.5 已真值替换 dummy，校验 mean/std/q01/q99 数值正确
2. ❌ ~~action 单位错（rad/deg）~~ — DIAG 显示 commanded 数值范围在度（如 -45°、+50°），与 dataset 一致
3. ❌ ~~action 是 delta 而非 absolute~~ — sim state 跟踪 commanded 验证是绝对值
4. ❌ ~~PD 没跟住~~ — 跟得很好，10° 滞后属正常
5. ❌ ~~Ray plumbing 阻断 eval~~ — standalone driver 绕过 Ray，链路全通
6. ❌ ~~reset 起始 pose OOD~~ — 改成 dataset home pose 后行为模式没本质改变（chunk 间仍震荡）

---

## 五、下一步选项（按性价比排序）

### ⭐ 推荐：放弃 Phase 2 SR 阈值，直接进 Phase 3 PPO
- SFT'd policy 作为 RL **action prior**（不是「能跑通的 baseline policy」）
- 用 dense reward shaping：手到 sponge 距离衰减 + sponge 到 bowl 距离衰减 + lift_height bonus
- PPO 在 sim 里 explore，自己 fine-tune 出 sim-specific behavior
- **这本来就是 RLinf 论文里 SFT → RL post-training 的正确范式**：SFT 给一个 reasonable prior，RL 在 target domain 微调
- 待办：修 Ray plumbing 三件套（VK ICD / CUDA_VISIBLE_DEVICES / botocore）+ 修 R1 ViewBackward0 inplace bug

### 备选 A：视觉对齐（domain randomization 风格）
- 提取 dataset overhead 真值帧（已有命令 `ffmpeg -i video.mp4 -vframes 1 frame.png`）
- 调 sim 相机 intrinsics/extrinsics 至肉眼匹配
- 换 sponge USD 用真实纹理（不是 CuboidCfg）
- 换 bowl USD 用 dataset 同型号
- **能把 Phase 2 SR 提一些但难达 30%**，因为动力学 mismatch 仍在；对 Phase 3 RL 有帮助（reduce domain gap → 减少 RL 探索成本）
- **代价**: 半天到 1 天

### 备选 B：sim 里收集 demo + 混合 SFT
- 用 IK + 脚本化关键路径生成 30-50 条 sim demo
- LeRobot SFT 在 mixed real+sim data 上 fine-tune
- 再 eval Phase 2
- **能让 Phase 2 SR 真正达标**，但延期 1-2 天，且偏离原 plan
- 优势：得到 sim-native policy，Phase 3 RL 起点高

### 不推荐：继续调 eval 参数
- camera sweep / reset pose / step count 都是 marginal，本质问题没动

---

## 六、价值产出（Phase 2 不全是浪费）

虽然没达到 SR 阈值，Phase 2 standalone eval 验证了以下链路：

1. ✅ `eval_pi05_sponge.py` standalone driver 可用（绕过 Ray 调通）
2. ✅ Phase 1.5 ckpt remap 正确（811/815 key matched，dry-run load_state_dict 通过）
3. ✅ norm_stats 真值落盘正确（state/action mean/std/q01/q99 in degrees）
4. ✅ env obs/action 格式（main_images, wrist_images, states, task_descriptions）与 RLinf model 接口匹配
5. ✅ action chunk 5×6 = 30 → `actions.view(B, 5, 6)` reshape 正确
6. ✅ episode_length_s override 机制可用（默认 5s → 30s+）
7. ✅ Sim reset pose 可在 sponge_bowl_cfg 通过 `self.scene.robot.init_state.joint_pos` 覆盖（不动 SO101_CFG 基础配置）

---

## 七、给 Phase 3 的输入清单

- **Init weights**: `outputs/sft_pi05_sponge/openpi_remapped/`（含 norm_stats）
- **Env**: `Isaac-Lift-Sponge-Bowl-SO101-v0`（非 Play 变体，PPO 用 num_envs 大批量）
- **Reset pose**: 已设为 dataset home（在 `sponge_bowl_cfg.py:78-90`）
- **Reward**: 当前是 object_goal_tracking + reaching；可能要加 `lift_z` shaping + sponge-bowl xy 距离衰减
- **必撞的 R1**: `openpi/models_pytorch/pi0_pytorch.py` 里 `ViewBackward0` inplace；PPO backward 必撞，到时候去掉那处 in-place op
- **必修的 R2 Ray plumbing 三件套**:
  1. `VK_ICD_FILENAMES` 在 Ray 子进程消失 → 已在 `so101_lift.py` make_env 闭包里 force-set，验证生效
  2. `CUDA_VISIBLE_DEVICES` 经 Ray accelerator manager 后变空 → 同闭包里 default 回 "0"，验证生效
  3. `botocore` 缺失 → `pip install botocore` 进 conda env（一行）

---

## 八、关键文件位置

- standalone eval: `/home/hlei/robotic/lerobot-rlinf/scripts/eval_pi05_sponge.py`
- 诊断输出样本: `/tmp/eval_pi05.log`
- ckpt: `/home/hlei/robotic/lerobot-rlinf/outputs/sft_pi05_sponge/openpi_remapped/`
- env cfg: `/home/hlei/robotic/lerobot-rlinf/src/lerobot_rlinf/tasks/lift/config/so101/sponge_bowl_cfg.py`
- RLinf model entry: `/home/hlei/RLinf/rlinf/models/embodiment/openpi/__init__.py:22 get_model()`
- RLinf so101 env wrapper (Ray-side): `/home/hlei/RLinf/rlinf/envs/isaaclab/tasks/so101_lift.py`
