# RLinf + Isaac Sim + LeRobot Pi 0.5 后训练方案
## SO-ARM100 (Seeed SO-100) 机械臂仿真强化学习训练

---

## 一、整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                        rlinf 训练主循环                       │
│                                                             │
│  ┌──────────────┐    obs_dict    ┌─────────────────────┐   │
│  │ Isaac Sim Env│ ─────────────► │  Pi 0.5 Actor       │   │
│  │ (OmniIsaac   │                │  (LeRobot 权重+结构) │   │
│  │  GymEnvs)    │ ◄───────────── │                     │   │
│  └──────────────┘  action_chunk  └─────────────────────┘   │
│         │                                  │                │
│         └──────── Trajectory Buffer ───────┘                │
│                         │                                   │
│                    PPO / GRPO                                │
│                    梯度更新 Pi 0.5                            │
└─────────────────────────────────────────────────────────────┘
```

**核心原则：** LeRobot 只贡献模型结构和预训练权重，rlinf 完全控制训练循环、数据采集、梯度更新。

---

## 二、机器人资产：SO-ARM100 URDF → Isaac Sim USD

### 2.1 资产来源（已确认）

TheRobotStudio 官方仓库已提供完整仿真资产：

```
https://github.com/TheRobotStudio/SO-ARM100
└── Simulation/
    └── SO101/
        ├── so101_new_calib.urdf   ← 推荐使用（关节零点在范围中点）
        ├── so101_old_calib.urdf
        ├── scene.xml              ← MuJoCo 版本（参考）
        └── meshes/                ← STL/OBJ 碰撞网格
```

Seeed 官方 Wiki 也提供了对应的 Isaac Sim 导入教程，路径为：
`/lerobot/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf`

### 2.2 URDF → USD 转换步骤

```python
# Isaac Sim 内置 URDF Importer，在 GUI 或脚本中执行：
from omni.isaac.urdf import _urdf

urdf_interface = _urdf.acquire_urdf_interface()
import_config = _urdf.ImportConfig()

# 关键配置项
import_config.merge_fixed_joints = False   # 保留所有关节，便于读取状态
import_config.fix_base = True              # 固定底座（桌面臂）
import_config.make_default_prim = True
import_config.self_collision_enabled = False
import_config.create_physics_scene = True
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength = 1000.0
import_config.default_position_drive_damping = 50.0

result, prim_path = urdf_interface.import_urdf(
    urdf_path="path/to/so101_new_calib.urdf",
    dest_path="/World/SO101",
    import_config=import_config
)
```

### 2.3 关节说明（SO-101，6 DOF）

| 关节索引 | 名称 | 范围 | 备注 |
|---------|------|------|------|
| 0 | shoulder_pan | ±π | 底座旋转 |
| 1 | shoulder_lift | ±π/2 | 肩部俯仰 |
| 2 | elbow_flex | ±π | 肘部 |
| 3 | wrist_flex | ±π/2 | 腕部俯仰 |
| 4 | wrist_roll | ±π | 腕部旋转 |
| 5 | gripper | [0, 100] → 归一化 [0,1] | 线性关节 |

> **注意：** LeRobot 中 gripper=0 表示全闭，gripper=100 表示全开。
> 当前 URDF 尚未完全反映这一映射，需在 action 处理层做转换。

---

## 三、Isaac Sim 环境改造（OmniIsaacGymEnvs → Pi 0.5 兼容）

### 3.1 OmniIsaacGymEnvs 原始问题

原始示例（如 `FrankaCabinet`）使用内存数据，无渲染相机：
```python
# 原始方式（不适用）
self.obs_buf[:, :7] = dof_pos  # 直接写内存 tensor，无图像
```

### 3.2 Pi 0.5 所需 obs 格式

```python
obs_dict = {
    "observation.images.cam_high":  Tensor[B, 3, 224, 224],  # 主相机 RGB
    "observation.images.cam_wrist": Tensor[B, 3, 224, 224],  # 腕部相机
    "observation.state":            Tensor[B, 14],            # 6关节pos+vel=12，+ee_xyz+ee_rpy=6 → 视任务取子集
}
```

> **LeRobot obs key 命名** 遵循 `observation.images.<cam_name>` 格式，
> 必须与 Pi 0.5 训练时的数据集 key 完全一致，否则 embedding lookup 出错。

### 3.3 相机配置

```python
from omni.isaac.sensor import Camera
import torch

class SO101Env(VecEnvBase):
    def _setup_cameras(self):
        self.cam_high = Camera(
            prim_path="/World/SO101/base_link/cam_high",
            resolution=(224, 224),
            frequency=30,
        )
        self.cam_wrist = Camera(
            prim_path="/World/SO101/wrist_link/cam_wrist",
            resolution=(224, 224),
            frequency=30,
        )
        self.cam_high.initialize()
        self.cam_wrist.initialize()

    def _get_camera_tensors(self):
        # 直接拿 GPU tensor，避免 CPU round-trip
        rgba_high  = self.cam_high.get_rgba()   # [H, W, 4] uint8 on GPU
        rgba_wrist = self.cam_wrist.get_rgba()

        img_high  = torch.as_tensor(rgba_high[..., :3],  device="cuda").permute(2, 0, 1).float() / 255.0
        img_wrist = torch.as_tensor(rgba_wrist[..., :3], device="cuda").permute(2, 0, 1).float() / 255.0

        # Batch 维度扩展（单 env 时）
        return img_high.unsqueeze(0), img_wrist.unsqueeze(0)  # [1, 3, 224, 224]
```

### 3.4 完整 step() 接口

```python
def step(self, actions: torch.Tensor):
    """
    actions: [B, 6]，归一化到 [-1, 1]，对应 6 个关节 delta 或绝对位置
    """
    # 1. 反归一化并施加动作
    joint_targets = self._denormalize_action(actions)
    self.articulation_view.set_joint_position_targets(joint_targets)

    # 2. 仿真步进
    self.sim.step()

    # 3. 采集 obs
    img_high, img_wrist = self._get_camera_tensors()  # [B,3,224,224]
    joint_pos = self.articulation_view.get_joint_positions()  # [B,6]
    joint_vel = self.articulation_view.get_joint_velocities()  # [B,6]
    ee_pose   = self._compute_ee_pose()                        # [B,6] xyz+rpy

    obs_dict = {
        "observation.images.cam_high":  img_high,
        "observation.images.cam_wrist": img_wrist,
        "observation.state": torch.cat([joint_pos, joint_vel], dim=-1),  # [B,12]
    }

    reward = self._compute_reward(obs_dict)
    done   = self._check_termination()
    info   = {"ee_pose": ee_pose, "joint_pos": joint_pos}

    return obs_dict, reward, done, info
```

---

## 四、LeRobot Pi 0.5 接入 rlinf

### 4.1 LoRA vs 全参数更新 —— 区别说明

| | LoRA 微调 | 全参数更新 |
|---|---|---|
| 原理 | 在原始权重矩阵旁插入低秩矩阵 A、B，只训练 A、B | 所有参数都参与梯度计算 |
| 显存 | 低（只存 LoRA 参数的梯度和优化器状态） | 高（完整模型的梯度 + Adam 二阶矩） |
| 风险 | 遗忘风险低，预训练知识保留好 | 容易过拟合，需要更多数据 |
| 推荐场景 | **后训练首选**，特别是 reward 信号稀疏时 | 数据量极大且任务分布与预训练差异大时 |
| Pi 0.5 建议 | 先做 LoRA，rank=16~64，只插在 action expert 的 attention 层 | 验证效果后再考虑是否解冻 vision encoder |

**结论：先用 LoRA，目标是 action expert 部分。** Pi 0.5 的 vision encoder（SigLIP 或 PaliGemma backbone）冻结，只让 action 解码器学习新任务的 reward 信号。

### 4.2 Pi 0.5 Actor Wrapper

```python
# rlinf/actors/pi05_actor.py

import torch
import torch.nn as nn
from lerobot.policies.pi0 import PI0Policy  # LeRobot 导入路径
from peft import get_peft_model, LoraConfig, TaskType

class Pi05ActorWrapper(nn.Module):
    """
    将 LeRobot Pi 0.5 包装为 rlinf 兼容的 actor。
    负责：
      1. obs key 名称转换
      2. 图像归一化（LeRobot 期望特定的 mean/std）
      3. action 输出格式统一
      4. log_prob 计算（PPO 所需）
    """

    def __init__(self, checkpoint_path: str, use_lora: bool = True, lora_rank: int = 32):
        super().__init__()

        # 加载 LeRobot Pi 0.5 预训练权重
        self.policy = PI0Policy.from_pretrained(checkpoint_path)

        # 冻结 vision encoder
        for name, param in self.policy.named_parameters():
            if "vision_encoder" in name or "image_encoder" in name:
                param.requires_grad_(False)

        # 插入 LoRA（如果启用）
        if use_lora:
            lora_config = LoraConfig(
                task_type=TaskType.CAUSAL_LM,  # flow matching 近似处理
                r=lora_rank,
                lora_alpha=lora_rank * 2,
                target_modules=["q_proj", "v_proj", "k_proj", "o_proj"],
                lora_dropout=0.05,
                bias="none",
            )
            self.policy = get_peft_model(self.policy, lora_config)
            self.policy.print_trainable_parameters()

    def _preprocess_obs(self, obs_dict: dict) -> dict:
        """
        Isaac Sim 输出 → LeRobot 期望格式
        主要处理：归一化、key 映射已在 env 中对齐，这里做类型/设备检查
        """
        processed = {}
        for key, val in obs_dict.items():
            if "images" in key:
                # LeRobot Pi 0.5 期望 [0,1] float32，env 已处理
                processed[key] = val.float().clamp(0.0, 1.0)
            else:
                processed[key] = val.float()
        return processed

    def forward(self, obs_dict: dict) -> torch.Tensor:
        """
        推理模式：返回 action chunk 的第一步
        返回 shape: [B, action_dim]
        """
        obs = self._preprocess_obs(obs_dict)
        with torch.autocast("cuda", dtype=torch.bfloat16):
            action_chunk = self.policy.select_action(obs)  # [B, chunk_T, A]
        # 取 chunk 第一步执行，后续步骤在 env 中 buffer
        return action_chunk[:, 0, :].float()

    def get_log_prob(self, obs_dict: dict, actions: torch.Tensor) -> torch.Tensor:
        """
        PPO 需要 log π(a|s)。
        Pi 0.5 基于 flow matching，log_prob 通过 ELBO 近似：
        对 action 加噪声后计算 flow matching loss 的负值作为代理。

        ⚠️  这是 PPO 接入 flow-matching policy 的核心挑战，
            需要根据 LeRobot Pi 0.5 的具体实现确认是否有
            compute_log_prob() 或等价接口。
        """
        obs = self._preprocess_obs(obs_dict)
        # 方案 A：如果 LeRobot 提供了 log_prob 接口
        # return self.policy.compute_log_prob(obs, actions)

        # 方案 B：用 flow matching loss 的负值近似（待验证）
        with torch.autocast("cuda", dtype=torch.bfloat16):
            loss = self.policy.compute_loss(obs, actions)
        return -loss.float()

    def get_value(self, obs_dict: dict) -> torch.Tensor:
        """
        PPO 的 critic value head。
        Pi 0.5 本身没有 value head，需要外挂一个轻量 MLP critic。
        """
        raise NotImplementedError("需要外挂独立 Critic，见 Pi05CriticWrapper")


class Pi05CriticWrapper(nn.Module):
    """
    独立的轻量 Critic，用于 PPO value 估计。
    不依赖 Pi 0.5 内部，只消费 obs 的 state 部分（可选加图像特征）。
    """
    def __init__(self, state_dim: int = 12, hidden_dim: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, obs_dict: dict) -> torch.Tensor:
        state = obs_dict["observation.state"]  # [B, state_dim]
        return self.net(state)  # [B, 1]
```

---

## 五、rlinf 训练主循环设计

### 5.1 关键接口约定

rlinf 框架通常期望 actor 提供以下接口（需根据实际源码核对）：

```python
# rlinf 期望的 actor 接口
class BaseActor:
    def act(self, obs) -> Tensor          # 返回 action
    def evaluate(self, obs, action)       # 返回 (log_prob, value, entropy)
    def parameters(self)                  # 可训练参数
```

`Pi05ActorWrapper` 需要适配这个接口，重点是 `evaluate()` 方法。

### 5.2 Trajectory Buffer 数据结构

```python
# 每条 transition 存储的字段
buffer_entry = {
    "obs": {
        "observation.images.cam_high":  Tensor[B, 3, 224, 224],
        "observation.images.cam_wrist": Tensor[B, 3, 224, 224],
        "observation.state":            Tensor[B, 12],
    },
    "action":   Tensor[B, 6],
    "reward":   Tensor[B, 1],
    "done":     Tensor[B, 1],
    "log_prob": Tensor[B, 1],   # 采集时的 log π_old(a|s)
    "value":    Tensor[B, 1],   # V(s) from critic
}
```

> **存储注意：** 图像数据量大（每帧 224×224×3×4 bytes ≈ 600KB），
> buffer 建议存 uint8 格式，计算时再转 float。
> num_steps=512, num_envs=4 → 约 1.2GB，需提前规划显存。

### 5.3 PPO 训练循环

```python
# rlinf/trainers/ppo_trainer.py（示意）

for iteration in range(total_iterations):

    # ── 1. Rollout 采集 ──────────────────────────────────
    buffer.clear()
    obs = env.reset()
    for step in range(num_steps):
        with torch.no_grad():
            action   = actor.forward(obs)
            log_prob = actor.get_log_prob(obs, action)
            value    = critic(obs)

        next_obs, reward, done, info = env.step(action)
        buffer.add(obs, action, reward, done, log_prob, value)
        obs = next_obs

    # ── 2. 计算 GAE 优势 ─────────────────────────────────
    with torch.no_grad():
        last_value = critic(obs)
    buffer.compute_gae(last_value, gamma=0.99, lam=0.95)

    # ── 3. PPO 更新（K 轮 epoch）────────────────────────
    for epoch in range(ppo_epochs):
        for batch in buffer.make_minibatches(batch_size=64):
            new_log_prob = actor.get_log_prob(batch.obs, batch.action)
            new_value    = critic(batch.obs)

            ratio    = (new_log_prob - batch.log_prob).exp()
            adv      = (batch.advantage - batch.advantage.mean()) / (batch.advantage.std() + 1e-8)

            # PPO clip loss
            pg_loss1 = -adv * ratio
            pg_loss2 = -adv * ratio.clamp(1 - clip_eps, 1 + clip_eps)
            pg_loss  = torch.max(pg_loss1, pg_loss2).mean()

            # Value loss
            v_loss = F.mse_loss(new_value.squeeze(), batch.returns)

            loss = pg_loss + vf_coef * v_loss

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(actor.parameters(), max_grad_norm)
            optimizer.step()

    # ── 4. Eval ──────────────────────────────────────────
    if iteration % eval_interval == 0:
        success_rate = eval_runner.run(actor, eval_env, n_episodes=20)
        logger.log({"success_rate": success_rate, "iteration": iteration})
```

---

## 六、Sim2Sim 对齐验证（进入 PPO 前的必做步骤）

目标：确保 LeRobot Pi 0.5 在 Isaac Sim 环境中的推理结果与其在 LeRobot 原生仿真（MuJoCo scene.xml）中一致，无数值偏差。

### 6.1 对齐检查清单

| 检查项 | 验证方法 | 通过标准 |
|-------|---------|---------|
| obs key 名称 | `assert set(obs.keys()) == set(policy.expected_obs_keys)` | 完全一致 |
| 图像归一化范围 | `assert img.min() >= 0 and img.max() <= 1` | [0,1] float32 |
| 图像尺寸 | `assert img.shape == (B, 3, 224, 224)` | 无缩放误差 |
| state 维度和顺序 | 对比 LeRobot 训练集的 `observation.state` 维度 | 相同 |
| action 空间范围 | 给定相同 obs，对比 Isaac 和 MuJoCo 中的 action 输出 | 差异 < 1e-3 |
| gripper 映射 | LeRobot [0,100] ↔ Isaac 关节角度 | 线性映射正确 |
| 坐标系 | Isaac 默认 Z-up，LeRobot URDF Z-up | 一致 ✓ |

### 6.2 对齐测试脚本

```python
def sim2sim_alignment_test(actor, isaac_env, n_steps=100):
    """
    固定随机种子，跑 n 步，检查动作输出的均值和方差是否在合理范围内。
    """
    torch.manual_seed(42)
    obs = isaac_env.reset()
    action_log = []

    for _ in range(n_steps):
        with torch.no_grad():
            action = actor.forward(obs)
        action_log.append(action.cpu())
        obs, _, done, _ = isaac_env.step(action)
        if done.any():
            obs = isaac_env.reset()

    actions = torch.stack(action_log)
    print(f"Action mean: {actions.mean(0)}")
    print(f"Action std:  {actions.std(0)}")
    print(f"Action range: [{actions.min():.3f}, {actions.max():.3f}]")

    # 验证没有 NaN 或极端值
    assert not actions.isnan().any(), "发现 NaN，obs 归一化或模型加载有问题"
    assert actions.abs().max() < 10.0, "动作范围异常，检查反归一化逻辑"
    print("✅ Sim2Sim 对齐检查通过")
```

---

## 七、执行计划与里程碑

### Week 1：资产准备与环境搭通

- [ ] Clone `TheRobotStudio/SO-ARM100`，提取 `so101_new_calib.urdf`
- [ ] 用 Isaac Sim URDF Importer 导入，验证关节数量和运动范围
- [ ] 在 OmniIsaacGymEnvs 基础上创建 `SO101PickTask`，跑通 `env.step()`（无图像）
- [ ] 接入双相机，验证 GPU tensor 路径（不经过 CPU）

### Week 2：obs 格式对齐

- [ ] 安装 LeRobot，加载 Pi 0.5 预训练权重，确认 `expected_obs_keys`
- [ ] 实现 `Pi05ActorWrapper`，先跑通 `forward()`（推理）
- [ ] 运行 `sim2sim_alignment_test()`，通过全部对齐检查
- [ ] 确认 gripper 映射和 action 反归一化逻辑

### Week 3：Rollout 数据流

- [ ] 实现 `Trajectory Buffer`，解决图像数据显存问题（uint8 存储）
- [ ] 接入 rlinf 的 rollout collector，验证 `(obs, action, reward, done, log_prob, value)` 完整流转
- [ ] 实现 `Pi05CriticWrapper`，验证 value 估计合理
- [ ] 跑通 eval 循环，记录 `success_rate`

### Week 4：PPO 接入与调试

- [ ] 实现 `get_log_prob()`（确认 Pi 0.5 的 flow matching log prob 接口）
- [ ] 接入完整 PPO 循环（先用小 `num_steps=128` 验证梯度流动）
- [ ] 验证 LoRA 参数正确更新（检查 `param.grad` 不为 None）
- [ ] 首次完整训练 run，观察 reward 曲线是否有上升趋势

### Week 5–6：稳定与扩展

- [ ] 调整 reward function，加入 dense reward
- [ ] 扩展 `num_envs=8`，验证并行相机采集稳定性
- [ ] 加入 domain randomization（光照、物体位置、摩擦系数）
- [ ] 整理 checkpoint 保存和 wandb logging

---

## 八、关键风险与应对

| 风险 | 严重程度 | 应对方案 |
|------|---------|---------|
| Pi 0.5 没有 `compute_log_prob()` 接口 | 高 | 用 REINFORCE（只需 reward，不需 log_prob）先跑通；或实现 flow matching log prob 近似 |
| 多 env 并行相机显存不足 | 中 | 先单 env 验证，逐步增加；或降低图像分辨率至 128×128 |
| URDF 导入后关节物理参数不准 | 中 | 对比真实机械臂运动，调整 `drive_strength` 和 `damping` |
| action chunk 与 PPO single-step 假设冲突 | 中 | 固定取 chunk 第一步；或实现展开模式的 multi-step PPO |
| Isaac Sim 与 OmniIsaacGymEnvs 版本兼容问题 | 低 | 锁定 Isaac Sim 4.2（Seeed Wiki 验证版本）|

---

## 九、依赖版本建议

```
isaac-sim          == 4.2.0
OmniIsaacGymEnvs   == 对应 Isaac Sim 4.2 分支
lerobot            == 最新 main（支持 pi0/pi0.5）
peft               >= 0.10.0
torch              >= 2.3.0
```