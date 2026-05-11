# RL 训练管线源码分析

基于 IsaacGym 的双足机器人强化学习训练框架。

**路径**: `LocomotionWithNP3O-masteroldx/` + `git_tinker/.../OmniBotCtrl/`

---

## 算法: NP3O (Neural Penalized PPO)

### 文件: `algorithm/np3o.py`

带安全约束的 PPO 变体:

```python
class NP3O:
    k_value = 3.0          # 拉格朗日乘子, 每步 ×1.0004 (上限 1.0)
    clip_param = 0.2
    gamma = 0.998, lam = 0.95
    desired_kl = 0.01      # 自适应 LR 目标
    dagger_update_freq = 20
```

### 核心损失函数

```
surrogate_loss = min(ratio * adv, clip(ratio, 1-ε, 1+ε) * adv)
cost_loss = k_value * Σ ReLU(cost_advantage + cost_violation)
total_loss = surrogate_loss
           + cost_viol_loss_coef * cost_loss
           + cost_value_loss_coef * cost_value_loss
           + value_loss_coef * value_loss
           + entropy_coef * entropy
           [+ imi_weight * imitation_loss]   # 可选 Teacher-Student 蒸馏
```

### Cost 约束类型 (Tinker)

| cost | 说明 |
|---|---|
| cost_pos | 关节位置超限 |
| cost_tau | 力矩超限 |
| cost_vel | 关节速度超限 |
| cost_feet | 足端离地时间违规 |
| cost_hip | 髋位置超限 |

---

## Actor-Critic 架构

### 文件: `modules/actor_critic.py` (1800+ 行)

10 种不同的 Actor-Critic 变体，核心模式:

```
Actor (Teacher):  特权观测 → PrivEncoder + ScanEncoder + HistoryEncoder → action
Actor (Student):  本体感觉 + 历史 → CNN History Encoder → action
Critic:           特权观测 → MLP → value
Cost Critic:      特权观测 → MLP → cost_value
```

### 关键编码器

| 编码器 | 说明 |
|---|---|
| StateHistoryEncoder | 1D-CNN (kernel=4/stride=2), 10 帧历史 → 64D latent |
| RnnStateHistoryEncoder | MLP + GRU |
| StateCausalTransformer | Causal Transformer + PositionalEncoding |
| MlpBarlowTwinsActor | MLP + Barlow Twins 自监督损失 |

### Tinker 使用的配置

```python
policy = ActorCriticMixedBarlowTwins  # MoE + BarlowTwins
teacher_act = False                   # Tinker 不使用 Teacher-Student
imi_flag = False                      # 不启用模仿学习
```

---

## 环境: LeggedRobot (IsaacGym)

### 文件: `envs/legged_robot.py` (1000+ 行)

#### Tinker 配置速查

| 参数 | 值 |
|---|---|
| 并行环境数 | 1024 |
| 动作空间 | 10 (5 per leg: yaw/roll/pitch/knee/ankle) |
| 本体感觉 | 39D per step |
| 观测 = 本体感觉 + 历史 | 44D × 10 frames = 440D (+ 特权隐变量) |
| 仿真频率 | 200Hz (dt=0.005) |
| 策略频率 | 50Hz (decimation=4) |
| 控制类型 | Position PD |

#### 观测空间 (39D single step)

| 索引 | 内容 | Scale |
|---|---|---|
| 0-2 | 基座角速度 | 1.0 |
| 3-5 | 欧拉角 (roll/pitch/yaw) | 1.0 |
| 6-8 | 速度指令 (vx/vy/vyaw) | 1.0 |
| 9-18 | 关节位置 - 默认姿态 | 1.0 |
| 19-28 | 关节速度 | 1.0 |
| 29-38 | 上一动作 | 1.0 |

#### 域随机化

| 参数 | 范围 |
|---|---|
| 摩擦 | [0.2, 2.75] |
| 基座质量 | ±0.5kg |
| 基座 CoM | ±0.05m |
| 电机强度 | [0.8, 1.2] |
| KP/KD | [0.8, 1.2] |
| 动作延迟 | [0, 10] steps |
| 观测噪声 | × noise_level |
| 随机推力 | 间隔 5s, max_vel=1.5m/s |

#### 奖励函数 (主要项)

| 奖励 | 权重 | 说明 |
|---|---|---|
| tracking_lin_vel | 1.0-3.0 | 指数衰减的线速度跟踪 |
| tracking_ang_vel | 0.5 | 角速度跟踪 |
| feet_clearance | 1.0 | 摆动足足端离地 |
| feet_air_time | 1.0 | 腾空时间奖励 |
| orientation | -1.0 | 躯干倾斜惩罚 |
| torques | -1e-6 | 力矩最小化 |
| dof_acc | -2.5e-7 | 加速度平滑 |
| action_rate | -0.01 | 动作变化率惩罚 |

---

## 训练入口

### `train.py`

```python
task_registry.register("Tinker", LeggedRobot, 
    TinkerConstraintHimRoughCfg(), TinkerConstraintHimRoughCfgPPO())

args.task = 'Tinker'
env, env_cfg = task_registry.make_env(name=args.task, args=args)
ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, ...)
ppo_runner.learn(num_learning_iterations=30000, init_at_random_ep_len=True)
```

### 训练超参数

| 参数 | 值 |
|---|---|
| max_iterations | 30000 |
| num_steps_per_env | 24 |
| 总样本/迭代 | 1024 × 24 = 24576 |
| learning_rate | 1e-4 |
| gamma | 0.98 |
| save_interval | 5000 |

---

## Sim2Sim Bridge

### `sim2sim_tinker.py` — MuJoCo 中直接推理

```
流程:
  MuJoCo (1000Hz) → Python 观测 (50Hz) → PyTorch 推理 → PD 控制 → MuJoCo
  
观测构建:
  obs = [omega×0.25, euler, cmd, (q-default), dq, last_actions]  # 39D
  policy_input = obs + zeros(priv) + zeros(scan) + history(10×44)  # ~736D
  
推理:
  policy.act_teacher(torch.tensor(policy_input).half())  # FP16
  
控制:
  target_q = action × 0.25 + default_dof_pos
  tau = (target_q - q) × kp + (target_dq - dq) × kd
  kp=9.0, kd=0.6, tau_limit=12 Nm
```

### UDP 桥接 (`sim2sim_tinker_udp.py`)

策略运行在 C++ 进程, MuJoCo 端接收 UDP 动作:

```
MuJoCo(Python) ←─UDP:8888─→ control_task(C++)
  → 发送观测 (37 floats = 148 bytes)
  ← 接收动作 (10 floats)
  kp=10.0, kd=0.4, decimation=18
```

---

## 模型导出

### `pt2tvm.py`

```python
# 加载 JIT 模型
model = torch.jit.load("model_jitt.pt")

# TVM 编译 (x86)
tvm.compile(model, target="llvm", opt_level=3)
→ policy_x64_cpu.so

# TVM 交叉编译 (ARM64 for Odroid C4)
tvm.compile(model, target="llvm -mtriple=aarch64-linux-gnu")
→ policy_arm64_cpu.so
```

---

## 配置文件层次

```
configs/
├── base_config.py           # BaseConfig (递归类实例化)
├── legged_robot_config.py   # LeggedRobotCfg + LeggedRobotCfgPPO
├── tinker_constraint_him.py # Tinker 基础配置 (1024 envs)
├── tinker_constraint_him_trot.py  # Tinker Trot 模式
├── tinker_constraint_him_stand.py # Tinker Stand 模式
├── tinker_constraint_him_cll.py   # 自定义 (64 envs, trimesh terrain)
├── go2_constraint_him.py    # Go2 四足配置 (2048 envs)
└── tinymal_constraint_him.py # TinyMal 四足
```

### Tinker CLL 自定义配置特点

- 更高速度指令: `lin_vel_x = [1.0, 3.0]` m/s
- 更高刚度: `stiffness = 25.0`, `damping = 0.5`
- 更大动作缩放: `action_scale = 0.35`
- 更短步态周期: `cycle_time = 0.3s`
- 更高 tracking 权重: `tracking_lin_vel = 3.0`
- 更激进的推力: `max_push_vel_xy = 0.8`

---

## 部署工具脚本

| 脚本 | 用途 |
|---|---|
| `tools/py_xbox.py` | Xbox 手柄 → UDP (IP 192.168.1.246:8001) |
| `tools/py_ui.py` | Tkinter 调试面板 (使能/步态/速度) |
| `tools/pt2tvm.py` | JIT → TVM .so |
| `tools/ethercat.sh` | EtherCAT 主站管理 (调试用) |
| `tools/make.sh` | CMake 构建 (x64/arm64) |
| `tools/run_mujoco.sh` | MuJoCo 仿真启动 |
| `tools/update_lib.sh` | SCP 部署 .so 到机器人 |
| `update1.sh/update2.sh` | 完整部署 (模型+参数+重启) |
