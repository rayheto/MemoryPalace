# Tinker RL 训练管线

基于 `/home/ubuntu/cll/LocomotionWithNP3O-masteroldx/` 的实际代码。

---

## 训练架构

```
train.py
  ├── 环境: IsaacGym (GPU, 1024+ 并行环境)
  ├── 算法: NP3O (Neural Probabilistic PPO, 带安全约束)
  ├── 模型: Actor-Critic MLP
  ├── 控制: 位置控制(PD), 100Hz 策略频率, 1000Hz 仿真
  └── 验证: MuJoCo sim2sim
```

---

## 快速启动

```bash
conda activate cll                          # IsaacGym 环境
cd /home/ubuntu/cll/LocomotionWithNP3O-masteroldx
export HEADLESS=1                           # 无头模式(服务器)
python train.py                             # 默认 task='Tinker'

# 可选 task:
#   Tinker, Tinymal, go2N3poHim, go2N3poTransP1, go2N3poTransP2
# 通过修改 train.py 底部的 args.task 切换
```

---

## 配置文件

| 文件 | 用途 |
|---|---|
| `configs/tinker_constraint_him.py` | Tinker V1 粗地形训练配置 |
| `configs/tinker_constraint_him_cll.py` | Tinker 自定义配置(trimmed terrain, 64 envs) |
| `configs/tinymal_constraint_him.py` | TinyMal 机器人配置 |
| `configs/go2_constraint_him.py` | Go2 四足配置 |
| `configs/legged_robot_config.py` | 基础配置类(LeggedRobotCfg, LeggedRobotCfgPPO) |
| `configs/base_config.py` | BaseConfig 递归实例化工具 |

---

## Tinker 训练配置速查 (tinker_constraint_him.py)

| 参数 | 值 |
|---|---|
| 并行环境数 | 1024 |
| 动作空间 | 10 (5 per leg × 2 legs) |
| 单步观测 | 39 本体感觉 + 187 高度图 + 特权隐变量 + 历史 |
| 控制类型 | 位置 PD 控制 |
| 仿真频率 | 1000 Hz |
| 策略频率 | 100 Hz (decimation=10) |
| 地形 | 高度场/三角网格 |
| 奖励尺度 | 速度跟踪(1.0)、角速度跟踪(0.5)、方向(-1.0)、力矩(-1e-6) 等 |

---

## NP3O 算法 (algorithm/np3o.py)

NP3O 在 PPO 基础上增加安全约束优化:

1. **Cost Critic**: 额外的值函数预测约束违反
2. **Constraint Lagrangian**: 拉格朗日乘子法处理约束
3. **Dual Update**: 每次 PPO 更新后调整拉格朗日乘子
4. **Trust Region**: 类似 PPO 的裁剪目标

核心损失:
```
L = L_policy + c1*L_value + c2*L_cost_value + λ*L_constraint
```

---

## 环境关键设计 (envs/legged_robot.py)

### 观测空间 (本体感觉 39D)
```
指令输入: sin(phase), cos(phase), vx, vy, vyaw (5D)
关节状态: dof_pos(10), dof_vel(10) (20D)
上一动作: actions(10) (10D)
IMU: 角速度(3), 欧拉角(3) (6D)
= 41D 本体感觉
```
(另有 187D 高度扫描 + 历史帧堆叠)

### 奖励函数
| 奖励项 | 权重 | 说明 |
|---|---|---|
| tracking_lin_vel | 1.0 | 跟踪指令线速度 |
| tracking_ang_vel | 0.5 | 跟踪指令角速度 |
| lin_vel_z | -2.0 | 抑制垂向速度 |
| ang_vel_xy | -0.05 | 抑制俯仰/滚转角速度 |
| orientation | -1.0 | 保持躯干水平 |
| base_height | -10.0 | 维持目标身高 |
| dof_acc | -2.5e-7 | 抑制关节加速度 |
| dof_vel | -1e-3 | 抑制关节速度 |
| torques | -1e-6 | 最小化力矩 |
| collision | -1.0 | 碰撞惩罚 |
| feet_air_time | 1.0 | 鼓励合理腾空时间 |
| feet_clearance | 1.0 | 鼓励足端离地高度 |

### 域随机化 (Sim2Real)
- 地面摩擦: [0.2, 1.3]
- 基座质量: +[-3, +3] kg
- 电机力矩: [0.8, 1.2] 倍
- 关节摩擦/阻尼: 随机
- 观测噪声: 关节位置 ±0.02, 角速度 ±1.5
- 随机推力: 间隔 5s, max 速度 1.5 m/s
- 动作延迟: 随机 1-10 步

---

## 训练产物

| 文件 | 说明 |
|---|---|
| `model_jitt.pt` | TorchScript JIT 模型 (已存在) |
| `modelt.pt` | 训练检查点 |
| `policy_x64_cpu.so` | TVM 编译的 x64 推理库 |
| `policy_arm64_cpu.so` | TVM 编译的 ARM64 推理库 (Odroid C4) |
| `logs/` | WandB 训练日志 + checkpoint |

---

## sim2sim 验证 (MuJoCo)

```bash
# 直接 MuJoCo 仿真
./simulate /home/ubuntu/cll/LocomotionWithNP3O-masteroldx/resources/tinker/xml/world_terrain.xml

# Python sim2sim (加载策略在 MuJoCo 中运行)
python sim2sim_tinker.py

# UDP 桥接 (模拟实机通信)
python sim2sim_tinker_udp.py
```

MuJoCo 模型在 `resources/tinker/xml/` 目录中。

---

## 与新版代码 (`new/LocomotionWithNP3O-masteroldx/`) 的差异

新版在 git_tinker 的 OmniBotCtrl 中，支持更多配置变体:
- `tinker_constraint_him.py` — 基础粗地形
- `tinker_constraint_him_trot.py` — 平面 Trotting
- `tinker_constraint_him_stand.py` — 站立
- `tinker_constraint_him_cll.py` — 自定义(64 envs, 固定 0.2m/s, 带 stairs/obstacles/slopes)

新增 `envs/legged_robot_2leg.py` — 专门的双足环境(高度图、深度相机、奖励/代价函数)。

---

## 外部训练管线参考

| 项目 | 机器人 | 算法特点 |
|---|---|---|
| `expressive-humanoid/` | H1 (19DOF) | PPO-Mimic, 动捕模仿, 6144并行环境, 特权信息蒸馏 |
| `unitree_rl_gym/` | G1/H1/Go2 | 标准 PPO, 4096并行, 宇树官方 |
| `git_zhihuijun/` | X1 (12DOF) | PPO + CNN 历史编码器 + 状态估计器, 步态阶段调度 |
| `git_zhongqin/` | ZqSA01 (12DOF) | PPO + 对称性损失, 300K 迭代 |
