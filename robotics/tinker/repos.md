# Tinker 代码仓库地图

基于 `/home/ubuntu/cll/` 的实际目录结构，理清各仓库的上下游关系。

---

## 核心仓库关系图

```
┌─────────────────────────────────────────────────────┐
│  OmniBotSeries-Tinker (git_tinker/)                  │
│  GitHub: golaced/OmniBotSeries-Tinker                │
│  ├── OmniBotCtrl/     ← RL训练 + 部署工具            │
│  ├── OmniBotHub/      ← 机载C++ (Odroid C4)          │
│  ├── OmniBotSeries/   ← 机械CAD                      │
│  ├── OmniBotTele/     ← 遥控器 (ESP32+STM32)         │
│  └── OmniBotHmi/      ← 上位机界面                    │
└──────────┬──────────────────────────────────────────┘
           │ 依赖
           ▼
┌─────────────────────────────────────────────────────┐
│  LocomotionWithNP3O-masteroldx/ (主训练代码)          │
│  算法: NP3O (PPO变体 + 安全约束)                      │
│  ┌──────────┐  ┌──────────┐                          │
│  │ IsaacGym  │  │  MuJoCo  │  双仿真后端              │
│  │ (GPU训练) │  │(sim2sim) │                          │
│  └──────────┘  └──────────┘                          │
├─────────────────────────────────────────────────────┤
│  new/LocomotionWithNP3O-masteroldx/ (新版)            │
│  多机器人: Tinker, Go2, TinyMal, Taitan              │
│  新增: legged_robot_2leg.py (双足专用环境)            │
└─────────────────────────────────────────────────────┘
```

---

## 各仓库详情

### 主训练仓库 (直接修改使用的)

| 路径 | 说明 | 状态 |
|---|---|---|
| `LocomotionWithNP3O-masteroldx/` | **主力训练代码**，默认训练 Tinker | 有训练产物(model_jitt.pt, policy_*.so) |
| `new/LocomotionWithNP3O-masteroldx/` | 新版训练代码，支持更多机器人 | 有 checkpoint，cll 自定义配置 |

### 硬件与全栈仓库

| 路径 | 说明 |
|---|---|
| `git_tinker/OmniBotSeries-Tinker-main/OmniBotCtrl/` | RL 训练配置 + CAN/SPI/TVM/MuJoCo/测试脚本 |
| `git_tinker/OmniBotSeries-Tinker-main/OmniBotHub/Linux/release/` | 预编译的机载二进制 + YAML 参数 |
| `git_tinker/OmniBotSeries-Tinker-main/OmniBotSeries/` | 机械 CAD (切割/机加/3D打印 STEP 文件) |
| `git_tinker/OmniBotSeries-Tinker-main/OmniBotTele/` | 遥控器固件(ESP32+STM32) + Altium PCB |

### URDF 模型

| 路径 | 内容 |
|---|---|
| `tinker/` | Tinker V1 URDF + 10 STL 网格 + ROS launch |
| `TinkerV2_URDF/` | Tinker V2 URDF (+ MuJoCo 编译器标签) |
| `git_tinker/.../OmniBotCtrl/resources/tinker/` | V1 URDF 多版本(含 `_inv`, `_inv_point` 等变体) |
| `git_tinker/.../OmniBotCtrl/resources/TinkerV2_URDF/` | V2 URDF (ROS Catkin) |

---

## 外部参考仓库 (不被 Tinker 直接依赖)

### Unitree 宇树系列

| 路径 | 来源 | 机器人 | 说明 |
|---|---|---|---|
| `unitree_rl_gym/` | `github.com/unitreerobotics/unitree_rl_gym` | G1/H1/Go2 | 宇树官方 RL 训练，legged_gym 框架 |
| `unitree_sdk2_python/` | `github.com/unitreerobotics/unitree_sdk2_python` | — | 宇树 Python SDK (DDS 通信) |

### 表情化人形运动 (ASE)

| 路径 | 来源 | 机器人 |
|---|---|---|
| `expressive-humanoid/` | `github.com/chengxuxin/expressive-humanoid` | H1 (19DOF) |

核心创新:
- CMU Mocap 动捕 → H1 全身模仿
- 特权信息蒸馏 (DACER 风格)
- 自适应运动难度课程学习
- 6144 并行环境训练

### 动捕重定向管线

| 路径 | 来源 | 机器人 | 核心技术 |
|---|---|---|---|
| `git_amp/mocap_retarget/` | 本地开发 | G1 (29DOF) | CasADi + IPOPT IK, Pinocchio FK, CMU ASF/AMC |

### 稚辉君 AgiBot X1

| 路径 | 来源 | 机器人 |
|---|---|---|
| `git_zhihuijun/agibot_x1_train/` | 稚辉君项目 | X1 (12DOF) |

核心创新:
- CNN 长历史编码器(66帧→64维)
- 状态估计器辅助损失(MSE 线速度预测)
- 多步态调度(walk/stand/sagittal/lateral/rotate)
- 导出链: .pt → .jit → .onnx → AimRT 部署

### EngineAI 中琴

| 路径 | 来源 | 机器人 |
|---|---|---|
| `git_zhongqin/engineai_legged_gym/` | EngineAI 机器人 | ZqSA01 (12DOF) |

核心创新: PPO + 对称性损失(L/R permutation), 300K 迭代

### VLN-CE 视觉导航

| 路径 | 来源 | 用途 |
|---|---|---|
| `git_vln_ce/VLN-CE-Isaac/` | `github.com/yang-zj1026/VLN-CE-Isaac` | Go2 + Matterport 视觉语言导航 |
| `git_vln_ce/model/` | — | 导航模型 |

### 其他工具链

| 路径 | 用途 |
|---|---|
| `fbx_sdk/` + `fbx_pythonbind/` | Autodesk FBX SDK 2020.3.2, CMU FBX 动捕数据导入 |
| `blender/` | Blender 4.4.1 (3D 建模/动画) |
| `sip/sip-4.19.3/` | PyQt 绑定工具 (FBX Python 绑定依赖) |
| `git_cmu/CMU_fbx.zip` | CMU 动捕 FBX 原始数据 |
| `LAFAN1_Retargeting_Dataset/` | LAFAN1 重定向数据集(G1/H1/H1_2) |
| `catkin_ws/` | ROS Catkin 工作空间 (当前为空) |

---

## 仿真后端

| 路径 | 版本 | 用途 | 状态 |
|---|---|---|---|
| `isaacgym/` | Preview 4 | GPU 加速物理仿真 | conda env `cll` |
| `IsaacGymEnvs-main/` | 1.5.1 | IsaacGym 基准任务 | 有训练日志 |
| `mujoco210/` | 2.1.0 | CPU 精确物理仿真 | Python 绑定未装 |
| `git_isaaclab/IsaacLab/` | 1.1.0 | 下一代框架(需 Isaac Sim) | **未配置** |

---

## 依赖链

```
Tinker 训练 (最小依赖):
  python3.8 + PyTorch 1.13 + CUDA 11.7
  └── isaacgym (Preview 4)    ← conda env: cll
  └── numpy

Tinker 部署 (Odroid C4):
  Ubuntu + PREEMPT_RT + SPI 共享内存 + CAN (MIT协议)
  └── TVM runtime (加载 .so)
  └── qpOASES (QP 求解器, 用于 MPC/WBC)

外部参考 (独立环境):
  expressive-humanoid → isaacgym + wandb + rsl_rl
  unitree_rl_gym      → isaacgym + legged_gym + rsl_rl
  agibot_x1_train     → isaacgym + wandb
  engineai_legged_gym → isaacgym + rsl_rl
  mocap_retarget      → ROS Noetic (Docker) + CasADi + Pinocchio
```

---

## Conda 环境

| 环境名 | 用途 | 激活方式 |
|---|---|---|
| `cll` | IsaacGym 训练 | `conda activate cll` 或别名 `cll` |
| `git_amp` | 动捕重定向 (Docker 内) | Docker: `cll_ros` 进入 |
| `zhihuijun` | X1 训练 (python3.8) | `conda activate zhihuijun` |
| `rlgpu` | IsaacGymEnvs | — |

---

## Docker 容器

| 容器名 | 镜像 | 用途 |
|---|---|---|
| `cll_ros` | `fishros2/ros:noetic-desktop-full` | ROS Noetic + 动捕重定向 |
| 启动: `cll_ros` (alias) → 交互式菜单 (r/e/s/c/d/t) |
