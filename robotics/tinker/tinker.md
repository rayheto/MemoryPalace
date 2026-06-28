---
tags:
  - robotics
  - tinker
  - subnode
---

# Tinker 双足机器人

自研 60cm 高性能双足机器人，基于强化学习的步态控制，已实现 Sim2Real 转移。整合 Pixar 灯神 + Disney BDX 风格设计。整机 BOM 成本 < 15,000 RMB。

> 实际代码位于 `/home/ubuntu/cll`，核心训练在 `LocomotionWithNP3O-masteroldx/`。

---

## 文档索引

| 主题 | 文档 | 核心要点 |
|------|------|---------|
| 硬件规格 | [hardware.md](hardware.md) | 机械尺寸、关节参数、CAN+STM32电机架构、Odroid C4规格、CAD/PCB文件位置 |
| RL 训练管线 | [training.md](training.md) | NP3O 算法、IsaacGym/MuJoCo 双后端、奖励函数设计、配置速查 |
| Sim2Real 部署 | [deployment.md](deployment.md) | 模型导出(JIT/ONNX/TVM)、Odroid RT内核安装、三Task架构(500Hz控制)、SPI+CAN+UDP协议栈 |
| 代码仓库地图 | [repos.md](repos.md) | 各 git 仓库关系、外部参考项目(宇树/稚辉君/EngineAI/VLN)、依赖链 |
| **源码深度分析** | **[src_analysis/](src_analysis/)** | **control_task, navigation_task, 训练管线, 固件, 通信协议** |

---

## 源码分析索引 ([src_analysis/](src_analysis/))

```
src_analysis/
├── omnibothub/           # 机载软件 (Odroid C4)
│   ├── overview.md       # 架构总览
│   ├── control_task.md   # 实时控制器 (C++, 500Hz, 7线程, 步态状态机, TVM RL)
│   ├── navigation_task.md # 通信桥接 (C++, UDP服务, 共享内存, 数据日志)
│   └── protocols.md      # SPI+CAN+共享内存+UDP 协议规格
├── omnibotctrl/          # 训练与仿真
│   ├── overview.md       # 架构总览
│   └── training_pipeline.md # NP3O算法, Actor-Critic, 域随机化, sim2sim
├── omnibottele/          # 遥控系统
│   ├── overview.md       # 架构总览
│   └── firmware.md       # STM32(nRF24L01+ANO)+ESP32(LVGL), 电机固件说明
└── omnibothmi/           # 地面站
    └── overview.md       # OmniRobHmi.exe, py_ui.py, py_xbox.py
```

---

## 一分钟速览

```
Tinker 机器人
├── 硬件: 10DOF(5×2腿), 60cm高, ~12kg(V2实机)/~4.4kg(V1 URDF)
├── 控制: STM32F4(CAN→电机) + Odroid C4(SPI→STM32, PREEMPT_RT, 500Hz控制)
├── 训练: IsaacGym(NP3O/PPO) + MuJoCo(sim2sim验证)
├── 部署: PyTorch → JIT → TVM → C++ 推理(Odroid C4)
└── 文档: 飞书知识库 (本地仅代码+CAD, 无原理图)
```

---

## 关键路径速查

```bash
# === 训练 ===
conda activate cll                              # 进入 isaacgym 环境
cd /home/ubuntu/cll/LocomotionWithNP3O-masteroldx
export HEADLESS=1
python train.py                                  # 默认训练 Tinker
python simple_play.py                            # 推理验证

# === MuJoCo 可视化 ===
./simulate /home/ubuntu/cll/LocomotionWithNP3O-masteroldx/resources/tinker/xml/world_terrain.xml

# === sim2sim UDP 部署测试 ===
python sim2sim_tinker.py                         # MuJoCo sim2sim
python sim2sim_tinker_udp.py                     # UDP 桥接测试

# === 模型导出 ===
python pt2onnx.py                                # PyTorch → ONNX
python pt2tvm.py                                 # PyTorch → TVM (Odroid C4 部署)

# === 实机部署 (Odroid C4, 自动启动) ===
# 见 /home/ubuntu/cll/git_tinker/OmniBotSeries-Tinker-main/OmniBotHub/Linux/release/
#   hardware_task_tinker   → STM32 固件 (CAN 电机通信 + IMU + SBUS)
#   control_task_tinker    → 实时控制 (VMC/RL推理 + 状态估计, 500Hz)
#   navigation_task_tinker → 通信桥接 (OCU/ROS UDP 接口)
```

---

## 目录对应关系

| 本地路径 (`/home/ubuntu/cll/`) | 用途 |
|---|---|
| `LocomotionWithNP3O-masteroldx/` | **主训练代码** (NP3O + MuJoCo) |
| `tinker/` | Tinker V1 URDF (ROS Catkin 包) |
| `TinkerV2_URDF/` | Tinker V2 URDF (更大的 10DOF 设计) |
| `git_tinker/OmniBotSeries-Tinker-main/` | 全栈开源项目(硬件CAD+PCB+固件+软件) |
| `git_tinker/.../OmniBotCtrl/` | RL 训练配置 + 部署工具(CAN/SPI/TVM/MuJoCo) |
| `git_tinker/.../OmniBotHub/` | 机载计算机软件(Odroid C4, C++ 二进制) |
| `git_tinker/.../OmniBotSeries/` | 机械 CAD(切割/机加/3D打印 STEP 文件) |
| `git_tinker/.../OmniBotTele/` | 遥控器(ESP32+STM32 固件 + Altium PCB) |
| `new/LocomotionWithNP3O-masteroldx/` | 新版训练代码(多机器人支持) |

---

## 外部参考项目 (也在 `/home/ubuntu/cll/`)

| 项目 | 路径 | 机器人 | 算法 |
|---|---|---|---|
| 宇树 RL Gym | `unitree_rl_gym/` | G1/H1/Go2 | PPO (legged_gym) |
| 表情化人形运动 (ASE) | `expressive-humanoid/` | H1 | PPO-Mimic (动捕模仿) |
| 动捕重定向 | `git_amp/mocap_retarget/` | G1 | CasADi+IPOPT IK 求解 |
| 稚辉君 X1 训练 | `git_zhihuijun/agibot_x1_train/` | AgiBot X1 | PPO + 历史CNN编码器 |
| EngineAI 训练 | `git_zhongqin/engineai_legged_gym/` | ZqSA01 | PPO + 对称性损失 |
| VLN-CE 导航 | `git_vln_ce/VLN-CE-Isaac/` | Go2 | 视觉语言导航 |

---

## 外部文档链接 (飞书)

| 内容 | 链接 |
|---|---|
| BOM + CAD + PCB | `hcn64ij2s2xr.feishu.cn/wiki/DPWDwNWaZiGNvpkbRrGcBXCHn7d` |
| 项目总览 | `hcn64ij2s2xr.feishu.cn/wiki/AZJxwlvEpiWnRrkeBbGc21U9n7f` |
| 装配指南 | `hcn64ij2s2xr.feishu.cn/wiki/FqmXwnRYliwYFbkXCO7csSHWnfh` |
| 软件部署 | `hcn64ij2s2xr.feishu.cn/wiki/MJ25wew9PijvXlkIoH7cZ9e4nCh` |
| RL 训练 | `hcn64ij2s2xr.feishu.cn/wiki/LJxZwgL1diHB8ykKU1acFnXknOf` |
| GitHub 上游 | `github.com/golaced/OmniBotSeries-Tinker` |
| B站介绍 | `bilibili.com/video/BV1FRZoY1ExW` |

