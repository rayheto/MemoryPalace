# OmniBotCtrl — 训练与仿真

基于 IsaacGym + MuJoCo 的 RL 训练与 sim2sim 验证。

## 文件索引

| 文档 | 内容 |
|---|---|
| [training_pipeline.md](training_pipeline.md) | NP3O 算法, 10 种 Actor-Critic 变体, 39D 观测空间, 域随机化, sim2sim 桥接, TVM 导出 |

## 源码路径

```
OmniBotCtrl/OmniBotCtrl/
├── train.py                     # 训练入口
├── play.py                      # 推理评估
├── algorithm/np3o.py            # NP3O 算法 (PPO + 拉格朗日约束)
├── runner/
│   ├── on_constraint_policy_runner.py  # 训练循环
│   └── rollout_storage.py       # Rollout 缓冲区 (含 Cost 通道)
├── modules/
│   ├── actor_critic.py          # 10 种 Actor-Critic 变体 (1800+ 行)
│   ├── common_modules.py        # MLP/CNN/GRU 构建块
│   ├── transformer_modules.py   # Causal Transformer
│   └── depth_backbone.py        # 深度图 CNN
├── envs/
│   ├── legged_robot.py          # IsaacGym 环境 (1000+ 行)
│   └── legged_robot_2leg.py     # 双足专用环境
├── configs/
│   ├── tinker_constraint_him.py       # Tinker 基础配置 (1024 envs)
│   ├── tinker_constraint_him_trot.py  # Trot 模式
│   ├── tinker_constraint_him_stand.py # Stand 模式
│   └── tinker_constraint_him_cll.py   # 自定义高速配置
├── sim2sim_tinker.py            # MuJoCo 直接推理
├── sim2sim_tinker_udp.py        # UDP 桥接
└── tools/
    ├── pt2tvm.py                # TVM 编译 (x64 + ARM64)
    ├── py_xbox.py / py_ui.py    # Xbox 手柄 / Tkinter 遥控
    └── *.sh                     # 构建/部署/测试脚本
```

## 关联仓库

`LocomotionWithNP3O-masteroldx/` 是主训练目录 (更早期版本)，`OmniBotCtrl/` 是从 OmniBotSeries-Tinker 仓库中提取的训练配置版本。
