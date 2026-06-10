# RLinf 四大具身环境对比

## 总览

| 特性 | LIBERO | ManiSkill3 | MetaWorld | CALVIN |
|---|---|---|---|---|
| 机械臂 | Franka Panda | 多种 (Franka 等) | Sawyer | Franka Panda |
| 仿真后端 | robosuite (MuJoCo) | SAPIEN (GPU 并行) | MuJoCo | PyBullet |
| 任务数 | ~100 (4个套件) | 25 Main | 50 (MT50) | 4 组 (A/B/C/D) |
| 核心卖点 | 终身学习 + 泛化 | GPU 大规模并行采样 | 元学习/多任务 | 长序列语言指令链 |

---

## LIBERO
- **论文定位**：终身机器人学习基准（lifelong / continual learning）
- **四个子套件**：
  - LIBERO-Spatial — 泛化到未见过的空间位置
  - LIBERO-Object — 泛化到未见过的物体
  - LIBERO-Goal — 泛化到未见过的任务目标
  - LIBERO-Long — 长序列任务（10+ 步），测试长期规划
- **关键难点**：任务数量多（100），需要策略在连续学习新任务时不遗忘旧任务，同时泛化到新场景

## ManiSkill3
- **论文定位**：通用操作技能基准，强调 GPU 并行仿真加速
- **25 个 Main 任务**，覆盖 6 大类：
  - 拿起放置（Pick-and-place）
  - 关节物体操作（Articulation）
  - 装配（Assembly）
  - 流体/软体操作
  - 灵巧手操作
  - 移动操作
- **关键难点**：任务多样性极高，从刚体到软体、流体都有；得益于 SAPIEN 的 GPU 并行，可同时跑数千个环境，适合需要大量样本的 RL 方法

## MetaWorld
- **论文定位**：元学习（meta-learning）和多任务强化学习基准
- **MT50 = 50 个原子化桌面操作任务**：reach、push、pick-place、door-open、drawer-close、button-press 等
- 每类任务有参数化变体（如不同目标位置），天然适配元学习
- **关键难点**：任务数量多（50），要求单一策略掌握互不相关的技能集合；每项技能本身简单，但跨技能推理困难

## CALVIN
- **论文定位**：语言条件的长序列操作（Language-Conditioned Long-Horizon Manipulation）
- **ABC→D 划分**：
  - 训练集：环境 A、B、C 中的任务
  - 测试集：环境 D（全新场景），测试泛化能力
- **核心机制**：用户通过自然语言指令链串行下达多个子任务（如 "open drawer" → "pick up block" → "place in drawer"），策略必须理解语言并按序完成
- **关键难点**：长序列规划 + 语言理解 + 零样本泛化到新环境 D；每个子任务最多 5 步，整条指令链可达 20+ 步

---

## 在 RLinf 中的表现对比

| 环境 | π0 SFT | π0.5 SFT | 要点 |
|---|---|---|---|
| LIBERO | 57.6% | **77.1%** | π0.5 提升巨大（+19.5%） |
| ManiSkill3 | 38.4% | 40.1% | 两者都低，任务多样性是瓶颈 |
| MetaWorld | 50.8% | 43.8% | π0.5 反而更差，MT50 不适合此架构 |
| CALVIN | 57.5% | 61.3% | π0.5 小幅提升 |
