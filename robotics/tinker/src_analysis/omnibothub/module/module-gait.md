# control_task_tinker — `gait_src/` 步态行为层（5d）

## 概览

`gait_src/` 是步态/行为层，在 VMC 主状态机 (`vmc_src/locomotion_sfm.cpp`) 调度下，实现 Tinker 机器人的三类具体行为：

| 文件 | 行数 | 行为 |
|------|------|------|
| `stand.cpp` | 216 | **静止站立** — PD 控制到默认姿态 |
| `rl.cpp` | 208 | **强化学习步态** — TVM 神经网络推理（条件编译 `RL_USE_TVM`） |
| `self_right.cpp` | 452 | **自主恢复** — 摔倒后 7 阶段站起 |

每个文件提供两个对外函数：`Gait_<Behavior>_Active(...)` 用于"进入此模式"的一次性初始化，`Gait_<Behavior>_Update(dt)` 用于每个控制周期更新。

---

## 一、`stand.cpp` — 静止站立

### 1.1 功能

- 把 14 个关节平滑插值到 `leg_motor_all.q_init[]`（YAML `kin_param.init_qXY` 加载）；
- 使用站立刚度 `leg_motor_all.stiff_stand`；
- 不输出力、纯关节 PD。

### 1.2 公共函数

#### `Gait_Stand_Active()` — [stand.cpp:28](gait_src/stand.cpp#L28)

进入站立模式的一次性初始化：
- `vmc_all.param.robot_mode = M_STAND_RC`；
- `vmc_all.gait_mode = STAND_RC`；
- 14 关节目标角 ← `leg_motor_all.q_init[i]`；
- 关节刚度 ← `leg_motor_all.stiff_stand`。

**被调用处**：
- [locomotion_sfm.cpp:812](vmc_src/locomotion_sfm.cpp#L812) — 从 IDLE 进入站立；
- [locomotion_sfm.cpp:933](vmc_src/locomotion_sfm.cpp#L933) — RL 切回 Stand 的备用分支。

#### `Gait_Stand_Update(float dt)` — [stand.cpp:41](gait_src/stand.cpp#L41)

每个控制周期（~5 ms）调用，核心代码在文件尾部 209~212 行：

```cpp
for(i=0;i<14;i++){
    leg_motor_all.q_set[i] = move_joint_to_pos_all(
        leg_motor_all.q_set[i],   // 当前设定角
        leg_motor_all.q_init[i],  // 目标初始角
        180,                      // 最大速度（deg/s）
        dt);
}
```

> 文件主体（约 41~209 行）有大量 `#if 0` 注释，是早期 VMC trot 站立逻辑的残留，当前**未参与编译**。实际逻辑只剩末尾这 4 行 PD 平滑插值。

**被调用处**：[locomotion_sfm.cpp:663](vmc_src/locomotion_sfm.cpp#L663)。

### 1.3 与其他模式的转移

| 来源状态 | 转入条件 | 目标 |
|---------|---------|------|
| IDLE | 按键 LL/RR | STAND_RC（调用 `Gait_Stand_Active`） |
| STAND_RC | Key X / `timer_auto_switch > 0.15s` 且 RL 就绪 | G_RL（调用 `Gait_RL_Active(1)`） — [locomotion_sfm.cpp:884](vmc_src/locomotion_sfm.cpp#L884) |
| STAND_RC | `auto_sit_down_t > 5s` | SOFT（`ocu.cmd_robot_state = 99`）— [locomotion_sfm.cpp:853-860](vmc_src/locomotion_sfm.cpp#L853-L860) |

---

## 二、`rl.cpp` — 强化学习步态

通过 **TVM 推理运行时** 加载 PyTorch 训练并编译的策略 `.so` 文件，实时输出 10 个被控关节的动作。

### 2.1 编译开关：`RL_USE_TVM`

整文件以 `#if RL_USE_TVM` 包围。

- **CMakeLists.txt** 中**未**显式 `add_definitions(-DRL_USE_TVM=...)`；
- 该宏应在 `vmc_inc/include.h` 或上游头文件中定义（取值 `0`/`1`）；
- 若取 0，所有 RL 代码块（包括函数体）被剔除，文件接近空函数。

代码结构：

```cpp
#if RL_USE_TVM
    #include "tvm2.h"
    #include <iomanip>

    Tvm::tvm2Class tvm;                     // TVM 推理器实例
    vecNf(39)        obs1;                  // 当前帧观测 (39)
    vecNf(39 * 10)   obs10;                 // 历史窗口 (390 = 39×10 帧)
    vecNf(10)        action, actionOld,
                     actionTmp, actionFilt; // 动作输出 (10)

    float smooth_rc    = 0.03;              // 命令平滑系数
    float dead_zone_rc = 0.01;              // 死区
    float cmd_x, cmd_y, cmd_rate;           // 当前命令
    float eu_ang_scale = 1;
    float omega_scale  = 0.25;
    float pos_scale    = 1.0;
    float vel_scale    = 0.05;
    float lin_vel      = 2.0;
    float ang_vel      = 0.25;

    int id_list[10]    = {0,1,2,3,4, 7,8,9,10,11};  // 被控关节 ID
    int rl_model_loaded = 0, first_loaded_model = 0;
#endif
```

被控的 10 个关节即 `id_list[]`：左腿前 5 个（0..4）+ 右腿前 5 个（7..11），跳过两腿后 2 个槽位（5/6/12/13）。

### 2.2 `Gait_RL_Active(char rst)` — [rl.cpp:41](gait_src/rl.cpp#L41)

**参数语义**：

| `rst` | 含义 |
|-------|------|
| 0 | "继续当前模型"（重入用，不重新加载） |
| 1 | **RL Trot**（快速走） |
| 2 | **RL Stand**（站立/原地缓动） |
| 3 / 4 / 5 | Dance 1 / 2 / 3 |
| 6 | Jump（跳跃） |

**初始化序列**：

1. **基础初始化**（lines 43-73）：
   - 设置 `robot_mode = M_RL`、`gait_mode = G_RL`；
   - 从 YAML 加载 14 关节刚度（乘以 `vmc_all.rl_gain` 缩放）；
   - 设 `vmc_all.net_run_dt = 0.01`（默认 100 Hz）；
   - 重置 IMU 姿态偏置等。

2. **TVM 模型加载**（lines 74-116，`#if RL_USE_TVM` 内）：

   ```cpp
   if (rst != vmc_all.rl_mode_used && rst != 0) {
       if (!first_loaded_model) {
           first_loaded_model = 1;
           action.setZero(); actionOld.setZero(); /* ... */
       }
       vmc_all.net_run_dt = 0.016;  // RL 步态实际 62.5 Hz

       switch (rst) {
           case 1: loaded = tvm.init("Model/Trot/policy_arm64_cpu.so",   39, 390, 10); break;
           case 2: loaded = tvm.init("Model/Stand/policy_arm64_cpu.so",  39, 390, 10); break;
           case 3: loaded = tvm.init("Model/Dance1/policy_arm64_cpu.so", 39, 390, 10); break;
           case 4: loaded = tvm.init("Model/Dance2/policy_arm64_cpu.so", 39, 390, 10); break;
           case 5: loaded = tvm.init("Model/Dance3/policy_arm64_cpu.so", 39, 390, 10); break;
           case 6: loaded = tvm.init("Model/Jump/policy_arm64_cpu.so",   39, 390, 10); break;
       }
       if (loaded) {
           rl_model_loaded = 1;
           vmc_all.rl_mode_used = rst;
           printf("RL:: Model policy=%d loaded good!\n", rst);
       } else {
           // 加载失败 → 回退到 Stand
           vmc_all.rl_mode_used = 0;
           ocu.cmd_robot_state  = 2;
           Gait_Stand_Active();
       }
   }
   ```

**`tvm.init(path, 39, 390, 10)` 含义**：
- `path` — 共享库；
- `39` — 输入 1：当前帧观测维度；
- `390` — 输入 2：历史窗口（10 帧）维度；
- `10` — 输出动作维度。

**模型文件路径**：相对可执行文件工作目录的 `Model/<类型>/policy_arm64_cpu.so`。

**被调用处**：
- [locomotion_sfm.cpp:884](vmc_src/locomotion_sfm.cpp#L884) — Stand → RL Trot（`rst=1`）；
- [locomotion_sfm.cpp:941](vmc_src/locomotion_sfm.cpp#L941) — RL 切到 Stand 模型（`rst=2`）；
- [locomotion_sfm.cpp:950](vmc_src/locomotion_sfm.cpp#L950) — 切回 RL Trot（`rst=1`）；
- [locomotion_sfm.cpp:1031](vmc_src/locomotion_sfm.cpp#L1031) — 踢球后恢复（`rst=0`）。

### 2.3 `Gait_RL_Update(float dt)` — [rl.cpp:120](gait_src/rl.cpp#L120)

每个 VMC 周期被调用，**只在累计达到 `net_run_dt`（默认 16 ms）时**实际跑一次推理。

**5 个步骤**：

1. **节拍累积**（lines 124-127）：
   ```cpp
   static float timer[10];
   timer[0] += dt;
   if (timer[0] > vmc_all.net_run_dt && rl_model_loaded) { /* 进入推理 */ }
   ```

2. **命令融合**（lines 128-149）：
   遥控/SDK 的 x/y/yaw 速度命令经"限幅 + 偏置 + 平滑 + 死区"：
   ```cpp
   cmd_x_temp = LIMIT(vmc_all.tar_spd.x, -MAX_SPD_X*0.5, MAX_SPD_X) + vmc_all.rl_commond_rl_rst[0] + …;
   cmd_y_temp = LIMIT(-vmc_all.tar_spd.y, -MAX_SPD_Y, MAX_SPD_Y) + …;
   // yaw 速率：遥控直接命令 或 反馈控制
   cmd_x = cmd_x*(1-smooth_rc) + (|cmd_x_temp|<dead_zone_rc ? 0 : cmd_x_temp) * smooth_rc;
   // cmd_y / cmd_rate 同理
   ```

3. **观测向量构造 `obs1[39]`**（lines 151-178）：

   | 索引 | 内容 |
   |------|------|
   | 0..2 | `-roll_rate, pitch_rate, yaw_rate` × `omega_scale` |
   | 3..5 | `-roll, pitch, -yaw` × `eu_ang_scale`（弧度） |
   | 6..8 | `cmd_x · lin_vel`, `cmd_y · lin_vel`, `cmd_rate · ang_vel` |
   | 9..18 | 关节角误差 `(q_now − default_action) · pos_scale`，对 10 个被控关节 |
   | 19..28 | 关节角速度 `qd_now · vel_scale` |
   | 29..38 | 上一帧动作 `action[i]` |

4. **历史窗口滑动**（lines 180-181）：
   ```cpp
   obs10.head<39*9>() = obs10.tail<39*9>();   // 丢弃最旧
   obs10.tail<39>()   = obs1;                 // 追加当前
   ```

5. **TVM 推理 + 后处理**（lines 183-206）：
   ```cpp
   tvm.in1 = obs1;
   tvm.in2 = obs10;
   tvm.run();                              // → tvm.out (10)

   action = actionOld * 0.2 + tvm.out * 0.8; // 指数平滑
   actionOld = tvm.out;
   for (int i=0;i<10;i++) action[i] = LIMIT(action[i], -5, 5);

   for (int i=0;i<10;i++) {
       float tgt = action[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];
       leg_motor_all.q_set[id_list[i]] = tgt * 57.3;  // rad → deg
   }
   ```

**被调用处**：[locomotion_sfm.cpp:666](vmc_src/locomotion_sfm.cpp#L666)（VMC 主循环）。

### 2.4 RL 模型清单

| 文件路径 | rst | 行为 |
|----------|-----|------|
| `Model/Trot/policy_arm64_cpu.so` | 1 | 快速行走 |
| `Model/Stand/policy_arm64_cpu.so` | 2 | 站立/缓动 |
| `Model/Dance1/policy_arm64_cpu.so` | 3 | 舞蹈 1 |
| `Model/Dance2/policy_arm64_cpu.so` | 4 | 舞蹈 2 |
| `Model/Dance3/policy_arm64_cpu.so` | 5 | 舞蹈 3 |
| `Model/Jump/policy_arm64_cpu.so` | 6 | 跳跃 |

需运行时在工作目录下放置完整 `Model/` 树。

---

## 三、`self_right.cpp` — 自主恢复

机器人侧翻角超过阈值（`|roll| > SAFE_ROLL ≈ 45°` 持续 ≥ 0.05s）时进入此模式，分 **7 个阶段**逐步把身体翻正、收拢腿到安全位置、最终回到站立。

### 3.1 文件级常量（lines 8-27）

```cpp
float k_roll_reset = 1.5;                    // 滚转缩放
float side_press_safe_45[3] = {120, 45, -35}; // 被压腿安全角
float side_free_safe_45[3]  = {60, 110, -3};  // 自由腿安全角

float side_press_reset_1[3] = {120, 110, -35};
float side_free_reset_1[3]  = {60, 110,  -3};
// reset_2..reset_5 同结构，渐进恢复
```

`vmc_all.param.safe_sita[3]` — 最终安全位置，由 YAML 加载。

### 3.2 辅助函数

#### `move_joint_to_pos(VMC *in, int joint, float tar, float max_spd, float dt)` — [self_right.cpp:31](gait_src/self_right.cpp#L31)

对**单条腿**的指定关节做平滑插值；关节误差 < 0.5° 时返回 1（"已到达"）。

- `joint` ∈ {0:髋俯仰, 1:膝, 2:髋外摆}；
- 用于 7 阶段中每阶段判定"全部关节是否就位"。

#### `move_joint_to_pos_all(float now, float tar, float max_spd, float dt)` — [self_right.cpp:72](gait_src/self_right.cpp#L72)

非绑定 VMC 的全局版本（被 `stand.cpp` 也复用）：

```cpp
err = tar - now;
out = now + LIMIT(err * k_self_right_p /*=35*/, -max_spd, max_spd) / 2.0 * dt;
```

### 3.3 入口与状态机

#### `Gait_Recovery_Active()` — [self_right.cpp:212](gait_src/self_right.cpp#L212)

进入恢复模式的一次性动作：
- `vmc_all.param.robot_mode = M_RECOVER`；
- `vmc_all.gait_mode = RECOVER`；
- `vmc_all.fall_self = 1`；
- 调用一次 `Gait_Recovery_Falling(0.005)` 立刻摆出安全姿态；
- 清空 4 条腿的接地标志、重置 `timer_self_reset[i]`、`self_right_state = 0`。

**被调用处**：[locomotion_sfm.cpp:992](vmc_src/locomotion_sfm.cpp#L992)。

#### `Gait_Recovery_Falling(float dt)` — [self_right.cpp:230](gait_src/self_right.cpp#L230)

摔倒瞬间的"安全姿态"：根据 roll 正负确定哪条腿被压在身下、哪条自由，把被压腿伸长（防卡）、自由腿收拢（防外伸）。

#### `Gait_Recovery_Update(float dt)` — [self_right.cpp:292](gait_src/self_right.cpp#L292)

7 个状态阶段：

| State | 行号范围 | 动作 |
|-------|---------|------|
| **0 Falling** | [:304-315](gait_src/self_right.cpp#L304-L315) | 持续重复安全姿态；等待角速率稳定 1 s |
| **1 Stabilize** | [:316-365](gait_src/self_right.cpp#L316-L365) | 按 roll 正负确定 press/free 腿；移动到 `side_*_safe_45`；角速率 < 10°/s 后进入下一阶段 |
| **2..5 Reset 1..4** | [:367-422](gait_src/self_right.cpp#L367-L422) | 递进地把 4 条腿移动到 `*_reset_1..4`（press/free 各有一套目标） |
| **6 Final** | [:424-449](gait_src/self_right.cpp#L424-L449) | 移动到全局安全 `safe_sita[3]`；`|roll| < 10°` 时清 `fall_self`，可选自动转 Stand |

每阶段进入下一阶段的条件：`move_joint_to_pos()` 对所有 12 个关节（4 腿 × 3）都返回 1（到达），且角度阈值满足。

**被调用处**：[locomotion_sfm.cpp:669](vmc_src/locomotion_sfm.cpp#L669)。

---

## 四、与 VMC 状态机的集成

### 4.1 全局状态机（基于 `ocu.cmd_robot_state` 与 `vmc_all.gait_mode`）

```
State 0 Safe (上电)
  └─ ok → State 1
State 1 IDLE
  └─ Key LL/RR → State 2 (Stand)         调用 Gait_Stand_Active()
State 2 STAND_RC
  ├─ Key X / 自动切换 → State 18         调用 Gait_RL_Active(1)
  └─ 5s 超时 → State 99 (SOFT)
State 18 RL Locomotion
  ├─ Key B / 自动 (Trot)  → Gait_RL_Active(2)  RL Stand
  ├─ Key X / 自动 (Stand) → Gait_RL_Active(1)  RL Trot
  ├─ Key A             → State 20 (Kick)
  └─ |roll|>SAFE_ROLL 持续 → State 12 (Recovery)  调用 Gait_Recovery_Active()
State 20 Kick
  └─ timer>0.05s → State 18              调用 Gait_RL_Active(0)
State 12 Recovery
  └─ 7 阶段完成 → State 2 (Stand)
State 99 SOFT (休眠)
  └─ Key ← → State 1
```

### 4.2 调用入口汇总表

| 文件 / 函数 | locomotion_sfm.cpp 调用行 | 触发条件 |
|------------|--------------------------|---------|
| `Gait_Stand_Active` | [:812](vmc_src/locomotion_sfm.cpp#L812), [:933](vmc_src/locomotion_sfm.cpp#L933), [:1017](vmc_src/locomotion_sfm.cpp#L1017) | 进入 STAND_RC |
| `Gait_Stand_Update` | [:663](vmc_src/locomotion_sfm.cpp#L663) | 每帧（STAND_RC） |
| `Gait_RL_Active` | [:884](vmc_src/locomotion_sfm.cpp#L884), [:941](vmc_src/locomotion_sfm.cpp#L941), [:950](vmc_src/locomotion_sfm.cpp#L950), [:1031](vmc_src/locomotion_sfm.cpp#L1031) | RL 模式切换 |
| `Gait_RL_Update` | [:666](vmc_src/locomotion_sfm.cpp#L666) | 每帧（G_RL） |
| `Gait_Recovery_Active` | [:992](vmc_src/locomotion_sfm.cpp#L992) | 严重侧翻 |
| `Gait_Recovery_Update` | [:669](vmc_src/locomotion_sfm.cpp#L669) | 每帧（RECOVER） |

---

## 五、依赖关系

| 依赖项 | 用途 |
|--------|------|
| `vmc_inc/locomotion_header.h` | 函数声明、VMC 数据结构 |
| `vmc_inc/base_struct.h` | `vmc_all`、`leg_motor_all`、`ocu` 等全局对象 |
| `math_inc/gait_math.h` | 辅助数学（夹角、限幅、180 度归一化等） |
| `math_src/eso_RL.cpp` | RL 步态的关节级 ESO 扰动观测 |
| `tvm2.h`（来自 `loong_ctrl` 动态库） | TVM 推理封装；仅在 `RL_USE_TVM=1` 时启用 |
| `inc/can.h` | 电机数据结构（被全模块共享） |
| `yaml-cpp` | `Gait_RL_Active()` 中直接读 `config_gait["imp_param"]` |

---

## 六、横向对比

| 特性 | `stand.cpp` | `rl.cpp` | `self_right.cpp` |
|------|-------------|---------|------------------|
| 控制方式 | 关节 PD 到 `q_init` | TVM 神经网络推理 | 多阶段关节 PD |
| 控制对象 | 14 关节 | 10 关节（`id_list`） | 12 关节（4×3） |
| 工作频率 | 每帧（~500 Hz） | 推理 62.5 Hz、调用每帧 | 每帧 |
| 输入信号 | 无（仅目标角） | IMU + 关节 + 命令 + 历史 | IMU 角度 + 关节角度 |
| 条件编译 | 无 | `#if RL_USE_TVM` | 无 |
| 模型外部依赖 | 无 | `Model/*/policy_arm64_cpu.so` | 无 |
| 失败回退 | — | 加载失败 → `Gait_Stand_Active()` | 7 阶段任何阶段卡住将循环重试 |
| 关键常量来源 | `q_init` / `stiff_stand` (YAML) | `default_action`, `action_scale`, `net_run_dt`, `imp_param.*` (YAML) | 文件内硬编码 `side_press/free_*[3]`, `k_self_right_p`, `safe_sita`(YAML) |
