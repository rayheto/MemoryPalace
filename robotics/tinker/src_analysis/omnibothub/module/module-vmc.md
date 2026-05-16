# control_task_tinker — `vmc_src/` 虚拟模型控制层（5c）

## 概览

`vmc_src/` 是 control_task_tinker 的**运动控制核心层**（VMC：Virtual Model Control）。从 `src/main.cpp` Thread_T1 主循环以 ~500 Hz 频率被驱动，负责：

1. **硬件抽象**：把 IMU、关节状态从共享内存读入并做姿态融合；
2. **状态机**：根据 `vmc_all.gait_mode` 在 stand / RL / recovery / kick / soft 等步态间切换；
3. **力/位置控制**：把上层步态产出的关节目标（`leg_motor_all.q_set[]`）与力期望转成电机扭矩；
4. **SDK 接口预留**：远程二次开发桩。

| 文件 | 行数 | 角色 |
|------|------|------|
| `locomotion_sfm.cpp` | 1104 | 主状态机 + 控制循环入口（**最核心**） |
| `hardware_interface.cpp` | 755 | IMU/编码器读入、姿态融合、写关节命令 |
| `force_imp_controller.cpp` | 105 | 力 + 阻抗混合控制器 |
| `sdk_api.cpp` | 22 | SDK 远程接口（大量空桩） |

对应头文件位于 [vmc_inc/](vmc_inc/)：`base_struct.h`（全局数据结构）、`gait_math.h`（雅可比、坐标转换、轨迹）、`include.h`（全局头入口）、`locomotion_header.h`（VMC API）。

---

## 一、`locomotion_sfm.cpp` — VMC 主状态机

文件结构上是一个被 Thread_T1 主循环周期性调用的**大 switch 状态机** + 步态调度。

### 1.1 控制循环入口

`locomotion()` 与相关入口位于约 [locomotion_sfm.cpp:482](vmc_src/locomotion_sfm.cpp#L482)。

执行序列（每周期）：

1. **case 0**：上电安全态；
2. **case 1**：完成初始化后转入 IDLE；
3. **case 2**（主控循环）：
   - 调用 `gait_switch(dt)`（约 [locomotion_sfm.cpp:734](vmc_src/locomotion_sfm.cpp#L734)）处理模式切换；
   - 按 `switch(vmc_all.gait_mode)` 分派到具体步态更新函数：
     - `Gait_Stand_Update(dt)` — [locomotion_sfm.cpp:663](vmc_src/locomotion_sfm.cpp#L663)；
     - `Gait_RL_Update(dt)` — [locomotion_sfm.cpp:666](vmc_src/locomotion_sfm.cpp#L666)；
     - `Gait_Recovery_Update(dt)` — [locomotion_sfm.cpp:669](vmc_src/locomotion_sfm.cpp#L669)；
   - 最后调用 `force_control_and_dis_*()` 系列，把目标关节角与期望力转成扭矩。

### 1.2 步态模式枚举

定义在 [vmc_inc/base_struct.h](vmc_inc/base_struct.h)：

| 值 | 名称 | 含义 |
|----|------|------|
| 0 | IDLE | 待命，不输出扭矩 |
| 1 | TROT | 经典 trot（备用，主要走 RL 路径） |
| 2 | STAND_RC | 站立 (Remote Control) |
| 3 | RECOVER | 摔倒自起 |
| 5 | G_RL | 强化学习步态 |
| 6 | G_KICK | 踢球动作 |
| 7 | SOFT | 软着陆 / 断电过渡 |

**典型转移**：STAND_RC ↔ G_RL（用户按键 / 自动切换计时器）；任何状态 → RECOVER（侧翻超阈值）。

### 1.3 状态机调用关系（外部视角）

```
src/main.cpp::Thread_T1 (500 Hz)
     │
     ▼
locomotion()                                  ← locomotion_sfm.cpp:482
     │
     ├─ 读 vmc_all / leg_motor_all (由硬件线程刷新)
     ├─ gait_switch(dt)                        ← :734  按键、自动切换、角度阈值
     ├─ switch(vmc_all.gait_mode):
     │    case STAND_RC:  Gait_Stand_Update    ← gait_src/stand.cpp:41
     │    case G_RL:      Gait_RL_Update       ← gait_src/rl.cpp:120
     │    case RECOVER:   Gait_Recovery_Update ← gait_src/self_right.cpp:292
     │    case TROT:      Gait_Trot_Update_v1  (备用 trot 路径)
     │    case G_KICK:    踢球分支
     │    case SOFT:      软着陆
     └─ force_control_and_dis_*()              ← 内部：把 q_set 与期望 GRF 转成 τ
```

---

## 二、`hardware_interface.cpp` — 硬件抽象与姿态融合

把 `src/memory_share.cpp` 暴露的 MEM_SPI 共享内存数据（IMU、电机反馈）**解析、滤波、坐标对齐**后填入 `vmc_all` / `leg_motor_all` 全局结构。是 VMC 唯一直接接触原始传感数据的地方。

### 2.1 主要函数（按调用次序）

| 函数 | 行号 | 职责 |
|------|------|------|
| `subscribe_imu_to_webot()` | [:15](vmc_src/hardware_interface.cpp#L15) | 读 IMU 数据；融合姿态（roll/pitch/yaw）；构造 3 种旋转矩阵 |
| `subscribe_webot_to_vmc()` | [:185](vmc_src/hardware_interface.cpp#L185) | 读电机当前角/角速度，映射到 4 条腿 × 3 关节的本地系 |

### 2.2 三种旋转矩阵

由 `subscribe_imu_to_webot()` 维护，存于全局 `robotwb`：

| 矩阵 | 含义 | 用途 |
|------|------|------|
| `Rn_b` | World → Body | 把世界系下规划的力/位置转回机身系 |
| `Rnn_b` | Yaw-aligned → Body | 解耦航向 |
| `Rn_g` | World → Gravity-aligned | 重力方向对齐参考系 |

调用 `math_src/RT_math.cpp`、`math_src/common_math.cpp` 提供的矩阵原语。IMU 角速率/欧拉角经过 `fliter_math.cpp` 的 `DigitalLPF` 平滑后才进入控制律。

### 2.3 关节状态读写

- **读**：把 MEM_SPI 中 14 个电机的位置/速度按"软件关节编号"映射到 `leg_motor_all.q_now[14]`、`qd_now[14]`、并按腿拆分到 `vmc[4]` 中（4 条腿，含模型坐标系下的足端期望/实际位置）；
- **写**：`leg_motor_all.q_set[14]` 与扭矩输出通过 `subscribe_*_to_webot()` 的对应"上行"函数写回 MEM_SPI。

---

## 三、`force_imp_controller.cpp` — 力/阻抗混合控制器

105 行，实现 VMC 中将"足端期望力 + 期望位置"合并为关节扭矩的核心控制律。位于 [vmc_src/force_imp_controller.cpp:42-105](vmc_src/force_imp_controller.cpp#L42-L105)。

### 3.1 控制律

```
τ_joint = Jᵀ · ( F_cmd_world          ← 上层力期望（站立/支撑相）
                + Kp · (p_des − p_now) ← 笛卡尔空间位置 PD
                + Kd · (ṗ_des − ṗ_now)
              )                         ← 全部在合适坐标系下计算
```

其中 `J` 来自 `kin_math.cpp::cal_jacobi_new()`，Kp/Kd 来自 YAML 的 `imp_param.imp_x/y/z_kp/kd`。

### 3.2 支撑相 / 摆动相

- **支撑相**：力主导；位置 PD 弱化，主要发挥扰动抑制；
- **摆动相**：位置主导（Bezier 轨迹 `bezier_math.cpp` 提供 p_des/ṗ_des），力期望近零。

切换由步态层（如 `gait_src/`）通过 `vmc[i].ground` 标志声明。

---

## 四、`sdk_api.cpp` — SDK 远程接口（预留）

仅 22 行，大部分函数为空桩（只保留签名）。预期对接外部 SDK 客户端用于：
- 实时读取机器人状态；
- 注入高层命令；
- 启停步态、调参。

目前线网络通道由 `src/main.cpp` 中 Thread_UDP_OCU 与 RL UDP 服务直接承担，此文件用作未来正式 SDK 的占位。

---

## 五、关键数据结构（来自 `vmc_inc/base_struct.h`）

集中定义于 [vmc_inc/base_struct.h:265-589](vmc_inc/base_struct.h#L265-L589) 范围内。

| 全局对象 | 类型 | 内容（概要） |
|---------|------|--------------|
| `vmc_all` | `VMC_ALL` | 机身姿态 / 期望姿态 / 步态模式 / RL 配置 / 命令偏置 / IMU 零偏 |
| `vmc[4]` | `VMC` × 4 | 4 条腿：髋位置、足端位置（多坐标系）、Jacobian、关节角、`ground` 接地标志 |
| `leg_motor_all` | `_LEG_MOTOR_ALL` | 14 自由度统一数组（kp/kd/stiff/q_now/q_set/q_init/q_max/q_min/扭矩） |
| `robotwb` | `RobotWB` | 旋转矩阵 `Rn_b`、`Rb_n` 等；当前 / 期望姿态（`now_att`、`exp_att`） |
| `ocu` | `OCU` | 遥控器/SDK 输入：`cmd_robot_state`（主状态机命令位）、按键、摇杆等 |
| `gait_ww` | `GAIT_WW` | 系统级自动切换开关（来自 YAML） |

---

## 六、与 math_src / gait_src 的交互

### 6.1 调用 `math_src/`

| math_src 模块 | 被 vmc_src 用法 |
|---------------|----------------|
| `kin_math.cpp` | FK/IK/Jacobian、力↔关节扭矩映射 |
| `RT_math.cpp` | 坐标变换 `converV_*`、矩阵运算 |
| `bezier_math.cpp` | 摆动相足端轨迹 |
| `imp_math.cpp` | 站立 / 支撑相力前馈 |
| `eso.cpp` | 姿态外环 ADRC 观测 |
| `fliter_math.cpp` | IMU 角速率平滑、跟踪微分器 |
| `common_math.cpp` | 矩阵/三角/四元数 |

### 6.2 被 `gait_src/` 反向调用

`locomotion()` 是 VMC 入口；步态层提供具体行为：
- `Gait_Stand_*` ← `gait_src/stand.cpp`
- `Gait_RL_*` ← `gait_src/rl.cpp`
- `Gait_Recovery_*` ← `gait_src/self_right.cpp`

详见 [module-gait.md](module-gait.md)。

### 6.3 与 `src/` 的边界

- `src/main.cpp` 调度 → `locomotion()`；
- `src/memory_share.cpp` 准备 IMU/电机数据 → `hardware_interface.cpp` 读取；
- `src/robot_param.cpp` 加载 YAML 到 `vmc_all`/`leg_motor_all` 字段，VMC 不直接访问 yaml-cpp。

---

## 七、状态机控制流总图

```
┌──────────────────────────────────────────────────────────────┐
│              Thread_T1 (500 Hz) 周期入口                      │
│                  src/main.cpp                                  │
└─────────────────────────┬─────────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │   locomotion()        │
              │ locomotion_sfm.cpp    │
              └─────────┬─────────────┘
                        │
        ┌───────────────┼───────────────────┐
        ▼               ▼                   ▼
  hardware_iface    gait_switch()       gait dispatch
  IMU+电机→vmc_all  按键/角度/计时器    switch(gait_mode)
                                              │
        ┌─────────────────────┬────────┬──────┴──────┬──────────┐
        ▼                     ▼        ▼             ▼          ▼
   STAND_RC               G_RL    RECOVER       G_KICK       SOFT
   stand.cpp           rl.cpp   self_right.cpp  …踢球…       软着陆
        └─────────────────────┴────────┴─────────────┘
                                │
                                ▼
                  leg_motor_all.q_set[14] / F_expect
                                │
                                ▼
            force_imp_controller.cpp / force_control_and_dis_*()
                                │
                                ▼
                  电机扭矩 → MEM_SPI → STM32 → CAN → 电机
```

---

## 八、安全与降级

- **侧翻保护**：`|att_ctrl[ROLr]| > SAFE_ROLL` 持续 > 0.05s → 进入 `RECOVER`（[locomotion_sfm.cpp:992](vmc_src/locomotion_sfm.cpp#L992)）；
- **超时保护**：`auto_sit_down_t > 5s` → `SOFT`（断扭矩待机）；
- **RL 模型加载失败**：`Gait_RL_Active()` 内回退到 `Gait_Stand_Active()`（见 [module-gait.md](module-gait.md) §2.3）；
- **角度限位**：所有 `leg_motor_all.q_set[]` 受 `q_max/q_min`（YAML 加载）约束。

---

## 九、扩展点

| 扩展位置 | 用途 |
|----------|------|
| `sdk_api.cpp` 空桩 | 接入正式 SDK |
| `mpc_locomotion/`（仓库内尚未创建） | 引入 MPC 力分配；`qpOASES` 已链入 |
| `vision_location/`（仓库内尚未创建） | 视觉/SLAM 输入到 `vmc_all` |

参见 [module-reserved.md](module-reserved.md)。
