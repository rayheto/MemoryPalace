# control_task_tinker — `Param_*` 配置文件模块（5e）

## 概览

`Param_*` 系列目录不参与编译，是**运行时**加载的机器人参数集合。每个目录对应一种机型/规格，内容为两份 YAML：

| 目录 | 机型 | 备注 |
|------|------|------|
| `Param_Tinker14/` | Tinker14（14 自由度双足，标准型） | 主参数集，与 `param_robot.yaml` 的 `robot: tinker14-small-01` 对应 |
| `Param_Tinker14_Zero/` | Tinker14 Zero（14 自由度，"零位"变体） | reset/init 关节角全部为 0，用于零位标定 |
| `Param_Tinker12/` | Tinker12（12 自由度，四足/精简型） | `kin_param.dof_num: [5,5,5,5]`，含 `kin_head` 字段；机型名 `tinymal-small-01` |
| `Param_Taitan14_Zero/` | Taitan14 Zero（14 自由度，Taitan 平台零位变体） | 关节限位更大（max_q03=147 等），扭矩 `tau_max` 更高 |

> 在 [CMakeLists.txt:84](../../../../../home/hlei/robotic/OmniBotSeries-Tinker/OmniBotHub/Linux/control_task2/CMakeLists.txt#L84) 中存在 `AUX_SOURCE_DIRECTORY(Param DIR_SRCS4)` 的占位条目，但仓库内并无 `Param/` 源码目录，4 个 `Param_*` 目录均不被 CMake 收录。

---

## 一、运行时加载机制

**硬编码路径：**

```cpp
// src/robot_param.cpp:8-9
YAML::Node config_robot=YAML::LoadFile("/home/odroid/Tinker/Param/param_robot.yaml");
YAML::Node config_gait=YAML::LoadFile("/home/odroid/Tinker/Param/param_gait.yaml");
```

参考 [robot_param.cpp:8-9](src/robot_param.cpp#L8-L9)。

**部署流程**：用户根据机型，把对应 `Param_Tinker14/` 等目录复制（或软链接）到目标机 odroid 用户的 `~/Tinker/Param/` 路径，启动时由 yaml-cpp 在**全局对象构造期**加载（即 main() 进入前已读完）。

**加载入口**：`robot_param_read()`（[robot_param.cpp:11](src/robot_param.cpp#L11)）——由 main 启动初始化调用，把 YAML 数据填入全局结构 `vmc_all`、`gait_ww`、`leg_motor_all` 及全局变量 `MAX_SPD_X/Y/RAD`。

---

## 二、文件 1：`param_robot.yaml` —— 机器人本体参数

按 YAML 顶层 key 分四段，共约 187 行。

### 2a. `robot_param` —— 几何与质量

| 字段 | 说明 | Tinker14 示例 |
|------|------|---------------|
| `L1` | 大腿长 (m) | 0.12 |
| `L2` | 小腿长 (m) | 0.14 |
| `L3` | 足端偏置 (m) | 0.075 |
| `L4` | 预留长度 | 0.0 |
| `H` | 站立基准高度 (m) | 0.306 |
| `W` | 髋部半宽 (m) | 0.09 |
| `Mess` | 整机质量 (kg) | 12.0 |
| `Ix/Iy/Iz_Body` | 机身惯量张量主对角元 | 0.04 / 0.075 / 0.06 |

Tinker12 多出 `Q_side_bias`，Taitan/Tinker12 在 `kin_param` 中含 `kin_head: [1,-1,1,-1]`（足端方向反向标志）。

### 2b. `kin_param` —— 腿部关节运动学参数（左腿 0xx / 右腿 1xx，7 关节/腿）

每个关节定义 5 类角度（**单位：度**），由 `robot_param.cpp` 全量加载：

| 前缀 | 含义 | 加载到 | 加载代码 |
|------|------|--------|----------|
| `init_qXY` | 上电初始关节角 | `leg_motor_all.q_init[i]` | [robot_param.cpp:78-92](src/robot_param.cpp#L78-L92) |
| `max_qXY` | 关节角上限 | `leg_motor_all.q_max[i]` | [robot_param.cpp:127-141](src/robot_param.cpp#L127-L141) |
| `min_qXY` | 关节角下限 | `leg_motor_all.q_min[i]` | [robot_param.cpp:143-157](src/robot_param.cpp#L143-L157) |
| `reset_qXY` | 复位姿态（关机/收拢） | `leg_motor_all.q_reset[i]` | [robot_param.cpp:111-125](src/robot_param.cpp#L111-L125) |
| `tau_maxXY` | 关节力矩上限 (N·m) | 后续部分加载 | — |

**索引约定**：`qXY` 中 X∈{0,1} 表示左/右腿，Y∈{0..6} 是该腿 7 个关节序号；映射成线性数组 `i = X*7 + Y`，故 `leg_motor_all.q_xxx[14]` 中 0~6 是左腿、7~13 是右腿。`dof_num: 14` 表明合计 14 自由度（左右各 7，部分关节 `_q05/_q06` 数值为 0 表示空槽）。

**Tinker14 vs Zero**：Zero 变体的 `reset_q*` 全 0、`init_q*` 数值不同（如 init_q02 从 56 → 32），用于物理零位标定姿态。

### 2c. `servo_param` —— 上身/伺服关节参数（同样 14 个槽位）

与 `kin_param` 结构对称，加载到 `leg_motor_all.q_set_servo_init/off/max/min`（[robot_param.cpp:160-222](src/robot_param.cpp#L160-L222)）。Tinker14 中仅左臂前 3 个 servo 启用（init_q00=-90, init_q01=-45, init_q02=0），其余为 0。

### 2d. `dyn_param` —— 动力学辨识参数（用于 RL/MPC 重力补偿）

```yaml
MESS_KIN1:    0.3   # 大腿连杆质量 kg
MESS_KIN12:   0.02  # 关节质量 kg
MESS_KIN2:    0.32  # 小腿连杆质量 kg
L_MESS_KIN1:  0.01  # 质心相对 l3 位置
L_MESS_KIN12: 0.45  # rate*l1
L_MESS_KIN2:  0.065 # l3
```

### 2e. `touch_st_stand` / `touch_swing` / `touch_td` —— 触地检测阈值

足端力/位移阈值（用于站立、摆动、触地三个阶段）：`st_td` 触地阈值、`st_lf` 离地阈值、`check_spd/td/lf` 速度与冗余检查阈值。

---

## 三、文件 2：`param_gait.yaml` —— 步态与控制器增益

约 182 行，按 5 个顶层段组织。

### 3a. `sys_param` —— 系统开关

| 字段 | 含义 | 加载到 |
|------|------|--------|
| `auto_gait_time` | 自动切换步态周期 (s) | `gait_ww.auto_gait_time` |
| `auto_gait_switch` | 是否自动切换 (0/1) | `gait_ww.auto_switch` |
| `auto_zmp_st_check` | ZMP 站立检查开关 | `gait_ww.auto_zmp_st_check` |
| `auto_mess_est` | 自动质量辨识开关 | `gait_ww.auto_mess_est` |

加载代码 [robot_param.cpp:17-20](src/robot_param.cpp#L17-L20)。

### 3b. `vmc_param` —— VMC 步态时序与偏置

| 字段 | 含义 |
|------|------|
| `stance_time` / `swing_time` | 支撑/摆动相时长 (s)，Tinker14 各 0.45 |
| `delay_time` | 步态间延迟 |
| `stand_z` | 站立高度比例（0.8 表示 H 的 80%） |
| `cog_off_x/y` `spd_off_x` | 质心/速度静态偏置 |
| `move_off_x` (数组), `move_off_y/pit/rol` | 运动模式下的姿态偏置 |
| `att_bias_pit/rol[_f]` | IMU 姿态零偏（正/翻倒两种状态） |
| `max_spd_x/y/rotate` | 操控速度上限（米/秒、度/秒） |

`max_spd_*` 加载到全局 [robot_param.cpp:22-24](src/robot_param.cpp#L22-L24)；姿态零偏加载到 `vmc_all.att_measure_bias[]`、`att_measure_bias_flip[]`（[robot_param.cpp:255-260](src/robot_param.cpp#L255-L260)）。

### 3c. `rl_gait` —— 强化学习策略参数

| 字段 | 含义 |
|------|------|
| `net_run_dt` | 神经网络推理周期 (s)，Tinker14=0.02 → 50 Hz |
| `action_scale` | 动作缩放因子，0.25 |
| `def_act0..def_act13` | 14 维默认动作（关节角偏置基准，rad） |
| `en_vel_off` | 速度偏置使能 |
| `vel_x_off` / `vel_y_off` | 速度命令零偏（用于现场标定）|

加载到 `vmc_all.net_run_dt`、`action_scale`、`default_action[14]`、`rl_commond_off[]`（[robot_param.cpp:230-251](src/robot_param.cpp#L230-L251)）。

### 3d. `imp_param` —— 阻抗/伺服增益

**全局**：
- `stiff_init` 上电刚度、`stiff_stand` 站立刚度（[robot_param.cpp:27-28](src/robot_param.cpp#L27-L28)）

**每关节**（14 个 `kp_qXY`、`kd_qXY`、`stiff_qXY`）：分别加载到 `leg_motor_all.kp[14]/.kd[14]/.stiff[14]`（[robot_param.cpp:30-77](src/robot_param.cpp#L30-L77)）。

**末端阻抗（足端在笛卡尔空间）**：
- `imp_x/y/z_kp` `imp_x/y/z_kd` —— 足端三轴 PD 增益
- `fb_x/y/z_kp` `fb_x/y/z_kd` —— 反馈（force feedback）增益
- `imp_x/y/z_fp` —— RC 遥控前馈增益

### 3e. `stand_param` & `selfright_param` —— 站立 & 自起姿态控制器

PID 三轴姿态控制器增益：
- `stand_param`: 站立步态用，分 pit/rol/yaw 三轴 + pos_x/y/z 三轴 PID
- `selfright_param`: 自起 (`self_right.cpp`) 步态用，单独一套增益
- `ground_mu` —— 摩擦系数（用于力分配）

---

## 四、变体差异速查

| 项目 | Tinker14 | Tinker14_Zero | Tinker12 | Taitan14_Zero |
|------|----------|---------------|----------|---------------|
| `robot:` 字符串 | `tinker14-small-01` | `tinker14-small-01` | `tinymal-small-01` | `taitan14-small-01`(类似) |
| `dof_num` | `14` (标量) | `14` | `[5,5,5,5]` (数组，四肢) | `14` |
| `kin_head` | 无 | 无 | `[1,-1,1,-1]` | 无 |
| `init_q03` (左腿膝) | -96° | -64° | 不同 | 65° |
| `max_q03` | 0 | 0 | — | 147 |
| `reset_q*` | 收拢姿态 | 全 0 | — | 不同 |
| 用途 | 正常步行 | 标定/调试 | 四足版 | Taitan 平台 |

---

## 五、与其他模块的交互

```
 [运行环境]                              [code]
 /home/odroid/Tinker/Param/             ┌───────────────┐
   ├─ param_robot.yaml  ──load──►  yaml-cpp ──►  robot_param_read()
   └─ param_gait.yaml   ──load──►              │ src/robot_param.cpp:11
                                                 │
                                                 ▼
                                   ┌─────────────┴─────────────┐
                                   ▼              ▼            ▼
                              leg_motor_all   vmc_all      gait_ww
                              (14×多组数组)   (RL/IMU/      (sys flags)
                                              VMC 偏置)
                                   │              │            │
                                   └──→ vmc_src/、gait_src/、math_src/ 全模块共享访问
```

- **生产者**：`src/robot_param.cpp` 全局对象 `config_robot` / `config_gait` 在程序启动时由 yaml-cpp 解析；`robot_param_read()` 把数据搬到全局结构。
- **消费者**：`vmc_src/` 读 `vmc_all.*`、`gait_src/rl.cpp` 读 `vmc_all.default_action[]/net_run_dt/action_scale`、`vmc_src/hardware_interface.cpp` 读 `leg_motor_all.q_init/q_max/q_min/kp/kd/stiff`、所有步态模块共享 `MAX_SPD_*`。
- **切换机型**：仅需替换部署目录下的两个 yaml，不需重新编译；机型识别由 yaml 内 `robot:` 字符串约定（代码不强校验）。
