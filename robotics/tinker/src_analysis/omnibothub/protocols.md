# 通信协议规格

Tinker 机器人系统内外部所有通信协议的详细规格。

---

## 协议层次总览

```
┌─────────────────────────────────────────┐
│ 外部接口 (UDP)                          │
│  OCU(8889) / SDK(8888) / ARM(9000)     │
│  ESP32(7070) / MAP(6666) / RL(10000)   │
├─────────────────────────────────────────┤
│ 进程间通信 (共享内存)                     │
│  MEM_SPI=0x0001 / MEM_CONTROL=4999     │
├─────────────────────────────────────────┤
│ 底层总线                                 │
│  CAN (2Mbps) / SPI (物理) / nRF24L01   │
└─────────────────────────────────────────┘
```

---

## 1. SPI 共享内存 (Odroid C4 ↔ STM32)

**键值**: `MEM_SPI = 0x0001`
**大小**: 2048 字节
**协议**: 全双工, 前半读缓冲区, 后半写缓冲区

### 仲裁

```c
struct shareMemory {
    int flag;     // 0 = control_task 可写, 1 = STM32 已写 (可读)
    unsigned char szMsg[2048];
};
```

- `flag == 1`: STM32 写入完毕 → control_task 读取 `szMsg[0..1023]`
- `flag == 0`: control_task 写入 `szMsg[1024..2047]` → STM32 读取后设 flag=1

### 序列化

所有 float 以大端序编码:
```c
// 写入
static void setDataFloat_mem(float f, int *cnt) {
    int i = *(int *)&f;
    buf[(*cnt)++] = (i >> 24) & 0xFF;  // byte[3]
    buf[(*cnt)++] = (i >> 16) & 0xFF;  // byte[2]
    buf[(*cnt)++] = (i >> 8) & 0xFF;   // byte[1]
    buf[(*cnt)++] = i & 0xFF;          // byte[0]
}

// 读取 (小端序机器)
static float floatFromData_spi(unsigned char *data, int *cnt) {
    int i = (data[*cnt+3] << 24) | (data[*cnt+2] << 16) |
            (data[*cnt+1] << 8)  | data[*cnt];
    *cnt += 4;
    return *(float *)&i;
}
```

### 读取帧格式 (STM32 → control_task, 偏移 0)

| 偏移(byte) | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | att[3] | float×3 | 姿态 (pitch/roll/yaw deg) |
| 12 | att_rate[3] | float×3 | 角速度 (deg/s) |
| 24 | acc_b[3] | float×3 | 机体加速度 (m/s²) |
| 36 | acc_n[3] | float×3 | 世界加速度 (m/s²) |
| 48 | imu_connect | char | IMU 连接标志 |
| 49 | att_usb[3] | float×3 | USB IMU 姿态 (备用) |
| 61 | att_rate_usb[3] | float×3 | USB IMU 角速度 |
| 73 | acc_b_usb[3] | float×3 | USB IMU 加速度 |
| 85 | q[14] | float×14 | 14 关节位置 (deg) |
| 141 | dq[14] | float×14 | 14 关节速度 (deg/s) |
| 197 | tau[14] | float×14 | 14 关节力矩 (Nm) |
| 253 | connect[14] | byte×14 | bit: node/motor/ready |
| 267 | q_servo[14] | float×14 | 14 舵机位置 |
| 323 | tau_servo[14] | float×14 | 14 舵机力矩 |
| 379 | bat_v | float | 电池电压 (V) |
| 383 | ocu 遥控数据 | mixed | 连接/按钮/摇杆 (20B) |

### 写入帧格式 (control_task → STM32, 偏移 1024)

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 1024 | en_motor, en_servo | byte×2 | 使能标志 |
| 1026 | reset_q, reset_err | byte×2 | 复位标志 |
| 1028 | Acc/Gyro/Mag_CALIBRATE | byte×3 | IMU 标定 |
| 1031 | beep_state | byte | 蜂鸣器状态 |
| 1032 | q_set[14] | float×14 | 目标位置 (deg) |
| 1088 | q_reset[14] | float×14 | 复位位置 |
| 1144 | tau_ff[14] | float×14 | 前馈力矩 (Nm) |
| 1200 | kp[14] | float×14 | 位置刚度 |
| 1256 | kd[14] | float×14 | 速度阻尼 |
| 1312 | stiff[14] | float×14 | 刚度乘子 |
| 1368 | reset_q_div[14] | byte×14 | 单独复位标志 |
| 1382 | q_set_servo[14] | float×14 | 舵机目标 (deg) |
| 1438 | tau_ff_servo[14] | float×14 | 舵机前馈力矩 |
| 1494 | stiff_servo[14] | float×14 | 舵机刚度 |
| 1550 | reset_q_div_servo[14] | byte×14 | 舵机单独复位 |

---

## 2. 控制共享内存 (control_task ↔ navigation_task)

**键值**: `MEM_CONTROL = 4999`
**大小**: 2048 字节
**仲裁**: `flag == 0` = control_task 可写, `flag == 1` = navigation_task 可读

### navigation → control

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0 | connect | char |
| 1 | ocu_mode | char |
| 2 | rc_spd_b[3] | float×3 |
| 14 | rc_rate_b[3] | float×3 |
| 26 | 10 按钮 (+1 偏移编码) | char×10 |
| 36 | request_gait (100/101/102/99) | char |
| 37 | exp_spd_o[3] (SDK 速度) | float×3 |
| 49 | exp_att_o[3] | float×3 |
| 61 | exp_datt_o[3] | float×3 |
| 73 | exp_pos_o[3] | float×3 |
| 85 | exp_q_o[4][3] | float×12 |
| 133 | exp_GRF_o[4][3] | float×12 |
| 181 | imu_pos[3] | float×3 |
| 193 | imu_att[3] | float×3 |
| 205 | gps_pos[3] | float×3 |
| 217 | Acc/Gyro/Mag_CALIBRATE | char×3 |

### control → navigation

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0 | connect_motor[14] | byte×14 |
| 14 | q_set[14] | float×14 |
| 70 | set_t[14] | float×14 |
| 126 | q_now[14] | float×14 |
| 182 | t_now[14] | float×14 |
| 238 | com_n_tar[3] / com_n_now[3] | float×6 |
| 262 | acc_b[3] | float×3 |
| 274 | dcom_n_tar[3] / dcom_n_now[3] | float×6 |
| 298 | ground_att_now[3] | float×3 |
| 310 | att_now[3] | float×3 |
| 322 | datt_now[3] | float×3 |
| 334 | att_tar[3] | float×3 |
| 346 | datt_tar[3] | float×3 |
| 358 | gait_state | byte |
| 359 | rc_mode | byte |
| 360 | temp_record[10] | float×10 |

---

## 3. UDP 协议

### RL 推理 (Port 10000, control_task ↔ Python/C++ 策略)

**观测请求** (control_task → 策略, 148 bytes):

```cpp
struct _msg_request_tinker {
    float trigger;         // 1=新策略步, 0=重复帧
    float command[4];      // vx, vy, vyaw (rad/s), reserved
    float eu_ang[3];       // roll, pitch, yaw (rad)
    float omega[3];        // 角速度 (rad/s)
    float acc[3];          // (未使用 =0)
    float q[10];           // 关节位置 (rad)
    float dq[10];          // 关节速度 (rad/s)
    float tau[10];         // (未使用 =0)
    float q_init[10];      // 默认关节角 (rad)
};
```

**动作响应** (策略 → control_task, 56 bytes):

```cpp
struct _msg_response_tinker {
    float q_exp[10];       // 期望关节偏移
    float dq_exp[10];      // (未使用)
    float tau_exp[10];     // (未使用)
};
```

动作处理:
```c
target_q = CLAMP(q_exp, -5, 5) * action_scale + default_action
motor_q_set = CLAMP(target_q * 57.3, q_min, q_max)
```

### OCU/ESP32 (Port 7070, control_task ↔ ESP32)

**ESP32 → control_task** (OCU 结构体):

```cpp
struct _msg_ocu_tinker {
    char Head1, Head2;     // 0xFA, 0xFB 帧头
    char gait_mode;        // 99=poweroff, 1=stand, 10=RL walk, 11=RL stand
    char auto_mode;        // 0=manual, 1=auto
    char head_mode;        // 头部控制模式
    char step_mode;        // 步态选择
    char cal_mode;         // 10=leg all, 11=leg sel, 20=arm all, 21=arm sel, 30=imu
    char cal_sel;          // 校准选择索引
    float att_cmd[3];      // 目标姿态
    float vel_cmd[3];      // 目标速度
    float pos_cmd[3];      // 目标位置
    int power_cnt;         // 心跳
};
```

**control_task → ESP32** (反馈):

```cpp
struct _msg_fb_tinker {
    char gait_mode;        // 0=idle, 1=stand, 2=RL
    char auto_mode;        // SDK 模式
    char head_mode;
    char step_mode;
    char heart_cnt;        // 心跳计数器
    float att[3];          // 当前姿态
    float vel[3];          // 目标速度
    float acc[3];          // 加速度
    float mag[3];          // 磁力计
    float gyro[3];         // 陀螺仪
    float baro;            // 气压
    float pos[3];          // 位置
    float q_leg[14];       // 腿关节角
    float q_arm[14];       // 手臂关节角
};
```

### 调试面板 (Port 8001, py_ui.py/py_xbox.py)

```python
cmd = struct.pack('<iiiiffff',
    99,             # head 魔数
    32,             # size
    1,              # id
    key,            # 1=Enable, 2=RC, 3=Idle, 4=Walk, 13=Disable, 15=Start, 17=Stop, 20=Damp, 21=Release
    joy_x, joy_y,   # 摇杆 X/Y (-1 ~ +1)
    joy_z, joy_w    # 摇杆 Z/W (yaw)
)
```

### OCU (Port 8889) + SDK (Port 8888)

自定义二进制协议，由 navigation_task 编解码 (见 [navigation_task.md](navigation_task.md))。

---

## 4. CAN 总线 (MIT Mini Cheetah 协议)

**物理层**: STM32F4 → CAN1 + CAN2, 2Mbps

### 帧格式

| CAN ID 范围 | 用途 |
|---|---|
| 0-19 | 系统参数 |
| 20-39 | 力矩指令 |
| 40-59 | 状态反馈 |
| 60-79 | 位置反馈 |
| 80-99 | 速度反馈 |
| 100-119 | MIT 模式反馈 |
| 140-159 | 力矩/电流下发 |
| 160-179 | 位置下发 |
| 200-219 | MIT 模式下发 |

### MIT 模式编码

| 字段 | 范围 | 位宽 | 缩放 |
|---|---|---|---|
| 位置 (P) | uint16 | 16bit | P_MIN=-180°, P_MAX=+180° |
| 速度 (V) | uint16 | 16bit | V_MIN=-2000, V_MAX=+2000 |
| KP | uint16 | 16bit | KP_MIN=0.0, KP_MAX=10.0 |
| KD | uint16 | 16bit | KD_MIN=0.0, KD_MAX=10.0 |
| 力矩 (T) | uint16 | 16bit | T_MIN=-10.0 Nm, T_MAX=+10.0 Nm |

### 电机连接检测

从 SPI 读取的每个电机状态字节:
- `connect = temp / 100 % 10` — 节点连接
- `connect_motor = (temp - connect*100) / 10` — 电机连接
- `ready = temp % 10` — 预备就绪

---

## 5. nRF24L01 无线协议 (遥控器 ↔ 机器人)

**物理**: 2.4GHz, 2Mbps, 通道 44, GFSK 调制
**帧长度**: 32 字节

### 发送包 (RC 命令)

| 偏移 | 字段 | 位宽 |
|---|---|---|
| 0-3 | THROTTLE | 2B (1000-2000us) |
| 4-7 | YAW | 2B |
| 8-11 | ROLL | 2B |
| 12-15 | PITCH | 2B |
| 16-25 | AUX[5] | 2B×5 |
| 26 | button_state | 1B |
| 27 | calib_flags | 1B |
| 28 | read_pid | 1B |
| 29 | acc_3d_step | 1B |
| 30-31 | checksum | 2B |

### 接收包 (8 种类型)

| 包 ID | 内容 |
|---|---|
| 0x01 | 姿态/速度/位置/GPS/状态/电池 |
| 0x02 | 调试值 (9 int16) + RC 通道 (5 int16) |
| 0x03-06 | PID 参数 (6 组) |
| 0x07 | 任务数据 |
| 0x08 | GPS 经纬度 (double) |

---

## 6. ANO 遥测协议 (USB VCP)

**物理**: USART1 @ 256kbps → USB Virtual COM Port

**帧格式**:
```
PC → STM32:  0xAA 0xAF <func_id(1B)> <len(1B)> <data(NB)> <checksum(1B)>
STM32 → PC:  0xAA 0xAA <func_id(1B)> <len(1B)> <data(NB)> <checksum(1B)>
```

**下行指令** (PC → STM32):
| func_id | 指令 |
|---|---|
| 0x01 | IMU 标定 (acc+gyro+mag) |
| 0x02 | PID 读请求 |
| 0x10-0x15 | PID 写入 (6 组) |
| 0x21-0x26 | 6 面加速度 3D 标定 |

**上行遥测**: 轮询发送多种数据类型 (状态/传感器/RC/电机/电源/速度/位置/PID)
