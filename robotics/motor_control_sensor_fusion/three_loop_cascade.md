<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# FOC 三环级联结构

## 一、为什么需要三环？

单环控制只能处理一个物理量的误差。电机需要同时兼顾**位置精度、速度平滑、电流（转矩）安全**三个层次，单环无法同时满足。

三环级联的设计思路：**外环的输出是内环的设定值（Setpoint）**，每个环只负责自己层次的物理量，带宽从内到外依次降低（内快外慢），形成天然的层次隔离。

```
位置环（最慢）→ 输出: 速度指令 ω_ref
    速度环（中速）→ 输出: 转矩/电流指令 Iq_ref
        电流环（最快）→ 输出: Vd, Vq → SVPWM
```

![三环级联结构框图](https://docs.simplefoc.com/extras/Images/angle_current_cascade_diagram.png)

---

## 二、电流环（最内环）

### 2.1 控制目标

控制电机绕组中的 **Id**（磁通电流）和 **Iq**（转矩电流），使其跟随上层给定值。

- Id_ref：通常为 0（SPMSM）；弱磁时为负值
- Iq_ref：由速度环 PI 输出决定，决定电磁转矩 $T_e = \frac{3p}{2}\psi_f I_q$

### 2.2 执行频率

**10–50 kHz**，与 PWM 载波频率同步。每个 PWM 周期执行一次：采样 → Clarke → Park → PI → 逆 Park → SVPWM。

### 2.3 PI 控制器

$$
V_d = K_{p,d}(I_{d,\text{ref}} - I_d) + K_{i,d}\int(I_{d,\text{ref}} - I_d)\,dt
$$

$$
V_q = K_{p,q}(I_{q,\text{ref}} - I_q) + K_{i,q}\int(I_{q,\text{ref}} - I_q)\,dt
$$

**交叉耦合解耦**（提高高速精度）：

$$
V_d^* = V_d - \omega_e L_q I_q, \quad V_q^* = V_q + \omega_e (L_d I_d + \psi_f)
$$

其中 $\omega_e$ 为电角速度，$L_d, L_q$ 为 dq 轴电感，$\psi_f$ 为永磁体磁链。

### 2.4 参数整定

电流环带宽通常设置为开关频率的 1/5～1/10（避免采样噪声放大）。由电机电气参数直接决定：

$$
K_p = \alpha_c L, \quad K_i = \alpha_c R
$$

其中 $\alpha_c$ 为期望电流环带宽（rad/s），$L$ 为相电感，$R$ 为相电阻。

---

## 三、速度环（中间环）

### 3.1 控制目标

使实际转速 ω 跟随**位置环**给出的速度设定值 ω_ref，PI 输出 Iq_ref（限幅后送入电流环）。

### 3.2 执行频率

**1–10 kHz**，远低于电流环，通常是电流环的 1/5～1/10。

### 3.3 速度反馈

编码器测速（增量编码器差分法）：

$$
\omega = \frac{\Delta\theta}{\Delta t}
$$

离散化后存在量化噪声，**必须加低通滤波器（LPF）**，否则噪声经微分放大后会引起电机抖动：

$$
\omega_{\text{filtered}}(s) = \frac{\omega_c}{s + \omega_c} \cdot \omega(s)
$$

滤波器截止频率 $\omega_c$ 通常取速度环带宽的 3～5 倍。

### 3.4 PI 控制器

$$
I_{q,\text{ref}} = K_{p,v}(\omega_{\text{ref}} - \omega) + K_{i,v}\int(\omega_{\text{ref}} - \omega)\,dt
$$

**积分抗饱和（Anti-windup）**：当 Iq_ref 被限幅时，停止积分累积（Back-Calculation 或 Clamping 法），防止积分漂移。

为何速度环需要 **PI 而不是 P**？  
仅用 P 环，稳态时若有负载干扰（恒定扰矩），误差不为零但输出不继续增大，产生**稳态速度误差**。I 环对恒定误差持续积分，最终将稳态误差消除到零。

---

## 四、位置环（最外环）

### 4.1 控制目标

使实际机械角度 θ 跟随位置设定值 θ_ref，输出速度指令 ω_ref（限幅后送入速度环）。

### 4.2 执行频率

**100–1000 Hz**，远低于速度环。

### 4.3 控制器选型

位置环通常用纯 **P 控制**即可（内环已消除稳态误差）：

$$
\omega_{\text{ref}} = K_{p,\theta}(\theta_{\text{ref}} - \theta)
$$

加入 D 项可提升动态响应（但对编码器噪声敏感），加入 I 项会引起积分饱和风险（因为位置误差通常可以通过速度环积分消除）。

**前馈补偿**（提升轨迹跟踪精度）：

$$
\omega_{\text{ref}} = K_{p,\theta}(\theta_{\text{ref}} - \theta) + \dot{\theta}_{\text{ref}}
$$

直接叠加参考轨迹的速度前馈，减少跟踪滞后。

---

## 五、三环带宽设计原则

| 环路 | 典型带宽 | 频率 |
|------|---------|------|
| 电流环 | 1–5 kHz | 10–50 kHz 执行 |
| 速度环 | 100–500 Hz | 1–10 kHz 执行 |
| 位置环 | 10–100 Hz | 100–1000 Hz 执行 |

**黄金法则**：相邻环路的带宽比至少为 5:1（内环比外环快 5 倍以上），否则各环相互干扰，系统震荡。

```
带宽比 ≥ 5:1
电流环 >> 速度环 >> 位置环
```

---

## 六、整定顺序

**从内到外**，逐环调参：

1. **电流环**：先断开速度环，给定阶跃 Iq_ref，调 Kp/Ki 使电流响应快且无过冲（Rise time ≈ 0.5–2 ms）
2. **速度环**：电流环闭好后，给定阶跃 ω_ref，调 Kp/Ki 使速度响应稳定（Rise time ≈ 5–20 ms）
3. **位置环**：速度环闭好后，给定阶跃位置，调 Kp 使位置无超调快速到达

---

## 七、开环 → 电流环 → 速度环 → 位置环 代码示例（SimpleFOC 风格）

```cpp
// 电流环：PI 控制 Iq，输出 Vq
float pid_current_q(float Iq_ref, float Iq_meas, float dt) {
    float error = Iq_ref - Iq_meas;
    integral_q += error * dt;
    integral_q = constrain(integral_q, -I_MAX, I_MAX);  // Anti-windup
    return Kp_q * error + Ki_q * integral_q;
}

// 速度环：PI 控制 ω，输出 Iq_ref
float pid_velocity(float omega_ref, float omega_meas, float dt) {
    float error = omega_ref - omega_meas;
    integral_v += error * dt;
    integral_v = constrain(integral_v, -Iq_MAX, Iq_MAX);
    return Kp_v * error + Ki_v * integral_v;
}

// 位置环：P 控制 θ，输出 ω_ref
float p_position(float theta_ref, float theta_meas) {
    return Kp_p * (theta_ref - theta_meas);
}

// 主循环（电流环，10 kHz）
void current_loop() {
    sample_currents();            // 采样 ia, ib
    clarke_park(theta_e);         // → Id, Iq
    Vq = pid_current_q(Iq_ref, Iq, dt);
    Vd = pid_current_d(0, Id, dt);
    inverse_park_svpwm(Vd, Vq, theta_e);  // → PWM
}
```

---

## 参考资料

- [SimpleFOC - Cascade Position Control](https://docs.simplefoc.com/angle_cascade_control)
- [Microchip - FOC 控制架构详解](https://onlinedocs.microchip.com/oxy/GUID-0A4BC4EE-29F5-4736-8125-17139B84E7B5-en-US-2/GUID-5CCF6974-52BB-4407-9796-20C86F178C71.html)
- [Berkeley Humanoid Lite - FOC Operation](https://berkeley-humanoid-lite.gitbook.io/docs/in-depth-contents/field-oriented-control-foc-operation)
- [PMD Corp - FOC Deep Dive](https://www.pmdcorp.com/resources/type/articles/get/field-oriented-control-foc-a-deep-dive-article)
