<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# FOC 弱磁控制（Field Weakening）

## 一、为什么需要弱磁控制？

PMSM 的反电动势（Back-EMF）与转速成正比：

$$
E_{back} = K_e \cdot \omega
$$

当转速升高到某个**基速（Base Speed）$\omega_{base}$** 时，反电动势接近直流母线电压 $V_{dc}$，逆变器没有足够的电压余量来驱动所需电流，速度无法继续提升——这就是**电压限制**：

$$
\sqrt{V_d^2 + V_q^2} \leq \frac{V_{dc}}{\sqrt{3}} \quad \text{（SVPWM线性区上限）}
$$

在此限制下，转速被"锁死"在基速。弱磁控制通过**注入负 Id 电流**，人为削弱气隙磁通，降低反电动势，使电机能在基速以上继续运行（代价是转矩能力下降）。

---

## 二、PMSM 电压方程与电流限制圆

### 2.1 稳态 dq 轴电压方程

$$
V_d = R_s I_d - \omega_e L_q I_q
$$

$$
V_q = R_s I_q + \omega_e (L_d I_d + \psi_f)
$$

其中：
- $R_s$：定子相电阻
- $L_d, L_q$：dq 轴电感（隐极 SPMSM 中 $L_d = L_q = L$）
- $\psi_f$：永磁体磁链（常数，由磁钢强度决定）
- $\omega_e$：电角速度

忽略电阻压降（高速时 $R_s I \ll \omega_e \psi_f$），电压约束简化为：

$$
(\omega_e L_d I_d + \omega_e \psi_f)^2 + (\omega_e L_q I_q)^2 \leq V_{s,max}^2
$$

这是 dq 电流平面上的一个**以 $(-\psi_f/L_d,\ 0)$ 为圆心的椭圆（电压限制椭圆）**，转速越高，椭圆越小。

### 2.2 电流限制圆

电机的额定电流限制：

$$
I_d^2 + I_q^2 \leq I_{s,max}^2
$$

这是 dq 平面上**以原点为圆心的电流限制圆**。

### 2.3 三个工作区域

![MTPA与弱磁工作区域](https://www.mathworks.com/help/mcb/gs/field-weakening-control-mtpa-pmsm-example_01.png)

| 工作区域 | Id | 特点 |
|---------|----|------|
| **恒转矩区**（$\omega < \omega_{base}$） | = 0（SPMSM）或 MTPA | 电压有余量，最大转矩输出 |
| **弱磁区 I**（$\omega_{base} < \omega < \omega_{max1}$） | < 0，逐渐增大 | 电流仍在限制圆上，转矩下降 |
| **弱磁区 II**（$\omega > \omega_{max1}$） | 更负，电流可能低于限制 | 电压和电流双重约束 |

---

## 三、弱磁的物理本质

永磁体产生的磁通 $\psi_f$ 是固定的，不能像励磁电机那样直接调节励磁电流。但可以用**定子 d 轴电流产生的去磁磁场**来**部分抵消**永磁体磁通：

$$
\psi_{total} = \psi_f + L_d I_d
$$

当 $I_d < 0$（注入负 d 轴电流），$\psi_{total} < \psi_f$，等效磁通下降，反电动势降低，腾出电压裕量驱动 $I_q$，使电机能在更高转速运行。

**代价**：定子铜损增加（$I_d$ 不为零），且相同电流模值下 $I_q$ 减小，转矩能力下降。

---

## 四、弱磁控制策略

### 4.1 基于电压限幅的弱磁（最常用）

检测 PI 输出的 $V_q$ 是否超过电压限制，超过则通过额外的弱磁 PI 控制器自动增加负 $I_d$：

```
检测 |Vd² + Vq²| > Vs_max²
        ↓ 是
弱磁PI: ΔId = -Kp_fw*(|Vs| - Vs_max) - Ki_fw*∫...
        ↓
Id_ref = max(Id_mtpa - ΔId, -Id_max)
```

**优点**：实现简单，无需电机参数  
**缺点**：动态响应稍慢（PI 积分延迟）

### 4.2 前馈弱磁（基于公式计算）

由电压方程直接解出所需的 $I_d$：

$$
I_d = \frac{\sqrt{V_{s,max}^2 - (\omega_e L_q I_q)^2} / \omega_e - \psi_f}{L_d}
$$

**优点**：无 PI 调节延迟，动态响应快  
**缺点**：依赖精确的电机参数（$L_d, L_q, \psi_f$），参数不准则过/欠弱磁

### 4.3 MTPA（最大转矩电流比）+ 弱磁联合

在恒转矩区，不强制 $I_d = 0$，而是沿 MTPA 曲线工作（利用磁阻转矩），提高效率；进入弱磁区后，沿最大转矩电压比（MTPV）曲线工作：

$$
\text{MTPA:}\ I_d = \frac{\psi_f - \sqrt{\psi_f^2 + 8(L_d-L_q)^2 I_s^2}}{4(L_d-L_q)}
$$

（对于 SPMSM $L_d = L_q$，MTPA 即 $I_d = 0$）

---

## 五、弱磁区的转矩-速度特性

```
转矩
 ↑
T_rated ──────────────┐
                       │ 弱磁区（T·ω ≈ 常数）
                       │\
                       │  \
                       │    \
─────────────────────────────→ 转速
        ω_base    ω_max1  ω_max2
```

弱磁区近似满足**恒功率**：$P = T \cdot \omega \approx \text{const}$

---

## 六、代码实现

```c
// 弱磁控制器（基于电压限幅反馈）
typedef struct {
    float Kp, Ki;
    float integral;
    float Vs_max;       // 电压限幅（= Vdc/√3）
    float Id_min;       // 最大负d轴电流
} FieldWeakeningCtrl;

float field_weakening_update(FieldWeakeningCtrl *fw, float Vd, float Vq, float dt) {
    float Vs = sqrtf(Vd * Vd + Vq * Vq);
    float error = fw->Vs_max - Vs;   // 误差：目标电压幅值 - 当前电压幅值

    // 只在电压饱和时激活（error < 0 意味着超出限制）
    if (error < 0) {
        fw->integral += error * dt;
        fw->integral = fmaxf(fw->integral, fw->Id_min);  // 限幅
    } else {
        fw->integral = fminf(fw->integral + error * dt, 0.0f);  // 允许缓慢恢复到0
    }

    float Id_fw = fw->Kp * error + fw->Ki * fw->integral;
    return fmaxf(Id_fw, fw->Id_min);  // 返回负d轴电流补偿量
}

// 在主 FOC 循环中
void foc_loop(void) {
    // ...正常FOC计算...
    float Id_ref_normal = 0.0f;  // 或 MTPA 计算值

    // 弱磁补偿
    float Id_fw_comp = field_weakening_update(&fw_ctrl, Vd_out, Vq_out, dt);
    float Id_ref = Id_ref_normal + Id_fw_comp;  // Id_fw_comp <= 0

    // 限制
    Id_ref = fmaxf(Id_ref, -Id_max);
    Id_ref = fminf(Id_ref, 0.0f);

    // ...继续电流环...
}
```

---

## 七、弱磁控制的关键参数

| 参数 | 典型值 | 说明 |
|------|--------|------|
| $V_{s,max}$ | $V_{dc}/\sqrt{3} \times 0.95$ | 留 5% 余量给解耦前馈 |
| $I_{d,min}$ | $-I_{rated}$ 到 $-0.5 I_{rated}$ | 过深弱磁会消磁风险 |
| 弱磁 PI Kp | 0.1–1.0 | 较小，避免振荡 |
| 弱磁 PI Ki | 10–100 | 保证稳态精度 |

---

## 八、注意事项

**消磁风险（Demagnetization）**  
过大的负 $I_d$ 会使永磁体部分消磁，导致 $\psi_f$ 永久下降。需设置 $|I_d| \leq I_{d,max}$（由电机厂家给出），并在高温时进一步降低限制（高温时磁钢矫顽力下降）。

**弱磁与制动**  
高速弱磁运行时若突然停止 PWM（如刹车），会产生高压浪涌（反电动势 > $V_{dc}$），可能击穿上管。需要软停策略：先降速到基速以下，再关断 PWM。

**与无感 FOC 的配合**  
无感 FOC 在高速区使用反电动势估算转速/角度，弱磁控制减小了反电动势幅值，可能影响估算精度，需要在弱磁区适当调整观测器增益。

---

## 参考资料

- [MATLAB - Field Weakening Control with MTPA](https://www.mathworks.com/help/mcb/gs/field-weakening-control-mtpa-pmsm.html)
- [Microchip AN2520 - 无感 FOC + 弱磁](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ApplicationNotes/ApplicationNotes/AN2520-Sensorless-Field-Oriented-Control-for-a-Permanent-Magnet-Synchronous-Motor-Using-a-PLL-Estimator-and-Equation-based-Flux-Weakening-DS00002520.pdf)
- [Infineon - 弱磁实现应用笔记](https://www.infineon.com/dgdl/Infineon-AN205401_FIELD_WEAKENED_IMPLEMENTATION_PMSM_DRIVE_FM3_MICROCONTROLLER-ApplicationNotes-v03_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0d55ffa86e69)
- [MATLAB - Field Weakening Control 概述](https://www.mathworks.com/discovery/field-weakening-control.html)
