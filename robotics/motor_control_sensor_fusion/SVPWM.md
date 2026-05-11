<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# SVPWM（空间矢量脉宽调制）

## 一、为什么需要 SVPWM？

逆 Park 变换输出的是 $V_\alpha, V_\beta$（两相静止坐标系中的电压矢量），但三相逆变器只能输出离散的开关状态，无法直接输出连续电压。**SVPWM 的任务**：用有限个开关状态的加权平均，在一个 PWM 周期内等效合成任意目标电压矢量。

相比正弦 PWM（SPWM），SVPWM 的直流母线利用率提升约 **15.5%**（$1/\sqrt{3}$ vs $1/2$），谐波性能更好，且天然适合数字实现。

---

## 二、三相逆变器的 8 个基本矢量

三相全桥逆变器（6 个 IGBT/MOSFET）有 $2^3 = 8$ 种开关组合：

| 矢量 | 上管状态 (A,B,C) | 电压 $U_{ab}, U_{bc}, U_{ca}$ |
|------|----------------|-------------------------------|
| $V_0$ | (0,0,0) 零矢量 | 0, 0, 0 |
| $V_1$ | (1,0,0) | $V_{dc}$, 0, $-V_{dc}$ |
| $V_2$ | (1,1,0) | 0, $V_{dc}$, $-V_{dc}$ |
| $V_3$ | (0,1,0) | $-V_{dc}$, $V_{dc}$, 0 |
| $V_4$ | (0,1,1) | $-V_{dc}$, 0, $V_{dc}$ |
| $V_5$ | (0,0,1) | 0, $-V_{dc}$, $V_{dc}$ |
| $V_6$ | (1,0,1) | $V_{dc}$, $-V_{dc}$, 0 |
| $V_7$ | (1,1,1) 零矢量 | 0, 0, 0 |

6 个非零矢量幅值相等（$= \frac{2}{3}V_{dc}$），相位间隔 60°，在复平面上构成**正六边形**：

![SVPWM 六边形矢量图](https://images.squarespace-cdn.com/content/v1/584729023e00bebf8abd6ba0/1537207349252-DQR4QNU0ZOSWADVRR1VR/3ph_fixed_animate.gif?format=2500w)

---

## 三、扇区划分

将 $V_\alpha, V_\beta$ 的范围划分为 6 个 60° 扇区（扇区 I～VI，逆时针编号）。

**快速扇区判断算法**（利用三个辅助量 $U_1, U_2, U_3$）：

$$
U_1 = V_\beta
$$
$$
U_2 = \frac{\sqrt{3}}{2}V_\alpha - \frac{1}{2}V_\beta
$$
$$
U_3 = -\frac{\sqrt{3}}{2}V_\alpha - \frac{1}{2}V_\beta
$$

| 条件 | $U_1 > 0$ | $U_2 > 0$ | $U_3 > 0$ | 扇区 |
|------|-----------|-----------|-----------|------|
| I | ✓ | ✗ | ✗ | I |
| II | ✓ | ✓ | ✗ | II |
| III | ✗ | ✓ | ✗ | III |
| IV | ✗ | ✓ | ✓ | IV |
| V | ✗ | ✗ | ✓ | V |
| VI | ✓ | ✗ | ✓ | VI |

用编码简化判断：

```c
int sector = (U1 > 0 ? 1 : 0) + (U2 > 0 ? 2 : 0) + (U3 > 0 ? 4 : 0);
// sector 值: 1→I, 3→II, 2→III, 6→IV, 4→V, 5→VI
```

---

## 四、作用时间计算

在扇区 I（$V_1$ 和 $V_2$ 之间），用 $T_1$（施加 $V_1$ 的时间）和 $T_2$（施加 $V_2$ 的时间）以及零矢量时间 $T_0$ 合成目标矢量 $V_{ref}$：

$$
V_{ref} \cdot T_s = V_1 \cdot T_1 + V_2 \cdot T_2 + V_0 \cdot T_0
$$

$$
T_1 = T_s \cdot \frac{\sqrt{3}}{V_{dc}} \left(V_\alpha \sin\frac{\pi}{3} - V_\beta \cos\frac{\pi}{3}\right)
\cdot \frac{\sin 60°}{\sin(\pi/3)}
$$

实际工程中通过变形后的逆 Clarke 矩阵**直接从 $V_\alpha, V_\beta$ 计算**，无需三角函数：

$$
X = V_\beta \cdot T_s / V_{dc}
$$
$$
Y = \left(\frac{\sqrt{3}}{2}V_\alpha + \frac{1}{2}V_\beta\right) \cdot T_s / V_{dc}
$$
$$
Z = \left(-\frac{\sqrt{3}}{2}V_\alpha + \frac{1}{2}V_\beta\right) \cdot T_s / V_{dc}
$$

各扇区的 $T_1, T_2$ 查表：

| 扇区 | $T_1$ | $T_2$ |
|------|-------|-------|
| I | $-Z$ | $X$ |
| II | $Z$ | $Y$ |
| III | $X$ | $-Y$ |
| IV | $Z$ | $-X$ |
| V | $-Z$ | $-Y$ |
| VI | $-X$ | $Y$ |

零矢量时间：$T_0 = T_s - T_1 - T_2$（需保证 $T_0 \geq 0$，即不过调制）

---

## 五、对称七段式 PWM

零矢量分配在每半个 PWM 周期的首尾，形成对称三角载波调制——减少电流纹波，降低谐波。

以扇区 I 为例，每个 PWM 周期内的开关序列：

```
T₀/4 | T₁/2 | T₂/2 | T₀/2 | T₂/2 | T₁/2 | T₀/4
 V₀     V₁     V₂     V₇     V₂     V₁     V₀
```

三相 PWM 比较值（占空比）计算：

$$
T_a = \frac{T_1 + T_2 + T_0}{2}, \quad
T_b = \frac{T_2 + T_0}{2} - \frac{T_1}{2}, \quad
T_c = \frac{T_0}{2}
$$

（各扇区需查表映射到具体的 A、B、C 相）

![七段 SVPWM 波形](https://imperix.com/doc/wp-content/uploads/2020/01/SVM-switching-sequence.png)

---

## 六、过调制（Overmodulation）

当 $|V_{ref}| > \frac{V_{dc}}{\sqrt{3}}$（六边形内切圆半径），目标矢量超出六边形范围，进入**过调制区**。

- **线性区**：$|V_{ref}| \leq \frac{V_{dc}}{\sqrt{3}}$，调制比 $m = \frac{|V_{ref}|}{V_{dc}/\sqrt{3}} \leq 1$
- **过调制一区**（$1 < m \leq 1.052$）：部分扇区饱和，需截断处理
- **六步换向（方波）**：$m = 1.1547$，直流母线利用率最高但谐波最大

单电阻/双电阻采样方案在过调制区会失效（见《电流采样》章节）。

---

## 七、代码实现（C 语言）

```c
typedef struct {
    float Va, Vb, Vc;  // 三相占空比 [0,1]
} SvpwmOutput;

SvpwmOutput svpwm(float Valpha, float Vbeta, float Vdc, float Ts) {
    // 1. 计算辅助量
    float X = Vbeta / Vdc;
    float Y = ( 0.8660254f * Valpha + 0.5f * Vbeta) / Vdc;  // √3/2 ≈ 0.866
    float Z = (-0.8660254f * Valpha + 0.5f * Vbeta) / Vdc;

    // 2. 扇区判断
    int n = (Vbeta > 0 ? 1 : 0) + (Y > 0 ? 2 : 0) + (Z > 0 ? 4 : 0);
    int sector_map[] = {0, 1, 3, 2, 6, 4, 5, 0};  // n → sector
    int sector = sector_map[n];

    // 3. 各扇区作用时间
    float T1, T2;
    switch (sector) {
        case 1: T1 = -Z; T2 =  X; break;
        case 2: T1 =  Z; T2 =  Y; break;
        case 3: T1 =  X; T2 = -Y; break;
        case 4: T1 = -X; T2 = -Z; break;  // 注意符号
        case 5: T1 = -Y; T2 = -Z; break;
        case 6: T1 = -X; T2 =  Y; break;
        default: T1 = 0; T2 = 0; break;
    }
    float T0 = 1.0f - T1 - T2;
    if (T0 < 0) { T1 *= (1.0f/(T1+T2)); T2 *= (1.0f/(T1+T2)); T0 = 0; }  // 过调制截断

    // 4. 计算三相比较值（对称七段式）
    float Ta = (T1 + T2 + T0) * 0.5f;
    float Tb = Ta - T1;
    float Tc = Tb - T2;

    // 5. 根据扇区映射 A/B/C 相
    SvpwmOutput out;
    switch (sector) {
        case 1: out.Va = Ta; out.Vb = Tb; out.Vc = Tc; break;
        case 2: out.Va = Tb; out.Vb = Ta; out.Vc = Tc; break;
        case 3: out.Va = Tc; out.Vb = Ta; out.Vc = Tb; break;
        case 4: out.Va = Tc; out.Vb = Tb; out.Vc = Ta; break;
        case 5: out.Va = Tb; out.Vb = Tc; out.Vc = Ta; break;
        case 6: out.Va = Ta; out.Vb = Tc; out.Vc = Tb; break;
        default: out.Va = 0.5f; out.Vb = 0.5f; out.Vc = 0.5f;
    }
    return out;
}
```

---

## 八、SVPWM vs SPWM 对比

| 指标 | SPWM | SVPWM |
|------|------|-------|
| 直流母线利用率 | 50% | 57.7%（提升 15.5%） |
| 实现方式 | 模拟/数字 | 纯数字 |
| 谐波含量 | 较高 | 较低（等效开关频率更高） |
| 过调制支持 | 无 | 有（需额外处理） |
| 与 FOC 配合 | 一般（需三路 SPWM） | 最佳（直接从 Vα,Vβ 计算） |

---

## 参考资料

- [Switchcraft - Space Vector PWM 图解教程](https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro)
- [imperix - SVM 实现细节](https://imperix.com/doc/implementation/space-vector-modulation)
- [Microchip AN - SVPWM 应用笔记](https://onlinedocs.microchip.com/oxy/GUID-AC0E172C-9656-4397-A490-08DF807DE2E8-en-US-2/GUID-E064C18E-FA54-4DD4-9E8B-31A3EDAFD643.html)
- [MATLAB - Space Vector Modulation](https://www.mathworks.com/discovery/space-vector-modulation.html)
