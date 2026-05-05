<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# FOC 电流采样

## 一、为什么电流采样是 FOC 的核心？

FOC 能比六步换向法更精确控制转矩，根本原因在于**实时掌握定子电流矢量的幅值和相位**。如开篇注中所述：

> 电机绕组是电感，电流滞后电压。要使磁场垂直于转子（最大转矩），电压必须有提前量。这个提前量只能通过电流采样获取。

Clarke → Park 变换的输入正是实时采样的相电流，因此电流采样的精度和时序直接决定 FOC 的性能。

---

## 二、三种采样拓扑对比

### 2.1 三电阻采样（Three-Shunt）

在三相逆变器的每个下桥臂串联一个采样电阻：

```
DC+ ─┬─[上管A]─┬─ A相 ─┐
     │         │       │ 电机
    [上管B]─┬─ B相 ─┤
     │         │       │
    [上管C]─┬─ C相 ─┘
     │    [Rs_A][Rs_B][Rs_C]
DC- ─┴────────────────────
```

**优点：**
- 每一时刻都能独立采样三相电流
- 可在任意 PWM 时刻采样，与调制深度无关
- 支持过调制（Overmodulation）和全总线电压利用
- OPAMP 压摆率/建立时间要求最低（采样窗口宽）

**缺点：**
- 三路 ADC + 三路运放，成本最高
- 三个电阻阻值需精确匹配，否则引入零漂

**适用场景：** 高性能伺服、要求过调制的应用（关节电机、工业伺服）

---

### 2.2 双电阻采样（Two-Shunt）

采样两相（通常 A、B 或 B、C），利用基尔霍夫定律重建第三相：

$$
i_c = -(i_a + i_b)
$$

```
DC+ ─┬─[上管A]─A─┐
     ├─[上管B]─B─┤ 电机
     └─[上管C]─C─┘
        [Rs_A][Rs_B]
DC- ───────────────
```

**优点：**
- 两路 ADC + 两路运放，成本适中
- 比单电阻方案精度高
- 任意时刻可采样（只要对应相下管导通）

**缺点：**
- 特定扇区/高调制深度下，某相占空比极窄，采样窗口不足
- 无法支持过调制（单相占空比为 0 时无法采样）
- 共模信号对两路运放的一致性要求较高

**适用场景：** 中端电机驱动器（大多数 FOC 控制板）

---

### 2.3 单电阻采样（Single-Shunt / DC Link Shunt）

仅在直流母线负极串一个采样电阻，通过测量直流母线电流重建相电流：

```
DC+ ─────────────────┐
       [上管A/B/C]   │ 三相桥
DC- ──[Rs]──[下管ABC]─┘
```

**工作原理：** 不同 PWM 区间内，母线电流 = 某相的相电流（取决于哪个上管开通），通过在特定时刻采样两次，重建两相电流：

| PWM 区间 | 母线电流等于 |
|----------|------------|
| $V_1$ 有效期 | $i_a$ |
| $V_2$ 有效期 | $-i_c$ |

**优点：**
- 单路 ADC + 单路运放，成本最低
- 只有一个采样电阻，无匹配问题

**缺点：**
- 必须在特定时间窗口内采样，对 ADC 触发时序要求极严格
- 调制比低或高（极宽/极窄占空比）时采样窗口 < 1 μs，无法准确采样
- 不支持过调制
- 高速电机中母线电流波形解码困难

**最窄可采样时间估算：**

$$
t_{min} = t_{OPAMP,settle} + t_{ADC,sample} \approx 1\text{–}3\ \mu s
$$

当某相 $T_{on} < t_{min}$ 时，需要进行**电流重建补偿（Current Reconstruction）**，算法复杂。

**适用场景：** 低成本消费品（电风扇、家电、低端无人机电调）

---

## 三、采样时序：在 PWM 周期的哪个时刻采样？

### 3.1 中心采样（Center-aligned Sampling）

在 PWM 载波的**波峰或波谷**（中心时刻）触发 ADC，此时三相上管同时关断（零矢量 $V_0$），母线电流为零，但**各相电流处于当前值的中点**——是一个周期内的平均值，纹波最小。

这是三电阻采样的标准做法。

### 3.2 双次采样（Dual Sampling for Single-Shunt）

在 PWM 半周期内设置两个 ADC 触发点，分别对应两个有效矢量的中心：

```
PWM载波  /\  /\  /\
触发点1 ↑     触发点2 ↑
采样A相     采样B相
```

两次采样间隔 = 相邻有效矢量的中心时刻差，由软件精确计算。

---

## 四、电流采样的噪声来源与滤波

### 4.1 主要噪声源

| 噪声来源 | 频率 | 影响 |
|---------|------|------|
| PWM 开关噪声 | $f_{PWM}$ 及谐波 | ADC 误采样，波形毛刺 |
| 寄生电感振铃 | 10–100 MHz | 采样窗口内振荡 |
| 接地反弹（GND Bounce） | 开关瞬间 | 零点漂移 |
| 热噪声 | 宽频 | 量化误差 |

### 4.2 硬件滤波

- **RC 低通滤波**：在运放输出与 ADC 输入之间加 RC（通常截止频率 200 kHz–2 MHz），滤除高频振铃
- **去耦电容**：采样电阻旁就近放置 100 nF 去耦电容

### 4.3 软件滤波

```c
// 一阶低通滤波（IIR）
float current_lpf(float raw, float prev, float alpha) {
    return alpha * raw + (1.0f - alpha) * prev;
    // alpha = Ts / (Ts + 1/(2π·fc))，fc 为截止频率
}
```

电流环带宽通常为 1–5 kHz，LPF 截止频率取 5–20 kHz（高于电流环带宽 5 倍以上，避免相位滞后影响稳定性）。

---

## 五、零电流偏置（DC Offset）校正

采样电路的运放、ADC 基准都存在零偏，需在上电后（电机不通电时）采集一次，记录偏置值 $I_{offset}$，后续减去：

```c
void calibrate_offset(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 1024; i++) {
        sum += adc_read();
        delay_us(100);
    }
    I_offset = (float)sum / 1024.0f;
}

float get_current(void) {
    return ((float)adc_read() - I_offset) * ADC_SCALE;
}
```

---

## 六、三种方案工程选型表

| 指标 | 单电阻 | 双电阻 | 三电阻 |
|------|--------|--------|--------|
| 成本 | ★☆☆ | ★★☆ | ★★★ |
| 精度 | ★☆☆ | ★★☆ | ★★★ |
| 过调制支持 | ✗ | ✗（低速可） | ✓ |
| 时序复杂度 | 高 | 中 | 低 |
| 适合场景 | 低成本消费品 | 通用 FOC | 高性能伺服 |
| 典型应用 | 电扇、家电 | 工业变频器 | 机器人关节、EV |

---

## 七、与 FOCnote 的关联

FOCnote 中提到：
> "电机发热导致电阻增大而导致电流减小，不进行闭环检测出力矩电流的具体最大值就会导致实际看起来力矩发生了衰减"

这正是**电流闭环**（电流环 = 最内环）的物理动机。没有电流采样，速度环输出的是 $U_{q,max}$ 电压设定，但温度变化、电阻变化都会导致实际电流（转矩）漂移；有了电流采样，内环直接控制 $I_q$，温漂自动被 PI 补偿。

---

## 参考资料

- [TI SPRACT7 - 单电阻 FOC 电流重建](https://www.ti.com/lit/an/spract7/spract7.pdf)
- [Microchip - 单电阻电流重建算法](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ApplicationNotes/ApplicationNotes/01299A.pdf)
- [TI E2E - 1/2/3 Shunt FOC 对比讨论](https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/1041426/foc-field-oriented-control-with-1-2-or-3-shunt-configurations)
- [NXP AN5327 - 单电阻无感 FOC 方案](https://www.nxp.com/docs/en/application-note/AN5327.pdf)
