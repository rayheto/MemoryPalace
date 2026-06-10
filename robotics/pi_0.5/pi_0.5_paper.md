# π₀.₅: A Vision-Language-Action Model with Open-World Generalization

> **Source**: https://arxiv.org/html/2504.16054v1
> **Authors**: Physical Intelligence (Kevin Black, Noah Brown, James Darpinian, Karan Dhabalia, Danny Driess, Adnan Esmail, Michael Equi, Chelsea Finn, Niccolo Fusai, Manuel Y. Galliker, Dibya Ghosh, Lachy Groom, Karol Hausman, Brian Ichter, Szymon Jakubczak, Tim Jones, Liyiming Ke, Devin LeBlanc, Sergey Levine, Adrian Li-Bell, Mohith Mothukuri, Suraj Nair, Karl Pertsch, Allen Z. Ren, Lucy Xiaoyang Shi, Laura Smith, Jost Tobias Springenberg, Kyle Stachowicz, James Tanner, Quan Vuong, Homer Walke, Anna Walling, Haohuan Wang, Lili Yu, Ury Zhilinsky)
> **本文为论文结构化摘录（中英对照重组），用于离线研究参考。完整内容请以 arxiv 原文为准。**

---

## Abstract（摘要）

VLA 模型在实验室内表现良好，但开放世界泛化能力仍存疑。π₀.₅ 在 π₀ 基础上引入**异构任务联合训练（co-training）**：多种机器人平台数据 + 高层语义子任务预测 + Web 多模态数据 + 其他来源。训练样本以「图像观测 + 语言指令 + 物体检测 + 子任务语义 + 低层动作」混合多模态形式给出。实验表明该跨源知识迁移对泛化至关重要，π₀.₅ 是首个端到端学习方法可在**全新家庭**完成厨房 / 卧室清洁等长时序、精细操作的系统。

---

## I. Introduction（引言）

核心问题：移动机械臂等具身系统只有走出实验室，才具备实用价值。学习方法可借助 scaling，但单纯 scale 不够——需要在**多个抽象层次**上具备泛化能力：

- 简单技能（抓取）：足够多样数据即可泛化
- 适配性技能：旧技能套用新场景
- 语义类技能（判断"袜子应放进哪个抽屉"）：需要场景语义理解

提出方案：通过 VLA 框架统一不同模态/数据源，并设计**层次化推理 + 联合训练**架构。

### 数据来源（约 400h 真实家庭移动机械臂数据 + 大量其它）
- ~400h 移动机械臂家庭任务数据（多个真实家庭）
- 多环境非移动机器人数据
- 实验室跨本体数据
- 高层语义任务预测样本
- Web 多模态样本（caption / VQA / 物体定位）

预训练初期 **97.6% 样本来自非移动机械臂数据源**。

### 推理流程（层次化）
1. **Pre-training**：在异构任务混合上训练
2. **Post-training**：针对移动操作，联合学习低层动作 + 高层语义子任务预测

推理时：
- 先预测语义子任务（high-level inference）
- 再以子任务为条件生成低层动作 chunk（low-level inference）

不同知识源服务于不同抽象层：低层动作受益于跨机器人动作数据，高层推理受益于 web 语义数据与人类指令。

---

## II. Related Work（相关工作）

### A. 通用机器人操作策略
VLA：在预训练 VLM 上微调，叠加表达力强的动作解码（flow matching / diffusion / action tokenization）。痛点：常在与训练匹配的环境内评估，未真正开放世界泛化。π₀.₅ 通过混合"第一手移动操作经验 + 其他来源"实现新环境（新厨房/卧室）泛化。

### B. 非机器人数据 co-training
- 早期：用 CV 数据初始化视觉编码器，或用 off-the-shelf planner
- VLA：天然支持图像-文本-动作多模态交错训练
- π₀.₅ 进一步把"其他机器人数据 + 高层子任务预测 + 人类口头指令"也并入

### C. 基于语言的机器人推理与规划
高层 reasoning 提升长程任务表现。π₀.₅ 采用**统一模型**做高/低层推理（CoT 风格），高层频率低于低层。

### D. 开放世界泛化的机器人学习系统
窄任务（抓取）可借特定方法泛化；通用 VLA 在新家中长程任务仍是开放问题。π₀.₅ 显著超越前作的难度与成功率。

---

## III. Preliminaries（预备知识）

VLA 通过模仿学习最大化条件动作对数似然：

$$\max_{\theta} \mathbb{E}_{(\mathbf{a}_{t:t+H}, \mathbf{o}_t, \ell) \sim \mathcal{D}} \log \pi_{\theta}(\mathbf{a}_{t:t+H}\mid \mathbf{o}_t, \ell)$$

- 观测 $\mathbf{o}_t$：图像 + 本体感知
- 大型自回归 transformer 主干，权重由 VLM 初始化
- 动作可用离散 tokenization（如 FAST），也可用 diffusion / flow matching 连续表示
- π₀.₅ 沿用 π₀ 的 **flow matching + action expert（类 MoE 的独立小权重）** 处理动作 token

---

## IV. The π₀.₅ Model and Training Recipe（模型与训练配方）

### IV-A. 架构

输出分布：

$$\pi_{\theta}(\mathbf{a}_{t:t+H}, \hat{\ell}\mid \mathbf{o}_t, \ell)$$

- $\mathbf{o}_t = [\mathbf{I}^1_t, \ldots, \mathbf{I}^n_t, \mathbf{q}_t]$：相机图像 + 机器人构型
- $\ell$：高层任务 prompt（如"put away dishes"）
- $\hat{\ell}$：模型输出的文本（子任务或 web 数据回答）
- $\mathbf{a}_{t:t+H}$：动作 chunk

分解：

$$\pi_{\theta}(\mathbf{a}_{t:t+H}, \hat{\ell}\mid \mathbf{o}_t, \ell) = \pi_{\theta}(\mathbf{a}_{t:t+H}\mid \mathbf{o}_t, \hat{\ell})\,\pi_{\theta}(\hat{\ell}\mid \mathbf{o}_t, \ell)$$

Transformer 处理 N 个多模态 token：

$$y_{1:N} = f\!\bigl(x_{1:N},\, A(x_{1:N}),\, \rho(x_{1:N})\bigr)$$

每个 $x_i$ 可以是：
- 文本 token $x_i^w \in \mathbb{N}$
- 图像 patch $x_i^I \in \mathbb{R}^{p\times p\times 3}$
- 中间 flow-matching 去噪动作值 $x_i^a \in \mathbb{R}^d$

- $\rho(x_i)$ 指示处理路径：图像走视觉编码器；文本走 embedding；动作 token 经独立 action expert 权重线性投影
- $A(x_{1:N})$：注意力可见性矩阵
- **非标准因果掩码**：图像 patch、文本 prompt、连续动作 token 之间使用**双向 attention**

输出拆分为 $(y_{1:M}^{\ell}, y_{1:H}^a)$：前 M 为文本 logits（采样 $\hat{\ell}$），后 H 为 action expert 输出（线性投影得到 $\mathbf{a}_{t:t+H}$）。本体感知作为文本 token 离散化输入。

### IV-B. 离散 + 连续动作联合表示

设 $\mathbf{a}^{\tau,\omega}_{t:t+H} = \tau \mathbf{a}_{t:t+H} + (1-\tau)\omega$，$\omega \sim \mathcal{N}(0,\mathbf{I})$，$\tau \in [0,1]$。模型预测 flow 场 $\omega - \mathbf{a}_t$。

- 离散 FAST token：训练快、可与 VLM next-token 任务统一；推理慢（自回归）
- 连续 flow matching：推理快但训练慢

**π₀.₅ 同时使用两者**，且 attention mask 保证两种动作表示**不相互看到**。联合损失：

$$\mathbb{E}_{\mathcal{D},\tau,\omega}\Bigl[H(x_{1:M}, f^{\ell}_{\theta}(\mathbf{o}_t,\ell)) + \alpha \bigl\|\omega - \mathbf{a}_{t:t+H} - f^a_{\theta}(\mathbf{a}^{\tau,\omega}_{t:t+H},\mathbf{o}_t,\ell)\bigr\|^2\Bigr]$$

- $H(\cdot,\cdot)$：文本 / FAST token 交叉熵
- $\alpha$：trade-off 超参（post-training 时 $\alpha=10.0$）
- **预训练阶段** $\alpha = 0$：等价标准 VLM 训练
- **后训练阶段** 加入 action expert（随机初始化），用 flow matching 非自回归预测连续动作

推理：自回归解文本 → 10 步 flow-matching 去噪 → 得到 $\mathbf{a}_{t:t+H}$。

### IV-C. Pre-training 数据组成

| 缩写 | 含义 | 描述 |
|------|------|------|
| **MM** | Mobile Manipulator | ~400h 移动机械臂家庭数据，约 100 个不同家庭 |
| **ME** | Multi-Environment static | 多家庭非移动机器人（单/双臂），form factor 不同 |
| **CE** | Cross-Embodiment lab | 实验室 wide-range 任务（清桌、叠衣等），含 OXE 与 π₀ 训练集扩展版 |
| **HL** | High-Level subtask | 高层指令拆分子任务（"clean bedroom" → "pick up pillow"），含 bounding box 标注 |
| **WD** | Web Data | image captioning（CapsFusion、COCO）；VQA（Cambrian-7M、PixMo、VQAv2）；物体定位（含额外室内/家居 bbox 数据） |

动作处理细节：
- 同时预测目标 **关节角** 与 **末端位姿**，通过 prompt `<<<control_mode>>> joint/end effector <<<control_mode>>>` 区分
- 用各数据集动作维度 **1% / 99% 分位数** 归一化到 [-1, 1]
- 动作维度固定为最大值，低维机器人 **0-padding**

### IV-D. Post-training

- 接在 280k 步离散 token 预训练之后
- 80k 步后训练，$\alpha = 10.0$
- 联合 next-token 预测（保留文本能力）+ flow matching 动作 expert（随机初始化）
- 数据：成功且不超过长度阈值的 MM + ME 动作子集；保留 WD；用 ME 切片的 HL
- 新增 **VI（Verbal Instruction）**：专家用户为机器人实时下达子任务级口头指令，结合学到的低层策略完成示教，提供"理想 high-level 输出"的训练样本

### IV-E. 机器人系统

- 双 6-DoF 臂 + 平行夹爪
- 全向轮式移动底盘
- 升降躯干（torso lift）
- 4 路单目 RGB 相机（前 / 后 / 双手腕）
- 状态/动作总维度 18–19 DoF
- 底盘动作：2D 线速度 + 1D 角速度
- torso：1D（上下）或 2D（上下 + 前后）
- 高层推理用全部 4 路相机；低层推理用手腕 + 前视
- π₀.₅ 直接以 **50 Hz** 输出臂目标位姿 / 夹爪 / torso + 底盘目标速度（带 action chunking）
- 底层只用简单 PD 跟踪，**无轨迹规划、无碰撞检测**——操作与导航完全 end-to-end

---

## V. Experimental Evaluation（实验）

主评测在**完全未见过的环境**进行：定量比较使用受控 mock home；最终评测使用训练集外的 3 个真实家庭。

研究问题：
1. π₀.₅ 能否在全新家庭完成复杂多阶段任务？
2. 泛化性能如何随训练环境数量 scaling？
3. 联合训练 recipe 各组件分别贡献多少？
4. 与其它 VLA 比表现如何？
5. 高层推理有多重要？

### V-A. 真实家庭泛化

在 3 个真实家庭（训练集外）评估"卧室清洁"与"厨房清洁"。
- 评分按子步骤百分比（如一半碗放进水池 ≈ 50%）
- 单任务 2–5 分钟，高层推理自主决定下一步（"pick up cup" 等）
- 泛化新颖度、任务长度、复杂度均**超过既往 VLA**

### V-B. Scaling: 训练环境数 vs 泛化

环境数序列：**3, 12, 22, 53, 82, 104** 个 location。
- 由于全 recipe 全运行代价过高：实验仅用"机器人动作混合（不含 MM）"先预训练，再以不同 MM 环境数做 post-training
- 后训练统一跑 **40k 步**，保证所有模型见到相同 unique sample 数

两种评测：
- **Type 1**：mock 家中端到端多阶段任务（放碗、装抽屉、收衣物、铺床）
- **Type 2**：细粒度语言指令跟随 + 新物体（in-distribution vs OOD 类别）

结论：
- 平均性能随训练 location 数单调上升
- 直接在测试家庭训练的 control 模型 ≈ 104-location 模型——说明 **co-training 已足以替代"目标家庭训练数据"**
- 缺少 co-training 任务的 baseline 即便能访问测试家庭数据，依然显著落后
- 语言跟随 in-distribution 提升快于 OOD，但 OOD 也稳步上升

### V-C. 数据源消融（co-training 组件）

Full recipe = MM + ME + CE + HL + WD（+ VI for post-training）

| 消融 | 说明 |
|------|------|
| no WD | 去掉 web 数据 |
| no ME | 去掉多环境非移动 |
| no CE | 去掉实验室跨本体 |
| no ME + no CE | 仅保留 MM 与 WD |

主要发现：
- 去 ME 或 CE 都显著下降 → **跨本体迁移很关键**（既迁移环境 ME，也迁移任务 CE）
- 同时去 ME + CE 更差
- no WD 在端到端任务上**统计不显著**，但显著降低 **OOD 物体语言跟随** —— web 数据主要扩展"对新物体类别的理解"
- 暗示 WD 真正贡献在**高层推理**上（V-E 验证）

### V-D. 与其它 VLA 对比

- **π₀**：原始 VLA
- **π₀-FAST+Flow**：用 Eq.(1) 联合 diffusion + FAST 预测，只用动作数据，不用 HL/WD

对比条件：
- 相同跨本体动作训练集
- 训练步数相当
- 差异：π₀.₅ 增加 HL + WD；用"离散预训练 + 后训练加 flow expert"的 hybrid 流程；π₀ 始终带 action expert；π₀-FAST+Flow 跟 hybrid 流程但不含高层推理

结果：π₀.₅ **显著优于** π₀ 与 π₀-FAST+Flow，即便 π₀ 延长到 **300k 步**仍不敌；FAST token 训练比纯 diffusion 更 compute-efficient。

### V-E. 高层推理重要性

策略组合（低层始终用 π₀.₅）：

| 方法 | 描述 |
|------|------|
| π₀.₅ | 高/低层同一模型 |
| no WD | 去 web 数据消融 |
| no VI | 去口头指令消融 |
| implicit HL | 推理时不显式跑高层，但训练含 HL 数据 |
| no HL | 训练与推理都不要 HL |
| GPT-4 | 用 GPT-4 做高层策略，prompt 提供任务描述 + 常用标签 |
| human HL | 专家人工高层 oracle |

结果：
- 完整 π₀.₅ **优于 human HL oracle**
- 第二好是 **implicit HL**（推理时不跑高层但训练有 HL 数据）→ 大部分增益来自"训练中混入子任务预测数据"
- no HL 显著最差之一
- VI 虽然只占高层 MM 样本 **~11%**，但移除会大幅掉点 → 关键监督
- no WD 显著降低，主要因为伤害了高层策略
- GPT-4 零样本最差 → 必须用机器人数据对 VLM 做适配

---

## VI. Discussion & Future Work

### 结论
- co-training 把多源数据有效迁移到单一 VLA，使中等规模（~400h）移动操作数据足以驱动新家庭的长程灵巧操作
- 在新家完成清厨房、清卧室、挂毛巾、铺床等

### 局限
- 仍有失败：陌生抽屉把手、物理上难开的柜门
- 部分可观测性差（机械臂遮挡擦拭区域）
- 高层有时"走神"（在装东西过程中反复开关同一抽屉）

### 未来方向
- 更复杂的 prompt / 偏好 / 长指令（依赖更丰富标注，包括合成）
- 引入记忆与上下文（多房间导航、地点记忆）
- 探索更多异构数据源；VI 已展示"人类即时口头监督"是强力新形式
- 目标：让下一代 VLA 在多种真实环境普遍泛化

---

## 关键超参数与训练规模速查

| 项 | 值 |
|---|---|
| 预训练步数 | 280k |
| 后训练步数 | 80k |
| Scaling 实验后训练步数 | 40k |
| 推理 flow-matching 去噪步数 | 10 |
| 后训练 $\alpha$ | 10.0 |
| 控制频率 | 50 Hz（带 action chunking） |
| 移动机械臂数据量 | ~400h |
| 训练家庭数 | ~100（scaling 序列：3/12/22/53/82/104） |
| 非移动数据占比（pre-training 初期） | 97.6% |
| 动作归一化分位 | 1% / 99% |
| VI 数据占高层 MM 样本比例 | ~11% |
| 总 DoF | 18–19 |
| 相机数 | 4（前 / 后 / 双手腕） |

---

## 备注

- 论文图、表细节、附录 A–E（架构超参数、tokenizer 细节、bbox 标注规范、prompt 模板等）未在本摘录中展开，建议结合 arxiv HTML 版核对
- 本文中 `attachments/` 目录可放原论文 PDF 与图片
