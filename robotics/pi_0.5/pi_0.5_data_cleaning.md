# π₀.₅ 数据清洗 / 数据处理总结

> Source: `2504.16054v1.pdf`（Physical Intelligence, "π₀.₅: a Vision-Language-Action Model with Open-World Generalization"）
> 本文按论文正文 IV-C / IV-D + 附录 B / E 梳理出 π₀.₅ 在数据层做的所有清洗、归一化、过滤、加工与增广步骤。

---

## 概览

π₀.₅ 并没有走"复杂自动清洗 pipeline"的路线，重点是**跨本体归一化 + 人工/半自动高质量标注**。所有处理可归纳为 9 件事，按数据流顺序如下。

| # | 阶段 | 处理 | 作用 |
|---|------|------|------|
| 1 | 训练前 | 1%/99% 分位数动作归一化 | 剔除 teleop 离群尖刺 |
| 2 | 训练前 | 动作空间对齐 + zero-pad + control-mode prompt | 跨本体可混训 |
| 3 | Post-training | 只留成功 + 短长度 episode | 提高动作质量 |
| 4 | 评测后 | 取消的 episode 剔除 | 评测层清洗 |
| 5 | 标注层 | 人工子任务 + bbox + policy relabeling | HL 数据加工 |
| 6 | VI 数据 | 半自动合成 | 过滤劣质 HL 标签 |
| 7 | Web 数据 | 选用公开集 + bbox 扩充 | 语义补足 |
| 8 | 训练时 | augmax 图像增广 | 视觉鲁棒性 |
| 9 | 训练时 | Flow timestep 偏置 + 截断 | 损失加权 |

---

## 1. 动作数据归一化（最关键的清洗步骤）

正文 IV-C 末尾：

- **逐数据集独立**统计每个动作维度的 **1% 和 99% 分位数**
- 按此把动作归一化到 `[-1, 1]`
- 用分位数而非 min/max → 天然剔除 teleop 中的离群点 / 抖动尖刺
- 等价于一次**鲁棒缩放 + 隐式 outlier clipping**

> 这条是论文里唯一一处显式的"过滤异常值"处理。

---

## 2. 动作空间对齐（跨本体融合的预处理）

正文 IV-C：

- 所有机器人统一到 **最大动作维度**
- 低维机器人的动作向量做 **zero-padding**
- 同一模型同时学**关节空间**与**末端位姿**两种控制目标，靠 prompt 区分：

  ```
  <<<control_mode>>> joint <<<control_mode>>>
  <<<control_mode>>> end effector <<<control_mode>>>
  ```

效果：MM / ME / CE 不同 form factor、不同控制模式的样本可放在同一个 batch 训练，无需切分 head。

---

## 3. Post-training 阶段的样本筛选

正文 IV-D：

- 仅保留 **成功 episode**
- 仅保留 **长度 < 固定阈值**的 episode
- 数据子集 = `MM ∪ ME` 经过滤后的子集
- **CE（实验室跨本体）在 post-training 阶段被整体丢弃**，让模型聚焦移动操作
- WD 和 HL 仍保留，以维持语义 / 视觉能力

---

## 4. 评测阶段的样本剔除

附录 B：

- 因机器人故障、超时、其它意外**取消的 episode 整体剔除**
- 再做双尾 t-test 统计显著性
- 属于**评测层清洗**，不进训练

---

## 5. 人工标注 + 重标注（数据加工，不是单纯过滤）

正文 IV-C（HL 部分）+ 图 4：

- **多子任务回合**人工标注语义子任务文本描述
- 当前观测下相关物体的 **bounding box** 人工标注，使模型在预测子任务前先预测 bbox
- 图 4 post-training 示例展示 **policy relabeling**：
  - "put plate in sink" → "put plate on rack"
  - "push the top drawer" → "pick up blue shirt"

→ 同一段动作可以挂多个不同的子任务标签，扩展语言多样性。

---

## 6. VI（Verbal Instruction）数据的半自动合成

正文 IV-D：

- 专家用户**实时口头**给已经训练好的低层策略下子任务命令
- 机器人在低层策略下完成任务
- 把 `(观测, 口头子任务)` 作为高质量 HL 监督样本

特点：
- **用低层策略 + 人作为高层标注器**，劣质 / 不可执行的子任务标签会被人当场过滤掉
- 仅占 HL MM 样本 **~11%**
- Ablation 显示去掉 VI 显著掉点 → 关键数据

---

## 7. Web 数据的整合

正文 IV-C（WD 部分）：

- 直接选用已清洗的公开集：CapsFusion / COCO / Cambrian-7M / PixMo / VQAv2
- **物体定位任务自行扩充**：额外抓取室内场景 / 家居物品的 bounding box 标注数据
- 没有提到自定义清洗规则 —— 假定上游公开集已清洗

---

## 8. 图像增广（训练时的预处理）

附录 E 给出 `augmax` 流水线（固定顺序）：

```python
transforms = [
    augmax.RandomCrop(int(width * 0.95), int(height * 0.95)),
    augmax.Resize(width, height),
    augmax.Rotate((-5, 5)),
    augmax.ColorJitter(brightness=0.3, contrast=0.4, saturation=0.5),
]
```

应用范围：所有输入图像（多路相机一致处理）。

---

## 9. Flow matching timestep 采样的偏置 + 截断

附录 E：

- 不使用均匀 $\tau \sim U(0,1)$
- 改用偏向**低噪声 timestep** 的 Beta 分布
  $$p(\tau) = \text{Beta}\!\left(\tfrac{s-\tau}{s};\ \alpha=1.5,\ \beta=1\right)$$
- 截断 $s = 0.999$
- $\tau > s$ 的样本**直接丢弃**，不参与训练

→ 相当于在 loss 输入侧丢弃"无用"样本（无需积分的 timestep），属于训练样本加权 / 过滤。

---

## 关键观察

1. **π₀.₅ 把"清洗"重心放在跨本体融合**：分位数归一化 + 维度对齐 + control-mode prompt，使异构数据可直接混训。
2. **质量保证靠"成功 + 短"过滤 + 人工 relabeling**，而非自动化清洗模型。
3. **VI 数据是亮点**：用已有低层策略做"动作可执行性过滤器"，让人只标注那些机器人真能完成的子任务，避免不可执行的 HL 标签污染数据。
4. **图像增广和 flow timestep 截断**属于训练侧预处理，对模型鲁棒性 / 收敛速度有影响。
5. **没有公开自动数据筛选 metric**（如基于轨迹平滑度、夹爪行为等自动剔除的脚本），这与 OXE 体系做法一致 —— 信任 teleop 操作员 + 分位数归一化即可。

---

## 对自有数据 pipeline 的可借鉴点

| π₀.₅ 做法 | 可直接复用的地方 |
|----------|-----------------|
| 1%/99% 分位数归一化 | 任何 imitation learning 数据集的动作 norm |
| 关节 + 末端双模式 prompt 分流 | 跨机器人 / 仿真 + 真机混训 |
| 成功 + 长度过滤 | post-training 阶段筛子集 |
| 多子任务 + bbox 人工标注 | 长程任务的 HL 监督 |
| Policy relabeling | 增加语言多样性 |
| VI 半自动合成 | 用现有低层策略 + 人做 HL 标注 |
| Beta timestep 截断 | flow matching / rectified flow 训练优化 |
