# Phase 2 pick_orange SFT eval — chain-collapse 诊断

**日期**: 2026-06-04
**对象**: `outputs/sft_pi05_pickorange/checkpoints/034000/pretrained_model`（lerobot 原生 PI05Policy，60 episodes × 2 envs × 1500 steps）
**结论**: SFT 单段 pick-place 已饱和（grasp→place 转化 100%），瓶颈是**链式串联坍塌**。整体 full-task SR = 6.7%，与 per-stage 条件概率乘积（6.6%）完美吻合。继续训 SFT 触不到 60-70% 整体 SR；下一步只能改 closed-loop 频率或进 RL。

---

## 一、关键指标修正

`scripts/eval_pi05_pickorange_lerobot.py` 原本打印的 `SR: x/y` 是**per-orange place rate**（分母 = episodes × 3 oranges），不是 episode 粒度成功率。orchestrator 的 `eval_results.tsv` 一直在记这个值，所以 030K=33%、032K=43%、034K=45% 这些数都属于"橘子粒度的 place 率"。

真·任务成功率（env `task_done` DoneTerm: 3 橘**同时**进盘 ±10cm + arm 回 rest pose）从来没被独立测过。这次 P0 加了 staged 统计后才看到。

| 指标 | 数值 | 备注 |
|---|---|---|
| per-orange place rate | 62 / 180 = **34.4%** | orchestrator 历史口径 |
| full-task SR | 4 / 60 = **6.7%** | env DoneTerm |
| ever all-3 on plate simul | 5 / 60 = 8.3% | 历史上有过 3 橘同时在盘 |
| ever rest-after-first-pick | 3 / 60 = 5.0% | arm 抓过后曾回 rest |

`ever_all3_simul_and_rest = 0/60` 是 instrumentation 假阴性（IsaacLab `env.step` 在 done 步返回的是 post-auto-reset obs，subtask flags 已被清零，所以 sticky-OR 抓不到成功瞬间）。authoritative 数仍是 `succ`（`term & ~trunc`）= 4。

---

## 二、链式衰减表（核心诊断）

```
[EVAL] per-orange grasp ever:  O1=21.7%  O2=41.7%  O3=45.0%
[EVAL] per-orange place ever:  O1=18.3%  O2=43.3%  O3=41.7%
[EVAL] cumulative grasps:      ≥1=71.7%  ≥2=26.7%  =3=10.0%
[EVAL] cumulative places:      ≥1=71.7%  ≥2=21.7%  =3=10.0%
```

| 过渡 | 条件成功率 | 解读 |
|---|---|---|
| grasp≥1 起手 | 71.7% | 起手够稳 |
| **P(grasp≥2 \| grasp≥1)** | **37.2%** | 🔴 抓完第 1 个后，63% 再也抓不到第 2 个 |
| P(grasp=3 \| grasp≥2) | 37.5% | 同样 ~63% drop |
| **P(place≥1 \| grasp≥1)** | **100%** | ✅ "只要这集里抓到了任意一颗，就一定有一颗被放上盘子" —— 注意：分子分母都不区分是哪颗橘子，所以这只能说"从抓到放的最后一公里没问题"，**不能**推出"第 1 段一定靠谱"。要看具体哪一颗的，请查 per-orange 行：O1 grasp 只有 21.7% |
| P(place≥2 \| place≥1) | 30.2% | 70% drop |
| P(place=3 \| place≥2) | 46.2% | |
| P(all3_simul \| place=3 ever) | 83.3% | 偶有"放上又被撞掉"，非主因 |
| P(success \| all3_simul) | 80.0% | parking 阶段也基本能完成，非主因 |

**乘法验证**：`0.717 × 0.302 × 0.462 × 0.833 × 0.80 = 6.6%` ≈ 实测 6.7%。
→ 纯链式衰减模型 + 各段近独立。**没有协同故障模式（如"前面成功反而后面更差"）**。

---

## 三、关于 per-orange 排序的反直觉发现

```
grasp:  O1=21.7%   O2=41.7%   O3=45.0%
place:  O1=18.3%   O2=43.3%   O3=41.7%
```

Orange 1 的抓取/放置率**反而最低**。不是因为 O1 物理上最难，而是说明策略**没学到"先 O1 再 O2 再 O3"的顺序先验**——它会先抓最容易够到的那个，无视 demo 中的固定顺序。这是 chain collapse 的旁证：策略没把"已完成 N 个"作为状态条件。

---

## 四、训练侧客观条件（无须再调，已 confirm 不是 SFT 训练设置问题）

| 项 | 值 | 评估 |
|---|---|---|
| 数据集 | LightwheelAI/leisaac-pick-orange | 60 eps / 36K frames @ 30fps |
| loss 末端 | 0.052 @ 34K step（epoch 7.5）| 已平台，再训过拟合 |
| normalization | STATE/ACTION = QUANTILES | ✅ 用户已主动选 |
| batch_size / lr | 8 / 2.5e-5 cosine | 标准 pi0.5 SFT |
| chunk_size / n_action_steps | 50 / 50 | 🔴 1.67s 开环 |
| freeze_vision_encoder | True | 标准 SFT 配方 |
| image_transforms.enable | False | 🟡 无增广 |
| use_amp | False，dtype bfloat16 | OK |

`loss=0.05` 在 QUANTILES 归一空间（target ∈ [-1,1]）就是 BC 下界，原因是 demo 本身有动作噪声 + 同观测多模态。继续训不会再降，已经看到 30K SR 比 28K 还差的过拟合信号。

---

## 五、回答用户原始三问

1. **"是不是因为多个抓取动作连续执行才差？"** —— **是，数据 100% 确认**。单段 pick→place = 100%，链式 4 段失败率 ≈ 93%。

2. **"loss 卡 0.05、想到 60-70% SR 合理吗？"**
   - 仅靠 SFT + 60 demos **不可能**到 60-70% 整体 SR：per-orange 34% × 3 段独立 ≈ 单位数百分比。
   - 60-70% **单段 SR** 也还差（现状 O2/O3 ≈ 42-45%）。
   - loss 0.05 是 BC 下界，**和 SR 解耦了**，再降 loss 也不会提 SR。

3. **"如何继续排查？"** —— P0 已做完（staged eval），结论清晰。下面是 P1 起的动作。

---

## 六、Next steps（按 ROI 排序）

| 优先级 | 动作 | 预期效果 | 工程量 |
|---|---|---|---|
| **P1** | 同 34K ckpt 改 `n_action_steps`: 50 → 10（chunk_size 不变，只是不消耗完），重跑 staged eval | 直接攻击 chain collapse；如果 SR 翻倍 → open-loop drift 是主因 | 1 行配置 |
| **P1** | 进 **Phase 3 RL**（RLinf PPO/RLOO），SFT prior 提供 100% per-stage 转化已经够当 init | RL 专治 long-horizon 串联，原本就是项目计划 | 较大 |
| P2 | 重训 SFT：`image_transforms.enable=True` + 解冻 vision encoder 顶 1-2 层 lr=1e-6 | 单段 SR +5-10pp（不解决链式问题）| 中 |
| P2 | 收 200+ demos | 最稳但慢 | 大 |
| P3 | 任务拆解：先训"pick 1 orange"独立策略再串联 | 工程复杂 | 大 |

**强建议先做 P1 短开环 eval**：一个变量隔离实验就能拍板下一步走"短开环"还是直接进 RL。

---

## 七、Instrumentation 备忘

- staged eval 在 [scripts/eval_pi05_pickorange_lerobot.py](../../../robotic/lerobot-rlinf/scripts/eval_pi05_pickorange_lerobot.py) 末尾输出
- env 的 `obs["subtask_terms"]["pick_orange00N"]` / `["put_orange00N_to_plate"]` 是 per-step bool tensor，sticky-OR 累积即可
- `is_so101_at_rest_pose` 从 `leisaac.utils.robot_utils` 拿，传入 raw `joint_pos`（radians）+ `env.scene["robot"].data.joint_names`
- `ever_all3_simul_and_rest` 因 IsaacLab 自动 reset 落在 env.step 内导致假阴性；判 episode 成功仍用 `term & ~trunc` 累积的 `succ`
- 完整原始日志: `/tmp/eval_pickorange_34000_staged.log`

---

## 八、相关文件

- 训练配置: `outputs/sft_pi05_pickorange/checkpoints/last/pretrained_model/train_config.json`
- eval orchestrator: `scripts/sft_eval_loop_pickorange.sh`（注意它仍解析旧版 "SR" 行作为 per-orange rate，未来更新需保持向后兼容）
- env 成功判据: `third_party/leisaac/source/leisaac/leisaac/tasks/pick_orange/mdp/terminations.py:task_done`
- obs 子任务：`third_party/leisaac/source/leisaac/leisaac/tasks/pick_orange/pick_orange_env_cfg.py:39-62`

---

## 九、P1 实验结果 — closed-loop n_action_steps 50 → 10 (2026-06-04)

**实验设置**：同 34K ckpt，唯一变量是 `--n-action-steps 10`（chunk_size=50 不变，只是不消耗完）；60 集 × 2 envs × 1500 steps，wall clock ~5 h。原始日志：`/tmp/eval_pickorange_34000_n10.log`。

实现：`scripts/eval_pi05_pickorange_lerobot.py` 加 `--n-action-steps` CLI flag，在 `PI05Policy.from_pretrained` 后直接覆写 `policy.config.n_action_steps`（`reset()` 和 `select_action()` 都现读 config，覆写即生效）。

### Headline 对比

| 指标 | n=50 baseline | **n=10 closed-loop** | Δ |
|---|---|---|---|
| **full-task SR** | 4/60 = 6.7% | **6/60 = 10.0%** | +3.3 pp |
| per-orange place rate | 62/180 = 34.4% | 89/180 = 49.4% | +15 pp |
| ever all-3 simul | 5/60 = 8.3% | **12/60 = 20.0%** | **2.4×** |
| ever rest-after-pick | 5.0% | 5.0% | 0 |

### 条件存活率 — 链断点变化

| 过渡 | n=50 | n=10 | Δ |
|---|---|---|---|
| P(grasp≥2 \| grasp≥1) | 37.2% | 55.6% | **+18 pp** |
| P(place≥1 \| grasp≥1) | 100% | 90.7% | 🟡 -9.3 pp（holding 时 re-query 偶致掉落）|
| **P(place≥2 \| place≥1)** | 30.2% | **57.1%** | **+27 pp**（最大改善）|
| P(all3_simul \| place=3) | 83.3% | 100% | "放完被撞掉" 消失 |
| **P(success \| all3_simul)** | **80%** | **50%** | 🔴 **parking 阶段反而退化** |

乘法验证 n=10：`0.90 × 0.571 × 0.429 × 1.0 × 0.50 = 11.0%` ≈ 实测 10%。链式独立模型仍成立。

### 关键结论

1. **chain collapse 主因 = open-loop drift**：诊断假设被数据直接验证。`P(place≥2|place≥1) +27 pp`、`ever_all3_simul 2.4×` 这两个指标都是单调对应"减少开环 drift"。
2. **per-orange 顺序偏好（O1 最低）没被纠正**：closed-loop 只是 uniform 上抬，没解决"无顺序先验"。这部分要靠 RL 或重训。
3. **新瓶颈在 parking**：`P(success|all3_simul) 80% → 50%`。可能：
   - 小样本（5 vs 12 trials，Wilson CI 都宽），可能假象
   - 真实回归：n=10 每 0.33s 重看图，3 橘放完后模型来回调整，反而远离 rest pose
4. **天花板算账**：当前 n=10 链 `0.90 × 0.571 × 0.429 = 22%` 是"3 橘曾全部放上"的上限，再乘 parking 50% = 11%。即便 parking 修回 80%，full-task SR 上限也只 17.6%。
5. **`n_action_steps` 这条单变量轴的天花板 ~17%，无法触达 60-70%**。继续提需要改变数据 / 增广 / 解冻 vision，或上 RL。

### 决策

- **优先级倒换**：原报告"P1 短开环 eval"现在已验证完毕，**P1 的真正下一步是 Phase 3 RL**（用 34K ckpt + `n_action_steps=10` 作 RL init，链先验更好、收敛更稳）
- 可选中间实验：`n_action_steps=20`（~3 h），看 parking 是否回升 + SR 是否到 13-15%。但即便最好情况，上限仍受限于上面算账，不改大方向
- SFT 训练侧改进（image_transforms / 解冻 vision）作为长期补丁，与 RL 并行考虑

---

## 十、HL 4-stage 状态机实验（OOD prompt 探测）

**追加时间**: 2026-06-04
**实验**: 同 34K ckpt + `n_action_steps=10` + 手编 HL 状态机
**改动**: 按 `subtask_terms` 的 placed 计数，每集动态切换 4 个 prompt
```
placed=0 → "Pick up the first orange and place it on the plate"
placed=1 → "Pick up the second orange and place it on the plate"
placed=2 → "Pick up the third orange and place it on the plate"
placed=3 → "Move robot arm back to rest position"
```
**已知 OOD 风险**: SFT 训练数据 60 集仅含一个固定 prompt `"Grab orange and place into plate"`，模型从未见过 "first/second/third orange" 或 "rest position"，PaliGemma 是否响应不可预期。

### 三组对照（同 ckpt 同 60 eps）

| 指标 | n=50 baseline | n=10 baseline | n=10 + HL-4 | HL-4 vs n=10 |
|---|---|---|---|---|
| **full-task SR** | 4/60 = 6.7% | 6/60 = 10.0% | **3/60 = 5.0%** | 🔴 **-5 pp** |
| per-orange place rate | 30.0% | ~40% | 32.8% | — |
| O1 grasp ever | 21.7% | **40.0%** | **11.7%** | 🔴 **-28 pp** |
| O2 grasp ever | 41.7% | 61.7% | 40.0% | 🔴 -22 pp |
| O3 grasp ever | 45.0% | 65.0% | 48.3% | 🔴 -17 pp |
| O1 place ever | 18.3% | 36.7% | 11.7% | 🔴 -25 pp |
| O2 place ever | 43.3% | 56.7% | 48.3% | 🟡 -8 pp |
| O3 place ever | 41.7% | 55.0% | 38.3% | 🔴 -17 pp |
| grasp ≥1 | 71.7% | 90.0% | 66.7% | 🔴 -23 pp |
| grasp ≥2 | 26.7% | 50.0% | 25.0% | 🔴 -25 pp |
| grasp =3 | 10.0% | 26.7% | 8.3% | 🔴 -18 pp |
| place ≥1 | 71.7% | 81.7% | 63.3% | 🔴 -18 pp |
| place ≥2 | 21.7% | 46.7% | 26.7% | 🔴 -20 pp |
| place =3 | 10.0% | 20.0% | 8.3% | 🔴 -12 pp |
| ever all3 simul | 8.3% | 20.0% | 8.3% | 🔴 -12 pp |
| **HL fires** | n/a | n/a | 5/60 = 8.3% | 状态机几乎没机会触发 |

### 条件存活率

| 过渡 | n=10 baseline | n=10 + HL-4 | Δ |
|---|---|---|---|
| P(grasp≥2 \| grasp≥1) | 55.6% | 37.5% | 🔴 -18 pp |
| P(grasp=3 \| grasp≥2) | 53.3% | 33.3% | 🔴 -20 pp |
| P(place≥1 \| grasp≥1) | 90.7% | 95.0% | 🟢 +4 pp |
| P(place≥2 \| place≥1) | 57.1% | 42.1% | 🔴 -15 pp |
| P(place=3 \| place≥2) | 42.9% | 31.2% | 🔴 -12 pp |
| P(all3_simul \| place=3) | 100% | 100% | — |
| **P(success \| all3_simul)** | 50.0% (6/12) | 60.0% (3/5) | 🟢 +10 pp |

乘法验证 HL-4: `0.667 × 0.421 × 0.312 × 1.0 × 0.60 = 5.3%` ≈ 实测 5.0%。链式独立模型仍成立。

### 关键发现

1. **HL-4 是净负面**：full-task SR 从 10% 降到 5%（-5 pp），几乎所有 grasp/place 指标都退化。
2. **smoking gun = O1 prompt 摧毁第一颗橘子的抓取**：
   - "Pick up the **first** orange" 的状态停留时间最长（episode 起手到 placed=1）
   - O1 grasp 率 40% → 11.7% (**-28 pp，3.4× 衰减**)
   - 与此同时 O1 是默认 prompt 下表现最好的橘子（n=10 baseline 中 O1=40%, O2=62%, O3=65%），HL-4 把这一切优势全部抹平
   - 解释：模型从未学过 ordinal "first"，新 prompt 等效于强制切换到 OOD 文本条件；attention 中的语言 token 干扰了 vision-language 对齐，导致整体动作策略劣化
3. **HL 状态机几乎没机会触发**：placed=3 整集 fire 率 = 8.3%（5/60），与 n=50 baseline 同水平；n=10 baseline 当时是 20%（12/60）→ 因为 placed=0 prompt 太差，链根本走不到 placed=3
4. **唯一"+"号项 P(success | all3_simul) 60% > 50%**：到达 placed=3 后 "rest position" prompt 看起来确实帮到 parking，但分母只 5，置信区间宽到没法下结论（Wilson 95% CI: [17%, 91%]）
5. **per-orange 反直觉再次出现**：HL-4 下 O1 grasp 11.7%, O2=40%, O3=48.3%，模型仍按"先抓最容易的"，并非按 ordinal 选择

### 结论

**单 prompt SFT 的代价 = HL 状态机直接不可用**。不是 PaliGemma 听不懂英语，而是 SFT 阶段把"语言条件 → 行为"的对齐窗口缩成了一个点 token（"Grab orange and place into plate"）。任何 prompt 偏离都会塌陷成 OOD，连"听不听"都谈不上。

#### 短期可行的 HL 变体（未验证）

如果还想用 HL 状态机做 zero-shot 改善 parking：
- **只切 1 处 prompt**：placed in {0,1,2} 全部用 training 原文 prompt，**只**在 placed=3 时切到 rest prompt。
- 这是最早 HL 实现的版本，没跑过完整 60 eps 数据（早期 PID 3217386 被改成 4-stage 之前已 kill）。
- 该版本预期上界：维持 n=10 baseline 的链式数据（grasp ≥1 = 90%、place ≥2|≥1 = 57.1%），只增强 parking。若 P(success|all3_simul) 能从 50% 抬到 80%（n=50 baseline 水平），上限：`0.90 × 0.571 × 0.429 × 1.0 × 0.80 = 17.6%`
- 但 5 个样本太少，需要真跑

#### 根治路径
- **训练侧**：SFT 时把 task prompt 做轻度 paraphrase 增广（5-10 条同义句），打开语言条件维度，HL 才有空间施展
- **RL 侧**：直接用 34K + n=10 作 init 跑 PPO，链先验 22%（`0.90 × 0.571 × 0.429`）即是 "placed=3 ever" 上限，RL 重点 reward shape parking
- HL 状态机这条路在当前 SFT 数据下走不通，转方向

### 决策

放弃 4-stage HL 状态机方向。两条候选：
- **A** (保守、~30 min)：跑 1-stage HL（只切 placed=3 prompt）验证 P(success|all3_simul) 是否真稳定在 60-80%
- **B** (主方向)：直接进 Phase 3 RL，34K ckpt + n=10 作 init
