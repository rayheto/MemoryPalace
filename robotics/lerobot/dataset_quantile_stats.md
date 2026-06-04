# LeRobot 数据集 quantile 统计量

## 是什么

数据集里每个维度（action / state / 有时 image）的 **q01（1% 分位）和 q99（99% 分位）**——即"忽略两端最极端 1% 样本后，这个维度的取值范围"。

## 归一化方法对比

| 方法 | 公式 | 问题 |
|------|------|------|
| mean/std | `(x - mean) / std` | 对离群点敏感，一个抖动样本把 std 拉大，正常值被压扁 |
| min/max | `(x - min) / (max - min)` | 更敏感，一个错误样本毁掉整个范围 |
| **quantile (q01/q99)** | `(x - q01) / (q99 - q01) * 2 - 1` | **自动裁掉两端 1% 极值**，对噪声/异常鲁棒 |

## 实际例子

某关节角正常范围 `[-1.5, 1.5]` rad，60 集示教里偶尔抖到 3.0 rad：

- min/max 归一化：按 `[-1.5, 3.0]` 算，正常动作全被压到 `[-1, 0]` 半边，浪费一半值域
- quantile 归一化：q99 ≈ 1.5（3.0 在 1% 极值里被忽略），正常动作铺满 `[-1, 1]`，训练更稳

## 在 Pi0 / Pi0.5 里的作用

- pi05_base 等 Pi0 系策略的 normalization layer **直接读 `q01` / `q99` 字段**
- 缺字段会报错，必须提供
- 训练前 normalize 输入、推理时 unnormalize 输出动作

## 为什么需要"补算"

- LeRobot **v2.1** 数据集的 `stats.json` 只存 mean/std/min/max
- **v3.0** 才加进 quantile 字段
- `lerobot.datasets.v30.convert_dataset_v21_to_v30` 转格式时**只搬数据不重算 stats**，quantile 字段是空的
- 必须再跑一次回填

## 补算命令

```bash
python -m lerobot.datasets.v30.augment_dataset_quantile_stats \
  --repo-id=<HF_REPO> \
  --root=<本地缓存根目录>
```

行为：扫一遍所有 episode，计算每个维度的 q01 / q99（可能还有 q05/q95），就地写回 `meta/stats.json`。

## 常用日志写法

```bash
... 2>&1 | tee /tmp/quantile.log | tail -30
```

`tee` 落盘完整日志，`tail -30` 只在终端显示尾部（这类脚本会刷一长串 per-episode 进度）。

## 触发场景

`lerobot-train` / SFT 启动时报 "missing quantile stats" / "q01 not found" —— 八成是数据集刚从 v2.1 转 v3.0 还没补算。跑一次上面的命令即可。
