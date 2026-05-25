# 把 LIPM reference + reward 接入 Tinker 训练框架

## Context

Tinker 当前的 trot 训练（`tinker_constraint_him_trot.py`）只用 `tracking_lin_vel` + `feet_air_time` + `trot_pattern1/2`（固定 cycle_time=0.5s）这一套奖励，**没有落脚点级别的物理参考**。RL policy 只知道"该到目标速度"，不知道每步**该往哪儿放脚**——这导致它在中高速场景下容易摔。

ModelBasedFootstepPlanning-IROS2024 的范式已经验证有效：**LIPM 用 XCoM 公式给出每步的最优落脚点 (u_x, u_y) 作为参考，RL 用 `_reward_contact_schedule` 跟踪 "在 schedule 要求接触时落到 u 点"**。这套范式直接搬到 Tinker 上能：

1. 让 policy 看到 lookahead 的落脚点，不是只看到速度命令；
2. 把 capture point 理论的稳定性先验植入 reward 场；
3. 把现在的固定 cycle_time trot 升级成 step_period 可命令变化的 LIPM 步频。

整个范式不引入 SLIP，**继续用 LIPM 的解析 cosh/sinh**——LIPM 在走路这个速度区间是被验证可行的，SLIP 留给后续 running gait 研究（见 [MemoryPalace/robotics/simulator/SLIP_3D_replace_LIPM_debug.md](/home/hlei/MemoryPalace/robotics/simulator/SLIP_3D_replace_LIPM_debug.md) 第 7 节）。

---

## 范围决策（已和用户确认）

- **完整移植**：phase clock + LIPM step gen + obs 扩展 + reward
- **目标配置**：`tinker_constraint_him_trot.py`
- **gait clock**：LIPM phase 取代固定 `cycle_time`，step_period 可命令采样

---

## 文件改动清单

### 1. 新增缓冲区与 phase clock — [envs/legged_robot.py](/home/hlei/robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/envs/legged_robot.py)

**`__init__` 末尾新增**（参考 IROS2024 humanoid_controller.py:78–98）：

```python
# LIPM gait state
self.phase = torch.zeros(self.num_envs, 1, device=self.device)
self.phase_count = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
self.update_count = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
self.step_period = torch.full((self.num_envs, 1), int(self.cfg.commands.step_period_init),
                              dtype=torch.long, device=self.device)
self.full_step_period = 2 * self.step_period

# LIPM physical quantities
self.CoM = torch.zeros(self.num_envs, 3, device=self.device)
self.w = torch.zeros(self.num_envs, 1, device=self.device)
self.ICP = torch.zeros(self.num_envs, 3, device=self.device)

# Step commands: (right_foot, left_foot) × (x, y, heading)
self.step_commands = torch.zeros(self.num_envs, 2, 3, device=self.device)
self.current_step = torch.zeros(self.num_envs, 2, 3, device=self.device)
self.prev_step_commands = torch.zeros(self.num_envs, 2, 3, device=self.device)
self.foot_on_motion = torch.zeros(self.num_envs, 2, dtype=torch.bool, device=self.device)
self.foot_on_motion[:, 1] = True   # left foot swings first
self.support_foot_pos = torch.zeros(self.num_envs, 3, device=self.device)

# Tracking error caches
self.step_location_offset = torch.zeros(self.num_envs, 2, device=self.device)

# Observation extension caches
self.phase_sin = torch.zeros(self.num_envs, 1, device=self.device)
self.phase_cos = torch.zeros(self.num_envs, 1, device=self.device)
self.contact_schedule = torch.zeros(self.num_envs, 1, device=self.device)
self.dstep_width = torch.full((self.num_envs, 1), self.cfg.commands.dstep_width, device=self.device)
```

### 2. 新增方法（粘贴/适配自 IROS2024 humanoid_controller.py）

按以下顺序加到 `LeggedRobot` 类（每个方法都有 IROS2024 源 file:line 可对照）：

| 方法 | IROS 源 | 作用 |
|------|---------|------|
| `_calculate_CoM()` | humanoid_controller.py:230–232 | 多刚体加权平均 → self.CoM |
| `_calculate_ICP()` | humanoid_controller.py:234–241 | w = √(g/z_c)，ICP = CoM + v/w |
| `_measure_step_offset()` | humanoid_controller.py:259–263 节选 | self.step_location_offset = ‖foot − step_commands‖ |
| `_update_phase_and_commands()` | humanoid_controller.py:284–305 + 287–289 | phase 推进、到期 reset、触发 step gen |
| `_generate_step_command_by_3DLIPM_XCoM()` | humanoid_controller.py:424–494 | **核心**：XCoM 公式生成 (u_x, u_y) |
| `smooth_sqr_wave(phase)` | IROS humanoid_controller.py 工具方法 | 平滑方波 → contact_schedule |

**核心公式**（粘自 humanoid_controller.py:468–488，无需改动，已是 vectorized PyTorch）：

```python
x_f = x_0*torch.cosh(T*w) + vx_0*torch.sinh(T*w)/w
vx_f = x_0*w*torch.sinh(T*w) + vx_0*torch.cosh(T*w)
# ... y 同理
eICP_x = x_f + support_foot_pos[:,0:1] + vx_f/w
b_x = dstep_length / (torch.exp(T*w) - 1)
u_x = eICP_x - b_x  # 旋转 theta 之后赋值给 step_commands
```

### 3. 接入 `_post_physics_step_callback`

在现有 callback 末尾追加：

```python
self._calculate_CoM()
self._calculate_ICP()
self._update_phase_and_commands()   # 内部：phase += 1/full_step_period，到期 call _generate_step_command_by_3DLIPM_XCoM
self._measure_step_offset()
self.contact_schedule = self.smooth_sqr_wave(self.phase)
self.phase_sin = torch.sin(2*torch.pi*self.phase)
self.phase_cos = torch.cos(2*torch.pi*self.phase)
```

### 4. observation 扩展 — [envs/legged_robot.py](/home/hlei/robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/envs/legged_robot.py) `compute_observations()` ~line 589

把 step_commands 旋转到 base frame 后拼进 obs_buf（参考 IROS2024 humanoid_controller.py:632–644 的 `_set_obs_variables`）：

```python
# 原 39 维基础上追加：
step_cmd_base_right = quat_rotate_inverse(self.base_quat,
    self.step_commands[:,0,:3] - self.root_states[:,0:3])
step_cmd_base_left  = quat_rotate_inverse(self.base_quat,
    self.step_commands[:,1,:3] - self.root_states[:,0:3])
obs_buf = torch.cat((obs_buf,
    step_cmd_base_right[:,:2],          # [2]
    step_cmd_base_left[:,:2],           # [2]
    self.phase_sin,                     # [1]
    self.phase_cos,                     # [1]
    self.contact_schedule,              # [1]
), dim=-1)                              # 总维度: 39 + 7 = 46
```

**配套更新**：`tinker_constraint_him_trot.py` 里 `env.num_observations = 46`，`policy.num_proprio = 46`，actor_critic 网络第一层尺寸自动随之变。

### 5. 新增 reward — [envs/legged_robot.py](/home/hlei/robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/envs/legged_robot.py) 末尾

复用现有 reward 注册机制（`_prepare_reward_function` 自动 `getattr(self, '_reward_' + name)`）：

```python
def _reward_contact_schedule(self):
    """ Alternate right/left contacts per LIPM phase, gated by step_location accuracy.
        IROS2024 humanoid_controller.py:823-830 """
    contact_rewards = (self.foot_contact[:,0].int() - self.foot_contact[:,1].int()) \
                      * self.contact_schedule.squeeze(1)
    a = 1.0
    swing_offset = self.step_location_offset[~self.foot_on_motion]
    tracking = 3.0 * torch.exp(-swing_offset.norm(dim=-1) / a)
    return contact_rewards * tracking

def _reward_step_location(self):
    """ Reward swing foot landing close to LIPM-predicted u_x, u_y """
    return torch.exp(-self.step_location_offset[self.foot_on_motion].norm(dim=-1) / 0.1)
```

### 6. command 采样扩展 — `_resample_commands()` ~line 1675

```python
self.step_period[env_ids] = torch.randint(
    self.command_ranges["sample_period"][0],
    self.command_ranges["sample_period"][1],
    (len(env_ids), 1), device=self.device)
self.full_step_period = 2 * self.step_period
```

`reset_idx`/`_reset_system` 里 reset `phase, phase_count, update_count, step_commands, foot_on_motion` 至初始（参考 humanoid_controller.py:143–167）。

### 7. config — [configs/tinker_constraint_him_trot.py](/home/hlei/robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/configs/tinker_constraint_him_trot.py)

```python
class commands:
    step_period_init = 25        # ~0.5s @ dt=0.02
    sample_period = [20, 35]     # 0.4 ~ 0.7s
    dstep_length = 0.3
    dstep_width = 0.2

class rewards(LeggedRobotCfg.rewards):
    # ...existing...
    class scales(LeggedRobotCfg.rewards.scales):
        # 关闭与 LIPM 冲突的固定 trot：
        feet_air_time = 0.0            # 原 3.0，由 contact_schedule 取代
        # 新增：
        contact_schedule = 3.0
        step_location = 1.5
```

`env.num_observations = 46`。

---

## 关键复用清单（不要重复造）

- **reward 注册机制** [envs/legged_robot.py:1334–1380](/home/hlei/robotic/OmniBotSeries-Tinker/OmniBotCtrl/OmniBotCtrl/envs/legged_robot.py#L1334-L1380) — 加 `_reward_xxx` 方法 + scale 配置即可
- **foot contact/positions** `self.feet_indices`, `self.contact_forces`, `self.feet_pos` — 已就绪
- **base frame 旋转工具** `quat_rotate_inverse` — Isaac Gym 内置
- **XCoM 公式** humanoid_controller.py:424–494 — vectorized 直接粘贴，无需改写

---

## 关键风险

1. **obs 维度变化** 会让现有 checkpoint 不可加载——必须从头训。
2. **step_period 变化**与 `feet_air_time` 关闭后，原有的稳定 trot 学习曲线会破坏。可能需要 curriculum：前 N 步 step_location reward 权重低，让 policy 先学站稳。
3. **support foot 切换时机**：IROS 用 `update_count >= step_period` 时间触发；Tinker 的 PD/contact 与 IROS 不完全一致，可能要小调阈值。
4. **`tinker_constraint_him_trot.py` 里 trot_pattern1/2 的引用点**要全部去掉，否则会和 LIPM phase 冲突。建 PR 前 grep 一遍。

---

## 验证

按以下顺序：

1. **冒烟（physics-only，无 RL）**：在 `play.py` 里禁用 policy，用零动作跑一个 env，print phase 推进、step_commands、step_location_offset 的轨迹 10s，确认数值都在合理范围（u 在 ±1m 内、phase 0→1→0 循环、step_period 切换正常）。
2. **短训（500 iter）**：在 `tinker_constraint_him_trot.py` 上跑训练，看 `contact_schedule` reward 是否随训练上升、`step_location_offset` 是否下降。
3. **回归对比**：保留原 trot 训练 checkpoint，对比新框架在 v=0.5/1.0/1.5 m/s 三个速度下的存活率。
4. **稳定速度边界**：找出 LIPM 框架下 policy 能稳定行走的最高 vx，作为后续 SLIP/running 研究的 baseline。

---

## 后续（不在本次改动范围）

- step_period 自动响应速度命令（dstep_length × T = v_d × T）；
- 把 SLIP 替代 LIPM 的实验（含 flight phase）按 SLIP_3D_replace_LIPM_debug.md §7 路线，在此框架上做增量替换；
- ZMP / 角动量 reward 补全，sim2real 时再考虑。
