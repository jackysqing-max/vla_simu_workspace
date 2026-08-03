# 由浅入深的算法讲义

这份讲义面向“已经学过一些深度学习、大模型、基础机器人学，但希望真正理解这个系统内部算法链条”的学生。

它的目标不是只回答“系统由哪些模块组成”，而是回答下面 6 个更深的问题：

1. 深度学习在这个系统里到底负责什么
2. 几何恢复是怎么把 2D mask 变成 3D 操作点的
3. 为什么不能把原始感知结果直接丢给大模型
4. 大模型在这里做的是哪一种推理问题
5. executor 为什么必须是状态机，而不是直接执行文本输出
6. 低层控制为什么还需要传统 IK 和连续性约束

建议配合版本：

- branch: `release/v0.1.4`
- commit: `f01d108`

---

## 1. 先给学生一个总框架

建议第一句话就说清楚：

“这个系统不是单一模型，而是一条由深度学习感知、3D 几何恢复、结构化状态表示、LLM 高层规划、执行状态机和机器人控制组成的分层闭环。”

用一个更算法化的 pipeline 写出来就是：

```text
图像 I + 文本 prompt
  -> 条件分割
  -> mask M 和 score s
  -> RGB-D 反投影得到点云 P
  -> 候选目标生成与主点选择
  -> 结构化世界状态 S
  -> (instruction, S) 输入 LLM
  -> 结构化计划 π
  -> 执行状态机
  -> IK / 关节目标
  -> 机械臂闭环执行
```

学生如果能看懂这条链，就已经理解了这个项目的核心。

---

## 2. 深度学习层：条件分割不是结果，而只是开始

这里对应：

- `sam3_mask_node`

### 2.1 它在做什么

这一层的输入是：

- 当前 RGB 图像 `I`
- 当前语义提示 `prompt`

输出是：

- 二值 mask `M`
- mask score `s`

可以写成：

\[
(M, s) = f_\theta(I, \text{prompt})
\]

也就是说，这里深度学习解决的是一个**条件分割问题**，不是检测框，也不是动作预测。

### 2.2 为什么这里用分割而不是框

因为后面系统要做的是：

- 从像素区域恢复 3D 几何
- 找到目标的稳定空间位置
- 最终让机械臂悬停到目标上方

如果只有检测框，目标区域太粗，深度恢复不稳定。  
mask 给了更细粒度的像素支持集，后面的几何恢复才有基础。

### 2.3 这层的局限

学生要理解：

- mask 只是“图像空间”的结果
- 还没有 3D 信息
- 也还没有世界坐标系里的稳定目标点

所以深度学习感知层不是终点，而是整个几何与规划链路的起点。

---

## 3. 几何恢复层：从 mask 到 3D 点云

这里对应：

- [mask_depth_fusion_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/mask_depth_fusion_node.py)
- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py)

### 3.1 基本数学

已知：

- mask 内像素坐标 `(u, v)`
- 对应深度 `z`
- 相机内参 `(f_x, f_y, c_x, c_y)`

就可以通过 pinhole camera model 做反投影：

\[
x = (u - c_x)\frac{z}{f_x}, \quad
y = (v - c_y)\frac{z}{f_y}, \quad
z = z
\]

这样就能把 mask 内每个有效像素恢复成相机坐标系下的 3D 点。

代码位置：

- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L44)
- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L67)

### 3.2 为什么不能直接取点云质心

这是很值得课堂讲的地方。

如果直接取 mask 点云的质心，会有几个问题：

- 质心常常落在物体内部，而不是操作上更合理的表面点
- 深度有噪声时，质心会抖动
- 对于立方体，真正想追踪的通常是“顶部中心”或“可接近点”

所以系统没有直接用 centroid，而是做了更复杂的候选点生成。

---

## 4. 目标候选生成：这一步是几何与聚类算法的结合

这里对应：

- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L203)

### 4.1 输入是什么

从 mask 区域里抽出：

- `xyz`：3D 空间点
- `rgb`：对应颜色
- `uv`：像素坐标

然后构造联合特征：

\[
f_i = \left[\lambda_{xyz}\frac{xyz_i}{\sigma_{xyz}}, \lambda_{rgb} rgb_i\right]
\]

这里的思想是：

- 只用几何可能分不清局部结构
- 只用颜色又会丢掉空间信息
- 所以把几何和颜色拼成联合特征

### 4.2 为什么还要做 PCA

因为联合特征维度更高，直接聚类会更不稳。

所以这里先做 PCA：

\[
z_i = \text{PCA}(f_i)
\]

得到更紧凑的 embedding，再在这个 embedding 里做聚类。

代码位置：

- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L104)

### 4.3 为什么用 k-means 和 mean shift 组合

这里做了两步：

1. `k-means`
   - 先把 mask 内样本分成若干 cluster
   - 每个 cluster 给一个初始 proposal

2. `mean shift`
   - 再在 3D 空间里把太近的 proposals 合并成 mode

这个设计的思想是：

- k-means 适合快速形成离散 proposal
- mean shift 适合把近邻模式融合成更稳定的中心

这比直接选单点或者直接平均更稳。

代码位置：

- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L116)
- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L160)

### 4.4 主候选是怎么选的

生成 `candidates_xyz` 后，系统先选离整个 mask 点云中心最近的候选：

\[
i^\* = \arg\min_i \|c_i - \bar{x}\|
\]

代码位置：

- [pointcloud_ops.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/pointcloud_ops.py#L194)

这一步本质上是一个 heuristic，但足够稳定，适合当前 demo。

---

## 5. 顶面点估计：从“物体点云”到“操作点”

这一步在当前系统里非常关键，也最能体现“机器人感知不是只看见物体，而是要找可执行目标点”。

对应代码：

- [mask_depth_fusion_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/mask_depth_fusion_node.py#L95)
- [mask_depth_fusion_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/perception_geometry/perception_geometry/mask_depth_fusion_node.py#L478)

### 5.1 做法

1. 把 mask 点云从 camera frame 变到 world frame
2. 找到 `z` 最大附近的一层点
3. 对这些 top-surface points 取：
   - `x, y` 的中位数
   - `z` 的均值

可以理解为：

\[
P_{top} = \{p \in P \mid z(p) \ge z_{max} - \epsilon\}
\]

\[
x^\* = \text{median}(x), \quad
y^\* = \text{median}(y), \quad
z^\* = \text{mean}(z)
\]

### 5.2 为什么这么做

因为你的任务是“hover above the cube”。

所以真正有意义的目标不是点云中心，而是：

- 物体顶部
- 且更接近几何中心的位置

这样机械臂悬停行为才更稳定，也更符合人对任务的直觉。

---

## 6. 状态表示层：为什么需要 `scene_object_registry_node`

这里对应：

- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py)

### 6.1 它在做什么

它把下面这些流式感知结果：

- 当前 prompt
- 当前 score
- 当前 valid
- 当前 3D keypoint
- TF 变换

整理成稳定的 object table：

```json
{
  "label": "red cube",
  "confidence": 0.93,
  "position_world": {...},
  "visible": true,
  "last_seen_age_sec": 0.2
}
```

### 6.2 为什么它是必须的

因为大模型更适合消费的是**结构化语义状态**，而不是原始连续信号。

如果 planner 直接读：

- `/perception/keypoint_3d`
- `/perception/valid`
- `/sam3/score`

那它要自己处理：

- 时间同步
- object identity
- visible / invisible 状态
- prompt 与检测之间的绑定

这会让 planner prompt 非常混乱。

### 6.3 算法上它在做什么

这不是“新模型”，而是一个轻量级时序状态估计器：

- freshness filtering
- TTL 过期清理
- visibility 判定
- camera frame 到 world frame 的转换

代码位置：

- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py#L150)
- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py#L209)

所以你可以对学生说：

“`scene_object_registry_node` 的本质，是把感知流压缩成 planner 可消费的结构化状态表示。”

---

## 7. 大模型规划层：不是自由生成，而是约束下的离散规划

这里对应：

- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py)
- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py)

### 7.1 这个问题本质上是什么

这里大模型做的不是连续控制，而是：

\[
\pi = g_\phi(\text{instruction}, \text{scene summary})
\]

其中输出 `π` 不是自然语言，而是结构化动作序列。

当前动作空间很小：

- `hover_target`
- `wait`

见 [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L9)

所以从算法角度讲，它更接近：

- constrained sequence generation
- symbolic task planning

而不是端到端 policy learning。

### 7.2 为什么 planner 要加 scene summary

planner 输入并不只是 instruction。  
它还显式加入了：

- system prompt
- 当前 scene summary
- 用户指令

见 [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L217)

这一步背后的思想是：

- 语言模型擅长处理语义上下文
- 所以要把几何世界状态摘要成它容易读懂的描述

### 7.3 为什么要有 schema 和 sanitize

真实系统不能直接信任大模型输出。

所以 current plan pipeline 实际上是：

\[
\text{raw text} \to \text{JSON extract} \to \text{sanitize} \to \text{executable plan}
\]

其中 `sanitize_plan(...)` 会：

- 修正非法 action
- 归一化 target_prompt
- 丢弃不可执行 step

见 [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L99)

### 7.4 为什么需要 fallback

系统要面对的不是“模型永远正确”，而是：

- 模型可能没返回合法 JSON
- 模型可能返回空 step
- 本地模型可能输出带 `<think>` 或代码块

所以 planner 有 deterministic fallback：

\[
\text{instruction} \to \text{ordered color mentions} \to \text{hover-only plan}
\]

见 [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L151)

这部分非常适合拿来给学生讲“系统鲁棒性设计”。

---

## 8. 执行层：为什么 executor 必须是状态机

这里对应：

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py)

### 8.1 如果没有状态机，会出什么问题

如果 plan 一出来就直接逐条发 prompt，系统会有两个大问题：

1. 旧目标的感知结果可能还残留
2. 新目标还没确认，tracking 就已经开始追了

所以 executor 当前是一个明确的 step state machine：

- `execution_loaded`
- `step_started`
- `step_waiting_target`
- `step_target_confirmed`
- `execution_complete`

### 8.2 它的核心算法是什么

对于每个 `hover_target` step：

1. 发布新的 `/sam3/prompt`
2. 关闭 tracking
3. 等待感知返回新的有效 keypoint
4. 满足 `target_reacquire_delay_sec`
5. 再打开 tracking
6. 检查当前末端与 hover target 的距离
7. 进入 `success_radius` 后，再满足 `dwell_sec`
8. 然后进入下一步

这就是典型的**事件驱动状态机 + 几何完成判定**。

关键代码：

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L210)
- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L231)

### 8.3 这一层为什么重要

它说明：

- LLM 给的是目标序列
- 不是执行细节
- 真正让系统闭环的是 executor 的状态机逻辑

---

## 9. 低层控制层：为什么还需要 IK 与连续性约束

这里对应：

- [iiwa_keypoint_tracker_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_keypoint_tracker_node.py)
- [ik_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/ik_utils.py)

### 9.1 输入是什么

tracker 读的是：

- 当前 3D keypoint
- 当前 joint state
- tracking enabled 开关

### 9.2 核心做法

它先把目标从 camera frame 变到 `world`，然后加上悬停偏置：

\[
p_{hover} = p_{world} + [0, 0, \Delta z]
\]

然后调用 IK：

\[
q_{des} = IK(q_{seed}, p_{hover}, R_{ee})
\]

这里 `q_seed` 用当前关节角，目的是让解更连续。  
如果保持初始末端姿态，就只允许平移不允许随意翻腕。

最后再做关节步长裁剪：

\[
q_{cmd} = q_{now} + \text{clip}(q_{des} - q_{now}, -\Delta_{max}, \Delta_{max})
\]

这一步对应代码：

- [iiwa_keypoint_tracker_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_keypoint_tracker_node.py#L11)
- [iiwa_keypoint_tracker_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_keypoint_tracker_node.py#L172)
- [ik_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/ik_utils.py#L32)

### 9.3 为什么大模型不直接做这一步

因为这一步是：

- 高频连续控制
- 强几何约束
- 对稳定性非常敏感

传统 IK + 连续性裁剪在这里更合适，也更安全。

---

## 10. 单卡资源调度：这也是系统算法的一部分

这里对应：

- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh)

很多学生容易把“资源调度”当成和算法无关的运维问题，但在机器人系统里不是这样。

当前单卡机器上：

- 本地 `Qwen3` 会占掉大部分显存
- `SAM3` 也需要 GPU

所以系统设计了 `staged_single_gpu`：

```text
先停 SAM3
-> 起 Qwen3 做规划
-> 规划完成后停 Qwen3
-> 再起 SAM3 做感知与执行
```

这说明一个非常现实的问题：

“系统算法不仅是模型结构和公式，还包括在资源约束下怎样维持闭环运行。”

---

## 11. 讲到这里时，你可以怎样总结

建议用下面这段收束：

“如果只从模块名看，这个系统像是在把 Qwen3 接到机械臂上；但从算法上看，它实际上串起了 5 种不同范式：  
第一，深度学习做条件分割；  
第二，几何投影和点云聚类做 3D 目标恢复；  
第三，结构化状态表示把连续感知流转换成 world state；  
第四，大模型在约束下做高层离散规划；  
第五，状态机、IK 和连续控制负责把计划变成机器人行为。  
所以这个系统真正值得学的地方，不是某一个模型，而是这些算法层之间的表示转换与衔接方式。”  

---

## 12. 这份讲义最适合怎么用

建议你这样用：

1. 先用它讲 15 到 20 分钟的算法导论
2. 再配合 `core_code_walkthrough.md` 进入代码
3. 最后让学生按 `student_reproduction_manual.md` 亲手跑系统

最推荐的配套材料顺序是：

1. `docs/teaching/algorithm_focused_lecture_notes.md`
2. `docs/teaching/one_page_architecture_talk.md`
3. `docs/teaching/core_code_walkthrough.md`
4. `docs/teaching/student_reproduction_manual.md`
