# 核心代码讲解

这份文档专门面向“已经有一定 AI 基础，准备读代码并复现系统”的学生。

目标不是覆盖整个仓库，而是解释最值得优先阅读的 5 个文件：

1. `start_llm_rekep_demo.sh`
2. `llm_task_planner_node.py`
3. `llm_task_executor_node.py`
4. `scene_object_registry_node.py`
5. `task_plan_utils.py`

建议配合版本：

- branch: `release/v0.1.4`
- commit: `11f5452`

## 1. 为什么先看这 5 个文件

如果学生第一次进这个仓库就去看所有节点，很容易迷路。  
这 5 个文件之所以重要，是因为它们恰好串起了当前系统最关键的 3 个问题：

1. 系统怎么被启动和编排
2. 自然语言怎么变成结构化计划
3. 计划怎么落到感知与执行链路

可以把它们理解成：

- `start_llm_rekep_demo.sh`
  - 系统编排器
- `llm_task_planner_node.py`
  - 高层规划适配器
- `llm_task_executor_node.py`
  - 执行状态机
- `scene_object_registry_node.py`
  - 感知到规划之间的中间层
- `task_plan_utils.py`
  - planner 和 executor 共用的“语言约束层”

---

## 2. `start_llm_rekep_demo.sh`

文件：

- `start_llm_rekep_demo.sh`

### 它解决什么问题

这个脚本不是简单地“依次启动几个节点”，而是整个 demo 的运行时编排器。

它要解决 4 件事：

1. 启动仿真、感知、planner、executor
2. 让 OpenAI 和本地 Qwen3 两种后端都能工作
3. 管理日志和 PID
4. 在单张 GPU 上协调 `Qwen3` 和 `SAM3`

### 学生读代码时先看哪里

先看参数区：

- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L8)

这里定义了系统的重要运行模式：

- `LLM_BACKEND`
- `GPU_EXECUTION_MODE`
- `SIM_GUI`
- `QWEN3_*`
- `SAM3_*`

这段的作用是告诉你：

“这不是一个固定脚本，而是一个支持多模式切换的运行控制器。”

### 再看哪几段

#### 1. 后台启动/停止逻辑

- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L125)
- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L146)
- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L173)
- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L200)

这些函数分别负责：

- 启动普通 ROS 节点
- 启动本地 Qwen3
- 启动 SAM3
- 停止进程

这里的核心思想是：

“整个 demo 被拆成多个可管理的后台进程，每个进程都有独立日志和 pid 文件。”

#### 2. 本地 Qwen3 的准备逻辑

- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L266)
- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L304)
- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L322)

这里回答两个问题：

1. 如果后端是本地 Qwen3，planner 参数怎么切换？
2. 如果 Qwen3 服务没起来，要不要自动启动？

学生读这块时要理解：

“planner 并不知道自己连的是 OpenAI 还是本地 vLLM，脚本在外面把参数文件准备好了。”

#### 3. `start)` 主流程

- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L374)

这是最关键的一段。

它决定：

- 仿真怎么启动
- SAM3 是否立即启动
- registry、tracker、planner、executor 怎么被拉起

这里你会看到当前系统的模块依赖顺序。

#### 4. `task)` 主流程

- [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh#L518)

这是 staged 模式最重要的代码。

它体现的是这条资源调度逻辑：

```text
停掉 SAM3
-> 起 Qwen3
-> 等 planner 出 plan
-> 停掉 Qwen3
-> 再起 SAM3 执行
```

学生读到这里时，应该能理解为什么系统更慢但更稳：

- 不是因为某个 node 算得慢
- 而是因为整个单卡系统需要做资源切换

### 给学生的一句话总结

`start_llm_rekep_demo.sh` 本质上不是“启动脚本”，而是当前系统的运行时调度层。

---

## 3. `llm_task_planner_node.py`

文件：

- `src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py`

### 它解决什么问题

planner 节点的职责是：

- 接收自然语言指令
- 读取场景摘要
- 请求大模型
- 把模型输出转换成结构化 `plan_json`

所以它本质上是一个“大模型适配器”，不是大模型本身。

### 学生读代码时先看哪里

#### 1. 参数定义和节点接口

- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L116)

这里可以先搞清楚它订阅/发布什么：

- 输入：
  - `/llm_task/instruction`
  - `/scene/objects_json`
- 输出：
  - `/llm_task/plan_json`
  - `/llm_task/status`

这是理解 planner 的最短入口。

#### 2. 本地/云端双后端支持

- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L225)
- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L244)
- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L285)

这里要让学生看到：

- `responses` 和 `chat_completions` 两种 payload 结构不一样
- 但最后都会被转换成统一 plan

这是系统设计里非常典型的“上层统一接口，下层适配不同后端”。

#### 3. prompt 组织逻辑

- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L217)
- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L267)

这两段很值得讲，因为它们说明：

- planner 不是只把原始自然语言直接发给模型
- 它还会把 `scene_registry_summary(...)` 拼进去
- 并且对动作空间施加约束

这正是“LLM 接机器人系统”的核心技巧之一：

不是只换模型，而是设计好输入和约束。

#### 4. 本地模型输出清洗

- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L41)
- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L62)

这一段很适合给学生讲“工程上的现实问题”：

- 本地模型不一定老老实实只返回 JSON
- 可能会夹带 `<think>`
- 可能会把 JSON 包在代码块里

所以 planner 先做文本清洗，再做 JSON 提取。

#### 5. fallback 逻辑

- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L314)
- [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py#L336)

这一块要告诉学生：

- 工程系统里，模型失败不是异常，而是常态之一
- 所以要有 deterministic fallback

当前 fallback 做得很简单：

- 从指令里抽颜色顺序
- 直接生成 hover plan

### 给学生的一句话总结

`llm_task_planner_node.py` 是把“不稳定的大模型输出”转换成“机器人能稳定执行的结构化计划”的关键桥接层。

---

## 4. `llm_task_executor_node.py`

文件：

- `src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py`

### 它解决什么问题

很多学生会误以为：

“plan 有了，执行就只是 for-loop 逐条发出去。”

这个文件正好告诉他们：不是。

它做的是一个执行状态机，负责：

- 加载 plan
- 逐步推进 step
- 切换当前语义目标
- 等待感知确认
- 控制 tracking 的开关

### 学生读代码时先看哪里

#### 1. 节点接口

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L54)

先看输入输出：

- 输入：
  - `/llm_task/plan_json`
  - `/iiwa7/joint_states`
  - `/perception/valid`
  - `/perception/keypoint_3d`
- 输出：
  - `/sam3/prompt`
  - `/llm_task/status`
  - `/llm_task/tracking_enabled`

这一步能帮学生建立执行器在全系统中的位置。

#### 2. `on_plan`

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L137)

这里做的事是：

- 读取 plan
- 重置状态机内部变量
- 进入 step 0

学生要注意：

- plan 到 executor 这里已经不是自然语言了
- 它已经是结构化 JSON

#### 3. `on_timer`

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L231)

这是整个 executor 的核心。

建议按这 4 个阶段读：

1. 当前有没有 plan
2. 当前 step 是 `wait` 还是 `hover_target`
3. 当前目标是否已经确认
4. 机械臂是否满足该 step 的完成条件

#### 4. 目标确认逻辑

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L210)
- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L269)

这是当前版本非常值得讲的改动。

它体现的是：

- executor 不会一切换 step 就立刻开 tracking
- 它先发新的 `/sam3/prompt`
- 然后等待感知返回“新的目标已经被重新锁定”
- 然后才放开 tracking

这能避免上一 step 的旧 keypoint 被误用。

这也是学生理解“为什么系统比想象中复杂”的一个好例子。

#### 5. 完成判定

- [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py#L334)

这里做了：

- FK 求当前末端位置
- 计算末端到 hover target 的距离
- 判断是否进入 `success_radius`
- 如果进入，再看 `dwell_sec`

这告诉学生：

- planner 给的是高层目标
- executor 把它翻成了可检查的几何完成条件

### 给学生的一句话总结

`llm_task_executor_node.py` 不是简单执行 plan，而是把 plan 变成“感知确认 + 跟踪使能 + 完成判定”的执行状态机。

---

## 5. `scene_object_registry_node.py`

文件：

- `src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py`

### 它解决什么问题

这是学生最容易忽视、但其实非常关键的中间层。

它解决的问题是：

“感知模块的输出对 planner 来说太底层、太分散了，怎么整理成大模型可以稳定理解的场景状态？”

也就是说，它负责把：

- 当前 prompt
- 当前 score
- 当前 3D keypoint
- 当前 valid 状态

整理成统一的 `/scene/objects_json`。

### 学生读代码时先看哪里

#### 1. 订阅和发布接口

- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py#L55)
- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py#L100)

这一步先搞清楚它接什么、发什么。

#### 2. `_update_active_object`

- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py#L150)

这是最关键的逻辑：

- 从 prompt 推导 canonical label
- 检查目标是否有效且新鲜
- 把 camera frame 下的 keypoint 变换到 `world`
- 更新对象表

这说明 registry 不是盲目缓存数据，而是在做“语义标签 + 几何位置 + 置信度”的融合。

#### 3. `_snapshot_scene`

- [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py#L209)

这里做的事是：

- 计算 `last_seen_age_sec`
- 判断对象是否 `visible`
- 清除过期对象
- 排序后输出整个 scene

学生应该从这里理解：

- `/scene/objects_json` 不是原始检测结果
- 而是“时间上被整理过”的世界状态表

### 为什么它在架构上很重要

这个节点的意义是把 perception 和 planner 解耦。

如果没有它：

- planner 就要直接读 `/perception/keypoint_3d`
- 还要自己理解 prompt、valid、TF、时间衰减
- prompt engineering 会非常混乱

所以它其实是这个系统里最典型的“中间表示层”。

### 给学生的一句话总结

`scene_object_registry_node.py` 把分散的感知结果整理成 planner 可消费的统一世界状态，是语言规划和视觉感知衔接起来的关键中间层。

---

## 6. `task_plan_utils.py`

文件：

- `src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py`

### 它解决什么问题

这是一个“规则层”和“约束层”。

它不直接跑 ROS，但它定义了：

- 什么动作是允许的
- 什么目标是允许的
- 模型输出怎么被清洗
- fallback 计划怎么生成
- scene registry 怎么被解析和摘要

所以它其实是 planner 和 executor 共享的协议层。

### 学生读代码时先看哪里

#### 1. schema 和动作空间

- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L9)
- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L18)

当前动作集合非常小：

- `hover_target`
- `wait`

这一步非常适合给学生讲：

- 为什么小动作集合有利于系统先跑通
- 为什么后续扩展 `grasp/place` 时要先改这里

#### 2. target normalization

- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L57)
- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L78)

这里回答的是：

“红色方块”“red cube”“red block”“红”这些不同写法，系统怎么统一？”

这部分虽然不炫，但在真实系统里非常重要。

#### 3. `sanitize_plan`

- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L99)

这是 planner 后处理的关键逻辑。

它会：

- 修正非法 action
- 规范 `step_index`
- 统一 target 名称
- 清掉无效 hover step

学生应当从这里学到：

- 大模型输出不能直接信
- 结构化后处理是必要步骤

#### 4. `infer_plan_from_instruction`

- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L151)

这个函数是 fallback planner。

它做的事情很简单：

- 从文本里按顺序抽颜色目标
- 构造 hover-only plan

这正好能给学生讲一个工程现实：

- fallback 不一定智能
- 但一定要 deterministic、可解释、可用

#### 5. `scene_registry_summary`

- [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py#L221)

这一步把结构化 scene registry 转成 planner prompt 里的场景摘要文本。

也就是说：

- registry 给的是 JSON
- LLM 吃的是自然语言描述
- 这中间还需要一个“摘要器”

### 给学生的一句话总结

`task_plan_utils.py` 定义了当前系统的语言约束、计划协议和 fallback 规则，是 planner/executor 共享的规则层。

---

## 7. 推荐学生按什么顺序读

最推荐的顺序是：

1. [start_llm_rekep_demo.sh](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/start_llm_rekep_demo.sh)
2. [task_plan_utils.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py)
3. [llm_task_planner_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py)
4. [scene_object_registry_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py)
5. [llm_task_executor_node.py](/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py)

为什么是这个顺序：

- 先知道系统怎么被启动
- 再知道系统允许什么 plan
- 再看 planner 怎么产出 plan
- 再看 scene 怎么组织
- 最后看 executor 怎么把 plan 落地

这样理解成本最低。

## 8. 学生读完后应该能回答的 5 个问题

1. 为什么 planner 不能直接把模型输出原样交给 executor？
2. 为什么需要 `scene_object_registry_node` 这一层？
3. 为什么 executor 在每一步里还要等待“目标确认”？
4. `task_plan_utils.py` 里的 fallback 为什么必要？
5. `start_llm_rekep_demo.sh` 为什么不只是一个简单 launcher？

## 9. 一句话总收束

如果让学生最后只记住一句话，我建议是：

“这 5 个文件分别对应系统的运行编排、计划协议、规划适配、世界状态中间层和执行状态机；把它们读懂，就等于读懂了当前 `v0.1.4` 版本最核心的 VLA pipeline。”
