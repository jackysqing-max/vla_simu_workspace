# 学生实验复现手册

这份手册面向“有一定 AI 基础，但第一次系统化复现机器人+大模型集成项目”的学生。

推荐复现版本：

- branch: `release/v0.1.4`
- tag: `v0.1.4`

本手册的目标不是一次性把全部原理讲完，而是让你按实验步骤，把系统真正跑起来，并且知道每一步在验证什么。

## 1. 你将复现到什么程度

完成本手册后，你应该能做到：

1. 构建当前工作区并理解主要模块分工
2. 跑通仿真和感知链路
3. 跑通本地 `Qwen3` 作为 planner 后端
4. 在单张 GPU 上通过 staged 模式完成一条完整任务：
   - 自然语言
   - 计划生成
   - 目标切换
   - 感知定位
   - 机械臂执行

## 2. 推荐学习顺序

不要一上来就跑完整链路。  
推荐按下面顺序走：

1. 先看系统结构
2. 再跑仿真和感知
3. 再跑 planner
4. 最后把本地 Qwen3 接进完整闭环

## 3. 复现前先读哪 3 份文档

先读：

- `README.md`
- `docs/v0.1.4_release_notes.md`
- `docs/qwen3_local_deployment.md`

如果你只想快速知道“系统为什么这么设计”，再看：

- `docs/qwen3_vla_integration_roadmap.md`

## 4. 环境准备

### 4.1 系统环境

推荐：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- 可用 NVIDIA GPU

### 4.2 工作区

```bash
cd ~/ros2_workspaces/humble
git clone git@github.com:jackysqing-max/vla_simu_workspace.git ros2_pybullet_ws
cd ros2_pybullet_ws
git switch release/v0.1.4
```

如果你更偏向稳定归档版本，也可以直接：

```bash
git checkout v0.1.4
```

### 4.3 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspaces/humble/ros2_pybullet_ws
colcon build --packages-select pybullet_ros2_sim
source install/setup.bash
```

验证：

```bash
ros2 pkg executables pybullet_ros2_sim
```

你应该能看到：

- `llm_task_planner_node`
- `llm_task_executor_node`
- `scene_object_registry_node`

## 5. 实验 A：先理解系统结构

### 目标

先知道系统里最重要的 5 个文件。

### 需要阅读的文件

- `start_llm_rekep_demo.sh`
- `src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py`
- `src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py`
- `src/pybullet_ros2_sim/pybullet_ros2_sim/scene_object_registry_node.py`
- `src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py`

### 你要搞清楚的事

1. planner 输入和输出是什么
2. executor 输入和输出是什么
3. perception 如何把目标变成 3D keypoint
4. `scene_object_registry_node` 为什么存在
5. staged single GPU 模式为什么存在

## 6. 实验 B：跑通仿真和感知链路

### 目标

不依赖 LLM，先确认机器人、相机、语义分割和跟踪链路能工作。

### 启动

```bash
cd ~/ros2_workspaces/humble/ros2_pybullet_ws
./start_semantic_tracking_demo.sh start
```

如果你处在无图形会话，改成：

```bash
SIM_GUI=false ./start_semantic_tracking_demo.sh start
```

### 验证点

检查进程状态：

```bash
./start_semantic_tracking_demo.sh status
```

检查关键 topic：

```bash
source install/setup.bash
ros2 topic list | rg "sam3|perception|iiwa7|sim/camera"
```

切换目标：

```bash
./start_semantic_tracking_demo.sh prompt "red cube"
./start_semantic_tracking_demo.sh prompt "blue cube"
```

### 这一实验在验证什么

它验证的是：

- PyBullet 仿真正常
- 相机图像正常
- SAM3 能根据 prompt 产出 mask
- `mask_depth_fusion_node` 能生成 3D keypoint
- tracker 能根据 keypoint 驱动机械臂

如果这一层不通，不要继续跑 LLM。

## 7. 实验 C：只验证 planner，不做完整执行

### 目标

确认 `planner` 能把自然语言转成结构化 `plan_json`。

### 方法 1：用 OpenAI 后端

```bash
cd ~/ros2_workspaces/humble/ros2_pybullet_ws
OPENAI_API_KEY=... ./start_llm_rekep_demo.sh start
./start_llm_rekep_demo.sh task "依次移动到红色方块、蓝色方块和黄色方块上方"
```

### 方法 2：用本地 Qwen3 后端

先起本地模型服务：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_workspaces/humble/ros2_pybullet_ws
source /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm/bin/activate
bash scripts/start_qwen3_vllm.sh
```

然后启动 planner：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
ros2 launch pybullet_ros2_sim llm_task_planner_qwen3_vllm.launch.py
```

发送指令：

```bash
ros2 topic pub --once /llm_task/instruction std_msgs/msg/String "{data: '依次移动到红色方块、蓝色方块和黄色方块上方'}"
```

看输出：

```bash
ros2 topic echo /llm_task/plan_json --once --field data
```

### 这一实验在验证什么

它验证的是：

- 模型服务能否被 planner 调用
- planner 是否能输出结构化 JSON
- target normalization 和 fallback 是否正常

## 8. 实验 D：完整复现本地 Qwen3 单卡闭环

### 目标

在单卡机器上跑通最稳的完整路径。

### 推荐命令

```bash
cd ~/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_rekep_demo.sh stop
SIM_GUI=false LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh start
SIM_GUI=false LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh task "依次移动到红色方块、蓝色方块和黄色方块上方"
```

### 为什么推荐这组参数

- `SIM_GUI=false`
  - 避免远程桌面或 X11 不稳定导致 PyBullet GUI 退出
- `LLM_BACKEND=qwen3_local`
  - 使用本地部署的 Qwen3
- `GPU_EXECUTION_MODE=staged_single_gpu`
  - 让 `Qwen3` 和 `SAM3` 轮流占 GPU，避免显存爆掉

### 关键验证命令

检查整体状态：

```bash
./start_llm_rekep_demo.sh status
```

看 planner/executor 状态：

```bash
source install/setup.bash
ros2 topic echo /llm_task/status --once
```

看 scene registry：

```bash
ros2 topic echo /scene/objects_json --once
```

看当前计划：

```bash
ros2 topic echo /llm_task/plan_json --once
```

### 这一实验在验证什么

它验证的是整个闭环：

- 自然语言
- planner
- executor
- 感知锁定
- 跟踪控制
- 机械臂执行

## 9. 最常见的问题

### 问题 1：机械臂不动

先检查：

```bash
./start_llm_rekep_demo.sh status
ros2 topic echo /llm_task/status --once
```

常见原因：

- planner 没出 plan
- `/scene/objects_json` 为空
- SAM3 没锁定当前目标
- tracker 还没被 executor 放开

### 问题 2：没有 PyBullet 窗口

如果你用了：

```bash
SIM_GUI=false
```

那就说明你启动的是无头仿真。  
这是正常行为，不是 bug。

### 问题 3：本地 Qwen3 和 SAM3 同时跑会爆显存

这是当前系统最现实的资源约束。  
单张 GPU 下优先使用：

```bash
GPU_EXECUTION_MODE=staged_single_gpu
```

### 问题 4：planner 返回 `planning_failed: no_steps`

当前版本已经加了 fallback，但你仍然应该检查：

- 指令里是否提到了受支持目标
- 本地模型是否返回了异常 JSON
- planner 日志里有没有解析错误

## 10. 学生实验报告建议回答的问题

建议每位学生最后回答下面 5 个问题：

1. 这个系统为什么采用分层架构，而不是端到端控制？
2. `Qwen3` 在这个系统里负责什么，不负责什么？
3. `scene_object_registry_node` 的作用是什么？
4. 为什么单卡机器需要 `staged_single_gpu`？
5. 如果要把动作从 `hover_target` 扩展到 `grasp/place`，你会优先改哪几个模块？

## 11. 复现成功的最低标准

如果你最终能做到下面 4 件事，就算完成了本实验：

1. 成功构建 `release/v0.1.4`
2. 成功启动仿真与感知链路
3. 成功让 planner 输出 `plan_json`
4. 成功让机械臂在仿真中对语言任务做出可观察动作

## 12. 建议的进一步扩展

复现完成后，可以尝试做下面几类扩展实验：

1. 修改 planner prompt，看计划是否更稳定
2. 调整 `target_reacquire_delay_sec`，观察速度与稳定性的 tradeoff
3. 增加新的动作 schema，比如 `move_above`、`inspect`
4. 扩展 `scene_object_registry_node` 的世界状态字段
5. 评估更大模型或更小模型对规划质量和速度的影响
