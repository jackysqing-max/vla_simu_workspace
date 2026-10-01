# VLA + RGB-D + SAM3 + Qwen3 项目分享稿

这份分享面向刚加入、准备继续做同类工作的同学。目标不是把所有代码细节一次讲完，而是帮助新同学快速理解：这个项目在做什么、系统怎么跑起来、每个模块负责什么、遇到问题应该从哪里排查。

建议分享时长：30-45 分钟。

分享重点：

- 如何部署和启动 SAM3。
- VLA 在本项目中具体怎么从 prompt 走到任务规划和执行。
- ROS2 在这个系统里到底承担什么角色。
- 本地 Qwen3 是怎么部署、怎么被 Python 代码调用的。
- Qwen3 和 SAM3 如何组成一条分步调用 GPU 资源的系统。
- 作为 Python 程序员，应该如何读这些节点代码。

在正式讲之前，先澄清几个最容易混淆的点：

第一，当前机器上的目录是：

```text
/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm
```

这里是 `.venvs`，不是 `.venv`。工作区里确实存在这个目录，所以如果有人没找到，通常只是少看了一个 `s`。

第二，本地部署和 HTTP 服务并不冲突。Qwen3 的模型权重、推理显存和服务进程都在你自己的机器上，这叫本地部署；它之所以还要提供一个本地 `http://127.0.0.1:8000/...` 接口，只是因为这样最方便被 ROS2 planner 节点调用。

第三，Qwen3 在当前系统里是真正的 LLM，不是一个“帮 SAM3 改 prompt 的壳”。它负责把自然语言任务变成结构化任务计划，SAM3 只是执行链上的视觉分割模块。

## 1. 一句话介绍

这个项目是在 ROS2 Humble 里搭建一条“语言任务 -> 大模型规划 -> 视觉关键点感知 -> 机器人执行/监控”的 VLA 原型链路。

目前项目分成两条主线：

- 仿真链路：PyBullet + iiwa + SAM3/ReKep 风格关键点 + 本地 Qwen3 任务规划。
- 真机视觉链路：RealSense RGB-D + SAM3 prompt 分割 + 深度点云聚类 + 实时关键点显示与 RobotMonitor 监控。

## 2. 为什么要做这件事

传统机器人任务通常需要手写状态机、手写目标点、手写规则。这个项目想验证一条更灵活的路径：

```text
自然语言任务
  -> LLM/VLA planner 生成步骤
  -> prompt 驱动 SAM3 找目标
  -> RGB-D 融合得到 3D keypoint / candidates
  -> tracker / executor 控制机器人靠近目标
  -> monitor 实时显示状态
```

当前系统还不是端到端 policy，也不是完整具身智能闭环。它更像一个可拆解、可调试的研究平台：每个模块都能单独看 topic、看日志、看图。

## 3. 当前系统架构

### 3.0 ROS2 在这里起什么作用

这套系统不是一个单体 Python 脚本，而是一组 ROS2 节点共同工作。ROS2 起到的是“进程编排 + 消息总线 + 时间戳同步 + 可视化排障”的作用。

可以把 ROS2 理解成项目里的交通系统：

- topic 是道路：图像、mask、prompt、keypoint、plan 都在 topic 上流动。
- node 是车辆：每个节点只负责一件事，比如 SAM3 分割、Qwen3 规划、深度融合、RobotMonitor 显示。
- launch 是调度员：一次性拉起相机、SAM3、fusion、viewer、monitor 等节点。
- parameter 是配置表：同一个节点可以通过参数切换 topic、模型设备、阈值、滤波强度。
- TF 是坐标系转换系统：把相机坐标下的 keypoint 转成机器人或 world 坐标下的目标点。

为什么不直接写成一个大 Python 文件：

- 图像、深度、大模型和机器人控制频率完全不同，拆成节点更容易并行。
- 一个模块坏了可以单独看日志和 topic，不必重启全部逻辑。
- 真机和仿真可以复用一部分 topic 接口。
- 可视化、监控、控制可以互不阻塞。

一句话：ROS2 让“大模型、视觉、点云、控制、显示”这些不同节奏的模块能稳定地连在一起。

### 3.1 仿真 VLA 主链

```text
用户输入任务
  -> /llm_task/instruction
  -> llm_task_planner_node
  -> OpenAI 或 本地 Qwen3(vLLM)
  -> /llm_task/plan_json
  -> llm_task_executor_node
  -> /sam3/prompt
  -> SAM3 mask
  -> mask_depth_fusion_node
  -> /perception/keypoint_3d
  -> iiwa_keypoint_tracker_node
  -> robot monitor / executor status
```

核心脚本：

- `start_llm_rekep_demo.sh`
- `start_llm_rekep_demo.local.sh`
- `scripts/start_qwen3_vllm.sh`

核心节点：

- `pybullet_ros2_sim/llm_task_planner_node.py`
- `pybullet_ros2_sim/llm_task_executor_node.py`
- `sam3_ros/sam3_mask_node.py`
- `perception_geometry/mask_depth_fusion_node.py`
- `pybullet_ros2_sim/iiwa_keypoint_tracker_node.py`

### 3.2 真机 RGB-D 关键点链路

```text
RealSense D456
  -> /camera/camera/color/image_raw
  -> /camera/camera/aligned_depth_to_color/image_raw
  -> /camera/camera/color/camera_info
  -> sam3_mask_node(prompt)
  -> /sam3/mask, /sam3/score
  -> mask_depth_fusion_node
  -> /perception/keypoint_3d
  -> /perception/keypoint_candidates
  -> /perception/keypoint_candidates_text
  -> tracking_overlay_viewer_node
  -> robot_monitor
```

核心 launch：

- `src/vla_rgbd_tools/launch/real_rgbd_keypoint_pipeline.launch.py`

当前窗口会显示：

- `SAM3 Mask`：prompt 目标的分割结果。
- `Masked 3D Point Cloud`：mask 内点云区域、轮廓、深度颜色、范围统计。
- `SAM3 Keypoint`：关键点叠加、prompt、score、3D 坐标。
- `RobotMonitor`：机器人状态图和 `Keypoint XYZ / m` 曲线。

## 4. Qwen3 是怎么部署的

本项目里的 Qwen3 是本地 vLLM 服务，不是 ROS 节点里直接加载模型。

默认位置：

```text
/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm
```

默认模型：

```text
Qwen/Qwen3-4B
```

默认服务：

```text
http://127.0.0.1:8000/v1/chat/completions
```

启动脚本：

```bash
bash scripts/start_qwen3_vllm.sh
```

项目里更推荐通过总启动脚本管理：

```bash
LLM_BACKEND=qwen3_local ./start_llm_rekep_demo.sh start
```

单卡显存紧张时，使用分阶段模式：

```bash
LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh start
```

这个模式的含义是：规划阶段让 Qwen3 占 GPU，规划完成后释放 Qwen3，再让 SAM3 占 GPU 做视觉执行，避免 Qwen3 和 SAM3 同时抢显存。

### 4.1 Qwen3 的本地部署步骤

部署思路是：单独创建 Python 虚拟环境，在里面安装 vLLM，然后把 Qwen3 暴露成一个 OpenAI-compatible HTTP 服务。

环境位置：

```bash
/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm
```

典型安装方式：

```bash
python3 -m venv /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm
source /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm/bin/activate
pip install "vllm==0.19.0" "openai==2.30.0"
```

启动脚本 `scripts/start_qwen3_vllm.sh` 里最重要的逻辑是：

```bash
exec "${VLLM_BIN}" serve "${MODEL}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --tensor-parallel-size "${TENSOR_PARALLEL_SIZE}" \
  --gpu-memory-utilization "${GPU_MEMORY_UTILIZATION}" \
  --max-model-len "${MAX_MODEL_LEN}" \
  --reasoning-parser qwen3 \
  --structured-outputs-config.enable_in_reasoning=True
```

这段的含义：

- `vllm serve` 把 Qwen3 启成 HTTP 服务。
- `--host` 和 `--port` 决定服务地址，默认是 `127.0.0.1:8000` 或 `0.0.0.0:8000`。
- `--gpu-memory-utilization` 控制 vLLM 可以使用多少显存。
- `--max-model-len` 控制上下文长度。
- `--reasoning-parser qwen3` 是为了兼容 Qwen3 的 reasoning 输出格式。

验证服务：

```bash
curl -s http://127.0.0.1:8000/health
curl -s http://127.0.0.1:8000/v1/models
```

如果这两个能正常返回，说明 Qwen3 服务已经在本地运行。

### 4.1.1 `vLLM` 是什么，为什么要装它

`vLLM` 不是模型本身，而是一个大模型推理引擎。可以把它理解成“专门负责高效运行 Qwen3 的服务器程序”。

为什么不用我们自己在 Python 节点里直接 `from transformers import AutoModelForCausalLM` 然后 `generate()`？

因为对当前项目来说，`vLLM` 有几个很实际的好处：

- 它本来就是为大模型推理优化的，显存利用率和吞吐比手写脚本更稳定。
- 它能把模型单独跑成一个独立进程，和 ROS2 节点解耦。
- 它提供 OpenAI-compatible API，planner 侧不需要因为“后端从 OpenAI 换到本地 Qwen3”而重写整套调用逻辑。
- 它更方便做健康检查、日志查看、端口管理和 staged GPU 调度。

所以这里安装 `vLLM`，不是因为它“更像 ChatGPT”，而是因为它更像一个成熟的本地 LLM 推理服务底座。

### 4.1.2 为什么本地部署还要把 Qwen3 启成 HTTP 服务

这是最常见的疑问之一。

“本地部署”描述的是模型跑在哪里：

- 模型文件在你机器上。
- 推理显存在你机器上。
- 服务进程在你机器上。

“HTTP 服务”描述的是别的程序怎么调用它：

- planner 节点不直接 import 模型。
- planner 节点通过 `http://127.0.0.1:8000/v1/chat/completions` 发请求。
- 请求虽然是 HTTP，但地址是本机回环地址 `127.0.0.1`，并没有把数据发去云端。

为什么这么设计：

- 规划器和模型进程解耦，planner 崩了不一定要重启模型，模型崩了也不一定拖死 ROS2。
- 本地 Qwen3 和 OpenAI 后端都能走同一套消息结构。
- 更容易做 staged GPU：需要规划时启 Qwen3，不需要时停 Qwen3。
- 更容易单独测试模型服务，而不用先把整条 ROS2 链路拉起来。

所以这里的 HTTP 只是“进程间通信协议”，不是“云端调用”的同义词。

### 4.1.3 OpenAI-compatible API 是不是为了无缝衔接 ChatGPT

更准确地说，是为了无缝衔接“OpenAI 风格的程序接口”，不是为了接 `chat.openai.com` 那个产品界面。

当前 planner 代码本来就支持两种后端：

- 真正的 OpenAI API
- 本地 Qwen3 的 OpenAI-compatible API

这意味着对 planner 而言，调用逻辑几乎可以保持一致：

```text
构造 messages
  -> POST 到 API
  -> 解析返回文本
  -> 提取 JSON plan
```

这样切换模型时，我们不必重写 planner 的业务逻辑。

### 4.2 Python planner 是怎么调用 Qwen3 的

关键代码在：

```text
src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py
```

作为 Python 程序员，可以按这个顺序读：

1. `LlmTaskPlanner.__init__()`：声明参数、创建 publisher/subscriber。
2. `on_instruction()`：收到 `/llm_task/instruction` 后开始规划。
3. `_plan_with_fallback()`：优先请求大模型，失败后走本地规则 fallback。
4. `_request_plan()`：真正发 HTTP 请求给 OpenAI 或本地 Qwen3。
5. `_build_chat_completions_payload()`：构造给 Qwen3 的 messages。
6. `_extract_json_object()`：从模型回复里提取 JSON。
7. `sanitize_plan()`：把模型输出清洗成系统能执行的 plan。
8. `self.pub_plan.publish(out)`：发布到 `/llm_task/plan_json`。

调用链可以画成：

```text
/llm_task/instruction
  -> on_instruction()
  -> _plan_with_fallback()
  -> _request_plan()
  -> urllib.request.urlopen(http://127.0.0.1:8000/v1/chat/completions)
  -> _extract_chat_completion_text()
  -> _extract_json_object()
  -> sanitize_plan()
  -> /llm_task/plan_json
```

Qwen3 接收到的不是一条裸 prompt，而是一组 messages：

```text
system: 你是桌面机器人任务规划器，只能 hover 或 wait
user: 当前场景摘要
user: 用户自然语言任务
```

这就是为什么模型会输出结构化任务计划，而不是随便聊天。

### 4.2.1 现在它是不是“真正的 LLM”

是的，它现在是一个真正的大语言模型，只是它在当前系统里的职责比较窄。

它当前负责的事情是：

- 理解自然语言任务。
- 结合场景摘要决定目标顺序。
- 输出可执行计划。

它当前不负责的事情是：

- 直接看图像做端到端控制。
- 直接输出关节角。
- 直接替代 SAM3 做视觉定位。

所以准确说法应该是：

```text
Qwen3 是当前系统里的高层任务规划 LLM
SAM3 是当前系统里的视觉 grounding / segmentation 模块
```

二者不是替代关系，而是串联关系。

### 4.3 prompt 到任务规划的例子

用户输入：

```text
依次移动到红色方块、蓝色方块和黄色方块上方
```

planner 期望得到类似：

```json
{
  "task_summary": "hover above red, blue, and yellow cubes in sequence",
  "steps": [
    {
      "step_index": 0,
      "action": "hover_target",
      "target_prompt": "red cube",
      "description": "Hover above the red cube",
      "success_radius_m": 0.06,
      "dwell_sec": 0.5
    },
    {
      "step_index": 1,
      "action": "hover_target",
      "target_prompt": "blue cube",
      "description": "Hover above the blue cube",
      "success_radius_m": 0.06,
      "dwell_sec": 0.5
    },
    {
      "step_index": 2,
      "action": "hover_target",
      "target_prompt": "yellow cube",
      "description": "Hover above the yellow cube",
      "success_radius_m": 0.06,
      "dwell_sec": 0.5
    }
  ],
  "planning_notes": ""
}
```

这个 JSON 发布到：

```text
/llm_task/plan_json
```

然后 executor 才开始逐步执行。

### 4.4 executor 如何把 plan 变成 SAM3 prompt

关键代码在：

```text
src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_executor_node.py
```

阅读顺序：

1. `on_plan()`：收到 `/llm_task/plan_json`，加载任务步骤。
2. `on_timer()`：高频循环推进当前 step。
3. `_publish_prompt(step["target_prompt"])`：把当前目标发到 `/sam3/prompt`。
4. 等 `/perception/valid` 和 `/perception/keypoint_3d`。
5. 用 TF 把 keypoint 转到目标坐标系。
6. 加上 `hover_offset_z`，得到机器人要悬停的位置。
7. 判断末端距离是否进入 `success_radius_m`。
8. 当前 step 满足后 `_advance_step()` 进入下一步。

所以 executor 不是直接控制 SAM3，它只是在每一步切换 SAM3 的文字目标：

```text
step 0 target_prompt="red cube"    -> /sam3/prompt
step 1 target_prompt="blue cube"   -> /sam3/prompt
step 2 target_prompt="yellow cube" -> /sam3/prompt
```

SAM3 负责找图像里的目标，fusion 负责把目标变成 3D keypoint，tracker/控制器负责让机器人运动。

### 4.5 Qwen3 和 SAM3 如何分步调用显卡资源

Qwen3 和 SAM3 都很吃 GPU，但它们并不需要一直同时运行。

单卡分阶段模式的思想：

```text
阶段 A：规划
  -> 启动 Qwen3(vLLM)
  -> 用户任务发给 planner
  -> Qwen3 输出 /llm_task/plan_json
  -> 停掉 Qwen3，释放 GPU

阶段 B：视觉执行
  -> 启动 SAM3
  -> executor 按 plan 逐步发布 /sam3/prompt
  -> SAM3 逐步输出 mask
  -> fusion 输出 keypoint
  -> tracker 执行
```

为什么这样做：

- Qwen3 只在“生成任务计划”的短时间内需要 GPU。
- SAM3 在视觉执行阶段持续需要 GPU。
- 单卡上让两者同时常驻，容易显存不足或速度变慢。

对应启动方式：

```bash
LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh start
```

对应执行任务：

```bash
LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu \
  ./start_llm_rekep_demo.sh task "依次移动到红色方块、蓝色方块和黄色方块上方"
```

可以把它理解成一个“GPU 接力棒”：

```text
Qwen3 拿 GPU -> 生成 plan -> 交还 GPU
SAM3 拿 GPU -> 按 plan 做视觉分割 -> 输出 keypoint
```

### 4.5.1 顺序跑和同时跑的区别

当前系统里，`Qwen3` 和 `SAM3` 有两种使用方式。

第一种是同时常驻：

```text
Qwen3 常驻 GPU
SAM3 也常驻 GPU
```

优点：

- 随时可以重新规划。
- 执行过程中如果想临时插入新任务，不需要重启模型。
- 交互体验更像“一个一直在线的智能体系统”。

缺点：

- 非常吃显存。
- 单卡上容易互相抢资源。
- 一旦两边都占大显存，推理速度会掉，甚至启动失败。

第二种是顺序分阶段：

```text
先跑 Qwen3 做规划
再停掉 Qwen3
再跑 SAM3 做执行视觉
```

优点：

- 更适合单卡机器。
- 显存更可控。
- 对当前这个“先规划、再执行”的任务形态很合适。

缺点：

- 执行过程中不适合频繁在线重规划。
- 系统更像“两阶段流水线”，而不是全时在线 agent。

结合你当前系统，我会更建议新同学这样理解：

- 如果任务是桌面方块序列导航，这种“先规划再执行”的 staged 结构很合适。
- 如果未来任务需要边看边问、边执行边改计划，同时常驻会更自然，但对硬件要求更高。

## 5. 最快启动流程

### 5.1 编译工作区

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5.2 启动本地 Qwen3 仿真 VLA demo

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
LLM_BACKEND=qwen3_local ./start_llm_rekep_demo.sh start
```

检查状态：

```bash
./start_llm_rekep_demo.sh status
```

发送任务：

```bash
./start_llm_rekep_demo.sh task "依次移动到红色方块、蓝色方块和黄色方块上方"
```

停止：

```bash
./start_llm_rekep_demo.sh stop
```

### 5.3 启动 RealSense 真机关键点 demo

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vla_rgbd_tools real_rgbd_keypoint_pipeline.launch.py \
  realsense_serial_no:=337122300149 \
  sam3_device:=cuda \
  prompt:=cup
```

如果关键点仍然偏跳，可以更稳一点：

```bash
ros2 launch vla_rgbd_tools real_rgbd_keypoint_pipeline.launch.py \
  realsense_serial_no:=337122300149 \
  sam3_device:=cuda \
  prompt:=cup \
  candidate_lock_radius_m:=0.04 \
  keypoint_filter_alpha:=0.10 \
  keypoint_jump_reset_m:=0.04
```

注意：不要默认使用 CPU 跑 SAM3，CPU 会非常慢。真机 demo 默认应该优先使用：

```text
sam3_device:=cuda
```

### 5.4 VLA 的具体流程是什么

这里的 VLA 不是“一个模型直接看图控制机器人”，而是一个分模块的 VLA-like pipeline。它把 Vision、Language、Action 拆成几个可调试节点。

### 阶段 1：Language，用户输入任务

用户输入自然语言：

```text
依次移动到红色方块、蓝色方块和黄色方块上方
```

这个任务会发布到：

```text
/llm_task/instruction
```

在 Python 里，`llm_task_planner_node.py` 订阅这个 topic：

```python
self.sub_instruction = self.create_subscription(
    String,
    self.instruction_topic,
    self.on_instruction,
    10,
)
```

意思是：只要 `/llm_task/instruction` 上出现新消息，就调用 `on_instruction()`。

### 阶段 2：Planning，Qwen3 生成结构化任务计划

`on_instruction()` 收到文本后，会调用本地 Qwen3 服务，把自然语言变成 JSON plan。

核心结果发布到：

```text
/llm_task/plan_json
```

这个 plan 不是自由文本，而是结构化步骤：

```text
step 0: hover above red cube
step 1: hover above blue cube
step 2: hover above yellow cube
```

这一步属于 VLA 里的 Language -> Action abstraction。Qwen3 不直接输出关节角，也不直接看图，而是输出“下一步应该找哪个目标、做什么动作”。

### 阶段 3：Execution，executor 按步骤切换 SAM3 prompt

`llm_task_executor_node.py` 订阅：

```text
/llm_task/plan_json
```

收到 plan 后，它不会一次把所有目标都发给 SAM3，而是一次只执行一个 step。

例如：

```text
当前 step = red cube
  -> 发布 /sam3/prompt = "red cube"
  -> 等待 /perception/keypoint_3d 有效
  -> 机器人移动到 keypoint 上方
  -> 满足 dwell 时间
  -> 进入下一步 blue cube
```

所以 `/sam3/prompt` 是连接大模型规划和视觉分割的桥。

### 阶段 4：Vision，SAM3 根据 prompt 找目标

SAM3 收到：

```text
/sam3/prompt = "red cube"
```

同时读取 RGB 图像：

```text
/camera/camera/color/image_raw
```

然后输出：

```text
/sam3/mask
/sam3/score
```

这一步属于 Vision：语言 prompt 被转换成图像上的目标区域。

### 阶段 5：RGB-D 融合，mask 变成 3D keypoint

`mask_depth_fusion_node.py` 同时读取：

```text
/sam3/mask
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

然后把 mask 内的深度点投影成 3D 点云，再聚类得到候选关键点。

输出：

```text
/perception/keypoint_3d
/perception/keypoint_candidates
/perception/valid
```

这一步把 2D segmentation 变成机器人能用的 3D 空间目标。

### 阶段 6：Action，tracker 或 executor 判断是否完成动作

机器人控制侧使用：

```text
/perception/keypoint_3d
```

把它转换到目标坐标系，加上悬停高度：

```text
hover_target = keypoint_world + [0, 0, hover_offset_z]
```

然后判断末端是否到达目标附近：

```text
distance(end_effector, hover_target) <= success_radius_m
```

如果满足，就进入下一个 task step。

### 这条链路的完整理解

```text
用户自然语言
  -> Qwen3 生成 JSON plan
  -> executor 逐步发布 target_prompt
  -> SAM3 根据 prompt 输出 mask
  -> RGB-D fusion 输出 3D keypoint
  -> tracker 控制机器人接近 keypoint
  -> executor 判断当前 step 完成
  -> 进入下一步
```

所以本项目中的 VLA 是“分层式 VLA”：

- V：SAM3 + RGB-D + keypoint fusion。
- L：Qwen3 语言规划。
- A：executor + tracker + robot monitor。

## 6. 关键模块说明

### 6.1 `sam3_ros`

作用：把 RGB 图像和文字 prompt 输入 SAM3，输出 mask 和 score。

主要 topic：

- 输入：相机 RGB 图像。
- 输入/更新：`/sam3/prompt`
- 输出：`/sam3/mask`
- 输出：`/sam3/score`

容易出问题的地方：

- prompt 没发到 `/sam3/prompt`。
- CUDA 不可用导致退回 CPU，速度明显变慢。
- mask 本身在背景和目标之间跳，后续 keypoint 也一定会跳。

### 6.1.1 SAM3 如何部署

SAM3 没有直接装在 ROS2 的系统 Python 里，而是放在独立虚拟环境中，避免和 ROS2 自带 Python 包冲突。

默认虚拟环境：

```text
/home/siqin/venvs/ros_vla
```

节点启动时默认使用：

```text
/home/siqin/venvs/ros_vla/bin/python
```

在真机 launch 里对应参数是：

```text
sam3_python:=/home/siqin/venvs/ros_vla/bin/python
sam3_device:=cuda
```

代码里真正加载模型的位置在：

```python
self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
self.processor = Sam3Processor.from_pretrained("facebook/sam3")
self.model.eval()
```

这几行说明：

- `Sam3Model.from_pretrained("facebook/sam3")` 会从本地缓存或 Hugging Face 加载 SAM3 权重。
- `.to(self.device)` 决定模型放在 `cuda` 还是 `cpu`。
- `Sam3Processor` 负责把 PIL 图像和文字 prompt 转成模型输入，也负责把模型输出后处理成 mask。
- `model.eval()` 表示推理模式，不做训练。

如果你要重新部署 SAM3，一般流程是：

```bash
python3 -m venv /home/siqin/venvs/ros_vla
source /home/siqin/venvs/ros_vla/bin/activate
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install transformers pillow numpy opencv-python
```

然后回到工作区编译 ROS2 包：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sam3_ros vla_rgbd_tools
source install/setup.bash
```

注意：具体 PyTorch CUDA 版本要和机器驱动匹配。验证 CUDA：

```bash
/home/siqin/venvs/ros_vla/bin/python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

如果输出 `False`，SAM3 会退回 CPU，速度会非常慢。

### 6.1.2 SAM3 Python 节点如何工作

核心文件：

```text
src/sam3_ros/sam3_ros/sam3_mask_node.py
```

作为 Python 程序员，先看 `Sam3MaskNode.__init__()`：

```python
self.declare_parameter("image_topic", "/image_raw")
self.declare_parameter("prompt", "glasses")
self.declare_parameter("device", "cpu")
self.declare_parameter("prompt_topic", "/sam3/prompt")
```

这说明节点的行为不是写死的，而是通过 ROS2 parameter 配置。

再看订阅和发布：

```python
self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
self.sub_prompt = self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
self.pub_mask = self.create_publisher(Image, "/sam3/mask", 1)
self.pub_score = self.create_publisher(Float32, "/sam3/score", 1)
```

这说明：

- 它订阅相机 RGB 图像。
- 它订阅 `/sam3/prompt` 来动态改变文字目标。
- 它发布 `/sam3/mask` 和 `/sam3/score`。

图像回调 `on_image()` 没有直接跑模型，而是只保留最新一帧：

```python
if self.q.full():
    self.q.get_nowait()
self.q.put_nowait((rgb, msg.header))
```

为什么这样设计：

- SAM3 推理比较慢。
- 相机帧率比推理帧率高。
- 如果每帧都排队，会处理过期图像。
- 所以队列只保留最新帧，避免延迟越积越大。

真正推理在后台线程 `infer_loop()`：

```text
最新 RGB 图像
  -> resize_keep_aspect()
  -> PIL Image
  -> processor(images=..., text=self.prompt)
  -> model(**inputs)
  -> post_process_instance_segmentation()
  -> 选最高分 mask
  -> 发布 /sam3/mask 和 /sam3/score
```

所以 SAM3 节点本质上是一个“低频异步视觉推理 worker”。

### 6.1.3 单独测试 SAM3

如果不想启动整条系统，可以单独跑 SAM3 节点：

```bash
source /opt/ros/humble/setup.bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
/home/siqin/venvs/ros_vla/bin/python -m sam3_ros.sam3_mask_node --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p prompt:=cup \
  -p device:=cuda \
  -p infer_hz:=2.0 \
  -p max_side:=640
```

另一个终端看输出：

```bash
ros2 topic echo /sam3/score --once
ros2 topic hz /sam3/mask
```

动态切换 prompt：

```bash
ros2 topic pub --once /sam3/prompt std_msgs/msg/String "{data: 'bottle'}"
```

如果 `/sam3/score` 有值、`/sam3/mask` 有频率，就说明 SAM3 部署基本成功。

### 6.2 `perception_geometry`

作用：把 SAM3 mask、RGB、aligned depth 和 camera_info 融合成 3D keypoint。

主要输出：

- `/perception/keypoint_3d`
- `/perception/keypoint_px`
- `/perception/keypoint_candidates`
- `/perception/keypoint_candidates_text`
- `/perception/masked_points`
- `/perception/valid`
- `/perception/keypoint_overlay`

当前算法思路：

```text
mask 内有效深度点
  -> 投影成 XYZ 点云
  -> 结合 RGB 特征
  -> PCA 压缩
  -> k-means 聚类
  -> mean-shift 合并候选点
  -> 历史最近邻锁定 candidate
  -> keypoint 低通滤波和跳变抑制
```

为什么 keypoint 会跳：

- RealSense 深度有噪声和空洞。
- SAM3 mask 边缘会逐帧变化。
- 如果 mask 跳到背景或其他物体，点云会彻底变。
- 如果使用 camera optical frame 做 top-surface，会把相机深度方向误当作“上表面”方向。

现在真机 launch 默认：

```text
use_top_surface_estimator:=false
```

也就是说真机优先输出稳定候选点，而不是错误地在 camera optical frame 里找 top surface。

### 6.3 `vla_rgbd_tools`

作用：提供真机 RGB-D pipeline launch 和实时可视化窗口。

核心文件：

- `real_rgbd_keypoint_pipeline.launch.py`
- `tracking_overlay_viewer_node.py`

窗口说明：

- `SAM3 Mask` 看分割是否稳定。
- `Masked 3D Point Cloud` 看点云区域、深度范围、目标轮廓。
- `SAM3 Keypoint` 看最终关键点是否有效、是否 stale。

排查建议：如果 keypoint 不稳定，先看 mask 窗口。如果 mask 自己不稳定，不要先怪聚类。

### 6.4 `robot_monitor`

作用：显示机器人状态和感知关键点。

当前新增了：

- 底部状态栏显示 prompt、keypoint、candidate summary。
- 终端每秒打印 keypoint 状态。
- 图上新增 `Keypoint XYZ / m` 面板，显示 x/y/z 三条曲线。

如果 RobotMonitor 图上没有 keypoint 曲线，通常说明：

- `/perception/keypoint_3d` 没有发布有限值。
- `/perception/valid` 一直是 false。
- keypoint stale。
- 没有重启新构建后的 RobotMonitor。

## 7. 常用 topic 排查

查看 topic：

```bash
ros2 topic list -t
```

看 prompt：

```bash
ros2 topic echo /sam3/prompt --once
```

看感知是否有效：

```bash
ros2 topic echo /perception/valid --once
```

看最终 3D keypoint：

```bash
ros2 topic echo /perception/keypoint_3d --once
```

看前几个候选点：

```bash
ros2 topic echo /perception/keypoint_candidates_text --once
```

看 mask 是否发布：

```bash
ros2 topic hz /sam3/mask
```

看点云是否发布：

```bash
ros2 topic hz /perception/masked_points
```

## 8. 常见问题和处理

### 8.1 Qwen3 没启动

现象：

```text
[--] qwen3 not running
```

处理：

```bash
curl -s http://127.0.0.1:8000/health
curl -s http://127.0.0.1:8000/v1/models
```

如果没服务，使用：

```bash
LLM_BACKEND=qwen3_local ./start_llm_rekep_demo.sh start
```

单卡建议：

```bash
LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh start
```

### 8.2 NVIDIA 驱动和内核脱节

现象：

- `nvidia-smi` 不可用。
- CUDA 不可用。
- SAM3/Qwen3 无法上 GPU。

常见原因：

- 系统更新了 Linux kernel，但 NVIDIA DKMS 模块没给新内核编译成功。
- Secure Boot 阻止了 NVIDIA 内核模块加载。
- 驱动装过，但当前启动的 kernel 不是当时编译模块的 kernel。

处理思路：

- 先确认 `nvidia-smi`。
- 再确认当前 kernel：`uname -r`。
- 检查 DKMS 状态：`dkms status`。
- 如果是 Secure Boot，确认是否需要关闭或重新签名模块。

### 8.3 SAM3 mask 正常，但点云窗口没东西

可能原因：

- aligned depth 没有发布。
- camera_info 没有发布。
- mask 和 depth 分辨率不一致。
- 深度范围过滤掉了所有点。

当前 viewer 会直接用：

```text
mask + aligned_depth + camera_info
```

生成显示点云，不再完全依赖 `/perception/masked_points`。

### 8.4 keypoint 大幅跳变

排查顺序：

1. 看 `SAM3 Mask` 是否稳定。
2. 看 `Masked 3D Point Cloud` 是否有明显背景点混进 mask。
3. 看 RobotMonitor 的 `Keypoint XYZ / m` 曲线是否在 invalid 后重新初始化。
4. 调小 `candidate_lock_radius_m` 和 `keypoint_filter_alpha`。

推荐保守参数：

```bash
candidate_lock_radius_m:=0.04
keypoint_filter_alpha:=0.10
keypoint_jump_reset_m:=0.04
keypoint_jump_hold_frames:=20
```

### 8.5 RobotMonitor 没有 keypoint 曲线

先确认是否用了新构建：

```bash
source install/setup.bash
```

再确认 topic：

```bash
ros2 topic echo /perception/keypoint_3d --once
```

如果输出是 NaN 或没有输出，问题在 perception，不在 Monitor。

如果 topic 有有限值但图上没有，重启 RobotMonitor 或整条 launch。

## 9. 给新同学的建议

先不要急着改大模型或控制器。这个项目最容易卡在视觉链路和 topic 时序上。

建议按这个顺序上手：

1. 先跑通 `./start_llm_rekep_demo.sh status`，理解有哪些进程。
2. 再跑真机 RGB-D pipeline，只看 SAM3 mask。
3. mask 稳定后，再看 point cloud。
4. point cloud 正常后，再看 keypoint candidates。
5. keypoint 稳定后，再接 RobotMonitor 和 tracker。
6. 最后再看 Qwen3 planner 和 task executor。

每次只改一个模块，改完立刻用 topic 验证，不要同时改 prompt、SAM3、fusion、monitor 和 planner。

## 9.1 如果目标变成“手术器械传递”，该怎么基于当前系统继续做

这个问题很关键，因为它能帮助新同学理解：当前系统不是最终答案，而是一个起点。

如果把当前系统直接照搬到手术器械传递，问题会很明显：

- 当前 planner 的动作空间过于简单，主要是 `hover_target` 和 `wait`。
- 当前视觉输出主要是单个关键点或少量 candidate，不足以描述器械的柄部、尖端、朝向和安全交接位姿。
- 当前 SAM3 更适合做 prompt 分割，不等于已经具备手术器械级别的鲁棒识别和跟踪。
- 当前 executor 的执行语义更像“移动到目标上方”，距离真正的 handover 还差很多层。

但它依然是一个很好的起点，因为系统骨架已经具备：

- 语言输入
- 任务规划
- prompt 驱动视觉
- 视觉到 3D 空间目标
- 机器人监控与执行

换句话说，当前系统已经把“V、L、A 三层怎么串起来”这件事搭好了。接下来要做的是把每一层的任务语义从“桌面方块”升级到“器械传递”。

### 一个更贴近手术器械传递的路线图

第一步，先把视觉目标从“单个物体中心”升级成“器械语义关键区域”。

例如对一把镊子，不应该只输出一个 keypoint，而至少要区分：

- 柄部可抓取区
- 尖端工作区
- 安全交接区
- 器械整体朝向

也就是说，当前的 `/perception/keypoint_3d` 未来更可能扩展成：

```text
handle_keypoint
tip_keypoint
handover_keypoint
instrument_pose
```

第二步，把 planner 的动作空间升级。

当前 planner 更像：

```text
hover red cube
wait
hover blue cube
```

器械传递需要更像：

```text
识别目标器械
确认器械当前是否可见且无遮挡
移动到可抓取区
稳定抓取
移动到交接区
调整朝向
等待对方接收或确认
释放
```

这意味着 Qwen3 未来不应只输出“目标物体名”，而应输出更丰富的 task schema。

第三步，把“结构化步骤”变成“自由文本 reasoning + 结构化执行表示”。

你刚才提到希望把它变成自由文本，这个方向是对的，但在当前系统里不能直接把自由文本喂给 executor。

因为 executor 目前需要的是可执行字段，比如：

- `action`
- `target_prompt`
- `success_radius_m`
- `dwell_sec`

所以更现实的做法不是“放弃结构化”，而是两层输出：

```text
Qwen3 自由文本解释 / reasoning
  + 一份结构化执行计划
```

这样做的好处是：

- 新同学和研究者能看到模型为什么这么规划。
- 执行器仍然可以拿结构化字段稳定运行。

第四步，引入场景状态和人机协作约束。

手术器械传递不是单纯的物体搬运，它包含：

- 谁在请求器械
- 请求的是哪种器械
- 当前器械是否在托盘上
- 当前是否被遮挡
- 当前是否已经被抓住
- 当前是否靠近病人危险区域
- 当前是否允许释放

这说明未来的 scene registry 不能只保存“objects 列表”，还要保存状态：

```text
instrument_type
visibility
graspable_region
handover_ready
human_request
safety_state
```

第五步，规划与执行之间要加入安全门。

当前桌面 demo 可以容忍一些误检和跳变，但器械传递不能。

结合你当前系统，更稳妥的研究路线应该是：

- 允许 Qwen3 提出“下一步建议”
- 允许视觉模块给出候选器械区域
- 允许规则层或人工确认通过后，才发给执行层

也就是说，未来的系统更可能是：

```text
Qwen3 负责高层语义规划
SAM3 / 视觉负责候选目标定位
规则层负责安全与状态检查
执行层负责真正动作
```

这比“一个模型包打天下”的思路更贴近你现在的系统结构，也更适合真实高风险场景。

## 10. 如何像 Python 程序员一样读这些代码

这套代码看起来节点很多，但 Python ROS2 节点基本都遵循同一个结构。

### 10.1 先看 `__init__()`

每个节点的 `__init__()` 一般回答四个问题：

- 这个节点叫什么。
- 它有哪些参数。
- 它订阅哪些 topic。
- 它发布哪些 topic。

例如 SAM3 节点：

```python
super().__init__("sam3_mask_node")
self.declare_parameter("image_topic", "/image_raw")
self.create_subscription(Image, self.image_topic, self.on_image, qos)
self.pub_mask = self.create_publisher(Image, "/sam3/mask", 1)
```

你只要看懂这几行，就知道这个节点的边界：

```text
输入 image_topic
输出 /sam3/mask
```

### 10.2 再看 callback

ROS2 是事件驱动的。callback 就是“收到消息后做什么”。

常见 callback：

- `on_image()`：收到图像。
- `on_prompt()`：收到 prompt。
- `on_instruction()`：收到用户任务。
- `on_plan()`：收到任务计划。
- `on_keypoint()`：收到关键点。

例如 planner：

```python
def on_instruction(self, msg: String):
    instruction = msg.data.strip()
    plan, used_fallback = self._plan_with_fallback(instruction)
    out.data = plan_to_json(plan)
    self.pub_plan.publish(out)
```

这段就是：

```text
收到自然语言
  -> 生成 plan
  -> 发布 plan_json
```

### 10.3 Timer 是节点自己的循环

有些节点不只是“收到消息才动”，还需要固定频率检查状态。

例如 executor：

```python
self.timer = self.create_timer(1.0 / max(self.control_hz, 1e-6), self.on_timer)
```

这表示 `on_timer()` 会按 `control_hz` 周期执行。

executor 的 `on_timer()` 负责：

- 当前有没有 plan。
- 当前 step 是 wait 还是 hover。
- prompt 是否已经发给 SAM3。
- keypoint 是否有效。
- 机器人是否到达目标。
- 是否进入下一步。

所以读 executor 时，不要只看 callback，要重点看 `on_timer()`。

### 10.4 Worker thread 是为了解决慢模型推理

SAM3 模型推理比相机帧率慢，所以不能在 `on_image()` 里直接推理，否则 ROS callback 会被卡住。

SAM3 节点使用：

```python
self.worker = threading.Thread(target=self.infer_loop, daemon=True)
self.worker.start()
```

设计思路：

```text
on_image() 只负责收最新图像
infer_loop() 低频跑模型
```

这是一种典型 Python 工程写法：把慢任务放到 worker thread，把 callback 做短。

### 10.5 读代码时画 topic 图

不要一上来逐行读。先画输入输出：

```text
llm_task_planner_node
  in:  /llm_task/instruction
  in:  /scene/objects_json
  out: /llm_task/plan_json
  out: /llm_task/status
```

```text
sam3_mask_node
  in:  color image
  in:  /sam3/prompt
  out: /sam3/mask
  out: /sam3/score
```

```text
mask_depth_fusion_node
  in:  /sam3/mask
  in:  aligned depth
  in:  camera_info
  out: /perception/keypoint_3d
  out: /perception/valid
```

画完 topic 图，再读函数调用链，效率会高很多。

### 10.6 读这套代码的推荐顺序

1. `sam3_mask_node.py`：最容易理解，输入图像和 prompt，输出 mask。
2. `mask_depth_fusion_node.py`：理解 mask 如何变成 3D keypoint。
3. `tracking_overlay_viewer_node.py`：理解显示窗口如何订阅和画图。
4. `llm_task_planner_node.py`：理解 Qwen3 如何被 HTTP 调用。
5. `llm_task_executor_node.py`：理解 plan 如何逐步变成 SAM3 prompt。
6. `start_llm_rekep_demo.sh`：理解整条 demo 如何被脚本编排。

读代码时最重要的一句话：

```text
先找 topic，再找 callback，再找 publish。
```

## 11. 当前版本还没有解决的问题

- SAM3 仍然可能因为 prompt 不明确而跳目标。
- ReKep 风格聚类目前用 RGB-D 特征近似，还不是完整视觉 foundation feature。
- 真机关键点仍然需要更强的时序目标身份跟踪。
- Qwen3 是高层 planner，不是端到端视觉动作模型。
- 单卡机器上 Qwen3 和 SAM3 仍然需要谨慎管理显存。

## 12. 推荐分享结构

可以按下面顺序讲：

1. 项目目标：从语言任务到机器人关键点执行。
2. 系统总架构：仿真链路和真机链路。
3. Qwen3 本地部署：为什么用 vLLM，单卡怎么跑。
4. SAM3 + RGB-D：prompt 如何变成 mask 和点云。
5. 关键点算法：mask 内点云、聚类、candidate、稳定器。
6. 可视化和 Monitor：如何确认系统真的在工作。
7. 常见问题：GPU、驱动、mask 跳、keypoint 跳、topic 无输出。
8. 后续工作：更强候选点选择、更稳定跟踪、接真实机器人控制。

最后可以给新同学一句话：

```text
不要把这个系统当黑盒跑。先看 topic，再看窗口，再看日志，最后才改算法。
```
