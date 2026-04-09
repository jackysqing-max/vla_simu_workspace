# Qwen3 本地部署与 ROS2/VLA 联调记录

这份文档整理了本工作区把 `Qwen3` 接入当前 `ROS2 + PyBullet + 感知/执行` 链路的完整过程。目标不是只让模型“能启动”，而是让它真正成为本地可调用的大模型推理模块，并已经验证到 `llm_task_planner_node` 可以通过本地 Qwen3 生成任务计划。

如果你想先看“这个版本相对 GitHub 归档版到底新增了什么、pipeline 是怎么变化的”，先看：

- `docs/v0.1.4_release_notes.md`

本记录基于 `2026-04-08` 这次实际联调结果。

## 1. 这次工作达成了什么

当前已经完成：

- 本地部署 `Qwen/Qwen3-4B`
- 用 `vLLM` 暴露 OpenAI-compatible 接口
- 让 `llm_task_planner_node` 同时兼容：
  - OpenAI `responses`
  - 本地 OpenAI-compatible `chat_completions`
- 为本地 Qwen3 增加独立的 ROS2 `launch` 和参数文件
- 完成一次真实联调：
  - `curl -> /v1/chat/completions` 成功
  - `ROS2 planner -> Qwen3 -> /llm_task/plan_json` 成功

一句话概括当前状态：

`Qwen3` 已经可以作为当前系统里的“本地任务规划大模型”使用。

## 2. 当前系统里 Qwen3 的位置

当前链路可以理解成：

```text
用户指令
  -> /llm_task/instruction
  -> llm_task_planner_node
  -> 本地 Qwen3(vLLM)
  -> /llm_task/plan_json
  -> llm_task_executor_node
  -> 感知提示/目标跟踪/机械臂执行
```

和场景相关的输入是：

```text
scene_object_registry_node
  -> /scene/objects_json
  -> llm_task_planner_node
```

所以当前 Qwen3 承担的是“高层语言规划器”角色，还不是端到端视觉控制策略。

## 3. 代码里改了哪些地方

这次接入涉及的关键文件如下。

### 3.1 Planner 节点

`src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py`

核心改动：

- 新增双后端模式：
  - `api_protocol=responses`
  - `api_protocol=chat_completions`
- 新增本地部署需要的参数：
  - `api_key`
  - `api_key_env_var`
  - `api_key_required`
  - `temperature`
  - `top_p`
  - `max_output_tokens`
  - `extra_request_body_json`
- 增强对本地模型返回结果的处理：
  - 提取 `chat.completions`
  - 剥离代码块包裹
  - 剥离 `<think>...</think>`
  - 从文本中稳健抽出 JSON

### 3.2 ROS2 启动与参数

新增：

- `src/pybullet_ros2_sim/launch/llm_task_planner_qwen3_vllm.launch.py`
- `src/pybullet_ros2_sim/config/llm_task_planner_qwen3_vllm.yaml`

这让本地 Qwen3 方案可以单独启动，不影响原先走 OpenAI 的配置。

### 3.3 vLLM 启动脚本

新增：

- `scripts/start_qwen3_vllm.sh`

这个脚本把本地模型服务的关键默认值固定下来，包括：

- 默认模型：`Qwen/Qwen3-4B`
- 默认端口：`8000`
- 默认 `gpu_memory_utilization=0.93`
- 默认 `max_model_len=32768`
- 默认 `HF_HUB_DISABLE_XET=1`

### 3.4 打包与安装

修改：

- `src/pybullet_ros2_sim/setup.py`
- `src/pybullet_ros2_sim/package.xml`

目的：

- 把 `launch/*.launch.py` 和 `config/*.yaml` 安装到 ROS 包里
- 补齐 `launch`、`launch_ros` 依赖

## 4. 本次验证环境

这次实际联调使用的是下面这套本机环境。

- 工作区：`/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws`
- Python 虚拟环境：`/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm`
- 模型服务：`vllm==0.19.0`
- Python OpenAI SDK：`openai==2.30.0`
- 模型：`Qwen/Qwen3-4B`
- GPU：`NVIDIA GeForce RTX 5080 Laptop GPU`
- 显卡驱动：`590.48.01`

## 5. 端到端部署流程

下面这套流程是已经跑通的最小可用路径。

### 5.1 准备 Python 环境

```bash
python3 -m venv /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm
source /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm/bin/activate
python -m pip install --upgrade pip
pip install "vllm==0.19.0" "openai==2.30.0"
```

### 5.2 启动本地 Qwen3

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm/bin/activate
bash scripts/start_qwen3_vllm.sh
```

当前脚本默认等价于：

```bash
QWEN_MODEL=Qwen/Qwen3-4B
QWEN3_HOST=0.0.0.0
QWEN3_PORT=8000
QWEN3_TENSOR_PARALLEL_SIZE=1
QWEN3_GPU_MEMORY_UTILIZATION=0.93
QWEN3_MAX_MODEL_LEN=32768
HF_HUB_DISABLE_XET=1
```

如果你想覆盖默认值，可以直接这样写：

```bash
QWEN_MODEL=Qwen/Qwen3-8B \
QWEN3_PORT=8001 \
QWEN3_GPU_MEMORY_UTILIZATION=0.92 \
QWEN3_MAX_MODEL_LEN=24576 \
bash scripts/start_qwen3_vllm.sh
```

### 5.3 构建 ROS2 包

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
colcon build --packages-select pybullet_ros2_sim
source install/setup.bash
```

### 5.4 启动 planner

```bash
ros2 launch pybullet_ros2_sim llm_task_planner_qwen3_vllm.launch.py
```

这个 launch 会读取：

- `src/pybullet_ros2_sim/config/llm_task_planner_qwen3_vllm.yaml`

当前参数文件里的关键设置是：

- `api_protocol: "chat_completions"`
- `api_base_url: "http://127.0.0.1:8000/v1/chat/completions"`
- `model: "Qwen/Qwen3-4B"`
- `api_key_required: false`
- `temperature: 0.7`
- `top_p: 0.8`
- `max_output_tokens: 512`
- `extra_request_body_json: '{"top_k": 20, "chat_template_kwargs": {"enable_thinking": false}}'`

如果你希望直接在仿真里一条命令把“仿真 + 感知 + 本地 Qwen3 planner + executor”整条链路拉起来，现在也可以直接用：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
LLM_BACKEND=qwen3_local ./start_llm_rekep_demo.sh start
```

这个脚本会：

- 启动 PyBullet RGB-D 仿真
- 启动 SAM3、fusion、scene registry、tracker、monitor
- 检查本地 `Qwen3` 服务是否已经 ready
- 如果本地服务未启动，则自动拉起 `scripts/start_qwen3_vllm.sh`
- 用本地 `chat_completions` 参数启动 `llm_task_planner_node`
- 启动 `llm_task_executor_node`

如果你只有一张显卡，并且发现 `Qwen3 + SAM3` 不能同时常驻，可以改用单卡分阶段模式：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh start
LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh task "依次移动到红色方块、蓝色方块和黄色方块上方"
```

这个模式的行为是：

- `start` 只启动基础仿真、fusion、registry、tracker、planner、executor
- 不让 `Qwen3` 和 `SAM3` 同时占 GPU
- `task` 时自动执行：
  - 先停掉 `SAM3`
  - 启动 `Qwen3` 做规划
  - 等 planner 发布计划
  - 停掉 `Qwen3`
  - 再启动 `SAM3` 做感知与执行

注意：

- staged 模式更适合用 `task` 单次指令，不适合 `shell` 交互模式
- 如果 `127.0.0.1:8000` 上已经有一个“脚本外部启动”的 Qwen3 服务在跑，staged 模式不会自动杀掉它；要先把那个外部服务停掉，脚本才能真正完成 GPU 交接

如果你是在纯终端环境、远程桌面环境，或者 X11 会话不稳定，也建议把 PyBullet 仿真切到无头模式，避免出现：

- `X connection to :1 broken`
- `iiwa_pybullet_rgbd_sim_node` 直接退出

对应启动方式：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
SIM_GUI=false LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu ./start_llm_rekep_demo.sh start
```

### 5.5 发送一条测试指令

```bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
ros2 topic pub --once /llm_task/instruction std_msgs/msg/String "{data: 'hover above the red cube then wait for 2 seconds'}"
```

### 5.6 查看 planner 输出

```bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
ros2 topic echo /llm_task/plan_json --once --field data
```

本次真实联调拿到的结果是：

```json
{"task_summary":"Hover above the red cube and wait for 2 seconds.","planning_notes":"No scene objects have been observed yet, so the red cube may not be visible. The robot can only hover above visible objects.","steps":[{"step_index":1,"action":"hover_target","target_prompt":"red cube","description":"Hover above the red cube.","success_radius_m":0.1,"dwell_sec":0.5,"wait_sec":0.0},{"step_index":2,"action":"wait","target_prompt":"","description":"Wait for 2 seconds.","success_radius_m":0.0,"dwell_sec":0.0,"wait_sec":2.0}]}
```

这说明本地 Qwen3 已经成功参与任务规划，而不是只把服务空跑起来。

## 6. 本次排查过的关键问题

下面这些坑都是这次实际遇到并解决掉的，后面基本可以少踩一遍。

### 6.1 `vllm` 启动参数和旧文档不完全一致

一开始尝试使用 `--enable-reasoning`，但 `vllm 0.19.0` 的 CLI 不接受这个参数。

最后采用的是：

```bash
--reasoning-parser qwen3
--structured-outputs-config.enable_in_reasoning=True
```

### 6.2 Hugging Face 权重下载卡住

现象：

- 模型分片下载长时间停在 `0.00/xxGB`
- 或者访问 `xet` 链路时超时

处理方式：

- 在脚本里默认设置 `HF_HUB_DISABLE_XET=1`

这不是 Qwen3 模型本身的问题，而是当前网络环境下 Hugging Face 的 Xet 下载链路不稳定。

### 6.3 显存足够加载模型，但不够默认 40960 上下文长度

最关键的一次失败出现在 KV cache 初始化阶段。

现象：

- 模型权重已经下载完成
- 模型也能加载进显存
- 但在 `max seq len = 40960` 时，KV cache 略超可用显存，服务启动失败

最后采用的稳定方案：

- `QWEN3_GPU_MEMORY_UTILIZATION=0.93`
- `QWEN3_MAX_MODEL_LEN=32768`

这组默认值已经写进启动脚本。

### 6.4 为什么 `Qwen3` 和 `SAM3` 会在单卡上抢爆显存

这是这次联调里最关键的工程现实。

在 `2026-04-09` 这台机器上，宿主机 `nvidia-smi` 看到的是：

- 显卡总显存：`16303 MiB`
- 当本地 `Qwen3` vLLM 服务运行时：
  - 已用显存：`15477 MiB`
  - 剩余显存：`366 MiB`
  - 主要计算进程：`VLLM::EngineCore`
  - 该进程占用：`15454 MiB`

也就是说，Qwen3 一旦常驻，这张卡几乎已经被占满。

从 vLLM 自己的启动日志看，显存主要花在这几块：

- 模型权重加载：约 `7.56 GiB`
- KV cache：约 `6.14 GiB`
- CUDA graph pool：约 `0.39 GiB`
- 再加上 PyTorch、allocator、运行时碎片和其他开销

这些加起来后，实际进程显存就接近 `15.1 GiB` 了。

而 `SAM3` 在 CUDA 上启动时，还需要继续把自己的视觉模型搬进同一张卡。我们在 `sam3.log` 里实际看到的失败是：

- `torch.OutOfMemoryError`
- 只是在再申请 `20 MiB` 时就失败了
- 错误时 GPU 剩余显存只有几 MiB 到几百 MiB

所以这里不是“配置差一点”，而是资源模型本身冲突：

- `Qwen3` 4B + vLLM 想长期占住大部分显存
- `SAM3` 也想在同一张卡上常驻
- 对单张约 `16 GiB` 的 GPU 来说，两者同时在线基本不可持续

这也是为什么需要单卡分阶段模式。

### 6.5 沙箱里的 `127.0.0.1` 不等于宿主机服务

在开发环境里，沙箱内的 `curl http://127.0.0.1:8000` 可能访问不到宿主机上的 vLLM 服务。

所以真正的服务验证需要在宿主机侧执行：

- `curl http://127.0.0.1:8000/health`
- `curl http://127.0.0.1:8000/v1/models`
- `curl http://127.0.0.1:8000/v1/chat/completions`

## 7. 这次实际通过了哪些验证

### 7.1 模型服务健康检查

```bash
curl -s http://127.0.0.1:8000/health
curl -s http://127.0.0.1:8000/v1/models
```

确认点：

- 服务 ready
- 模型列表里能看到 `Qwen/Qwen3-4B`

### 7.2 OpenAI-compatible 接口验证

实际发送了一条最小请求：

```bash
curl -s -X POST http://127.0.0.1:8000/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{"model":"Qwen/Qwen3-4B","messages":[{"role":"user","content":"Return JSON only: {\"status\":\"ok\"}"}],"temperature":0.0,"max_tokens":64,"chat_template_kwargs":{"enable_thinking":false}}'
```

返回结果是：

```json
{"status":"ok"}
```

### 7.3 ROS2 planner 联调验证

已经确认：

- `ros2 launch pybullet_ros2_sim llm_task_planner_qwen3_vllm.launch.py` 能正常启动
- planner 收到 `/llm_task/instruction` 后会调用本地 `/v1/chat/completions`
- planner 成功向 `/llm_task/plan_json` 发布结构化计划

## 8. 日常使用的最短命令集

如果你后面只想快速重启整套链路，按下面顺序就够了。

### 8.1 启动模型服务

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm/bin/activate
bash scripts/start_qwen3_vllm.sh
```

### 8.2 启动 planner

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source install/setup.bash
ros2 launch pybullet_ros2_sim llm_task_planner_qwen3_vllm.launch.py
```

### 8.3 发指令

```bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
ros2 topic pub --once /llm_task/instruction std_msgs/msg/String "{data: 'hover above the blue cube then wait for 1 second'}"
```

### 8.4 看结果

```bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
ros2 topic echo /llm_task/plan_json --once --field data
```

## 9. 当前边界和现实限制

现在这套链路已经能工作，但还要明确它当前的能力边界。

- Qwen3 目前是语言规划器，不直接做视觉编码
- 当前 planner 支持的动作仍然很少，主要是：
  - `hover_target`
  - `wait`
- 规划对场景理解仍依赖已有感知链路，而不是模型直接看图
- 还没有完整的失败恢复和闭环重规划

所以现在更准确的定位是：

“本地大模型已经成功接入现有 VLA 框架的高层规划层。”

而不是：

“已经变成了端到端统一 VLA 模型。”

## 10. 下一步建议

后续推进建议按照下面顺序：

1. 先把 `scene/objects_json -> planner -> executor` 这条闭环跑稳定。
2. 再扩充动作集合，让 planner 输出真正更像机器人技能序列。
3. 最后再推进多模态输入、失败恢复、分层 VLA。

更完整的架构演进计划，见：

- `docs/qwen3_vla_integration_roadmap.md`
