# LLM + VLM + RCM 腔镜机器人 Demo 完整搭建、修改日志与指令手册

更新时间：2026-06-29

工作区：

```text
/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
```

本文档描述当前工作区中三套相关 Demo：

1. KUKA iiwa 的 RCM、直线和平面虚拟夹具 Demo。
2. SAM3 + RGB-D 的穿孔中心与入孔轴线感知 Demo。
3. Qwen3 任务分解 + SAM3 感知 + 安全入孔 + RCM 圆轨迹的完整 Demo。

本文档以当前代码和实机验证结果为准。文中所有命令默认在 Ubuntu 22.04、
ROS 2 Humble 和 Bash 下执行。

---

## 1. 当前完成状态

当前已经完成并实际验证：

- 将旧 `franka_task.zip` 中的 RCM、直线和平面虚拟夹具思想迁移到当前项目。
- 将原 Franka/libfranka 接口改为 PyBullet KUKA iiwa ROS 2 接口。
- 在 KUKA 末端增加 220 mm 可视化长杆手术器械。
- 加入 dVRK Large Needle Driver 420006 末端模型。
- 加入 `phantom_centered.stl` 腔镜工作空间模型。
- 在 PyBullet 中显示 RCM 点、器械尖端、末端轨迹、孔中心和孔轴。
- 增加沿孔轴安全入孔的状态机，避免直接横向穿过 phantom 表面。
- 使用 SAM3 文本提示 `circular hole` 分割穿孔区域。
- 使用 RGB-D 深度提取三维孔中心。
- 使用 phantom CAD 标定值提供隐藏通道的入孔轴线。
- 对孔中心和轴线进行多帧稳定判断并锁定。
- 使用本地 Qwen3-4B 将自然语言任务拆成四个受约束动作。
- 通过 surgical executor 串行控制感知、对轴、入孔和 RCM 圆轨迹。
- 在单张 16 GB GPU 上使用 Qwen3/SAM3 分阶段显存调度。
- 记录 RCM 误差、末端误差和器械尖端误差到 CSV。

完整链路的最后一次验证结果：

```text
Qwen3 planner status: planned: 4 steps
SAM3 prompt: circular hole
locked port center: (0.7024, 0.0003, 0.4068) m
locked inward axis: (0.335, 0.000, -0.942)
motion end state: RCM_HOLD
task end state: plan_completed
circle duration: approximately 7.0 s
mean RCM error during circle: 0.999 mm
maximum RCM error during circle: 1.712 mm
mean end-effector tracking error: 1.139 mm
maximum end-effector tracking error: 2.066 mm
```

---

## 2. 当前验证环境

当前机器上的已验证配置：

| 项目 | 当前值 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 系统 Python | Python 3.10.12 |
| GPU | NVIDIA GeForce RTX 5080 Laptop GPU |
| GPU 显存 | 16303 MiB |
| NVIDIA 驱动 | 590.48.01 |
| Qwen 模型 | `Qwen/Qwen3-4B` |
| vLLM | 0.19.0 |
| Qwen 环境 PyTorch | 2.10.0+cu128 |
| SAM3 模型 | `facebook/sam3` |
| SAM 环境 Transformers | 5.2.0 |
| SAM 环境 PyTorch | 2.10.0+cu128 |
| NumPy | 1.26.4 |
| PyBullet | 当前系统 Python 中可导入 |

虚拟环境位置：

```text
Qwen3: /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm
SAM3:  /home/siqin/venvs/ros_vla
```

---

## 3. 系统总体架构

完整任务链路：

```text
Natural-language instruction
  -> /llm_task/instruction
  -> llm_task_planner_node
  -> local Qwen3 through vLLM
  -> /llm_task/plan_json
  -> surgical_rcm_task_executor_node
       |
       +-> /sam3/prompt
       |    -> sam3_mask_node
       |    -> /sam3/mask + /sam3/score
       |    -> vlm_port_pose_node
       |    -> locked port center + locked port axis
       |
       +-> /rcm_virtual_fixtures/start
       +-> /rcm_virtual_fixtures/start_pivot
       +-> /rcm_virtual_fixtures/hold_pivot
            -> rcm_virtual_fixture_node
            -> /iiwa7/joint_desired
            -> iiwa_pybullet_sim_node
            -> KUKA iiwa motion in PyBullet
```

RGB-D 感知链路：

```text
iiwa_pybullet_sim_node
  -> /sim/camera/color/image_raw
  -> /sim/camera/aligned_depth_to_color/image_raw
  -> /sim/camera/color/camera_info

sam3_mask_node
  <- /sim/camera/color/image_raw
  <- /sam3/prompt
  -> /sam3/mask
  -> /sam3/score
  -> /sam3/active_prompt

vlm_port_pose_node
  <- SAM3 mask + score
  <- RGB + aligned depth + CameraInfo
  -> /vlm_rcm/port_point_raw
  -> /vlm_rcm/port_axis_raw
  -> /vlm_rcm/locked_port_point
  -> /vlm_rcm/locked_port_axis
  -> /vlm_rcm/port_ready
```

运动控制链路：

```text
rcm_virtual_fixture_node
  <- /iiwa7/joint_states
  <- locked visual port pose
  <- task stage gate commands
  -> constrained Cartesian target
  -> damped PyBullet inverse kinematics
  -> /iiwa7/joint_desired
  -> PyBullet KUKA position control
```

---

## 4. 代码目录和文件职责

### 4.1 新增 RCM package

```text
src/rcm_virtual_fixtures/
├── package.xml
├── setup.py
├── launch/
│   └── rcm_virtual_fixture_demo.launch.py
├── rcm_virtual_fixtures/
│   ├── rcm_virtual_fixture_node.py
│   ├── surgical_rcm_task_executor_node.py
│   └── vlm_port_pose_node.py
├── meshes/
│   ├── phantom_centered.stl
│   └── dvrk_lnd_420006/
└── urdf/
    └── dvrk_lnd_420006_tip.urdf
```

节点职责：

- `rcm_virtual_fixture_node.py`
  - RCM、直线和平面轨迹生成。
  - 安全入孔状态机。
  - PyBullet IK 和关节位置命令。
  - RCM/末端/器械尖端误差计算。
  - CSV 记录。
- `vlm_port_pose_node.py`
  - 接收 SAM3 mask、RGB 和深度图。
  - 估计三维孔中心。
  - 检查孔位与 CAD 预期位置的偏差。
  - 多帧稳定后锁定孔中心和孔轴。
  - 发布图像 overlay。
- `surgical_rcm_task_executor_node.py`
  - 接收 LLM 四步计划。
  - 给 SAM3 发送 prompt。
  - 等待感知锁定。
  - 按步骤启动对轴、入孔和圆轨迹。
  - 一圈后切换到 `RCM_HOLD`。

### 4.2 修改的原 package

```text
src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_pybullet_sim_node.py
src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py
src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py
src/pybullet_ros2_sim/config/llm_task_planner_qwen3_rcm.yaml
```

主要修改：

- `iiwa_pybullet_sim_node.py`
  - 增加 phantom STL 加载。
  - 增加长杆器械和 dVRK 末端可视化。
  - 增加 RCM 点、TIP 点和轨迹点。
  - 增加孔中心青色圆环和孔轴青色箭头。
  - 增加 RGB-D 相机。
  - 增加 `/sim/camera/enabled`，锁定孔位后可关闭 RGB-D 渲染。
- `llm_task_planner_node.py`
  - 增加 RCM 专用 schema 和 system prompt。
  - 增加 `enable_rcm_actions`。
  - `/llm_task/plan_json` 改为 reliable + transient-local。
  - 支持 Qwen3 结构化 JSON 输出。
  - 增加正常的 ROS external shutdown 处理。
- `task_plan_utils.py`
  - 增加四个 RCM 动作。
  - 增加确定性安全排序。
  - 固定插入深度、圆轨迹半径和圈数。
  - 增加无 LLM 时的本地 fallback。
- `llm_task_planner_qwen3_rcm.yaml`
  - Qwen3-4B、256 输出 token、90 秒超时。
  - 关闭 Qwen3 thinking。
  - 启用 RCM 动作。

### 4.3 顶层启动脚本

```text
start_rcm_virtual_fixture_demo.sh
start_vlm_rcm_port_perception_demo.sh
start_llm_vlm_rcm_demo.sh
```

- `start_rcm_virtual_fixture_demo.sh`
  - 独立运行 RCM、line 或 plane 虚拟夹具。
- `start_vlm_rcm_port_perception_demo.sh`
  - 只运行 PyBullet RGB-D、SAM3 和孔位感知。
- `start_llm_vlm_rcm_demo.sh`
  - 运行 Qwen3 规划和完整 VLM-RCM 任务。
  - 自动管理单 GPU 分阶段切换。

---

## 5. 从零搭建环境

如果当前工作区已经可以运行，可跳到第 6 节。

### 5.1 安装 ROS 2 和基础依赖

以下命令假定 ROS 2 Humble 已正确加入 apt 源：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-image-transport
```

首次使用 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经初始化，可忽略该提示。

### 5.2 安装工作区 ROS 依赖

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

### 5.3 安装 PyBullet

当前仿真节点使用系统 Python：

```bash
python3 -m pip install --user --upgrade pybullet
```

验证：

```bash
python3 -c "import pybullet; print('PyBullet import OK')"
```

### 5.4 创建 SAM3 环境

SAM3 环境必须能访问系统 ROS 2 Python 包，因此使用
`--system-site-packages`：

```bash
python3 -m venv --system-site-packages /home/siqin/venvs/ros_vla
source /home/siqin/venvs/ros_vla/bin/activate
python -m pip install --upgrade pip
```

安装当前已验证版本：

```bash
pip install \
  "transformers==5.2.0" \
  "Pillow==12.1.1" \
  "numpy==1.26.4"
```

根据本机 CUDA 安装 PyTorch。当前机器使用 CUDA 12.8 wheel：

```bash
pip install torch torchvision \
  --index-url https://download.pytorch.org/whl/cu128
```

验证：

```bash
python -c \
  "import torch, transformers; print(torch.__version__); print(transformers.__version__); print(torch.cuda.is_available())"
```

预期最后一项为：

```text
True
```

SAM3 首次运行时会从 Hugging Face 下载：

```text
facebook/sam3
```

如果模型需要账号授权：

```bash
pip install huggingface_hub
huggingface-cli login
```

### 5.5 创建 Qwen3/vLLM 环境

Qwen 环境位置：

```bash
python3 -m venv \
  /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm

source \
  /home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm/bin/activate

python -m pip install --upgrade pip
pip install "vllm==0.19.0" "openai==2.30.0"
```

验证：

```bash
vllm --version
python -c "import torch; print(torch.cuda.is_available())"
```

完整 RCM 脚本使用的 Qwen3 参数不是通用 Qwen 脚本中的高显存默认值，而是：

```text
QWEN3_GPU_MEMORY_UTILIZATION=0.60
QWEN3_MAX_MODEL_LEN=4096
QWEN_MODEL=Qwen/Qwen3-4B
QWEN3_PORT=8000
```

这是当前 16 GB GPU 上验证过的配置。

### 5.6 构建工作区

完整构建：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

只构建本 Demo 需要的核心包：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  pybullet_ros2_sim \
  sam3_ros \
  rcm_virtual_fixtures

source install/setup.bash
```

独立 RCM 脚本还会启动 bridge 和 monitor。若要使用这些窗口，建议直接完整构建，
或额外构建：

```bash
colcon build --symlink-install \
  --packages-select \
  iiwa_state_udp_bridge \
  robot_monitor
```

### 5.7 验证 package 和可执行节点

```bash
source /opt/ros/humble/setup.bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash

ros2 pkg prefix rcm_virtual_fixtures
ros2 pkg executables rcm_virtual_fixtures
```

预期包含：

```text
rcm_virtual_fixtures rcm_virtual_fixture_node
rcm_virtual_fixtures surgical_rcm_task_executor_node
rcm_virtual_fixtures vlm_port_pose_node
```

验证 SAM3 ROS package：

```bash
set +u
source /opt/ros/humble/setup.bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
source /home/siqin/venvs/ros_vla/bin/activate
set -u

python -c "import sam3_ros; print(sam3_ros.__file__)"
```

### 5.8 构建和静态检查

本次使用的检查命令：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws

bash -n \
  start_rcm_virtual_fixture_demo.sh \
  start_vlm_rcm_port_perception_demo.sh \
  start_llm_vlm_rcm_demo.sh

python3 -m py_compile \
  src/pybullet_ros2_sim/pybullet_ros2_sim/task_plan_utils.py \
  src/pybullet_ros2_sim/pybullet_ros2_sim/llm_task_planner_node.py \
  src/rcm_virtual_fixtures/rcm_virtual_fixtures/rcm_virtual_fixture_node.py \
  src/rcm_virtual_fixtures/rcm_virtual_fixtures/vlm_port_pose_node.py \
  src/rcm_virtual_fixtures/rcm_virtual_fixtures/surgical_rcm_task_executor_node.py

git diff --check

colcon build --symlink-install \
  --packages-select pybullet_ros2_sim rcm_virtual_fixtures
```

当前结果：

- Bash 语法检查通过。
- Python 编译检查通过。
- `git diff --check` 通过。
- `pybullet_ros2_sim` 和 `rcm_virtual_fixtures` 构建通过。
- 完整真实 Qwen3 和 fallback 两条运行链路均通过。

包级 `colcon test`：

```bash
colcon test \
  --packages-select pybullet_ros2_sim rcm_virtual_fixtures \
  --event-handlers console_direct+
```

当前 `rcm_virtual_fixtures` 没有自动测试。`pybullet_ros2_sim` 的
ament flake8/pep257 会报告仓库既有的代码风格问题；这些问题不影响当前构建和
端到端运行，但在发布下一个版本前应统一清理。

---

## 6. 模型、尺寸和坐标约定

### 6.1 Phantom

模型：

```text
src/rcm_virtual_fixtures/meshes/phantom_centered.stl
```

当前参数：

```text
STL 尺寸: 320 x 260 x 166 mm
STL 原点: 底面中心
顶部开孔中心: local (0, 0, 164) mm
PyBullet scale: 0.001
PyBullet base position: (0.701726, 0.0, 0.240701) m
预期孔中心: (0.701726, 0.0, 0.404701) m
```

注意：STL 使用毫米建模，PyBullet 加载时乘 `0.001` 转换为米。

### 6.2 KUKA 末端器械

当前几何参数：

```text
tool_length_m = 0.220
tool_radius_m = 0.006
rcm_lambda = 0.650
```

对应：

```text
wrist_to_rcm = lambda * tool_length = 0.143 m
rcm_to_tip = (1 - lambda) * tool_length = 0.077 m
```

### 6.3 dVRK Large Needle Driver

URDF：

```text
src/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf
```

网格、来源和许可证：

```text
src/rcm_virtual_fixtures/meshes/dvrk_lnd_420006/
```

该模型和长杆目前是可视化模型，不参与接触和碰撞力计算。

### 6.4 世界坐标

当前系统统一使用：

```text
frame_id = world
单位 = m, rad, s
KUKA 工具轴 = link-7 local +Z
孔轴方向 = 从 phantom 外部指向内部
```

---

## 7. 控制和几何方法

### 7.1 当前控制类型

当前不是力矩控制，也不是零空间阻抗控制。

当前方法是：

```text
任务空间目标
  -> PyBullet damped inverse kinematics
  -> 关节目标 /iiwa7/joint_desired
  -> PyBullet joint position control
```

IK 使用：

- 关节上下限。
- joint damping。
- 上一次 IK 结果作为下一次 seed。
- 初始构型 `q0` 作为 `restPoses`，用于偏置冗余自由度。
- `max_joint_step_rad` 限制每周期关节跳变。

因此当前只有“通过 rest pose 尽量保持原构型”的零空间偏置，不是显式
null-space impedance。

### 7.2 器械尖端

设：

- 末端位置为 \(p_e\)。
- 工具轴单位向量为 \(z\)。
- 工具长度为 \(L\)。

器械尖端：

\[
p_{tip}=p_e+Lz
\]

### 7.3 RCM 点

初始几何模式中：

\[
p_{rcm}=(1-\lambda)p_e+\lambda p_{tip}
\]

视觉融合模式中不重新计算 RCM 点，而是直接使用锁定的孔中心：

\[
p_{rcm}=p_{port}^{locked}
\]

### 7.4 RCM 圆轨迹

令：

- \(d_e=\lambda L\)：RCM 到 KUKA wrist 的距离。
- \(d_t=(1-\lambda)L\)：RCM 到器械尖端的距离。
- \(r\)：尖端圆轨迹半径。
- \(\theta=\arcsin(r/d_t)\)：器械轴摆角。
- \(\omega=v/r\)：圆轨迹角速度。

控制器生成随时间旋转的器械轴 \(z(t)\)，然后计算：

\[
p_{tip}^{d}(t)=p_{rcm}+d_t z(t)
\]

\[
p_e^{d}(t)=p_{rcm}-d_e z(t)
\]

因此：

- 器械总长度保持不变。
- wrist 和 tip 位于同一器械轴线上。
- 器械轴始终穿过固定 RCM 点。

当前完整 Demo：

```text
r = 0.020 m
v = 0.018 m/s
one-circle duration = 2*pi*r/v = approximately 6.98 s
```

### 7.5 RCM 误差

实际器械轴线为：

\[
\ell(s)=p_e+s z
\]

固定孔中心到实际器械轴线的径向距离：

\[
e_{radial}
=
\left\|
(p_{rcm}-p_e)
-
\left[(p_{rcm}-p_e)^Tz\right]z
\right\|
\]

当前发布的总 RCM error 是实际轴线上投影点和期望 RCM 点之间的距离。

### 7.6 直线虚拟夹具

设直线锚点为 \(p_0\)，方向单位向量为 \(d\)：

\[
p_d(t)=p_0+s(t)d
\]

`s(t)` 使用三角波，使末端沿直线往返运动。

### 7.7 平面虚拟夹具

设平面中心为 \(p_0\)，法向为 \(n\)，平面内基向量为 \(b_1,b_2\)：

\[
p_d(t)
=
p_0+r\cos(\omega t)b_1+r\sin(\omega t)b_2
\]

控制器再移除法向分量，保证目标位于约束平面内。

---

## 8. 穿孔视觉识别方法

### 8.1 输入

```text
SAM3 prompt: circular hole
RGB: /sim/camera/color/image_raw
Depth: /sim/camera/aligned_depth_to_color/image_raw
CameraInfo: /sim/camera/color/camera_info
```

### 8.2 孔中心

处理顺序：

1. SAM3 用 `circular hole` 获取语义 mask。
2. 在 mask 周围 annulus 区域采样顶板深度。
3. 使用 RANSAC 拟合 phantom 顶部平面。
4. 在 mask 邻域搜索相对顶板后退至少 25 mm 的深度区域。
5. 提取孔腔连通区域和几何中心。
6. 使用 CameraInfo 将像素和深度反投影到相机坐标。
7. 变换到 `world` 坐标。
8. 检查结果是否在预期孔位置 70 mm 范围内。
9. 连续多帧稳定后锁定。

当前稳定判定：

```text
stable_frames = 3
stability_window = 5
max_center_spread_m = 0.006
max_axis_spread_deg = 6.0
```

### 8.3 孔轴

当前 phantom 的孔道位于表面内部，单个外部 RGB-D 视角不能完整观察隐藏孔道，
因此：

- 孔中心来自 SAM3 + RGB-D。
- 入孔轴来自 phantom CAD 标定。

当前向内轴线：

```text
(0.335067, 0.0, -0.942194)
```

该状态在输出中标记为：

```text
axis_source=calibrated_port_geometry
```

不要在 PPT 中把当前轴线描述为“完全由单目视觉估计”。更准确的描述是：

```text
SAM3/RGB-D 检测孔中心，CAD 先验提供隐藏孔道轴线。
```

### 8.4 可视化颜色

图像 overlay：

- 绿色：SAM3 mask。
- 绿色小十字：mask 质心。
- 青色大十字：深度修正后的三维孔中心投影。
- 青色箭头：入孔方向。

PyBullet：

- 青色圆环：锁定孔中心。
- 青色箭头：锁定入孔轴。
- 红色小球：RCM 点。
- 黄色小球：期望器械尖端。
- 绿色点：器械尖端轨迹。

---

## 9. LLM 任务规划

### 9.1 推荐输入

完整英文任务：

```text
locate the circular laparoscopic port, align the surgical tool with the insertion axis, insert through the port while establishing the RCM constraint, then execute a circular trajectory while keeping the RCM point fixed
```

### 9.2 四步动作

```text
1. localize_rcm_port
2. align_tool_axis
3. establish_rcm
4. execute_rcm_circle
```

### 9.3 最终计划示例

```json
{
  "task_summary": "Locate the port, establish RCM, and execute a circle.",
  "planning_notes": "The port pose must be locked before robot motion.",
  "steps": [
    {
      "step_index": 1,
      "action": "localize_rcm_port",
      "target_prompt": "circular hole",
      "success_radius_m": 0.006,
      "dwell_sec": 0.5,
      "insertion_depth_m": 0.0,
      "trajectory_radius_m": 0.0,
      "trajectory_cycles": 0.0
    },
    {
      "step_index": 2,
      "action": "align_tool_axis",
      "target_prompt": "circular hole",
      "success_radius_m": 0.006,
      "dwell_sec": 0.5,
      "insertion_depth_m": 0.0,
      "trajectory_radius_m": 0.0,
      "trajectory_cycles": 0.0
    },
    {
      "step_index": 3,
      "action": "establish_rcm",
      "target_prompt": "circular hole",
      "success_radius_m": 0.006,
      "dwell_sec": 0.8,
      "insertion_depth_m": 0.077,
      "trajectory_radius_m": 0.0,
      "trajectory_cycles": 0.0
    },
    {
      "step_index": 4,
      "action": "execute_rcm_circle",
      "target_prompt": "",
      "success_radius_m": 0.006,
      "dwell_sec": 0.0,
      "insertion_depth_m": 0.077,
      "trajectory_radius_m": 0.02,
      "trajectory_cycles": 1.0
    }
  ]
}
```

### 9.4 安全层职责

Qwen3 原始输出只需要：

```json
{
  "step_index": 1,
  "action": "localize_rcm_port"
}
```

确定性安全层负责：

- 补齐四步。
- 强制正确顺序。
- 固定 prompt。
- 固定容差。
- 固定插入深度。
- 固定圆半径。
- 固定为一整圈。

LLM 不允许直接输出：

- 关节角。
- 关节力矩。
- 任意末端位姿。
- 无约束自由空间轨迹。

---

## 10. 指令集合：通用环境初始化

每个手动 ROS 2 终端先执行：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查 ROS 环境：

```bash
echo "$ROS_DISTRO"
ros2 node list
ros2 topic list
```

预期：

```text
humble
```

---

## 11. 指令集合：独立虚拟夹具 Demo

### 11.1 RCM 模式

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws

RCM_FIXTURE_MODE=rcm \
./start_rcm_virtual_fixture_demo.sh start
```

### 11.2 直线虚拟夹具

```bash
RCM_FIXTURE_MODE=line \
./start_rcm_virtual_fixture_demo.sh start
```

### 11.3 平面虚拟夹具

```bash
RCM_FIXTURE_MODE=plane \
./start_rcm_virtual_fixture_demo.sh start
```

### 11.4 查看状态

```bash
./start_rcm_virtual_fixture_demo.sh status
./start_rcm_virtual_fixture_demo.sh logs
```

### 11.5 停止

```bash
./start_rcm_virtual_fixture_demo.sh stop
```

### 11.6 OBS 录制入口

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

./recording_scripts/11_rcm_virtual_fixture_recording.sh rcm
./recording_scripts/11_rcm_virtual_fixture_recording.sh line
./recording_scripts/11_rcm_virtual_fixture_recording.sh plane
```

每次只运行一个模式。切换模式前先停止。

### 11.7 常用调参示例

```bash
RCM_SPEED_MPS=0.012 \
RCM_TRAJECTORY_RADIUS_M=0.018 \
RCM_MAX_JOINT_STEP_RAD=0.012 \
RCM_FIXTURE_MODE=rcm \
./start_rcm_virtual_fixture_demo.sh start
```

调整工具：

```bash
RCM_TOOL_LENGTH_M=0.280 \
RCM_TOOL_RADIUS_M=0.008 \
RCM_FIXTURE_MODE=rcm \
./start_rcm_virtual_fixture_demo.sh start
```

注意：`RCM_TOOL_LENGTH_M` 同时传给控制器和可视化模型，不能只改其中一侧。

无 GUI：

```bash
RCM_GUI=false \
RCM_START_MONITOR=false \
./start_rcm_virtual_fixture_demo.sh start
```

不显示 phantom：

```bash
RCM_SHOW_PHANTOM=false \
./start_rcm_virtual_fixture_demo.sh start
```

### 11.8 直接使用 ROS 2 launch

不需要 bridge、monitor 和 CSV 管理时，可以直接启动核心仿真与 controller：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch \
  rcm_virtual_fixtures \
  rcm_virtual_fixture_demo.launch.py \
  fixture_mode:=rcm \
  gui:=true \
  speed_mps:=0.018 \
  trajectory_radius_m:=0.020 \
  tool_length_m:=0.220
```

直线：

```bash
ros2 launch \
  rcm_virtual_fixtures \
  rcm_virtual_fixture_demo.launch.py \
  fixture_mode:=line
```

平面：

```bash
ros2 launch \
  rcm_virtual_fixtures \
  rcm_virtual_fixture_demo.launch.py \
  fixture_mode:=plane
```

---

## 12. 指令集合：仅穿孔感知 Demo

### 12.1 启动

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_vlm_rcm_port_perception_demo.sh start
```

### 12.2 状态和日志

```bash
./start_vlm_rcm_port_perception_demo.sh status
./start_vlm_rcm_port_perception_demo.sh logs
```

### 12.3 查看检测结果

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /vlm_rcm/status
ros2 topic echo /vlm_rcm/locked_port_point
ros2 topic echo /vlm_rcm/locked_port_axis
ros2 topic echo /vlm_rcm/port_ready
ros2 topic echo /sam3/active_prompt
```

### 12.4 修改 SAM3 prompt

```bash
./start_vlm_rcm_port_perception_demo.sh prompt "circular hole"
```

等价 ROS 2 命令：

```bash
ros2 topic pub --once \
  /sam3/prompt \
  std_msgs/msg/String \
  "{data: 'circular hole'}"
```

### 12.5 重新识别和锁定

如果 RGB-D 已被关闭，先重新启用：

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /sim/camera/enabled \
  std_msgs/msg/Bool \
  "{data: true}"
```

清除旧锁定：

```bash
ros2 topic pub --once \
  /vlm_rcm/reset_lock \
  std_msgs/msg/Bool \
  "{data: true}"
```

### 12.6 无窗口模式

```bash
VLM_RCM_GUI=false \
VLM_RCM_DISPLAY_OVERLAY=false \
./start_vlm_rcm_port_perception_demo.sh start
```

### 12.7 停止

```bash
./start_vlm_rcm_port_perception_demo.sh stop
```

---

## 13. 指令集合：完整 Qwen3 + SAM3 + RCM Demo

### 13.1 启动规划阶段

终端 1：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh start
```

脚本会：

1. 启动本地 Qwen3/vLLM。
2. 等待 `http://127.0.0.1:8000/health`。
3. 启动 `llm_task_planner_node`。
4. 启动 `surgical_rcm_task_executor_node`。
5. 暂时不启动 SAM3，避免 GPU OOM。

### 13.2 发送默认任务并自动切换到执行阶段

终端 2：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh task
```

脚本会：

1. 发送默认英文任务。
2. 等待 `/llm_task/plan_json`。
3. 打印格式化后的四步 JSON。
4. 停止 planner 和脚本管理的 Qwen3。
5. 等待显存释放。
6. 启动 PyBullet、RGB-D、SAM3 和孔位估计。
7. 启动 RCM controller。
8. executor 使用内存中保留的计划继续执行。

### 13.3 发送显式任务

```bash
./start_llm_vlm_rcm_demo.sh task \
  "locate the circular laparoscopic port, align the surgical tool with the insertion axis, insert through the port while establishing the RCM constraint, then execute a circular trajectory while keeping the RCM point fixed"
```

### 13.4 查看运行状态

```bash
./start_llm_vlm_rcm_demo.sh status
./start_llm_vlm_rcm_demo.sh logs
```

ROS 2 状态：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /llm_task/status
ros2 topic echo /surgical_rcm/status
ros2 topic echo /surgical_rcm/active_step_json
ros2 topic echo /vlm_rcm/status
ros2 topic echo /rcm_virtual_fixtures/stage
ros2 topic echo /rcm_virtual_fixtures/status
```

任务完成时应看到：

```text
plan_completed: controller holding final RCM pose
```

和：

```text
RCM_HOLD
```

### 13.5 无 Qwen3 的本地 fallback

该模式用于验证除 LLM 推理外的全部链路：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws

LLM_BACKEND=local_fallback \
VLM_RCM_GUI=true \
./start_llm_vlm_rcm_demo.sh start

./start_llm_vlm_rcm_demo.sh task
```

此时 `/llm_task/status` 会显示：

```text
planned_fallback: 4 steps
```

真实 Qwen3 模式应显示：

```text
planned: 4 steps
```

### 13.6 无 GUI 完整测试

因为 staged 模式在 `task` 命令中才启动仿真，所以无 GUI 变量必须传给
`task` 命令：

```bash
VLM_RCM_GUI=false \
VLM_RCM_DISPLAY_OVERLAY=false \
./start_llm_vlm_rcm_demo.sh task
```

### 13.7 停止完整系统

```bash
./start_llm_vlm_rcm_demo.sh stop
```

---

## 14. LLM 输入输出和节点检查

### 14.1 检查 Qwen3 服务

```bash
curl -fsS http://127.0.0.1:8000/health
```

查看模型：

```bash
curl -s http://127.0.0.1:8000/v1/models | python3 -m json.tool
```

### 14.2 检查 ROS 节点

```bash
ros2 node list | sort
```

规划阶段应包含：

```text
/llm_task_planner_node
/surgical_rcm_task_executor_node
```

执行阶段应包含：

```text
/iiwa_pybullet_sim_node
/sam3_mask_node
/vlm_port_pose_node
/rcm_virtual_fixture_node
/surgical_rcm_task_executor_node
```

### 14.3 检查 topic 连接

```bash
ros2 topic info -v /llm_task/instruction
ros2 topic info -v /llm_task/plan_json
ros2 topic info -v /sam3/prompt
ros2 topic info -v /vlm_rcm/locked_port_point
ros2 topic info -v /iiwa7/joint_desired
```

### 14.4 监听 LLM 输入

先运行：

```bash
ros2 topic echo /llm_task/instruction
```

再在另一终端发送任务。输出为：

```yaml
data: locate the circular laparoscopic port, ...
---
```

### 14.5 手动发送给 planner

```bash
ros2 topic pub --once \
  /llm_task/instruction \
  std_msgs/msg/String \
  "{data: 'locate the circular laparoscopic port, align the surgical tool with the insertion axis, insert through the port while establishing the RCM constraint, then execute a circular trajectory while keeping the RCM point fixed'}"
```

这条命令确实是把用户 prompt 发送给 ROS planner。planner 再通过 HTTP 把它发送
给 Qwen3。

注意：在单 GPU staged 模式中，完整任务应优先使用：

```bash
./start_llm_vlm_rcm_demo.sh task
```

因为它还负责 Qwen3/SAM3 的 GPU 交接。单独 `ros2 topic pub` 不会自动完成该交接。

### 14.6 查看完整 plan JSON

推荐：

```bash
ros2 topic echo --once \
  /llm_task/plan_json \
  --field data \
  | sed '/^---$/d' \
  | python3 -m json.tool
```

必须删除 ROS 2 echo 最后的 `---`，否则 `python3 -m json.tool` 会报：

```text
Extra data
```

最稳定的展示方式仍然是：

```bash
./start_llm_vlm_rcm_demo.sh task
```

它会自动捕获并格式化 JSON。

---

## 15. 手动控制 RCM 状态机

正常完整 Demo 不需要手动发布这些 topic。本节只用于调试。

### 15.1 允许 controller 开始对轴和入孔

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /rcm_virtual_fixtures/start \
  std_msgs/msg/Bool \
  "{data: true}"
```

### 15.2 允许开始 RCM 圆轨迹

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /rcm_virtual_fixtures/start_pivot \
  std_msgs/msg/Bool \
  "{data: true}"
```

### 15.3 停止圆轨迹并保持最终位姿

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /rcm_virtual_fixtures/hold_pivot \
  std_msgs/msg/Bool \
  "{data: true}"
```

### 15.4 安全状态顺序

```text
WAITING_FOR_START
  -> PREINSERT_HOLD
  -> APPROACH_PORT
  -> PORT_DWELL
  -> INSERT_THROUGH_PORT
  -> INSERTED_DWELL
  -> WAITING_FOR_PIVOT
  -> RCM_PIVOT
  -> RCM_HOLD
```

状态进入下一阶段需要同时满足：

- 当前阶段最短时间已经到达。
- 实际器械尖端误差进入 6 mm 阈值。
- 连续满足指定 settle cycles。

---

## 16. ROS 2 Topic 总表

### 16.1 LLM

| Topic | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/llm_task/instruction` | `std_msgs/String` | 输入 | 用户自然语言任务 |
| `/llm_task/plan_json` | `std_msgs/String` | 输出 | 受约束任务 JSON |
| `/llm_task/status` | `std_msgs/String` | 输出 | planning/planned/fallback/error |

### 16.2 SAM3 和视觉

| Topic | 类型 | 说明 |
|---|---|---|
| `/sam3/prompt` | `std_msgs/String` | 输入视觉 prompt |
| `/sam3/active_prompt` | `std_msgs/String` | 当前生效 prompt |
| `/sam3/mask` | `sensor_msgs/Image` | SAM3 二值 mask |
| `/sam3/score` | `std_msgs/Float32` | mask score |
| `/vlm_rcm/port_point_raw` | `geometry_msgs/PointStamped` | 未锁定孔中心 |
| `/vlm_rcm/port_axis_raw` | `geometry_msgs/Vector3Stamped` | 未锁定孔轴 |
| `/vlm_rcm/locked_port_point` | `geometry_msgs/PointStamped` | 锁定孔中心 |
| `/vlm_rcm/locked_port_axis` | `geometry_msgs/Vector3Stamped` | 锁定孔轴 |
| `/vlm_rcm/port_valid` | `std_msgs/Bool` | 当前帧是否有效 |
| `/vlm_rcm/port_ready` | `std_msgs/Bool` | 是否已稳定锁定 |
| `/vlm_rcm/status` | `std_msgs/String` | 感知状态 |
| `/vlm_rcm/reset_lock` | `std_msgs/Bool` | 重新锁定 |
| `/vlm_rcm/overlay` | `sensor_msgs/Image` | 可视化 overlay |

### 16.3 Surgical executor

| Topic | 类型 | 说明 |
|---|---|---|
| `/surgical_rcm/status` | `std_msgs/String` | 当前任务执行状态 |
| `/surgical_rcm/active_step_json` | `std_msgs/String` | 当前步骤 |
| `/rcm_virtual_fixtures/start` | `std_msgs/Bool` | 启动对轴/入孔 |
| `/rcm_virtual_fixtures/start_pivot` | `std_msgs/Bool` | 启动圆轨迹 |
| `/rcm_virtual_fixtures/hold_pivot` | `std_msgs/Bool` | 停止并保持 |
| `/sim/camera/enabled` | `std_msgs/Bool` | 开关 RGB-D 渲染 |

### 16.4 RCM controller

| Topic | 类型 | 说明 |
|---|---|---|
| `/iiwa7/joint_states` | `sensor_msgs/JointState` | 实际关节状态 |
| `/iiwa7/joint_desired` | `std_msgs/Float64MultiArray` | 关节位置目标 |
| `/iiwa7/control_mode` | `std_msgs/Int8` | `1` 为位置模式 |
| `/rcm_virtual_fixtures/stage` | `std_msgs/String` | 运动阶段 |
| `/rcm_virtual_fixtures/status` | `std_msgs/String` | 误差状态摘要 |
| `/rcm_virtual_fixtures/metrics` | `std_msgs/Float64MultiArray` | 在线误差 |
| `/rcm_virtual_fixtures/rcm_point` | `geometry_msgs/PointStamped` | 期望 RCM 点 |
| `/rcm_virtual_fixtures/tip_point` | `geometry_msgs/PointStamped` | 期望尖端 |
| `/rcm_virtual_fixtures/locked_port_point` | `geometry_msgs/PointStamped` | controller 冻结的孔中心 |
| `/rcm_virtual_fixtures/locked_port_axis` | `geometry_msgs/Vector3Stamped` | controller 冻结的孔轴 |

---

## 17. 参数集合

### 17.1 完整 LLM 脚本

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `LLM_BACKEND` | `qwen3_local` | `qwen3_local` 或 `local_fallback` |
| `GPU_EXECUTION_MODE` | `staged_single_gpu` | 单 GPU 分阶段运行 |
| `QWEN3_AUTO_START` | `true` | 自动启动 Qwen |
| `QWEN_MODEL` | `Qwen/Qwen3-4B` | 模型 |
| `QWEN3_PORT` | `8000` | vLLM 端口 |
| `QWEN3_GPU_MEMORY_UTILIZATION` | `0.60` | Qwen GPU 占用上限 |
| `QWEN3_MAX_MODEL_LEN` | `4096` | 最大上下文 |
| `RCM_SPEED_MPS` | `0.018` | 圆轨迹线速度 |
| `RCM_TRAJECTORY_RADIUS_M` | `0.020` | 圆轨迹半径 |
| `RCM_TOOL_LENGTH_M` | `0.220` | 工具总长度 |
| `RCM_LAMBDA` | `0.650` | RCM 在工具上的比例 |

### 17.2 感知脚本

| 环境变量 | 默认值 |
|---|---:|
| `SAM3_DEVICE` | `cuda` |
| `SAM3_PROMPT` | `circular hole` |
| `SAM3_INFER_HZ` | `1.0` |
| `SAM3_MAX_SIDE` | `640` |
| `SAM3_SCORE_TH` | `0.05` |
| `SAM3_MASK_TH` | `0.35` |
| `VLM_RCM_GUI` | `true` |
| `VLM_RCM_DISPLAY_OVERLAY` | `true` |
| `VLM_RCM_RGBD_HZ` | `4.0` |
| `VLM_RCM_SHOW_TOOL` | `false` |
| `VLM_RCM_SHOW_DVRK_LND` | `false` |

完整 LLM Demo 会把工具和 dVRK 末端设为可见。

### 17.3 RCM 运动

| 参数 | 完整 Demo 值 | 说明 |
|---|---:|---|
| `publish_hz` | 100 Hz | 控制发布频率 |
| `tool_length_m` | 0.220 m | 工具长度 |
| `rcm_lambda` | 0.650 | RCM 比例 |
| `trajectory_radius_m` | 0.020 m | 尖端圆半径 |
| `speed_mps` | 0.018 m/s | 圆轨迹线速度 |
| `preinsert_clearance_m` | 0.040 m | 孔外预插入距离 |
| `port_standoff_m` | 0.012 m | 孔口外停留距离 |
| `insertion_speed_mps` | 0.018 m/s | 入孔速度 |
| `stage_position_tolerance_m` | 0.006 m | 阶段完成位置阈值 |
| `stage_settle_cycles` | 12 | 连续稳定周期 |
| `max_joint_step_rad` | 0.018 rad | 单周期最大关节变化 |

---

## 18. 误差记录和 CSV

### 18.1 在线 metrics

```bash
ros2 topic echo /rcm_virtual_fixtures/metrics
```

数组顺序：

```text
0  elapsed time
1  RCM total error
2  tool-tip tracking error
3  plane error
4  line error
5  desired end-effector x
6  desired end-effector y
7  desired end-effector z
8  end-effector tracking error
9  RCM radial error
10 RCM signed axial error
```

### 18.2 CSV 文件

独立 RCM Demo：

```text
run_logs/rcm_metrics_YYYYMMDD_HHMMSS.csv
run_logs/rcm_metrics_latest.csv
```

完整 LLM/VLM/RCM Demo：

```text
run_logs/llm_vlm_rcm_metrics.csv
```

CSV 包含：

- ROS time。
- elapsed time。
- 总 RCM error。
- 径向和轴向 RCM error。
- EE tracking error。
- tool-tip tracking error。
- 所有 desired/actual Cartesian position。

### 18.3 快速统计

统计整个 CSV：

```bash
awk -F, '
NR > 1 {
  n++;
  sum_rcm += $3;
  sum_ee += $6;
  sum_tip += $7;
  if ($3 > max_rcm) max_rcm = $3;
  if ($6 > max_ee) max_ee = $6;
  if ($7 > max_tip) max_tip = $7;
}
END {
  printf "samples=%d\n", n;
  printf "mean_rcm=%.6f m max_rcm=%.6f m\n", sum_rcm/n, max_rcm;
  printf "mean_ee=%.6f m max_ee=%.6f m\n", sum_ee/n, max_ee;
  printf "mean_tip=%.6f m max_tip=%.6f m\n", sum_tip/n, max_tip;
}' run_logs/llm_vlm_rcm_metrics.csv
```

如果只统计圆轨迹，应根据 controller 日志中进入和离开 `RCM_PIVOT` 的 ROS time
筛选 CSV 时间区间。

---

## 19. 日志和 PID

运行日志：

```text
run_logs/llm_vlm_rcm_qwen3.log
run_logs/llm_vlm_rcm_planner.log
run_logs/llm_vlm_rcm_executor.log
run_logs/llm_vlm_rcm_controller.log
run_logs/vlm_rcm_port_sim.log
run_logs/vlm_rcm_port_pose.log
run_logs/vlm_rcm_port_sam3.log
```

PID 文件：

```text
run_pids/llm_vlm_rcm_*.pid
run_pids/vlm_rcm_port_*.pid
run_pids/rcm_*.pid
```

查看最近日志：

```bash
tail -n 100 run_logs/llm_vlm_rcm_planner.log
tail -n 100 run_logs/llm_vlm_rcm_executor.log
tail -n 100 run_logs/llm_vlm_rcm_controller.log
tail -n 100 run_logs/vlm_rcm_port_pose.log
```

跟踪关键状态：

```bash
tail -f \
  run_logs/llm_vlm_rcm_executor.log \
  run_logs/llm_vlm_rcm_controller.log \
  run_logs/vlm_rcm_port_pose.log
```

---

## 20. OBS/PPT 推荐录制顺序

### 20.1 片段 A：LLM 任务拆解

画面只录终端：

```bash
./start_llm_vlm_rcm_demo.sh start
./start_llm_vlm_rcm_demo.sh task
```

重点保留：

- USER TASK。
- PLAN JSON。
- 四步动作。
- `TARGET_PROMPT HANDOFF`。
- `planned: 4 steps`，说明是真实 Qwen3。

### 20.2 片段 B：SAM3 孔位感知

```bash
./start_vlm_rcm_port_perception_demo.sh start
```

录制：

- SAM3 绿色 mask。
- 青色孔中心。
- 青色孔轴。
- PyBullet 中对应的圆环和箭头。

### 20.3 片段 C：安全入孔

完整链路执行时录制：

```text
PREINSERT_HOLD
-> APPROACH_PORT
-> PORT_DWELL
-> INSERT_THROUGH_PORT
-> INSERTED_DWELL
```

重点展示器械先对轴，再沿孔轴跨越表面。

### 20.4 片段 D：RCM 圆轨迹

录制：

- 红色 RCM 点基本不动。
- 工具轴始终通过 RCM 点。
- 黄色目标尖端。
- 绿色圆形轨迹。
- dVRK 末端姿态变化。

### 20.5 片段 E：定量结果

终端：

```bash
ros2 topic echo /rcm_virtual_fixtures/status
```

或者展示 CSV 统计：

```text
mean RCM error = 0.999 mm
max RCM error = 1.712 mm
```

---

## 21. 常见问题与排查

### 21.1 卡在 waiting for local Qwen3 service

检查：

```bash
tail -n 100 run_logs/llm_vlm_rcm_qwen3.log
nvidia-smi
curl -v http://127.0.0.1:8000/health
```

常见原因：

- 模型第一次下载。
- vLLM 正在编译 CUDA graph。
- 端口 8000 被占用。
- GPU 显存不足。

检查端口：

```bash
ss -ltnp | grep ':8000'
```

### 21.2 Qwen 和 SAM3 同时 OOM

完整 Demo 必须使用：

```text
GPU_EXECUTION_MODE=staged_single_gpu
```

并让 `start_llm_vlm_rcm_demo.sh` 自己启动和停止 Qwen。

如果 8000 端口上已有脚本外部启动的 Qwen，先停止它：

```bash
pkill -INT -f "vllm serve Qwen/Qwen3-4B"
```

然后重新：

```bash
./start_llm_vlm_rcm_demo.sh start
./start_llm_vlm_rcm_demo.sh task
```

### 21.3 `task` 等待 plan 超时

检查：

```bash
./start_llm_vlm_rcm_demo.sh status
ros2 node list
ros2 topic info -v /llm_task/instruction
ros2 topic info -v /llm_task/plan_json
tail -n 100 run_logs/llm_vlm_rcm_planner.log
```

新的脚本会检查 planner/executor 是否存活，并在启动早期失败时自动重试。

### 21.4 JSON 显示省略号

普通终端宽度或 ROS 2 输出格式可能让长字符串看起来被截断。使用：

```bash
ros2 topic echo --once \
  /llm_task/plan_json \
  --field data \
  | sed '/^---$/d' \
  | python3 -m json.tool
```

### 21.5 `python3 -m json.tool` 报 Extra data

原因是 ROS 2 echo 在 JSON 后输出 `---`。使用：

```bash
sed '/^---$/d'
```

删除分隔符。

### 21.6 SAM3 一直没有 mask

检查：

```bash
ros2 topic hz /sim/camera/color/image_raw
ros2 topic echo /sam3/active_prompt
tail -n 100 run_logs/vlm_rcm_port_sam3.log
```

确认：

- SAM3 已经加载完成。
- prompt 是 `circular hole`。
- CUDA 可用。
- RGB-D 相机未被关闭。

重新打开相机：

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /sim/camera/enabled \
  std_msgs/msg/Bool \
  "{data: true}"
```

### 21.7 孔中心没有锁定

检查：

```bash
ros2 topic echo /vlm_rcm/status
ros2 topic echo /vlm_rcm/port_valid
```

可能原因：

- mask 面积过小。
- score 过低。
- 深度孔腔区域小于阈值。
- 检测点离预期 CAD 孔位超过 70 mm。
- 连续帧中心抖动超过 6 mm。

### 21.8 孔轴为什么不是视觉估计

当前单个外部 RGB-D 视角看不到隐藏孔道，所以使用 CAD 标定轴：

```text
(0.335067, 0.0, -0.942194)
```

未来若需要纯视觉轴线，需要：

- 斜视第二相机。
- 内窥镜相机。
- 可见的圆柱孔道。
- 多视角点云或 CAD registration。

### 21.9 controller 不动

检查：

```bash
ros2 topic echo /rcm_virtual_fixtures/stage
ros2 topic echo /rcm_virtual_fixtures/status
ros2 topic hz /iiwa7/joint_states
ros2 topic hz /iiwa7/joint_desired
```

常见等待状态：

- 等机器人初始化。
- 等 task start。
- 等 port ready。
- visual port safety gate 拒绝。
- 等 pivot start。

### 21.10 轨迹只有四分之一圈

旧问题是 `trajectory_cycles` 被默认 `0.0` 覆盖。当前安全层已经固定：

```json
"trajectory_cycles": 1.0
```

一圈应持续约：

```text
2*pi*0.020/0.018 = 6.98 s
```

### 21.11 机械臂姿态扭动

当前使用 position IK。可尝试：

```bash
RCM_MAX_JOINT_STEP_RAD=0.012 \
RCM_SPEED_MPS=0.012 \
./start_rcm_virtual_fixture_demo.sh start
```

代码已经：

- 使用上一 IK 结果作为 seed。
- 使用初始构型作为 rest pose。
- 限制每周期关节步长。

但这仍不是显式零空间阻抗。

### 21.12 PyBullet 窗口打不开

无 GUI 运行：

```bash
VLM_RCM_GUI=false \
VLM_RCM_DISPLAY_OVERLAY=false \
./start_llm_vlm_rcm_demo.sh task
```

检查：

```bash
echo "$DISPLAY"
```

### 21.13 ROS shutdown traceback

planner、executor 和 RCM controller 已处理：

```text
ExternalShutdownException
```

如果日志仍出现旧 traceback，重新构建并 source：

```bash
colcon build --symlink-install \
  --packages-select pybullet_ros2_sim rcm_virtual_fixtures

source install/setup.bash
```

---

## 22. 停止和恢复

### 22.1 正常停止完整链路

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh stop
```

### 22.2 正常停止感知

```bash
./start_vlm_rcm_port_perception_demo.sh stop
```

### 22.3 正常停止独立 RCM

```bash
./start_rcm_virtual_fixture_demo.sh stop
```

### 22.4 检查残留进程

```bash
ps -eo pid,ppid,stat,etime,cmd | \
  grep -E \
  "vllm|sam3_mask_node|vlm_port_pose|surgical_rcm|rcm_virtual_fixture_node|iiwa_pybullet_sim_node" \
  | grep -v grep
```

### 22.5 必要时清理残留

先使用脚本 stop。只有 stop 无效时再执行：

```bash
pkill -INT -f "vllm serve"
pkill -INT -f "sam3_mask_node"
pkill -INT -f "vlm_port_pose_node"
pkill -INT -f "surgical_rcm_task_executor_node"
pkill -INT -f "rcm_virtual_fixture_node"
pkill -INT -f "iiwa_pybullet_sim_node"
```

再次检查：

```bash
nvidia-smi
ros2 node list
```

---

## 23. 修改日志

以下日志描述 `v1.2.0` 归档版本之后当前工作区中的 RCM/VLM 集成工作。
当前 Git 分支：

```text
release/v1.2.0
```

当前 HEAD：

```text
1c16506 release: archive v1.2.0 medical recording handover
```

当前 RCM/VLM 变更仍位于 working tree，尚未作为新的 Git tag 发布。

### 阶段 1：旧 Franka Demo 迁移

- 导入并分析：

```text
/home/siqin/old_ws/ysq_proj/rcm_decouple/src/franka_pkg/src/franka_task.zip
```

- 新建 `rcm_virtual_fixtures` ROS 2 Python package。
- 将 libfranka callback 和 torque/velocity 接口替换为：

```text
/iiwa7/joint_states
/iiwa7/joint_desired
/iiwa7/control_mode
```

- 保留 RCM、lineVF 和 planeVF 的任务空间几何意义。

### 阶段 2：轨迹和误差可视化

- 增加固定 RCM 点。
- 增加器械尖端目标点。
- 增加实际尖端轨迹点。
- 增加在线 metrics。
- 增加 CSV 记录。
- 增加 RCM 总误差、径向误差、轴向误差、EE error 和 tip error。

### 阶段 3：长杆手术器械

- 在 KUKA link-7 的 `+Z` 方向加入长杆。
- 控制器和渲染共用 `tool_length_m`。
- 增加 shaft radius、mount、tip trace 参数。

### 阶段 4：dVRK 末端

- 加入 dVRK Large Needle Driver 420006 网格。
- 新建 URDF。
- 加入 jaw angle 和 tip offset。
- 保存网格来源和许可证。
- 当前只用于可视化，不参与碰撞。

### 阶段 5：Phantom

- 加入用户建模的 `phantom_centered.stl`。
- 使用毫米到米比例 `0.001`。
- 将 phantom 底部贴在桌面顶面。
- 将孔中心与 RCM 工作区对齐。
- 缩小 RCM marker，保证孔口可见。

### 阶段 6：安全入孔

- 增加状态机：

```text
PREINSERT_HOLD
APPROACH_PORT
PORT_DWELL
INSERT_THROUGH_PORT
INSERTED_DWELL
WAITING_FOR_PIVOT
RCM_PIVOT
RCM_HOLD
```

- 穿越 phantom 表面时只允许沿孔轴移动。
- 阶段切换同时使用时间和实际尖端位置误差。

### 阶段 7：孔中心和孔轴可视化

- 增加 RGB-D 相机。
- 增加孔中心青色圆环。
- 增加入孔轴青色箭头。
- 增加相机 overlay。

### 阶段 8：SAM3 孔位感知

- 新增 `vlm_port_pose_node`。
- 使用 SAM3 mask 限制孔位语义区域。
- 使用深度拟合顶板和孔腔。
- 提取并锁定 world-frame 孔中心。
- 使用 CAD 标定轴描述隐藏孔道方向。
- 增加稳定窗口和安全 gate。

### 阶段 9：LLM 任务拆解

- 在 planner 增加四个 RCM 动作。
- 增加 RCM JSON schema。
- 增加 RCM 专用 system prompt。
- 使用 Qwen3-4B 真实生成四步计划。
- 将数值运动参数移到确定性安全层。

### 阶段 10：Surgical executor

- 新增 `surgical_rcm_task_executor_node`。
- 串行执行四步计划。
- 等孔位稳定后才启动机械臂。
- 进入 `WAITING_FOR_PIVOT` 后才允许圆轨迹。
- 一整圈后发布 hold 并进入 `RCM_HOLD`。

### 阶段 11：单 GPU 调度

- Qwen3 使用约 60% GPU 显存。
- 计划完成后停止 Qwen3。
- 显存释放后启动 SAM3。
- executor 使用 retained plan 继续任务。
- 锁定孔位后关闭 RGB-D 渲染，恢复 PyBullet 控制频率。

### 阶段 12：可靠性修复

- `/llm_task/plan_json` 使用 transient-local QoS。
- stage gate command 使用 transient-local QoS。
- 增加后台进程启动存活检查。
- 增加启动失败自动重试。
- `task` 前检查 planner 和 executor。
- 增加 planner/executor external shutdown 处理。
- 修复 `trajectory_cycles=0.0` 导致四分之一圈的问题。
- 最终安全层固定一整圈。

---

## 24. 当前已知限制

1. 当前是 PyBullet IK 位置控制，不是力矩级 RCM 或零空间阻抗控制。
2. dVRK 末端和长杆目前无碰撞模型。
3. phantom 在录制配置中默认关闭碰撞，安全性来自几何状态机而不是接触反馈。
4. 孔轴依赖 CAD 标定，不是纯视觉估计。
5. 单 GPU staged 模式一次 `start` 适合执行一次规划任务。重新规划时建议
   `stop -> start -> task`。
6. 当前四步计划固定执行圆轨迹，不包含撤回、组织目标或缝合动作。
7. 当前控制安全 gate 主要检查孔位偏差、轴角、阶段尖端误差和关节步长，
   还没有完整碰撞检测、关节力矩监控或临床安全认证。
8. 当前 Demo 仅用于研究、仿真和 PPT 展示。

---

## 25. 后续推荐工作

优先级从高到低：

1. 将 RCM controller 改成 ROS 2 action server。
2. 增加显式动作 result、超时、失败原因和 retry。
3. 增加沿孔轴安全撤回动作。
4. 加入碰撞检测和 phantom 接触约束。
5. 使用第二相机或多视角点云估计真实孔轴。
6. 增加组织目标点和缝合入口点 prompt。
7. 实现显式 null-space posture task。
8. 进一步实现 torque-level null-space impedance。
9. 为四步 planner、executor 状态机和轨迹误差增加自动测试。
10. 将当前 working tree 归档为新的 Git release/tag。

---

## 26. 最短执行清单

### 完整真实 Qwen3 Demo

终端 1：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh start
```

终端 2：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh task
```

终端 3：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /surgical_rcm/status
```

停止：

```bash
./start_llm_vlm_rcm_demo.sh stop
```

### 仅视觉

```bash
./start_vlm_rcm_port_perception_demo.sh start
./start_vlm_rcm_port_perception_demo.sh stop
```

### 仅 RCM

```bash
RCM_FIXTURE_MODE=rcm ./start_rcm_virtual_fixture_demo.sh start
./start_rcm_virtual_fixture_demo.sh stop
```

### 直线/平面

```bash
RCM_FIXTURE_MODE=line ./start_rcm_virtual_fixture_demo.sh start
RCM_FIXTURE_MODE=plane ./start_rcm_virtual_fixture_demo.sh start
```
