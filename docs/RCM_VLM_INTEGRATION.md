# RCM 与 VLM/LLM 手术任务系统

完整环境搭建、修改日志、参数、指令集合和排障说明见：

- [LLM_VLM_RCM_FULL_BUILD_AND_COMMAND_GUIDE.md](LLM_VLM_RCM_FULL_BUILD_AND_COMMAND_GUIDE.md)

## 完整 LLM + SAM3 + RCM Demo

默认使用本地 Qwen3 生成受约束的四步计划。当前显卡无法同时常驻 Qwen3 和
SAM3，因此脚本采用分阶段 GPU 调度：先运行 Qwen3，计划生成后自动关闭 Qwen3，
再启动 SAM3、PyBullet 和 RCM 控制器。

终端 1：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh start
```

Qwen3 显示 ready 后，在终端 2 发送默认任务：

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_llm_vlm_rcm_demo.sh task
```

也可以显式发送同义指令：

```bash
./start_llm_vlm_rcm_demo.sh task \
  "locate the circular laparoscopic port, align the surgical tool with the insertion axis, insert through the port while establishing the RCM constraint, then execute a circular trajectory while keeping the RCM point fixed"
```

查看状态或停止：

```bash
./start_llm_vlm_rcm_demo.sh status
ros2 topic echo /surgical_rcm/status
ros2 topic echo /rcm_virtual_fixtures/stage
./start_llm_vlm_rcm_demo.sh stop
```

无 Qwen3 时可用确定性计划器测试其余链路：

```bash
LLM_BACKEND=local_fallback ./start_llm_vlm_rcm_demo.sh start
./start_llm_vlm_rcm_demo.sh task
```

LLM 只输出动作名称，安全层固定视觉提示、阈值、插入深度和轨迹参数。最终
执行计划严格为：

1. `localize_rcm_port`：发送 `circular hole` 给 SAM3，稳定后锁定孔中心和孔轴。
2. `align_tool_axis`：移动到孔外预插入位姿，使器械轴与锁定孔轴共线。
3. `establish_rcm`：沿孔轴插入 77 mm，将孔中心建立为固定 RCM 点。
4. `execute_rcm_circle`：以 20 mm 半径执行一整圈末端轨迹并保持 RCM 约束。

主要任务接口：

- `/llm_task/instruction`：用户自然语言任务。
- `/llm_task/plan_json`：四步受约束计划。
- `/surgical_rcm/status`：当前执行步骤和完成状态。
- `/surgical_rcm/active_step_json`：当前活动步骤。
- `/rcm_virtual_fixtures/stage`：底层运动阶段。
- `/rcm_virtual_fixtures/metrics`：RCM 和末端轨迹误差。

## 当前可运行的孔位感知 Demo

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_vlm_rcm_port_perception_demo.sh start
```

停止：

```bash
./start_vlm_rcm_port_perception_demo.sh stop
```

当前 demo 只运行感知，不启动 RCM 运动。SAM3 使用 `circular hole`
定位穿孔局部区域；深度图从局部顶板平面中提取后方凹陷区域，并将完整孔腔
区域的质心投影回顶板，得到三维孔中心。连续三帧稳定后锁定结果。

彩色窗口中，绿色为 SAM3 mask，绿色小十字为 mask 质心，青色大十字为
深度修正后的孔中心，青色箭头为入孔方向。PyBullet 中使用同一组锁定结果
绘制青色圆环和轴线。

当前 STL 的外部图像只显示平面孔口，单个外部 RGB-D 视角无法直接观测隐藏
通道的完整三维轴线。因此孔中心来自 `SAM3 + RGB-D`，入孔轴采用 phantom
CAD 标定值 `(0.335067, 0, -0.942194)`。节点状态会明确输出
`axis_source=calibrated_port_geometry`，不会把标定轴误称为纯视觉结果。

主要输出：

- `/vlm_rcm/overlay`
- `/vlm_rcm/status`
- `/vlm_rcm/locked_port_point`
- `/vlm_rcm/locked_port_axis`
- `/vlm_rcm/port_ready`

## 1. 系统接口

- LLM 输入：`/llm_task/instruction`
- LLM 计划：`/llm_task/plan_json`
- SAM3 提示：`/sam3/active_prompt`
- 三维关键点：`/perception/keypoint_3d`
- 目标平面法向：`/perception/object_plane_normal`
- 目标平面切向：`/perception/object_plane_tangent`
- RCM 状态：`/rcm_virtual_fixtures/status`
- RCM 误差：`/rcm_virtual_fixtures/metrics`
- 已锁定孔中心：`/vlm_rcm/locked_port_point`
- 已锁定孔轴：`/vlm_rcm/locked_port_axis`
- 机器人命令：`/iiwa7/joint_desired`

`surgical_rcm_task_executor_node` 不直接发布关节量，只负责按计划发出阶段门控。
`rcm_virtual_fixture_node` 是当前唯一的 `/iiwa7/joint_desired` 发布者。

## 2. 推荐职责划分

1. VLM/LLM 只负责语义理解、目标选择和受约束任务分解，不直接产生关节量。
2. SAM3 + GMS/深度融合负责把语义目标变成端口中心、组织目标点、器械点和表面法向。
3. Surgical executor 负责顺序执行、视觉点锁定、阶段完成判断和控制权切换。
4. RCM controller 只执行 `align`、`insert`、`pivot`、`retract` 等几何原语。
5. Safety supervisor 检查孔径、插入轴、工作空间、跟踪误差和状态机顺序。

## 3. 当前计划 JSON

```json
{
  "task_summary": "Locate the port, establish RCM, and execute a circle.",
  "steps": [
    {
      "step_index": 1,
      "action": "localize_rcm_port",
      "target_prompt": "circular hole"
    },
    {
      "step_index": 2,
      "action": "align_tool_axis",
      "target_prompt": "circular hole"
    },
    {
      "step_index": 3,
      "action": "establish_rcm",
      "target_prompt": "circular hole",
      "insertion_depth_m": 0.077
    },
    {
      "step_index": 4,
      "action": "execute_rcm_circle",
      "target_prompt": "",
      "trajectory_radius_m": 0.02,
      "trajectory_cycles": 1.0
    }
  ]
}
```

这四个动作已加入 planner schema、确定性 fallback 和 surgical executor。
LLM 不能直接输出关节角、位姿或力矩命令，也不能改变动作顺序。

## 4. 视觉几何锁定

端口识别应同时使用：

- SAM3 mask：限定“穿孔/端口”目标区域。
- 深度点云：拟合端口附近上表面。
- 端口中心：mask 边界或孔轮廓的三维几何中心。
- 插入轴：优先使用孔轴；无法直接估计时使用局部表面法向。

关键点和法向连续稳定后生成一次 `port_pose` 快照。机械臂开始对孔后停止实时
更新该快照，避免关键点跳变把器械轴拉向孔壁。只有任务重试或显式重新定位时
才重新解锁视觉。

PyBullet 使用青色圆环显示锁定孔中心，使用青色箭头显示向 phantom 内部的
锁定孔轴。`start_vlm_rcm_port_perception_demo.sh` 已由视觉锁定节点生成这两个
world-frame 结果；RCM 运动 demo 的下一阶段将直接消费这些锁定结果。

## 5. 安全顺序

```text
LOCALIZE_PORT
  -> LOCK_PORT_POSE
  -> PREINSERT_HOLD
  -> APPROACH_PORT
  -> PORT_DWELL
  -> INSERT_THROUGH_PORT
  -> INSERTED_DWELL
  -> WAITING_FOR_PIVOT
  -> RCM_PIVOT
  -> RCM_HOLD
```

跨越 phantom 表面时只允许沿锁定的孔轴移动，不允许横向平移或改变姿态。
每个阶段必须同时满足最短持续时间和实际尖端误差阈值，才能进入下一阶段。
若 RCM 径向误差、关节极限或 IK 残差超限，应保持当前位置并报告失败，而不是
跳到下一步。

## 6. 后续扩展

下一阶段可加入组织目标点、缝合入口点、沿孔轴撤回、失败重试和显式控制权
仲裁。推荐最终把运动原语改为 ROS 2 action server，使 executor 可以收到
每一步的结构化 result 和错误原因。
