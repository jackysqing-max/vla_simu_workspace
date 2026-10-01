# 多孔 phantom RCM 定位版本汇总

日期：2026-08-01

## 目标

将原有单孔/先验式 RCM demo 改成多孔 phantom 场景下的语言引导定位流程。当前版本优先完成“只定位、不运动”的闭环：根据语言描述选择目标孔，生成孔中心、表面等效轴和入路轴，并在视觉窗口与 PyBullet 中同步显示。

## 主要改动

1. 替换并对齐多孔 phantom 模型
   - 新增 `phantom_multi.STL` 和 `phantom_multi_raw.STL`。
   - 将多孔模型坐标对齐到原 `phantom_centered.stl` 的中心、姿态和放置方式。
   - 启动脚本默认加载多孔 phantom。

2. SAM3 多候选孔感知
   - SAM3 节点不再只使用单个 best mask，而是合并多个满足阈值的候选 mask。
   - VLM/RGB-D 节点可以从同一帧中提取多个孔候选。

3. 语言驱动选孔
   - 新增 `/vlm_rcm/language_command`，用于传递“相机视角左上角”等语言指令。
   - 支持相机/图像参考系和 phantom 自身参考系的方向解析。
   - 每个候选孔都会计算语言选择分数和排序。

4. 孔中心与孔轴估计
   - 孔中心优先使用 SAM aperture ellipse 的表面开口中心。
   - 输出表面等效轴 `surface_equivalent_axis`。
   - 增加基于多深度截面圆心拟合的 `section_center_axis`。
   - 默认 RCM 入路轴仍使用 calibrated axis，可通过 `VLM_RCM_AXIS_MODE=section_centers` 切换。

5. PyBullet 可视化
   - 黄色球：真实 locked port point。
   - 绿色球：候选孔中心。
   - 青色箭头：最终入路轴。
   - 洋红色箭头：表面等效轴。
   - 显示每个候选孔的分数。

6. 修复 RGB-D 反投影横向偏差
   - 修正 PyBullet 相机内参：`fx = fy`。
   - 原错误会让横向世界距离被压缩为 `height / width = 0.75`。
   - 修复后左上孔从约 `(0.6018, -0.0561, 0.4067)` 校正为 `(0.6018, -0.0748, 0.4067)`，与 STL 理论孔心 `(0.6017, -0.0750, 0.4067)` 基本一致。

7. 只定位不运动入口
   - 新增 `docs/recording_scripts/vlm_rcm_locate_capture.py`。
   - 可用命令：

```bash
./start_vlm_rcm_port_perception_demo.sh locate "定位相机视角左上角的孔"
```

或：

```bash
./start_llm_vlm_rcm_demo.sh locate "定位相机视角左上角的孔"
```

## 验证结果

已运行：

```bash
python3 -m py_compile \
  src/pybullet_ros2_sim/pybullet_ros2_sim/sim_camera.py \
  src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_pybullet_sim_node.py \
  src/rcm_virtual_fixtures/rcm_virtual_fixtures/vlm_port_pose_node.py

colcon build --packages-select pybullet_ros2_sim rcm_virtual_fixtures --symlink-install

./start_vlm_rcm_port_perception_demo.sh locate "定位相机视角左上角的孔"
```

定位输出：

```text
target=left+up reference=image selected=H3
locked_point_world=(0.6018, -0.0748, 0.4067)
```

## 当前建议

下一步先确认 PyBullet 黄色球是否稳定落在目标孔中心。如果确认无误，再把 locked port point、入路轴和 RCM 运动 primitive 接起来，进入自动入路和 RCM 轨迹执行阶段。
