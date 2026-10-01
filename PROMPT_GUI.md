# HiSurg LLM Demo Prompt GUI

Launch the common task-entry window:

```bash
cd ~/ros2_workspaces/humble/ros2_pybullet_ws
./start_prompt_gui.sh
```

Select a demo, optionally start its stack, enter a natural-language task and
press **发送给 LLM**. `Ctrl+Enter` also submits the task. Output from the
existing launcher is shown in the lower log panel.

The LLM-capable demo launchers also open the matching window when `task` has no
instruction, for example:

```bash
./start_llm_vlm_rcm_demo.sh task
./start_medical_grasp_demo.sh task
./start_open_vocab_household_demo.sh task
```

Passing an explicit task remains compatible with scripts and automation:

```bash
./start_llm_vlm_rcm_demo.sh task \
  "定位相机视角下左上角的孔，在20度锥形姿态范围内自动选择最接近的可达入路姿态，并建立RCM"
```

The GUI invokes each launcher with an argument list rather than a shell command,
so prompt text is passed as one argument without shell interpolation.
