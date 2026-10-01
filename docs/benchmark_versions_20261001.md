# Benchmark依赖版本

`main`整合截至07e5c1e的仿真和8月多孔demo，冻结标签`benchmark-v1-20261001`。当前benchmark v2在独立`benchmark/v2`分支中发展；主研究入口为https://github.com/jackysqing-max/hisurg_ras_ws/tree/benchmark/v2 。

V5是phantom资产版本；本标签不改软件历史VERSION。现有PR #1的e86eb66由本次整合纳入。旧发布分支保留，无force push。

v2专用可靠RGB-D包装目前位于主仓库scripts/pipeline_v2_sim.py及pipeline_v2_bound_pose.py；本仓库共享几何/验证代码没有因此改写。
