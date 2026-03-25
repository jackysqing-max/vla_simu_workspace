# Changelog

## v0.1.1 - 2026-03-25

- Added a visible raised table to the RGB-D PyBullet scene instead of a buried table URDF.
- Added multiple tabletop cubes with different colors so text prompts such as `red cube` select a specific object.
- Replaced the single-centroid depth fusion logic with a ReKep-style clustering proposal stage:
  - masked RGB-D sampling
  - PCA feature compression
  - k-means grouping
  - 3D mean-shift proposal merging
- Kept the existing `/perception/keypoint_3d` topic for tracker compatibility and added `/perception/keypoint_candidates` for clustered proposals.
- Added `start_rekep_demo.sh` as a one-command launcher for:
  - RGB-D simulation
  - SAM3 mask generation in `~/venvs/ros_vla`
  - clustering-based keypoint fusion
  - keypoint tracker
  - state bridge
  - robot monitor
