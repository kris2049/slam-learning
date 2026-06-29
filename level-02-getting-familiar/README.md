# Level 2: SLAM 入门 (Getting Familiar)

> **目标**: 掌握视觉 SLAM 的核心组件：特征提取/匹配、多视图几何、非线性优化、因子图、建图
> **预计时间**: 4-6 周
> **前置要求**: Level 1 全部通过

## 学习清单

### 特征提取与匹配
- [ ] 关键点检测器: SIFT, FAST, ORB, AKAZE
- [ ] 描述子匹配: Brute-Force, FLANN, Kd-Tree
- [ ] 深度学习特征: SuperPoint, SuperGlue, LightGlue
- [ ] 光流跟踪: KLT Tracker
- [ ] 词袋模型: Bag of Visual Words

### 多视图几何
- [ ] 2D-2D: 基础矩阵/本质矩阵/单应矩阵
- [ ] 2D-3D: P3P, PnP
- [ ] 3D-3D: ICP
- [ ] 外点剔除: RANSAC

### 非线性优化
- [ ] 最小二乘, 重投影误差
- [ ] 高斯-牛顿法, Levenberg-Marquardt
- [ ] 李群李代数: SO(3), SE(3)
- [ ] Bundle Adjustment
- [ ] 位姿图优化
- [ ] Schur 补/稀疏性

### 因子图优化
- [ ] 因子图概念
- [ ] g2o / GTSAM 使用

### 建图
- [ ] 点云, 八叉树地图
- [ ] TSDF, Surfel
- [ ] 占据栅格地图

### 传感器与评测
- [ ] IMU, LiDAR 基础
- [ ] ATE/RPE 评测指标
- [ ] KITTI / TUM / EuRoC 数据集

## 资源
- 《视觉SLAM十四讲》第5-11讲
- Cyrill Stachniss: Factor Graphs for SLAM
- Ceres-solver 官方教程

```bash
cd exercises/
/usr/bin/python3 exercise_01_feature_matching.py
/usr/bin/python3 exercise_02_multiview_geometry.py
/usr/bin/python3 exercise_03_nonlinear_optimization.py
/usr/bin/python3 exercise_04_factor_graph.py

cd experiments/
/usr/bin/python3 experiment_01_bundle_adjustment.py
/usr/bin/python3 experiment_02_pose_graph.py
/usr/bin/python3 experiment_03_mapping.py

cd tests/
/usr/bin/python3 test_02_slam_foundations.py
```
