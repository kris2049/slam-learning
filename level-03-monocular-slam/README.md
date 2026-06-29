# Level 3: 单目视觉 SLAM

> **目标**: 理解单目 SLAM 的完整 pipeline — 前端追踪、后端优化、回环检测、建图
> **预计时间**: 4-6 周
> **前置要求**: Level 2 全部通过

## 核心论文 (按学习顺序)

| 系统 | 年份 | 关键贡献 | 必读 |
|------|------|----------|:--:|
| PTAM | 2007 | Frontend/Backend分离, 关键帧, 并行线程 | ⭐ |
| ORB-SLAM | 2015 | 自动初始化, 共视图, 三线程架构 | ⭐⭐ |
| LSD-SLAM | 2014 | 直接法, 光度误差, 半稠密建图 | ⭐ |
| DSO | 2016 | 光度BA, 滑动窗口, 稀疏直接法 | ⭐⭐ |
| ORB-SLAM3 | 2020 | 多地图, IMU融合, Atlas系统 | ⭐⭐ |
| DROID-SLAM | 2021 | 可微分BA, 端到端学习 | ⭐ |
| DUSt3R/MASt3R | 2024 | 基础模型, Pointmap回归 | ⭐ |
| NICE-SLAM | 2022 | NeRF-SLAM, 层次特征网格 | ⭐ |
| MonoGS | 2024 | 3DGS-SLAM, 单目高斯泼溅 | ⭐ |

## 学习清单

### 特征法 SLAM
- [ ] VO vs SLAM: 局部 vs 全局+回环
- [ ] 尺度不确定性
- [ ] 共视图 (Covisibility Graph)
- [ ] 三种线程: Tracking, LocalMapping, LoopClosing
- [ ] 地图点管理: 生成, 剔除, 合并
- [ ] 自动初始化: Homography vs Fundamental 选择

### 直接法 SLAM
- [ ] 光度误差 vs 几何误差
- [ ] 半稠密建图
- [ ] 滑动窗口优化

### 学习法 SLAM
- [ ] DROID-SLAM: 可微分BA
- [ ] 基础模型: DUSt3R, MASt3R, VGGT

### 神经表示 SLAM
- [ ] NeRF-SLAM: iMAP → NICE-SLAM → Co-SLAM
- [ ] 3DGS-SLAM: SplaTAM → MonoGS → RTG-SLAM

### 语义 SLAM
- [ ] ConceptFusion, LERF, ConceptGraphs

## 实验

```bash
cd exercises/
/usr/bin/python3 exercise_01_orb_slam_pipeline.py     # ORB-SLAM 核心流程
/usr/bin/python3 exercise_02_direct_method.py         # 直接法光度误差
/usr/bin/python3 exercise_03_sliding_window.py         # 滑动窗口与边缘化

cd experiments/
/usr/bin/python3 experiment_01_covisibility_graph.py   # 共视图构建
/usr/bin/python3 experiment_02_loop_closure.py         # 回环检测模拟

cd tests/
/usr/bin/python3 test_03_monocular_slam.py
```
