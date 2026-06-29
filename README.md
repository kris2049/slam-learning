# SLAM 学习路线图 — 实践版

> 基于 [changh95/visual-slam-roadmap](https://github.com/changh95/visual-slam-roadmap) 的动手学习计划。
> 每个级别包含：**理论笔记** → **编程练习** → **实验验证** → **测试**

## 学习进度

| 级别 | 主题 | 状态 | 完成日期 |
|:---:|------|:---:|:---:|
| 1 | 基础：编程、数学、射影几何、相机、图像 | 🔴 | — |
| 2 | SLAM 入门：特征匹配、多视图几何、优化、建图 | ⬜ | — |
| 3 | 单目视觉 SLAM | ⬜ | — |
| 4 | RGB-D 视觉 SLAM | ⬜ | — |
| 5 | 深度学习 + SLAM | ⬜ | — |
| 6 | VIO / VINS | ⬜ | — |
| 7 | 双目 SLAM | ⬜ | — |
| 8 | 协作 / 多机器人 SLAM | ⬜ | — |
| 9 | LiDAR & 视觉-LiDAR 融合 | ⬜ | — |
| 10 | 事件相机 SLAM | ⬜ | — |
| 11 | 世界模型 & 空间 AI | ⬜ | — |

## 使用方式

```bash
# 每个级别按顺序学习:
#   study/  →  学习教材 (先读这个!)
#   exercises/ → 编程练习 (边学边练)
#   experiments/ → 实验验证 (跑起来看结果)
#   tests/ → 综合测试 (通过=掌握)

cd level-01-beginner

# 1️⃣ 先读教材
cat study/01-programming.md
cat study/02-mathematics.md
cat study/03-projective-geometry.md
cat study/04-camera-and-image.md

# 2️⃣ 做练习
/usr/bin/python3 exercises/exercise_01_linear_algebra.py

# 3️⃣ 跑实验
/usr/bin/python3 experiments/experiment_01_camera_model.py

# 4️⃣ 通过测试
/usr/bin/python3 tests/test_01_basics.py
```

## 规则

1. **顺序学习**：必须通过当前级别的所有测试才能进入下一级
2. **动手优先**：每个概念都有对应代码练习
3. **实验驱动**：每个模块包含至少一个可运行的实验
4. **Git 记录**：每完成一个练习/实验，commit 一次
