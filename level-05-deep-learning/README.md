# Level 5: 深度学习 + SLAM

> **目标**: 用深度学习方法替代/增强 SLAM 各组件
> **预计时间**: 3-4 周 | **前置**: Level 2-3

## 四大方向
| 方向 | 代表工作 | 关键词 |
|------|----------|--------|
| A. 前端 | SuperPoint, SuperGlue, MiDaS, RAFT | 特征/深度/光流 |
| B. 后端 | DROID-SLAM, Theseus, SE-Sync | 可微分优化 |
| C. 系统 | DPVO, DeepFactors | 端到端 VO/SLAM |
| D. 语义 | ConceptFusion, LERF, SAM | 场景理解 |

```bash
cd exercises/ && /usr/bin/python3 exercise_01_deep_features.py
cd experiments/ && /usr/bin/python3 experiment_01_differentiable_ba.py
cd tests/ && /usr/bin/python3 test_05_deep_learning.py
```
