# Level 4: RGB-D 视觉 SLAM

> **目标**: 理解深度相机 SLAM — 体积融合、Surfel、TSDF
> **预计时间**: 2-3 周 | **前置**: Level 2

## 核心系统
| 系统 | 年份 | 关键贡献 |
|------|------|----------|
| KinectFusion | 2011 | GPU, TSDF 体积融合 |
| ElasticFusion | 2015 | Surfel 面元, 非刚体变形 |
| BundleFusion | 2016 | 局部到全局优化 |
| DSP-SLAM | 2021 | DeepSDF 形状先验 |

## 学习清单
- [ ] ICP 配准: point-to-point, point-to-plane
- [ ] TSDF: 截断符号距离函数, 体积融合
- [ ] Surfel: 面元模型
- [ ] 回环: 随机蕨编码 (ElasticFusion)
- [ ] 动态场景: DynamicFusion, 运动场

```bash
cd exercises/ && /usr/bin/python3 exercise_01_tsdf_fusion.py
cd experiments/ && /usr/bin/python3 experiment_01_surfel_map.py
cd tests/ && /usr/bin/python3 test_04_rgbd_slam.py
```
