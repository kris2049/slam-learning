# Level 1: 基础篇（Beginner）

> **目标**: 建立视觉 SLAM 所需的编程、数学和图像处理基础
> **预计时间**: 2-3 周（取决于已有基础）
> **前置要求**: 无

## 学习清单

### 1. 编程基础
- [ ] Python: numpy 矩阵运算、matplotlib 可视化
- [ ] C++: 指针、OOP（了解即可，后续级别需要）
- [ ] Bash/Linux: 终端基本操作

### 2. 数学基础
- [ ] 线性代数: 向量/矩阵、行列式、逆矩阵、SVD、特征值/特征向量
- [ ] 概率统计: 高斯分布、贝叶斯定理
- [ ] 微积分: 求导、泰勒展开

### 3. 射影几何
- [ ] 针孔相机模型 → 图像投影
- [ ] 相机标定: 内参/外参、镜头畸变
- [ ] 刚体运动: 欧拉角/四元数/旋转矩阵
- [ ] 齐次坐标与变换
- [ ] 对极几何 → 本质矩阵/基础矩阵
- [ ] 三角化

### 4. 相机与图像
- [ ] 相机硬件: 镜头、传感器、分辨率
- [ ] 图像基础: 彩色/灰度、阈值化、高斯模糊
- [ ] 角点检测: Harris Corner
- [ ] 边缘检测: Sobel & Canny
- [ ] 立体视觉: 视差、深度

## 资源

| 资源 | 链接 |
|------|------|
| **视觉SLAM十四讲** (高翔) | [书籍](https://github.com/gaoxiang12/slambook2) |
| **Multiple View Geometry** (Hartley & Zisserman) | 第2章、第9章 |
| **Cyrill Stachniss SLAM 讲座** | [YouTube](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQh_J16IMwDlji18SWQ2PZ6) |

## 如何开始

```bash
# 1. 先做练习（边学边练）
cd exercises/
python3 exercise_01_linear_algebra.py
python3 exercise_02_probability.py
python3 exercise_03_projective_geometry.py
python3 exercise_04_image_processing.py

# 2. 跑实验（验证理解）
cd experiments/
python3 experiment_01_camera_model.py
python3 experiment_02_epipolar_geometry.py
python3 experiment_03_feature_detection.py

# 3. 通过测试
cd tests/
python3 test_01_basics.py
```
