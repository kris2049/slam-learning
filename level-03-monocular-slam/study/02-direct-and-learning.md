# 第2部分: 直接法 SLAM & 学习法 SLAM & 神经表示

---

## 2.1 直接法 vs 特征法 — 根本哲学分歧

| | 特征法 (ORB-SLAM) | 直接法 (DSO/LSD-SLAM) |
|---|-------------------|----------------------|
| **使用什么** | 稀疏关键点 | 所有高梯度像素 |
| **优化什么** | 几何误差 (像素距离) | 光度误差 (像素强度差) |
| **对纹理需求** | 有角点即可 | 需要梯度，对低纹理敏感 |
| **对光照** | 鲁棒（描述子光照不变） | 敏感（依赖光度恒定假设） |
| **地图** | 稀疏3D点 | 半稠密/稠密 |
| **计算量** | 轻量 | 较重 |

---

## 2.2 LSD-SLAM (Large-Scale Direct SLAM) — 2014, Engel et al.

**第一个大尺度直接法 SLAM**。

### 核心思想

用**高梯度像素**做跟踪和建图。

### 跟踪 (Tracking)

$$\min_{\xi} \sum_{p \in \Omega} \left\| I_{ref}(p) - I_{cur}(\omega(p, D_{ref}(p), \xi)) \right\|^2$$

- $D_{ref}(p)$：参考帧的深度图（半稠密）
- $\omega$：warp 函数——给定参考帧像素和深度，在当前帧中预测对应位置

**与特征法的关键区别**：
- 不需要特征提取 → 可在纹理弱但梯度够的场景工作
- 直接使用像素强度优化 → 利用了更多图像信息

### 建图 (Mapping)

- 对每个高梯度像素，沿对极线做**立体匹配**估计深度
- 深度估计带有**概率分布**（高斯分布）
- 多帧观测 → 用卡尔曼滤波融合 → 深度越来越精确

### 回环与全局优化

- 用 FAB-MAP (基于 SURF + BoVW) 做回环检测
- Sim(3) 对齐 → 位姿图优化

---

## 2.3 DSO (Direct Sparse Odometry) — 2016, Engel et al.

**直接法 + 稀疏 + 光度标定**

### DSO 的两个关键改进

**1. 光度标定**

相机的亮度响应不是线性的：
- 镜头**渐晕 (Vignette)**：图像边缘比中心暗
- **伽马校正**：$I_{out} = I_{in}^{\gamma}$
- **曝光时间**：自动曝光会改变整体亮度

DSO 对每个相机做光度标定，在优化中补偿这些因素：

$$I(x) = G(tV(x)B(x))$$

其中 $G$ = 响应函数，$V$ = Vignette，$B$ = 场景辐照度，$t$ = 曝光时间。

**2. 滑动窗口 + 边缘化**

DSO 维持 7 个关键帧的滑动窗口。窗口外的旧帧被边缘化→信息压缩到剩余变量。

### 稀疏 vs 半稠密

DSO 不像 LSD-SLAM 用所有高梯度像素，而是采样**稀疏的高梯度点**：
- 在图像上均匀采样
- 选择梯度最大的点
- → 更快的速度，同时保持了直接法的优势

### DSO 的局限

- ❌ 没有回环检测和全局优化 → 长时间累积漂移
- ❌ 纯单目 → 尺度漂移

### LDSO (2018) — DSO + 回环

用 DBoW2 (ORB 描述子) 做回环检测，弥补 DSO 的最大短板。

---

## 2.4 混合法: SVO (Semi-direct Visual Odometry) — 2014

**结合特征法和直接法的优点**：

- 用 FAST 检测特征点（特征法）
- 用直接法跟踪这些点（直接法）→ 亚像素精度
- Bundle Adjustment 优化位姿和3D点

**结果**：非常快（>100 FPS），适合无人机等快速运动场景。

---

## 2.5 学习法 SLAM

### DROID-SLAM (2021, Teed & Deng) — 里程碑

**端到端可微分的 SLAM**。

核心创新：**可微分 Bundle Adjustment**

传统 BA 用 Gauss-Newton 迭代求解 → 不可微分。
DROID-SLAM 将 BA 的一步迭代实现为可微分的神经网络层 → 可用梯度下降训练。

**架构**：
```
图像对 → 稠密光流 (RAFT-like) → 可微分 BA 层 → 位姿 + 深度
```

训练后，DROID-SLAM 在未见过的数据集上表现优异。

### DPVO (2023) — DROID-SLAM 轻量版

- Patch-based：用图像块代替稠密光流
- 30+ FPS 实时
- 更适合实际部署

### MAC-VO (2024)

添加度量感知 → 解决单目的尺度模糊。

### VoT (2025, Visual Odometry with Transformers)

用 Transformer 架构估计视觉里程计。

---

## 2.6 基础模型 SLAM — 2024-2026 最新前沿

### DUSt3R (2024, Wang et al.)

从**一对图像**直接回归 3D Pointmap（不依赖相机内参！）。

**革命性意义**：不需要特征提取 → 不需要匹配 → 不需要标定 → 直接从像素到 3D。

### MASt3R (2024)

DUSt3R + 局部特征匹配 → 更好的精度。

### MASt3R-SLAM (2025)

将 MASt3R 作为实时的稠密 SLAM 前端。

### VGGT (2025, Meta — CVPR 2025 Best Paper)

从 N 张图像前馈推理**所有**的位姿、深度、点云、特征轨迹。

**这是范式转变**：从"优化求解"到"直接推理"。

### VGGT-SLAM (2025)

将 VGGT 作为实时 SLAM 的前端。

---

## 2.7 神经表示 SLAM

### NeRF-SLAM 系列

| 系统 | 年份 | 表示 | 特点 |
|------|------|------|------|
| **iMAP** | 2021 | 单个 MLP | 首个 NeRF-SLAM |
| **NICE-SLAM** | 2022 | 层次特征网格 | 可扩展 |
| **Co-SLAM** | 2023 | Hash Grid | 5-10× 快 |
| **ESLAM** | 2023 | Tri-plane | O(N²) 内存 |
| **GO-SLAM** | 2023 | 全局优化 | 回环闭合 |

### 3DGS-SLAM 系列 (最新主流)

| 系统 | 年份 | 特点 |
|------|------|------|
| **SplaTAM** | 2024 | 首个 3DGS-SLAM |
| **MonoGS** | 2024 | 单目 3DGS |
| **RTG-SLAM** | 2024 | Jetson 上 25 FPS |

**Gaussian Splatting 优势**：
- 显式表示（非 MLP）→ 渲染极快
- 可微分 → 梯度下降优化
- 比 NeRF 快 100+ 倍

---

## 2.8 语义 / 语言锚定 SLAM

### ConceptFusion (2023, MIT)

将 CLIP 特征融合到 3D 地图 → 可用自然语言查询地图。

### LERF (2023)

Language Embedded Radiance Fields：DINO 多尺度特征嵌入 NeRF。

### ConceptGraphs (2023)

开放词汇 3D 场景图：SAM + CLIP + LLM → 空间关系推理。

### LEGS (2025)

Language Embedded Gaussian Splats：实时可查询 3D。

---

> **练习**: `exercises/exercise_02_direct_method.py` (直接法), `exercises/exercise_03_sliding_window.py` (滑动窗口)
> **实验**: `experiments/experiment_01_covisibility_graph.py`
