# Level 4 教材: RGB-D 视觉 SLAM

## 1. RGB-D 相机原理

| 技术 | 原理 | 代表设备 | 测距范围 |
|------|------|----------|----------|
| 结构光 | 投射红外散斑→变形→深度 | Kinect v1, RealSense SR305 | 0.3-3m |
| ToF | 红外脉冲往返时间 | Kinect v2, Azure Kinect | 0.5-5m |
| 主动立体 | 投射红外纹理+双目匹配 | RealSense D435 | 0.2-10m |

## 2. KinectFusion (2011, Newcombe et al.)

### 核心创新：GPU 加速的 TSDF 体积融合

**Tracking**: 当前深度图 → 3D点云 + 法向量 → coarse-to-fine ICP 配准

**Mapping**: 将当前深度帧融合到全局 TSDF 体积
- 每个体素存储截断距离 D 和权重 W
- 加权平均: D_new = (W_old·D_old + D_curr) / (W_old + 1)

**局限**: 体积增长与场景大小成立方关系 → 只能处理房间大小

### Kintinuous (2012) — 超越房间大小

**Volume Shifting**: 相机移出当前体积时，TSDF 体积整体平移 → 无限地图

## 3. ElasticFusion (2015, Whelan et al.)

### 基于 Surfel 的建图

地图 = 非结构化的 Surfel 集合（面元 = 带半径的圆盘）

### 非刚体变形

回环检测到后，不是硬对齐，而是**弹性变形**整个 Surfel 地图。

### 回环检测: 随机蕨编码 (Random Fern Encoding)

对每个关键帧编码 → Fern 数据库搜索 → 回环候选 → 局部非刚体对齐。

## 4. BundleFusion (2016, Dai et al.)

全局优化的 RGB-D SLAM：每一帧都与所有历史帧做匹配 → 局部到全局的层次化优化。

## 5. DSP-SLAM (2021)

结合 ORB-SLAM2 + DeepSDF 形状先验：
- 对检测到的物体，用 DeepSDF（学习到的隐式形状）做稠密重建
- 物体级建图

## 6. ORB-SLAM2/3 RGB-D 模式

使用 RGB-D 深度初始化地图点（无需对极几何+三角化），其他流程与单目一致。

---

> **练习**: `exercises/exercise_01_tsdf_fusion.py`
