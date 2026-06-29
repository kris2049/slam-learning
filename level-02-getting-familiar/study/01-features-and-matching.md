# Level 2 学习教材: SLAM 入门

> 覆盖 visual-slam-roadmap Level 2 的全部内容:
> 特征提取/匹配、多视图几何、非线性优化、因子图、建图、传感器、评测

---

# 第1部分: 特征提取与匹配

## 1.1 关键点检测器

视觉 SLAM 前端的第一步：从图像中提取**可重复**检测的特征点。

### SIFT (Scale-Invariant Feature Transform) — 1999/2004

**核心思想**：在不同尺度空间中寻找稳定的极值点。

**SIFT 四步骤**：

1. **尺度空间极值检测 (Scale-space Extrema Detection)**
   - 用不同 $\sigma$ 的高斯核对图像做模糊 → 构建高斯金字塔
   - 相邻层相减 → DoG (Difference of Gaussian) 金字塔
   - 在 DoG 空间中找局部极值（3×3×3 邻域）

2. **关键点定位 (Keypoint Localization)**
   - 对极值点做精确的亚像素定位（泰勒展开 + 二次拟合）
   - 去除低对比度和边缘响应的点

3. **方向分配 (Orientation Assignment)**
   关键点周围区域计算梯度方向直方图（36 bins）
   - 主方向 = 直方图峰值
   - 实现旋转不变性

4. **描述子生成 (Descriptor Generation)**
   - 将关键点周围 16×16 区域旋转到主方向
   - 分成 4×4 个子区域，每个统计 8 方向梯度直方图
   - → **128 维浮点描述子**

**SIFT 的优缺点**：
- ✅ 对尺度、旋转、光照变化鲁棒
- ✅ 特征区分度极高
- ❌ **很慢**（无法实时，>100ms/帧）
- ❌ 专利保护（until 2020）

### FAST (Features from Accelerated Segment Test) — 2006

**核心思想**：如果圆周上有 12 个连续像素都比中心亮/暗，就是角点。

```
        16
    15      1
  14    圆    2
  13    ·p    3
  12          4
    11      5
      10  6
        9 8 7
```

**FAST 三步**：

1. **快速排除**：检查第1,5,9,13点——如果 ≥3 个不比中心亮/暗，跳过
2. **完整检查**：检查全部16点，看是否有12连续的一致
3. **非极大值抑制**：相邻角点只保留响应最大的

**速度**：比 SIFT 快 100+ 倍，CPU 上 >1000 FPS。

**但 FAST 有缺陷**：
- ❌ 没有方向信息（不是旋转不变）
- ❌ 没有尺度信息（不是尺度不变）
- ❌ 没有描述子（不能直接匹配）

**→ oFAST (oriented FAST)**：计算灰度质心(Intensity Centroid)，赋予方向
**→ 尺度不变**：在图像金字塔每层提取 FAST

### ORB (Oriented FAST + Rotated BRIEF) — 2011

**ORB-SLAM 的基石！**

ORB = oFAST 检测器 + rBRIEF 描述子

**oFAST**：FAST + 灰度质心方向
$$\theta = \arctan\left(\frac{\sum_{y} y \cdot I(x,y)}{\sum_{x} x \cdot I(x,y)}\right)$$

**rBRIEF (rotated BRIEF)**：
- BRIEF = 在关键点周围随机选 256 对像素，比较大小 → 256-bit 二进制串
- rBRIEF：根据方向 $\theta$ 旋转采样模式 → 旋转不变性

**ORB 特征总结**：
- 检测器：oFAST（快速 + 有方向）
- 描述子：rBRIEF（256-bit = 32字节，Hamming 距离匹配）
- 尺度：图像金字塔（8层，缩放因子 1.2）
- 速度：比 SIFT 快 ~100 倍
- 质量：足够好，不如 SIFT 但 SLAM 够用

### AKAZE (Accelerated KAZE) — 2013

非线性扩散滤波（保留边缘），比 SIFT 快，比 ORB 精度高。适合特定场景。

---

## 1.2 描述子匹配

有了两帧的特征描述子，如何找到对应关系？

### Brute-Force (暴力匹配)

计算描述子 A 中每个特征与 B 中所有特征的距离，取最近的。

$$match_i = \arg\min_j \|desc_A^i - desc_B^j\|$$

- 浮点描述子 (SIFT) 用 L2 距离
- 二进制描述子 (ORB) 用 **Hamming 距离**（XOR 后数1的位数）

### FLANN (Fast Library for Approximate Nearest Neighbors)

**近似最近邻搜索**：牺牲一点精度，换 10-100× 加速。

内部使用：
- **Kd-Tree**：将描述子空间树状划分，快速定位最近邻
- **LSH (Locality Sensitive Hashing)**：将相似描述子哈希到同一桶
- **HBST (Hierarchical Clustering)**：层次聚类树

### Lowe's Ratio Test（黄金标准）

**一个匹配是否可靠？**

检查最近邻距离 $d_1$ 和次近邻距离 $d_2$：

$$\frac{d_1}{d_2} < 0.7 \ (\text{或}0.8)$$

直觉：如果最近的匹配明显比第二近的好 → 可靠。
如果两者差不多 → 可能是模糊/重复纹理 → 丢弃。

**Lowe's ratio test 是 SLAM 中最常用、最重要的匹配质量过滤**。

---

## 1.3 深度学习特征 (Level 5 预览)

### SuperPoint (2018) — 自监督关键点检测+描述

- Homographic Adaptation：对图像随机做单应变换，要求检测器一致
- 共享 VGG 编码器 + 检测头 + 描述头
- 描述子 256 维浮点

### SuperGlue (2020) — 基于 GNN 的特征匹配

- 将匹配问题建模为**图的最优传输**
- Self-Attention → 特征增强
- Cross-Attention → 寻找对应
- Sinkhorn 算法 → 软分配矩阵

### LightGlue (2023) — SuperGlue 加速版

- 自适应深度/宽度：简单匹配对提前停止
- 5-10× 加速，精度与 SuperGlue 持平

### 实践建议

ORB-SLAM = ORB 特征（传统方法，成熟稳定）
如果做研究 → SuperPoint + LightGlue（深度学习，SOTA）

---

## 1.4 光流跟踪 (Optical Flow)

**与特征匹配的根本区别**：

| | 特征匹配 | 光流跟踪 |
|---|---------|----------|
| 输入 | 两张图像，各自提取特征 | 第一帧的特征位置 + 两张图像 |
| 方法 | 描述子距离 | 亮度恒定假设 + 最小二乘 |
| 速度 | 较慢（需提取+匹配） | 快（只优化位置） |
| 鲁棒性 | 对遮挡/大运动鲁棒 | 对光照变化敏感 |

### KLT 光流 (Kanade-Lucas-Tomasi Tracker)

**核心假设**：
1. **亮度恒定**：$I(x,y,t) = I(x+dx, y+dy, t+dt)$
2. **小运动**：$(dx, dy)$ 很小 → 可用一阶泰勒展开
3. **空间一致性**：相邻像素运动相同

**数学推导**：

$$I(x+dx, y+dy, t+dt) = I(x,y,t) + I_x dx + I_y dy + I_t dt = I(x,y,t)$$
$$\Rightarrow I_x u + I_y v = -I_t$$

其中 $u=dx/dt, v=dy/dt$，$I_x, I_y$ 是图像梯度，$I_t$ 是时间梯度。

对窗口内所有像素：
$$\begin{bmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{bmatrix} \begin{bmatrix} u \\ v \end{bmatrix} = -\begin{bmatrix} \sum I_x I_t \\ \sum I_y I_t \end{bmatrix}$$

最小二乘解 → $(u, v)$。

**金字塔 KLT**（实践中的标准做法）：
在高斯金字塔上从粗到细迭代 → 处理大运动。

---

## 1.5 词袋模型 (Bag of Visual Words, BoVW)

用于**回环检测**（识别以前见过的地方）。

### 构建过程

1. **离线训练**：从大量图像提取 ORB 描述子 → K-means 聚类 → 生成**视觉词典**（如 $10^6$ 个词）
2. **在线使用**：每帧图像的 ORB 描述子 → 量化到最近的视觉词 → 词频直方图

### 相似度计算

两帧的 BoVW 向量 $v_1, v_2$：
$$\text{score} = 1 - \frac{1}{2}\left|\frac{v_1}{|v_1|} - \frac{v_2}{|v_2|}\right|$$

### DBoW2/DBoW3（ORB-SLAM 使用）

- 层次化词汇树（Hierarchical Vocabulary Tree）
- 快速检索（毫秒级）
- 存储每帧的 BoVW（用于回环候选）
- 存储`关键帧→视觉词→特征`的倒排索引（用于快速特征匹配）

---

> **练习**: `exercises/exercise_01_feature_matching.py`
