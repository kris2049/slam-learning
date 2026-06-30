# 模块1: SLAM 所需的数学基础

> 本模块覆盖 visual-slam-roadmap Level 1 中「Mathematics」的全部知识点。
> 目标是让你能看懂 SLAM 论文中的所有数学公式。

---

## 1.1 线性代数 — SLAM 的骨架

SLAM 中一切都在**向量空间**中进行：3D 点坐标、相机速度、平移向量都是向量；旋转、变换都是矩阵。

### 1.1.1 向量 (Vector)

物理含义：空间中的一个点或方向。

```
a = [3, 0, 0]ᵀ    # 沿X轴的单位方向 (模长为3)
b = [0, 4, 0]ᵀ    # 沿Y轴
```

**向量模长 (L2 Norm)**

$$\|a\| = \sqrt{a_1^2 + a_2^2 + a_3^2} = \sqrt{9 + 0 + 0} = 3$$

在 SLAM 中，两个 3D 点之间的欧氏距离就是差的模长。

**点积 (Dot Product)**

$$a \cdot b = a^\top b = a_1b_1 + a_2b_2 + a_3b_3$$

几何意义：
$a \cdot b = \| a \| \cdot \| b \| \cdot \cos\theta$

- 若 $a \cdot b = 0$，则两向量**正交**（SLAM中常用于判断方向是否垂直）
- 若 $a \cdot b > 0$，夹角 < 90°
- 用于计算**重投影误差**中像素误差的平方和

**叉积 (Cross Product)**

$$a \times b = \begin{bmatrix} a_2b_3 - a_3b_2 \,  a_3b_1 - a_1b_3 \,  a_1b_2 - a_2b_1 \end{bmatrix}$$

几何意义：结果向量垂直于 a 和 b，模长 = $\|a\| \cdot \|b\| \cdot \sin\theta$

SLAM 用途：
- 计算**平面法向量**（两向量叉积即得法向量）
- 构造**反对称矩阵**（用于对极几何中的本质矩阵）

**反对称矩阵 (Skew-Symmetric Matrix)**

任意向量 $v = [x, y, z]^\top$ 可构造：

$$v^\wedge = \begin{bmatrix} 0 & -z & y \\\\ z & 0 & -x \\\\ -y & x & 0 \end{bmatrix}$$

性质：
$a^\wedge b = a \times b$（反对称矩阵乘向量 = 叉积）

这正是**本质矩阵** $E = t^\wedge R$ 中的 $t^\wedge$。

---

### 1.1.2 矩阵 (Matrix)

**SLAM 中最重要的矩阵类型：**

| 矩阵类型 | 符号 | 用途 |
|----------|------|------|
| 旋转矩阵 | $R$ | $R^\top R = I$, $\det(R)=1$ 3D旋转变换|
| 相机内参 | $K$ | 将3D相机坐标映射到2D像素 |
| 投影矩阵 | $P = K[R \mid t]$ | 世界3D → 图像2D |
| Hessian矩阵 | $H = J^\top J$ | 优化中的二阶信息 |
| 协方差矩阵 | $\Sigma$ | 描述状态不确定性 |

**矩阵的秩 (Rank)**

矩阵的秩 = 线性无关的行/列数。

- SLAM 中 $E$ 矩阵秩必须为2（八点法恢复后要强制 SVD 置最小奇异值为0）
- $H$ 矩阵的秩决定了解的唯一性（秩不足 → 需要先验约束）

**行列式 (Determinant)**

- $\det(R) = 1$：旋转矩阵保持体积
- $\det(H)$：优化问题的可解性指标

**逆矩阵**

- 旋转矩阵的特殊性质: $R^{-1} = R^\top$（正交矩阵）
- 一般矩阵求逆用 LU 分解或 SVD
- SLAM 中 $K^{-1}$ 用于从像素坐标恢复归一化相机坐标

**QR 分解**

$$A = QR$$

Q 是正交矩阵，R 是上三角矩阵。SLAM 中用于：
- 最小二乘问题的稳定求解

---

### 1.1.3 奇异值分解 (SVD) — 最重要！

$$A = U \Sigma V^\top$$

- $U$: 左奇异向量($A$ 的列空间正交基)
- $\Sigma$: 对角矩阵，非负奇异值降序排列
- $V$: 右奇异向量($A$ 的行空间正交基)

**SLAM 中的4个关键用途：**

1. **求解齐次线性方程 $Ax = 0$（八点法求 F/E 矩阵）**
   解 = $V$ 的最小奇异值对应的列（最后一列）

2. **求解非齐次方程 $Ax = b$ 的最小二乘解**
   $x = V \Sigma^{-1} U^\top b$

3. **ICP 中从点云对应求最优刚体变换**
   $H = \sum (p_i - \bar{p})(q_i - \bar{q})^\top$，SVD 分解 H 即得最优旋转

4. **强制矩阵秩约束**
   如 $E$ 矩阵秩必须为2 → 置最小奇异值为0再重构

**实践示例（从八点法求 F 矩阵）：**
```python
A = ...  # 8×9矩阵，每行来自一对匹配点的对极约束
_, _, Vt = np.linalg.svd(A)
F = Vt[-1].reshape(3, 3)      # 最小奇异值对应的解

# 强制秩为2
U, S, Vt = np.linalg.svd(F)
S[2] = 0                       # 最小奇异值置零
F_rank2 = U @ np.diag(S) @ Vt  # 合法的本质矩阵
```

---

### 1.1.4 特征值与特征向量

$$A v = \lambda v$$

- $v$：特征向量（变换后方向不变）
- $\lambda$：特征值（变换后的缩放倍数）

**SLAM 中的关键应用：**

1. **Harris 角点检测**
   图像梯度协方差矩阵 $M$ 的两个特征值：
   - $\lambda_1, \lambda_2$ 都大 → **角点**（两个方向都有大梯度）
   - $\lambda_1 \gg \lambda_2$ 或反之 → **边缘**（只一个方向有大梯度）
   - 都小 → **平坦区域**

   Harris 响应: $R = \lambda_1\lambda_2 - k(\lambda_1+\lambda_2)^2$
   等价于 $R = \det(M) - k \cdot \text{trace}(M)^2$

2. **PCA（主成分分析）**
   协方差矩阵的特征向量 = 数据的主要方向
   SLAM 中用于点云法向量估计

3. **可观性分析 (Observability Analysis)**
   VIO 系统中 FIM (Fisher Information Matrix) 的特征值：
   - 零特征值 → 对应不可观状态（如VIO中的全局偏航角）

---

### 1.1.5 刚体变换与齐次坐标

**旋转矩阵 R ∈ SO(3)**

$$SO(3) = \{R \in \mathbb{R}^{3\times3} \mid R^\top R = I,\ \det(R) = 1\}$$

3个自由度（绕X、Y、Z轴各一个旋转角）。

绕 Z 轴旋转 $\theta$:

$$R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\\\ \sin\theta & \cos\theta & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$

**齐次变换矩阵 T ∈ SE(3)**

$$T = \begin{bmatrix} R & t \\\\ 0 & 1 \end{bmatrix} \in \mathbb{R}^{4\times4}$$

6个自由度（3旋转 + 3平移）。

变换一个点: $p' = T p$，其中 $p$ 是齐次坐标 $[x, y, z, 1]^\top$

**变换合成**：
$$T_{ac} = T_{ab} \cdot T_{bc}$$

世界点 → 相机坐标：
$$P_{cam} = T_{cw} \cdot P_{world} = T_{wc}^{-1} \cdot P_{world}$$

**旋转的三种表示（欧拉角 / 旋转向量 / 四元数）← 都表示 SO(3)**

| 表示 | 参数数 | 优点 | 缺点 |
|------|--------|------|------|
| 欧拉角 | 3 | 直观 | 万向锁，不连续 |
| 旋转向量 | 3 | 紧凑 | 奇异点(θ=0附近) |
| 四元数 | 4 | 无奇异性，平滑插值 | 不直观 |

**SLAM 实践**: ORB-SLAM 使用四元数 + 平移向量表示位姿；优化时在 se(3) 李代数上进行（6维向量）。

---

## 1.2 概率与统计 — SLAM 的不确定性语言

SLAM 的本质是概率推断问题：

$$P(\text{map}, \text{pose} \mid \text{observation})$$

> 给定传感器观测，地图和轨迹的**后验概率**最大是多少？

### 1.2.1 高斯分布 (正态分布)

**一维高斯**：
$$p(x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$$

- $\mu$：均值（最可能的值）
- $\sigma$：标准差（不确定性度量）

**为什么 SLAM 几乎只用高斯分布？**
1. 中心极限定理：大量独立误差之和 ≈ 高斯
2. 高斯分布在边缘化和条件化下封闭（卡尔曼滤波的基础）
3. 负对数把乘积转为平方和 → 最小二乘优化

**68-95-99.7 规则**：
- $[\mu-\sigma, \mu+\sigma]$ 包含 ~68% 的概率
- $[\mu-2\sigma, \mu+2\sigma]$ 包含 ~95% 的概率
- $[\mu-3\sigma, \mu+3\sigma]$ 包含 ~99.7% 的概率

**多元高斯**:

$$p(\mathbf{x}) = \frac{1}{(2\pi)^{n/2}|\Sigma|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x}-\mu)^\top\Sigma^{-1}(\mathbf{x}-\mu)\right)$$

- $\mu$：n维均值向量
- $\Sigma$：n×n协方差矩阵（描述各维度间的关联）

SLAM 中：相机位姿的协方差矩阵描述了你的**不确定性椭球**。

**协方差矩阵的几何意义**：
- 对角元素：各变量的方差
- 非对角元素：变量间的相关性
- 特征值决定椭球轴长，特征向量决定椭球方向

---

### 1.2.2 贝叶斯定理

$$P(A \mid B) = \frac{P(B \mid A) \cdot P(A)}{P(B)}$$

- $P(A)$：**先验**（看到任何数据之前的信念）
- $P(B \mid A)$：**似然**（给定A，B出现的概率）
- $P(A \mid B)$：**后验**（看到B后，对A的更新信念）

**SLAM 的贝叶斯框架**：

$$\underbrace{P(X_{1:t}, M \mid Z_{1:t}, U_{1:t})}_{\text{posterior: }P(\text{map, trajectory} \mid \text{observations, controls})} \propto \underbrace{P(Z_t \mid X_t, M)}_{\text{observation model}} \cdot \underbrace{P(X_t \mid X_{t-1}, U_t)}_{\text{motion model}}$$

> 每次新的传感器数据到来，就用贝叶斯定理**更新**我们对世界状态的信念。

---

### 1.2.3 最大似然估计 (MLE) 与最大后验估计 (MAP)

**MLE（最大似然）**:

$$\hat{\theta}_{\text{MLE}} = \arg\max_\theta P(D \mid \theta)$$

「哪种参数最可能产生我们观察到的数据？」

SLAM 中：给定匹配点，最可能的相机位姿 → **这就是 PnP 的精神！**

**MAP（最大后验）**:

$$\hat{\theta}_{\text{MAP}} = \arg\max_\theta P(\theta \mid D) = \arg\max_\theta P(D \mid \theta)P(\theta)$$

比 MLE 多了一个先验 $P(\theta)$。

SLAM 中：
- 纯 BA = MLE（最小化重投影误差）
- 带先验的 BA = MAP（如固定第一帧位姿）

**为什么最小化平方误差等同于 MLE？**

假设观测噪声是高斯分布: 

$z = h(x) + \epsilon,\ \epsilon \sim \mathcal{N}(0, \sigma^2)$

$$P(z \mid x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(z - h(x))^2}{2\sigma^2}\right)$$

取负对数：
$$-\log P(z \mid x) = \frac{(z - h(x))^2}{2\sigma^2} + \text{const}$$

所以：
$$\arg\max_x P(z \mid x) = \arg\min_x (z - h(x))^2$$

**这就是 Bundle Adjustment 的数学本质！** 最小化重投影误差 ≈ 最大化高斯噪声假设下的似然。

---

### 1.2.4 泰勒展开

$$f(x) \approx f(x_0) + f'(x_0)(x - x_0) + \frac{1}{2}f''(x_0)(x - x_0)^2 + \cdots$$

SLAM 中的关键应用：
- **一阶泰勒** → 线性化非线性函数（高斯-牛顿法的基础）
- **二阶泰勒** → Hessian矩阵（牛顿法）
- **李代数扰动** → 在 se(3) 上用一阶泰勒将位姿优化问题线性化

---

## 1.3 微积分

### 1.3.1 导数与梯度

- 一阶导数 → 变化率
- 梯度 $\nabla f$ → 多变量函数的最速上升方向
- SLAM 中：**雅可比矩阵 J** 就是多维函数对所有变量的偏导数矩阵

### 1.3.2 对数与指数

- $\ln(e^x) = x$
- $\ln(ab) = \ln a + \ln b$

SLAM 中：
- 取对数把乘积变加和 → 负对数似然 → 最小二乘
- $\exp$ 映射 se(3) → SE(3)（李代数到李群）
- $\log$ 映射 SE(3) → se(3)（李群到李代数）

---

## 📝 自检清单

在进入 Level 2 之前，确保你能回答：

- [ ] 叉积在 SLAM 中的两个核心用途是什么？
- [ ] SVD 分解后如何求解 $Ax=0$？
- [ ] 为什么旋转矩阵的逆等于转置？
- [ ] 协方差矩阵的特征值告诉你什么？
- [ ] 为什么最小化重投影误差 = 最大似然估计？
- [ ] 多元高斯分布在 SLAM 中代表什么？
- [ ] 齐次变换矩阵 T 有哪四个部分？各有什么含义？

---

> **下一步**: 完成 `exercises/exercise_01_linear_algebra.py` 和 `exercises/exercise_02_probability.py` 来巩固这些数学概念。
