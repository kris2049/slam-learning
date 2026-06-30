# 第2部分: 非线性优化 — SLAM 的数学引擎

> SLAM 后端的所有问题最终都归结为**最优化**：找到最优的相机位姿和3D点位置。
> 本部分是最重要的理论基础。

---

## 2.1 最小二乘优化

### 2.1.1 SLAM 的优化目标

SLAM 中有两类变量要优化：
- $\mathbf{X} = \{x_1, ..., x_N\}$：相机位姿（每个 $x_i \in SE(3)$）
- $\mathbf{L} = \{l_1, ..., l_M\}$：3D路标点

目标：最小化**重投影误差**

$$\min_{\mathbf{X}, \mathbf{L}} \sum_{i=1}^N \sum_{j \in \text{obs}(i)} \| \mathbf{z}_{ij} - \pi(x_i, l_j) \|^2_{\Sigma_{ij}}$$

其中：
- $\mathbf{z}_{ij}$：相机 $i$ 观测到路标 $j$ 的像素坐标
- $\pi(x_i, l_j)$：将路标 $j$ 投影到相机 $i$ 的预测像素
- $\|\cdot\|_{\Sigma}$：马氏距离（加权L2范数，$\Sigma$ 是信息矩阵）

### 2.1.2 非线性最小二乘

一般形式：
$$\min_{\theta} \sum_{k=1}^K \|r_k(\theta)\|^2 = \min_{\theta} \|r(\theta)\|^2$$

其中 $r_k(\theta)$ 是第 $k$ 个残差（观测 - 预测）。

**核心困难**：$r_k(\theta)$ 是非线性的（投影是透视变换） → 不能直接求解。

**解决方案**：迭代线性化 → 高斯-牛顿法 (Gauss-Newton) / Levenberg-Marquardt

---

## 2.2 高斯-牛顿法 (Gauss-Newton, GN)

### 算法推导

将残差在 $\theta_k$ 处一阶泰勒展开：

$$r(\theta_k + \Delta\theta) \approx r(\theta_k) + J(\theta_k) \Delta\theta$$

其中 $J = \frac{\partial r}{\partial \theta}$ 是**雅可比矩阵**。

代入代价函数：
$$\|r + J\Delta\theta\|^2 = (r + J\Delta\theta)^\top(r + J\Delta\theta)$$
$$= r^\topr + 2r^\topJ\Delta\theta + \Delta\theta^\topJ^\topJ\Delta\theta$$

对 $\Delta\theta$ 求导并令其为零：
$$\frac{\partial}{\partial \Delta\theta} = 2J^\topr + 2J^\topJ\Delta\theta = 0$$

$$\Rightarrow J^\topJ \Delta\theta = -J^\topr$$

$$\Rightarrow H \Delta\theta = -g$$

其中 $H = J^\topJ$ 是近似的**Hessian矩阵**，$g = J^\topr$ 是梯度。

### GN 算法流程

```
1. 给定初始值 θ₀
2. for k = 0, 1, 2, ...:
   3. 计算残差 r(θₖ) 和雅可比 J(θₖ)
   4. 构建 H = JᵀJ, g = Jᵀr
   5. 求解 H Δθ = -g  →  Δθ
   6. 更新 θₖ₊₁ = θₖ + Δθ
   7. if |Δθ| < ε: break
```

### GN 的优缺点

- ✅ 不需要计算 Hessian（二阶导）
- ✅ $H = J^\topJ$ 至少半正定
- ❌ $H$ 可能奇异（$J$ 列不满秩）→ 无法求解
- ❌ 步长可能太大，导致发散

---

## 2.3 Levenberg-Marquardt (LM)

**LM = GN + 阻尼**

$$(J^\topJ + \lambda I) \Delta\theta = -J^\topr$$

$\lambda$ 是**阻尼因子**：
- $\lambda$ 大 → 退化为梯度下降（步长小，方向 = -梯度）→ 稳定但慢
- $\lambda$ 小 → 近似高斯-牛顿 → 快但可能不稳定

**$\lambda$ 的自适应调整**：

- 如果新误差 < 旧误差：接受更新，$\lambda \leftarrow \lambda / 2$
- 如果新误差 > 旧误差：拒绝更新，$\lambda \leftarrow 2\lambda$

这就是 Levenberg-Marquardt 的核心：**在 GN（快）和 GD（稳）之间自动切换**。

---

## 2.4 李群与李代数 — 在位姿空间上做优化

### 2.4.1 为什么需要李代数？

优化变量是**位姿** $T \in SE(3)$。但 $SE(3)$ 不是线性空间：

- $T_1 + T_2$ 无意义（$T_1 + T_2 \notin SE(3)$）
- 直接用矩阵做优化不保证 $R^\topR = I, \det(R)=1$

**解决方案**：在位姿的**切空间**上做优化。

### 2.4.2 关键映射

| 映射 | 方向 | 含义 |
|------|------|------|
| $\exp(\xi^\wedge)$ | se(3) → SE(3) | 6维向量 → 4×4变换矩阵 |
| $\log(T)$ | SE(3) → se(3) | 4×4矩阵 → 6维向量 |

$$\xi = \begin{bmatrix} \rho \\ \phi \end{bmatrix} \in \mathbb{R}^6$$

- $\rho$：平移分量
- $\phi$：旋转分量（旋转向量）

### 2.4.3 扰动模型（实际使用的方式）

在位姿 $T$ 上施加微小扰动 $\delta\xi$：

$$T(\delta\xi) = \exp(\delta\xi^\wedge) \cdot T$$

$$\frac{\partial (Tp)}{\partial \delta\xi} = [I_{3\times3} \mid -(Rp)^\wedge]$$

这就是 **SE(3) 上的雅可比**。SLAM 优化器就是在 se(3) 上迭代更新 6 维向量。

### 2.4.4 GN/LM 在 SE(3) 上的一步迭代

```
当前位姿: T ∈ SE(3)
1. 计算残差 r(T) 和雅可比 J（在 se(3) 切空间）
2. 求解 H Δξ = -g
3. 更新 T ← exp(Δξ^) · T
```

---

## 2.5 Bundle Adjustment (BA) — 一切优化之母

**BA = 同时优化所有相机位姿和所有3D点**

$$\min_{\{T_i\}, \{P_j\}} \sum_{i,j} \|\pi(T_i, P_j) - z_{ij}\|^2$$

### BA 的 H 矩阵结构

假设 $N$ 个相机，$M$ 个路标。变量排列：$[T_1,...,T_N, P_1,...,P_M]$。

$$H = \begin{bmatrix} B & E \\ E^\top & C \end{bmatrix}$$

- **B**：相机-相机块（N×N 块对角，每块 6×6）
- **C**：路标-路标块（M×M 块对角，每块 3×3）
- **E**：相机-路标交叉项

**关键观察**：$H$ 高度稀疏！

- $E$ 只在「相机 $i$ 观测到路标 $j$」处非零
- $C$ 是块对角的（路标之间无关联）

### 舒尔补 (Schur Complement) — BA 加速的核心

由于 $C$ 是块对角的（容易求逆），可以先消去路标变量：

$$S \Delta\mathbf{x}_c = \mathbf{b}_c - E C^{-1} \mathbf{b}_p$$

其中 $S = B - E C^{-1} E^\top$（**舒尔补**）。

1. 求解小系统 $S \Delta\mathbf{x}_c = ...$ → 相机更新
2. 回代 $\Delta\mathbf{x}_p = C^{-1}(\mathbf{b}_p - E^\top\Delta\mathbf{x}_c)$ → 路标更新

**复杂度**：$O(N^3)$ 而非 $O((N+M)^3)$，巨大加速！

### 边缘化 (Marginalization) — 滑动窗口 BA 的基础

从窗口中移除旧帧时，不能直接丢弃。需要用 Schur 补将其信息**压缩**到剩余变量上：

$$H_{marg} = H_{bb} - H_{ab}^\top H_{aa}^{-1} H_{ab}$$

边缘化产生 **fill-in**：原本稀疏的 $H_{bb}$ 变得稠密 → 不再是精确稀疏。
DSO 和 VINS-Mono 使用边缘化维持固定大小的滑动窗口。

---

## 2.6 位姿图优化 (Pose Graph Optimization, PGO)

回环检测后，需要全局修正所有位姿。

只优化**位姿之间的相对约束**（里程计边 + 回环边），不优化路标：

$$\min_{\{T_i\}} \sum_{(i,j) \in \mathcal{E}} \|\log(T_{ij}^{-1} T_i^{-1} T_j)\|^2$$

其中 $T_{ij}$ 是里程计测量的位姿 $i$ 到 $j$ 的相对变换。

**PGO 的特点**：
- 变量数 = 位姿数（远小于 BA）
- 回环边提供长距离约束 → 消除累积漂移
- ORB-SLAM 的回环后全局优化就是 PGO

---

## 2.7 鲁棒核函数 (Robust Kernel)

现实中有外点（错误匹配），平方误差会被外点主导。

**解决方案**：用鲁棒核函数限制外点的影响。

| 核函数 | 公式 | 特点 |
|--------|------|------|
| Huber | $\rho(e) = \begin{cases} e^2/2 & |e|<\delta \\ \delta(|e|-\delta/2) & \text{else} \end{cases}$ | 平滑过渡 |
| Cauchy | $\rho(e) = \frac{\delta^2}{2}\log(1+(e/\delta)^2)$ | 更激进地抑制 |
| Tukey | 超过阈值直接置0 | 完全拒绝外点 |

ORB-SLAM 使用 Huber 核。

---

> **练习**: `exercises/exercise_03_nonlinear_optimization.py`
