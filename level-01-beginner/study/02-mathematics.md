# 模块2: SLAM 所需的数学基础

> 本模块覆盖 visual-slam-roadmap Level 1 中「Mathematics」的全部知识点。
> 目标是让你能看懂 SLAM 论文中的所有数学公式。
> 每个公式都配有**完整的数值计算示例**。

---

## 1.1 线性代数 — SLAM 的骨架

SLAM 中一切都在**向量空间**中进行：3D 点坐标、相机速度、平移向量都是向量；旋转、变换都是矩阵。

### 1.1.1 向量 (Vector)

物理含义：空间中的一个点或方向。

```
a = [3, 0, 0]ᵀ    # 沿X轴的方向 (模长为3)
b = [0, 4, 0]ᵀ    # 沿Y轴
```

#### 向量模长 (L2 Norm)

$$\|a\| = \sqrt{a_1^2 + a_2^2 + a_3^2}$$

> **示例 1** — 计算向量模长
>
> 给定
> $$a = [3, 0, 0]^\top$$:
> $$\|a\| = \sqrt{3^2 + 0^2 + 0^2} = \sqrt{9} = 3$$
>
> 给定
> $$v = [1, 2, 2]^\top$$:
> $$\|v\| = \sqrt{1^2 + 2^2 + 2^2} = \sqrt{1 + 4 + 4} = \sqrt{9} = 3$$
>
> 在 SLAM 中，两个 3D 点
> $$P_1=[1,2,3]^\top$$,
> $$P_2=[4,6,3]^\top$$
> 之间的欧氏距离:
> $$\|P_2 - P_1\| = \|[3, 4, 0]^\top\| = \sqrt{9 + 16 + 0} = \sqrt{25} = 5$$

#### 点积 (Dot Product)

$$a \cdot b = a^\top b = a_1b_1 + a_2b_2 + a_3b_3$$

几何意义：
$$a \cdot b = \| a \| \cdot \| b \| \cdot \cos\theta$$

> **示例 2** — 点积与夹角计算
>
> $a = [1, 2, 3]^\top$, $b = [4, -5, 6]^\top$
>
> $$a \cdot b = 1\times4 + 2\times(-5) + 3\times6 = 4 - 10 + 18 = 12$$
>
> 模长:
> $$\|a\| = \sqrt{1+4+9} = \sqrt{14} \approx 3.742$$,
> $$\|b\| = \sqrt{16+25+36} = \sqrt{77} \approx 8.775$$
>
> 夹角:
> $$\cos\theta = \frac{a \cdot b}{\|a\|\|b\|} = \frac{12}{3.742 \times 8.775} \approx 0.3653$$,
> $$\theta \approx 68.6^\circ$$
>
> ---
>
> **示例 2b** — 正交判断 (SLAM 中常用)
>
> $a = [1, 0, 0]^\top$, $b = [0, 1, 0]^\top$
>
> $$a \cdot b = 1\times0 + 0\times1 + 0\times0 = 0$$
>
> 点积为0 → 两向量**正交**（垂直）。X轴和Y轴互相垂直。

- 若
- $$a \cdot b = 0$$
- ，则两向量**正交**（SLAM中常用于判断方向是否垂直）
- 若
- $$a \cdot b > 0$$
- ，夹角 < 90°
- 用于计算**重投影误差**中像素误差的平方和

#### 叉积 (Cross Product)

$$a \times b = \begin{bmatrix} a_2b_3 - a_3b_2 \\ a_3b_1 - a_1b_3 \\ a_1b_2 - a_2b_1 \end{bmatrix}$$

几何意义：结果向量垂直于 a 和 b，模长 =
$$\|a\| \cdot \|b\| \cdot \sin\theta$$

> **示例 3** — 叉积计算
>
> $a = [1, 2, 3]^\top$, $b = [4, 5, 6]^\top$
>
> $$a \times b = \begin{bmatrix} 2\times6 - 3\times5 \\ 3\times4 - 1\times6 \\ 1\times5 - 2\times4 \end{bmatrix} = \begin{bmatrix} 12 - 15 \\ 12 - 6 \\ 5 - 8 \end{bmatrix} = \begin{bmatrix} -3 \\ 6 \\ -3 \end{bmatrix}$$
>
> 验证垂直性:
> $$(a \times b) \cdot a = -3\times1 + 6\times2 + (-3)\times3 = -3+12-9 = 0$$
> ✓
>
> SLAM 用途：
> - 计算**平面法向量**：空间中两个非平行向量叉积即得法向量。如 a=[1,0,0]^T, b=[0,1,0]^T，则 a×b=[0,0,1]^T（Z轴法向量，即XY平面的法向）
> - 构造**反对称矩阵**（用于对极几何中的本质矩阵）

#### 反对称矩阵 (Skew-Symmetric Matrix)

任意向量
$$v = [x, y, z]^\top$$
可构造：

$$v^\wedge = \begin{bmatrix} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \end{bmatrix}$$

性质：
$$a^\wedge b = a \times b$$
（反对称矩阵乘向量 = 叉积）

> **示例 4** — 构造反对称矩阵并验证
>
> $v = [1, 2, 3]^\top$:
> $$v^\wedge = \begin{bmatrix} 0 & -3 & 2 \\ 3 & 0 & -1 \\ -2 & 1 & 0 \end{bmatrix}$$
>
> 验证
> $$v^\wedge b = v \times b$$
> （用
> $$b=[4,5,6]^\top$$
> ）:
> $$v^\wedge b = \begin{bmatrix} 0 & -3 & 2 \\ 3 & 0 & -1 \\ -2 & 1 & 0 \end{bmatrix} \begin{bmatrix} 4 \\ 5 \\ 6 \end{bmatrix} = \begin{bmatrix} 0\times4 + (-3)\times5 + 2\times6 \\ 3\times4 + 0\times5 + (-1)\times6 \\ (-2)\times4 + 1\times5 + 0\times6 \end{bmatrix} = \begin{bmatrix} -15+12 \\ 12-6 \\ -8+5 \end{bmatrix} = \begin{bmatrix} -3 \\ 6 \\ -3 \end{bmatrix}$$
>
> 这与示例3中
> $$v \times b$$
> 结果一致 ✓
>
> 这正是**本质矩阵**
> $$E = t^\wedge R$$
> 中的
> $$t^\wedge$$

---

### 1.1.2 矩阵 (Matrix)

**SLAM 中最重要的矩阵类型：**

| 矩阵类型 | 符号 | 用途 |
|----------|------|------|
| 旋转矩阵 |
$$R$$
|
$$R^\top R = I$$,
$$\det(R)=1$$
3D旋转变换|
| 相机内参 |
$$K$$
| 将3D相机坐标映射到2D像素 |
| 投影矩阵 |
$$P = K[R \mid t]$$
| 世界3D → 图像2D |
| Hessian矩阵 |
$$H = J^\top J$$
| 优化中的二阶信息 |
| 协方差矩阵 |
$$\Sigma$$
| 描述状态不确定性 |

#### 矩阵的秩 (Rank)

矩阵的秩 = 线性无关的行/列数。

> **示例 5** — 计算矩阵的秩
>
> $$A = \begin{bmatrix} 1 & 2 & 3 \\ 2 & 4 & 6 \\ 3 & 6 & 9 \end{bmatrix}$$
>
> 观察：第2行 = 第1行 × 2，第3行 = 第1行 × 3 → 只有1个独立行 →
> $$\text{rank}(A) = 1$$
>
> $$B = \begin{bmatrix} 1 & 2 & 3 \\ 4 & 5 & 6 \\ 7 & 8 & 9 \end{bmatrix}$$
>
> 第3行 = 2×第2行 − 第1行 (
> $$[14,16,18] - [1,2,3] = [13,14,15]$$
> ≠ [7,8,9])... 实际上这个矩阵的秩=2（行列式为0，但前两行线性无关），因为
> $$\det(B) = 1(45-48) - 2(36-42) + 3(32-35) = -3 + 12 - 9 = 0$$

- SLAM 中
- $$E$$
- 矩阵秩必须为2（八点法恢复后要强制 SVD 置最小奇异值为0）
- $$H$$
- 矩阵的秩决定了解的唯一性（秩不足 → 需要先验约束）

#### 行列式 (Determinant)

> **示例 6** — 计算行列式
>
> 2×2 矩阵:
> $$\det\begin{bmatrix} a & b \\ c & d \end{bmatrix} = ad - bc$$
>
> $$A = \begin{bmatrix} 2 & 1 \\ 3 & 4 \end{bmatrix} \quad \Rightarrow \quad \det(A) = 2\times4 - 1\times3 = 8 - 3 = 5$$
>
> 3×3 矩阵:
> $$R_z(30^\circ) = \begin{bmatrix} \cos30^\circ & -\sin30^\circ & 0 \\ \sin30^\circ & \cos30^\circ & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0.866 & -0.5 & 0 \\ 0.5 & 0.866 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$\det(R_z) = 0.866 \times (0.866\times1 - 0\times0) - (-0.5) \times (0.5\times1 - 0\times0) + 0 \times (...)$$
> $$= 0.866 \times 0.866 + 0.5 \times 0.5 = 0.75 + 0.25 = 1$$
>
> 旋转矩阵
> $$\det(R) = 1$$
> （保持体积和手性） ✓

- $$\det(R) = 1$$
- ：旋转矩阵保持体积
- $$\det(H)$$
- ：优化问题的可解性指标

#### 逆矩阵

- 旋转矩阵的特殊性质:
- $$R^{-1} = R^\top$$
- （正交矩阵）
- 一般矩阵求逆用 LU 分解或 SVD
- SLAM 中
- $$K^{-1}$$
- 用于从像素坐标恢复归一化相机坐标

> **示例 7** — 计算逆矩阵
>
> $$A = \begin{bmatrix} 2 & 1 \\ 3 & 4 \end{bmatrix}$$
>
> 2×2 逆矩阵公式:
> $$A^{-1} = \frac{1}{\det(A)}\begin{bmatrix} d & -b \\ -c & a \end{bmatrix} = \frac{1}{ad-bc}\begin{bmatrix} d & -b \\ -c & a \end{bmatrix}$$
>
> $$A^{-1} = \frac{1}{5}\begin{bmatrix} 4 & -1 \\ -3 & 2 \end{bmatrix} = \begin{bmatrix} 0.8 & -0.2 \\ -0.6 & 0.4 \end{bmatrix}$$
>
> 验证:
> $$A A^{-1} = \begin{bmatrix} 2 & 1 \\ 3 & 4 \end{bmatrix} \begin{bmatrix} 0.8 & -0.2 \\ -0.6 & 0.4 \end{bmatrix} = \begin{bmatrix} 1.6-0.6 & -0.4+0.4 \\ 2.4-2.4 & -0.6+1.6 \end{bmatrix} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$$
> ✓
>
> ---
>
> **示例 7b** — 旋转矩阵的特殊性
>
> $$R = R_z(30^\circ) = \begin{bmatrix} 0.866 & -0.5 & 0 \\ 0.5 & 0.866 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$R^{-1} = R^\top = \begin{bmatrix} 0.866 & 0.5 & 0 \\ -0.5 & 0.866 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
> （即旋转
> $$-30^\circ$$
> ，因为
> $$\cos(-30^\circ)=\cos30^\circ$$,
> $$\sin(-30^\circ)=-\sin30^\circ$$
> ）

#### QR 分解

$$A = QR$$

Q 是正交矩阵，R 是上三角矩阵。SLAM 中用于最小二乘问题的稳定求解。

> **示例 8** — QR 分解（Gram-Schmidt）
>
> $$A = \begin{bmatrix} 1 & 2 \\ 3 & 2 \end{bmatrix}$$
>
> 列向量:
> $$a_1 = [1,3]^\top$$,
> $$a_2 = [2,2]^\top$$
>
> **Step 1**: $u_1 = a_1 = [1,3]^\top$
> $$q_1 = \frac{u_1}{\|u_1\|} = \frac{[1,3]^\top}{\sqrt{10}} \approx [0.3162, 0.9487]^\top$$
>
> **Step 2**:投影:
> **Step 2**:$$r_{12} = q_1^\top a_2 = 0.3162\times2 + 0.9487\times2 = 2.5298$$
> $$u_2 = a_2 - r_{12}q_1 = \begin{bmatrix} 2 \\ 2 \end{bmatrix} - 2.5298\begin{bmatrix} 0.3162 \\ 0.9487 \end{bmatrix} = \begin{bmatrix} 2 - 0.8 \\ 2 - 2.4 \end{bmatrix} = \begin{bmatrix} 1.2 \\ -0.4 \end{bmatrix}$$
> $$q_2 = \frac{u_2}{\|u_2\|} = \frac{[1.2,-0.4]^\top}{\sqrt{1.44+0.16}} = \frac{[1.2,-0.4]^\top}{\sqrt{1.6}} \approx [0.9487, -0.3162]^\top$$
>
> **结果**:
> $$Q = \begin{bmatrix} 0.3162 & 0.9487 \\ 0.9487 & -0.3162 \end{bmatrix}, \quad R = \begin{bmatrix} \|u_1\| & r_{12} \\ 0 & \|u_2\| \end{bmatrix} = \begin{bmatrix} 3.1623 & 2.5298 \\ 0 & 1.2649 \end{bmatrix}$$
>
> 验证:
> $$QR = A$$
> ...
> $$\begin{bmatrix}0.3162\times3.1623 & 0.3162\times2.5298+0.9487\times1.2649 \\ 0.9487\times3.1623 & 0.9487\times2.5298+(-0.3162)\times1.2649\end{bmatrix} = \begin{bmatrix}1 & 2 \\ 3 & 2\end{bmatrix} = A$$
> ✓

---

### 1.1.3 奇异值分解 (SVD) — 最重要！

$$A = U \Sigma V^\top$$

- U：左奇异向量（A 的列空间正交基）
- Σ：对角矩阵，非负奇异值降序排列
- V：右奇异向量（A 的行空间正交基）

> **示例 9** — SVD 手算演练（2×2 矩阵）
>
> $$A = \begin{bmatrix} 3 & 1 \\ 1 & 3 \end{bmatrix}$$
>
> **Step 1**:计算
> **Step 1**:$$A^\top A$$
> $$A^\top A = \begin{bmatrix} 3 & 1 \\ 1 & 3 \end{bmatrix} \begin{bmatrix} 3 & 1 \\ 1 & 3 \end{bmatrix} = \begin{bmatrix} 10 & 6 \\ 6 & 10 \end{bmatrix}$$
>
> **Step 2**:求
> **Step 2**:$$A^\top A$$
> **Step 2**:的特征值和特征向量
> $$\det(A^\top A - \lambda I) = \det\begin{bmatrix} 10-\lambda & 6 \\ 6 & 10-\lambda \end{bmatrix} = (10-\lambda)^2 - 36 = 0$$
> $$\lambda^2 - 20\lambda + 64 = 0 \quad \Rightarrow \quad \lambda_1 = 16,\ \lambda_2 = 4$$
>
> 奇异值:
> $$\sigma_1 = \sqrt{16} = 4$$,
> $$\sigma_2 = \sqrt{4} = 2$$
>
> **Step 3**:求特征向量（即
> **Step 3**:$$V$$
> **Step 3**:的列）
>
> 对
> $$\lambda_1=16$$:
> $$(A^\top A - 16I)v_1 = \begin{bmatrix} -6 & 6 \\ 6 & -6 \end{bmatrix} v_1 = 0 \Rightarrow v_1 = \frac{1}{\sqrt{2}}[1, 1]^\top$$
>
> 对
> $$\lambda_2=4$$:
> $$(A^\top A - 4I)v_2 = \begin{bmatrix} 6 & 6 \\ 6 & 6 \end{bmatrix} v_2 = 0 \Rightarrow v_2 = \frac{1}{\sqrt{2}}[1, -1]^\top$$
>
> $$V = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 & 1 \\ 1 & -1 \end{bmatrix} \approx \begin{bmatrix} 0.7071 & 0.7071 \\ 0.7071 & -0.7071 \end{bmatrix}$$
>
> **Step 4**:求
> **Step 4**:$$U$$
> **Step 4**:的列:
> **Step 4**:$$u_i = Av_i / \sigma_i$$
>
> $$u_1 = \frac{1}{4}\begin{bmatrix} 3 & 1 \\ 1 & 3 \end{bmatrix} \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\ 1 \end{bmatrix} = \frac{1}{4\sqrt{2}}\begin{bmatrix} 4 \\ 4 \end{bmatrix} = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\ 1 \end{bmatrix}$$
> $$u_2 = \frac{1}{2}\begin{bmatrix} 3 & 1 \\ 1 & 3 \end{bmatrix} \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\ -1 \end{bmatrix} = \frac{1}{2\sqrt{2}}\begin{bmatrix} 2 \\ -2 \end{bmatrix} = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\ -1 \end{bmatrix}$$
>
> **最终 SVD**:
> $$A = \begin{bmatrix} 0.7071 & 0.7071 \\ 0.7071 & -0.7071 \end{bmatrix} \begin{bmatrix} 4 & 0 \\ 0 & 2 \end{bmatrix} \begin{bmatrix} 0.7071 & 0.7071 \\ 0.7071 & -0.7071 \end{bmatrix}$$

**SLAM 中的4个关键用途：**

1. **求解齐次线性方程（八点法求 F/E 矩阵）**

   $$Ax = 0$$

   解 = V 的最小奇异值对应的列（最后一列）

2. **求解非齐次方程的最小二乘解**

   $$Ax = b$$

   $$x = V \Sigma^{-1} U^\top b$$

3. **ICP 中从点云对应求最优刚体变换**

   $$H = \sum (p_i - \bar{p})(q_i - \bar{q})^\top$$

   SVD 分解 H 即得最优旋转

4. **强制矩阵秩约束**
   如
   $$E$$
   矩阵秩必须为2 → 置最小奇异值为0再重构

> **示例 10** — SVD 求解最小二乘问题
>
> $$A = \begin{bmatrix} 1 & 2 \\ 3 & 4 \\ 5 & 6 \end{bmatrix}, \quad b = \begin{bmatrix} 1 \\ 1 \\ 1 \end{bmatrix}$$
>
> 求
> $$x$$
> 使
> $$\|Ax - b\|^2$$
> 最小化。
>
> 使用 SVD 解:
> $$x = V \Sigma^{-1} U^\top b$$
>
> 用 numpy 计算（概念演示）:
> ```python
> import numpy as np
> A = np.array([[1,2],[3,4],[5,6]])
> b = np.array([1,1,1])
> U, S, Vt = np.linalg.svd(A, full_matrices=False)
> x = Vt.T @ np.diag(1/S) @ U.T @ b
> # x ≈ [-1.0, 1.0]
> # 验证: Ax = [1,1,1]ᵀ = b（恰好有精确解！）
> ```

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

- $$v$$
- ：特征向量（变换后方向不变）
- $$\lambda$$
- ：特征值（变换后的缩放倍数）

> **示例 11** — 手算特征值和特征向量
>
> $$A = \begin{bmatrix} 2 & 1 \\ 1 & 2 \end{bmatrix}$$
>
> **Step 1**:特征方程
> **Step 1**:$$\det(A - \lambda I) = 0$$
> $$\det\begin{bmatrix} 2-\lambda & 1 \\ 1 & 2-\lambda \end{bmatrix} = (2-\lambda)^2 - 1 = 0$$
> $$\lambda^2 - 4\lambda + 3 = 0 \quad \Rightarrow \quad (\lambda-1)(\lambda-3) = 0$$
> $$\lambda_1 = 3,\ \lambda_2 = 1$$
>
> **Step 2**: 求特征向量
>
> 对
> $$\lambda_1=3$$:
> $$(A-3I)v = \begin{bmatrix} -1 & 1 \\ 1 & -1 \end{bmatrix}v = 0 \Rightarrow v_1 = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\ 1 \end{bmatrix}$$
>
> 对
> $$\lambda_2=1$$:
> $$(A-I)v = \begin{bmatrix} 1 & 1 \\ 1 & 1 \end{bmatrix}v = 0 \Rightarrow v_2 = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\ -1 \end{bmatrix}$$
>
> **验证**：
>
> $$Av_1 = \begin{bmatrix}2&1\\1&2\end{bmatrix}\frac{1}{\sqrt{2}}\begin{bmatrix}1\\1\end{bmatrix} = \frac{1}{\sqrt{2}}\begin{bmatrix}3\\3\end{bmatrix} = 3v_1$$
>
> ✓

**SLAM 中的关键应用：**

1. **Harris 角点检测**
   图像梯度协方差矩阵
   $$M$$
   的两个特征值：
   - $$\lambda_1, \lambda_2$$
   - 都大 → **角点**（两个方向都有大梯度）
   - $$\lambda_1 \gg \lambda_2$$
   - 或反之 → **边缘**（只一个方向有大梯度）
   - 都小 → **平坦区域**

   > **示例 12** — Harris 响应计算
   >
   > 假设某像素处的结构张量:
   > $$M = \begin{bmatrix} 100 & 50 \\ 50 & 100 \end{bmatrix}$$
   >
   > 特征值:
   > $$\det(M-\lambda I) = (100-\lambda)^2 - 2500 = 0$$
   > $$\lambda^2 - 200\lambda + 7500 = 0 \Rightarrow \lambda_1 = 150,\ \lambda_2 = 50$$
   >
   > 两个特征值都大 → **角点**！
   >
   > Harris 响应 (
   > $$k=0.04$$
   > ):
   > $$R = \det(M) - k\cdot\text{trace}(M)^2 = (100\times100 - 50\times50) - 0.04 \times (200)^2$$
   > $$= 7500 - 0.04 \times 40000 = 7500 - 1600 = 5900$$
   >
   > 对比平坦区域:
   > $$M = \begin{bmatrix} 1 & 0.1 \\ 0.1 & 1 \end{bmatrix}$$,
   > $$\det=0.99$$,
   > $$\text{trace}=2$$
   > $$$$
   > $$R = 0.99 - 0.04 \times 4 = 0.83 \quad\text{(small → flat)}$$
   > $$$$

   Harris 响应:
   $$R = \lambda_1\lambda_2 - k(\lambda_1+\lambda_2)^2$$
   等价于
   $$R = \det(M) - k \cdot \text{trace}(M)^2$$

2. **PCA（主成分分析）**
   协方差矩阵的特征向量 = 数据的主要方向
   SLAM 中用于点云法向量估计

3. **可观性分析 (Observability Analysis)**
   VIO 系统中 FIM (Fisher Information Matrix) 的特征值：
   - 零特征值 → 对应不可观状态（如VIO中的全局偏航角）

---

### 1.1.5 刚体变换与齐次坐标

#### 旋转矩阵 R ∈ SO(3)

$$SO(3) = \{R \in \mathbb{R}^{3\times3} \mid R^\top R = I,\ \det(R) = 1\}$$

3个自由度（绕X、Y、Z轴各一个旋转角）。

绕 Z 轴旋转
$$\theta$$:

$$R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

绕 X 轴旋转
$$\theta$$:

$$R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{bmatrix}$$

绕 Y 轴旋转
$$\theta$$:

$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

> **示例 13** — 旋转一个 3D 点
>
> 绕 Z 轴旋转
> $$45^\circ$$
> (
> $$\cos45^\circ = \sin45^\circ = \frac{\sqrt{2}}{2} \approx 0.7071$$
> )
>
> $$R_z(45^\circ) = \begin{bmatrix} 0.7071 & -0.7071 & 0 \\ 0.7071 & 0.7071 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
>
> 点
> $$P = [2, 0, 3]^\top$$
> 旋转后:
> $$P' = R_z P = \begin{bmatrix} 0.7071 & -0.7071 & 0 \\ 0.7071 & 0.7071 & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 2 \\ 0 \\ 3 \end{bmatrix} = \begin{bmatrix} 0.7071\times2 + (-0.7071)\times0 + 0\times3 \\ 0.7071\times2 + 0.7071\times0 + 0\times3 \\ 0\times2 + 0\times0 + 1\times3 \end{bmatrix} = \begin{bmatrix} 1.414 \\ 1.414 \\ 3 \end{bmatrix}$$
>
> 原本在 X 轴上的点
> $$(2,0,3)$$
> 旋转
> $$45^\circ$$
> 后到了
> $$(1.414, 1.414, 3)$$
> ，Z 不变。

#### 齐次变换矩阵 T ∈ SE(3)

$$T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} \in \mathbb{R}^{4\times4}$$

6个自由度（3旋转 + 3平移）。

变换一个点:
$$p' = T p$$
，其中
$$p$$
是齐次坐标
$$[x, y, z, 1]^\top$$

> **示例 14** — 完整刚体变换
>
> 先绕 Z 轴旋转
> $$90^\circ$$
> (
> $$\cos90^\circ=0$$,
> $$\sin90^\circ=1$$
> )，再平移
> $$[1, 2, 0]^\top$$:
>
> $$R = R_z(90^\circ) = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad t = \begin{bmatrix} 1 \\ 2 \\ 0 \end{bmatrix}$$
>
> $$T = \begin{bmatrix} 0 & -1 & 0 & 1 \\ 1 & 0 & 0 & 2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> 点
> $$P_{world} = [3, 0, 5]^\top$$
> 变换到相机坐标:
> $$P_{cam} = T \begin{bmatrix} 3 \\ 0 \\ 5 \\ 1 \end{bmatrix} = \begin{bmatrix} 0\times3 + (-1)\times0 + 0\times5 + 1\times1 \\ 1\times3 + 0\times0 + 0\times5 + 1\times2 \\ 0\times3 + 0\times0 + 1\times5 + 0\times1 \\ 0+0+0+1 \end{bmatrix} = \begin{bmatrix} 1 \\ 5 \\ 5 \\ 1 \end{bmatrix}$$
>
> 即
> $$P_{cam} = [1, 5, 5]^\top$$

**变换合成**：
$$T_{ac} = T_{ab} \cdot T_{bc}$$

世界点 → 相机坐标：
$$P_{cam} = T_{cw} \cdot P_{world} = T_{wc}^{-1} \cdot P_{world}$$

> **示例 15** — 变换合成
>
> $$T_{ab}$$
> : 从 A 到 B（旋转
> $$90^\circ$$
> 绕Z，平移
> $$[1,0,0]$$
> ）
> $$T_{bc}$$
> : 从 B 到 C（旋转
> $$90^\circ$$
> 绕Z，平移
> $$[0,1,0]$$
> ）
>
> $$T_{ab} = \begin{bmatrix} 0 & -1 & 0 & 1 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad T_{bc} = \begin{bmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> $$T_{ac} = T_{ab} \cdot T_{bc} = \begin{bmatrix} -1 & 0 & 0 & 1 \\ 0 & -1 & 0 & 1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> 复合变换: 绕Z旋转
> $$180^\circ$$
> (
> $$90^\circ+90^\circ$$
> )，平移
> $$[1,1,0]^\top$$

#### 旋转的三种表示

| 表示 | 参数数 | 优点 | 缺点 |
|------|--------|------|------|
| 欧拉角 | 3 | 直观 | 万向锁，不连续 |
| 旋转向量 | 3 | 紧凑 | 奇异点(θ=0附近) |
| 四元数 | 4 | 无奇异性，平滑插值 | 不直观 |

> **示例 16** — 三种表示之间的转换
>
> 绕 Z 轴旋转 60°：
>
> $$R_z(60^\circ) = \begin{bmatrix} 0.5 & -0.866 & 0 \\ 0.866 & 0.5 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
>
> **欧拉角** (ZYX)：
>
> $$(60^\circ, 0^\circ, 0^\circ)$$
>
> **旋转向量** (轴角)：
>
> $$\theta = 60^\circ = \frac{\pi}{3}$$
>
> 轴：
>
> $$\mathbf{n} = [0, 0, 1]^\top$$
>
> 旋转向量：
>
> $$\theta\mathbf{n} = [0, 0, \frac{\pi}{3}]^\top$$
>
> **四元数**：
>
> $$q = (\cos\frac{\theta}{2}, \mathbf{n}\sin\frac{\theta}{2}) = (\cos30^\circ, 0, 0, \sin30^\circ) = (0.866, 0, 0, 0.5)$$

**SLAM 实践**: ORB-SLAM 使用四元数 + 平移向量表示位姿；优化时在 se(3) 李代数上进行（6维向量）。

---

## 1.2 概率与统计 — SLAM 的不确定性语言

SLAM 的本质是概率推断问题：

$$P(\text{map}, \text{pose} \mid \text{observation})$$

> 给定传感器观测，地图和轨迹的**后验概率**最大是多少？

### 1.2.1 高斯分布 (正态分布)

#### 一维高斯

$$p(x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$$

- $$\mu$$
- ：均值（最可能的值）
- $$\sigma$$
- ：标准差（不确定性度量）

> **示例 17** — 一维高斯的概率密度计算
>
> 深度传感器测量深度，噪声服从
> $$\mathcal{N}(\mu=5.0\text{m},\ \sigma=0.1\text{m})$$
>
> 在
> $$x=5.05\text{m}$$
> 处的概率密度:
> $$p(5.05) = \frac{1}{0.1\sqrt{2\pi}} \exp\left(-\frac{(5.05-5.0)^2}{2 \times 0.01}\right) = \frac{1}{0.1\times2.5066} \exp\left(-\frac{0.0025}{0.02}\right)$$
> $$= \frac{1}{0.25066} \times e^{-0.125} \approx 3.989 \times 0.8825 \approx 3.52$$
>
> 在
> $$x=5.3\text{m}$$
> 处 (偏离3σ):
> $$p(5.3) = \frac{1}{0.1\sqrt{2\pi}} \exp\left(-\frac{0.09}{0.02}\right) = 3.989 \times e^{-4.5} \approx 3.989 \times 0.0111 \approx 0.044$$
>
> 可以看到，越偏离均值，概率密度急剧下降。

**为什么 SLAM 几乎只用高斯分布？**
1. 中心极限定理：大量独立误差之和 ≈ 高斯
2. 高斯分布在边缘化和条件化下封闭（卡尔曼滤波的基础）
3. 负对数把乘积转为平方和 → 最小二乘优化

**68-95-99.7 规则**：
- $$[\mu-\sigma, \mu+\sigma]$$
- 包含 ~68% 的概率
- $$[\mu-2\sigma, \mu+2\sigma]$$
- 包含 ~95% 的概率
- $$[\mu-3\sigma, \mu+3\sigma]$$
- 包含 ~99.7% 的概率

> **示例 18** — 68-95-99.7 规则应用于 SLAM
>
> 相机位姿估计的 X 坐标
> $$\sim \mathcal{N}(\mu=2.0\text{m},\ \sigma=0.05\text{m})$$:
>
> - 68% 概率真实值在
> - $$[1.95,\ 2.05]$$
> - m
> - 95% 概率真实值在
> - $$[1.90,\ 2.10]$$
> - m
> - 99.7% 概率真实值在
> - $$[1.85,\ 2.15]$$
> - m
>
> 这就是 SLAM 中**不确定性椭球**的一维版本。σ 越小 = 估计越精确。

#### 多元高斯

$$p(\mathbf{x}) = \frac{1}{(2\pi)^{n/2}|\Sigma|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x}-\mu)^\top\Sigma^{-1}(\mathbf{x}-\mu)\right)$$

- $$\mu$$
- ：n维均值向量
- $$\Sigma$$
- ：n×n协方差矩阵（描述各维度间的关联）

SLAM 中：相机位姿的协方差矩阵描述了你的**不确定性椭球**。

> **示例 19** — 多元高斯协方差
>
> 二维位姿
> $$(x, y)$$
> 的协方差矩阵:
> $$\Sigma = \begin{bmatrix} 0.04 & 0.01 \\ 0.01 & 0.09 \end{bmatrix}$$
>
> 解读：
> - $$\sigma_x^2 = 0.04 \Rightarrow \sigma_x = 0.2\text{m}$$
> - （X方向不确定度）
> - $$\sigma_y^2 = 0.09 \Rightarrow \sigma_y = 0.3\text{m}$$
> - （Y方向更不确定）
> - 协方差
> - $$0.01 > 0$$
> - → X 和 Y 正相关（X偏大时Y也偏大）
>
> 求
> $$\Sigma$$
> 的特征值和特征向量:
> $$\det\begin{bmatrix} 0.04-\lambda & 0.01 \\ 0.01 & 0.09-\lambda \end{bmatrix} = (0.04-\lambda)(0.09-\lambda) - 0.0001 = 0$$
> $$\lambda^2 - 0.13\lambda + 0.0035 = 0 \Rightarrow \lambda_1 \approx 0.093,\ \lambda_2 \approx 0.037$$
>
> 不确定性椭圆的半轴长:
> $$\sqrt{\lambda_1} \approx 0.305\text{m}$$,
> $$\sqrt{\lambda_2} \approx 0.192\text{m}$$
> 椭圆方向由对应特征向量决定。

**协方差矩阵的几何意义**：
- 对角元素：各变量的方差
- 非对角元素：变量间的相关性
- 特征值决定椭球轴长，特征向量决定椭球方向

---

### 1.2.2 贝叶斯定理

$$P(A \mid B) = \frac{P(B \mid A) \cdot P(A)}{P(B)}$$

- $$P(A)$$
- ：**先验**（看到任何数据之前的信念）
- $$P(B \mid A)$$
- ：**似然**（给定A，B出现的概率）
- $$P(A \mid B)$$
- ：**后验**（看到B后，对A的更新信念）

> **示例 20** — 贝叶斯定理：SLAM 回环检测
>
> 假设：
> - 回环的先验概率
> - $$P(\text{loop}) = 0.1$$
> - （只有10%的概率经过之前的地方）
> - 如果真的是回环，特征匹配成功的概率
> - $$P(\text{many matches} \mid \text{loop}) = 0.95$$
> - 如果不是回环，特征匹配成功的概率
> - $$P(\text{many matches} \mid \text{no loop}) = 0.05$$
> - （偶尔也会误匹配很多）
>
> 现在观察到了很多匹配。回环的后验概率是多少？
>
> **Step 1**: 计算全概率
> $$$$
> $$P(\text{many matches}) = P(\text{many matches}\mid\text{loop})P(\text{loop}) + P(\text{many matches}\mid\text{no loop})P(\text{no loop})$$
> $$$$
> $$= 0.95 \times 0.1 + 0.05 \times 0.9 = 0.095 + 0.045 = 0.14$$
>
> **Step 2**: 贝叶斯更新
>
> $$P(\text{loop} \mid \text{many matches}) = \frac{0.95 \times 0.1}{0.14} = \frac{0.095}{0.14} \approx 0.679$$
>
> 观察到很多匹配后，回环概率从 10% 更新到了 **68%**！但仍有 32% 可能不是回环。
>
> ORB-SLAM 正是这样：先用 BoW 匹配找到候选回环帧，再通过几何验证（Sim3 对齐）进一步确认。

#### SLAM 的贝叶斯框架

$$\underbrace{P(X_{1:t}, M \mid Z_{1:t}, U_{1:t})}_{\text{posterior: }P(\text{map, trajectory} \mid \text{observations, controls})} \propto \underbrace{P(Z_t \mid X_t, M)}_{\text{observation model}} \cdot \underbrace{P(X_t \mid X_{t-1}, U_t)}_{\text{motion model}}$$

> 每次新的传感器数据到来，就用贝叶斯定理**更新**我们对世界状态的信念。

---

### 1.2.3 最大似然估计 (MLE) 与最大后验估计 (MAP)

#### MLE（最大似然）

$$\hat{\theta}_{\text{MLE}} = \arg\max_\theta P(D \mid \theta)$$

「哪种参数最可能产生我们观察到的数据？」

SLAM 中：给定匹配点，最可能的相机位姿 → **这就是 PnP 的精神！**

> **示例 21** — MLE: 从测量估计真实深度
>
> 三次深度测量:
> $$z = [4.9, 5.1, 5.0]$$
> 米，假设噪声
> $$\mathcal{N}(0, \sigma^2)$$
>
> 似然函数:
> $$P(z \mid \mu) = \prod_{i=1}^3 \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(z_i - \mu)^2}{2\sigma^2}\right)$$
>
> 取负对数（去掉常数）:
> $$-\log P(z \mid \mu) \propto \sum_{i=1}^3 (z_i - \mu)^2 = (4.9-\mu)^2 + (5.1-\mu)^2 + (5.0-\mu)^2$$
>
> 求导使之为0:
> $$\frac{d}{d\mu} = -2(4.9-\mu) - 2(5.1-\mu) - 2(5.0-\mu) = 0$$
> $$3\mu = 4.9 + 5.1 + 5.0 = 15.0 \quad \Rightarrow \quad \hat{\mu}_{\text{MLE}} = 5.0$$
>
> MLE = **样本均值**！（这是高斯假设的自然结果）

#### MAP（最大后验）

$$\hat{\theta}_{\text{MAP}} = \arg\max_\theta P(\theta \mid D) = \arg\max_\theta P(D \mid \theta)P(\theta)$$

比 MLE 多了一个先验
$$P(\theta)$$

> **示例 22** — MAP: 带先验的深度估计
>
> 测量:
> $$z = [5.2, 5.3]$$
> 。先验: 深度
> $$\sim \mathcal{N}(4.5, 1.0^2)$$
> （我们"相信"大概是4.5m，但不太确定）。
>
> 似然
> $$P(z \mid \mu) \propto \exp(-\frac{(5.2-\mu)^2 + (5.3-\mu)^2}{2\sigma^2})$$
>
> 先验
> $$P(\mu) \propto \exp(-\frac{(\mu-4.5)^2}{2\times1.0})$$
>
> 负对数后验（假设
> $$\sigma^2=0.01$$
> ）:
> $$-\log P(\mu \mid z) \propto \frac{(5.2-\mu)^2 + (5.3-\mu)^2}{2\times0.01} + \frac{(\mu-4.5)^2}{2\times1.0}$$
> $$= 50[(5.2-\mu)^2 + (5.3-\mu)^2] + 0.5(\mu-4.5)^2$$
>
> 最小化:
> $$\frac{d}{d\mu} = -100(5.2-\mu) - 100(5.3-\mu) + (\mu-4.5) = 0$$
> $$200\mu - 1050 + \mu - 4.5 = 0 \quad \Rightarrow \quad 201\mu = 1054.5 \quad \Rightarrow \quad \hat{\mu}_{\text{MAP}} \approx 5.25$$
>
> 对比 MLE =
> $$(5.2+5.3)/2 = 5.25$$
> ...在这个例子中 MLE 和 MAP 一样，因为
> $$\sigma^2$$
> 很小而先验很宽。如果先验窄（置信高），MAP 会被「拉」向先验。
>
> 实际上：纯 BA = MLE（最小化重投影误差），带先验的 BA = MAP（如固定第一帧位姿）。

#### 为什么最小化平方误差等同于 MLE？

假设观测噪声是高斯分布: 

$z = h(x) + \epsilon,\ \epsilon \sim \mathcal{N}(0, \sigma^2)$

$$P(z \mid x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(z - h(x))^2}{2\sigma^2}\right)$$

取负对数：
$$-\log P(z \mid x) = \frac{(z - h(x))^2}{2\sigma^2} + \text{const}$$

所以：
$$\arg\max_x P(z \mid x) = \arg\min_x (z - h(x))^2$$

> **示例 23** — 重投影误差 = MLE
>
> 3D点投影到图像：预测像素
> $$(u_{\text{pred}}, v_{\text{pred}}) = h(X, R, t, K)$$
>
> 实际观测像素:
> $$(u_{\text{obs}}, v_{\text{obs}}) = (321.5, 240.3)$$
>
> 重投影误差:
> $$e = \sqrt{(321.5 - u_{\text{pred}})^2 + (240.3 - v_{\text{pred}})^2}$$
>
> 最小化
> $$\sum e^2$$
> = 最大化
> $$\prod \exp(-e^2/2\sigma^2)$$
> = **在高斯噪声假设下最大化似然**！
>
> **这就是 Bundle Adjustment 的数学本质！** 最小化重投影误差 ≈ 最大化高斯噪声假设下的似然。

---

### 1.2.4 泰勒展开

$$f(x) \approx f(x_0) + f'(x_0)(x - x_0) + \frac{1}{2}f''(x_0)(x - x_0)^2 + \cdots$$

SLAM 中的关键应用：
- **一阶泰勒** → 线性化非线性函数（高斯-牛顿法的基础）
- **二阶泰勒** → Hessian矩阵（牛顿法）
- **李代数扰动** → 在 se(3) 上用一阶泰勒将位姿优化问题线性化

> **示例 24** — 一阶泰勒展开：线性化投影函数
>
> 假设深度
> $$Z=5$$
> ，相机在原点。3D 点的 X 坐标投影:
> $$u(X) = f_x \cdot \frac{X}{Z} + c_x = 500 \cdot \frac{X}{5} + 320 = 100X + 320$$
>
> 在
> $$X_0 = 1.0$$
> 处展开:
> $$u(X) \approx u(1.0) + u'(1.0)(X - 1.0)$$
> $$u(1.0) = 100\times1.0 + 320 = 420$$
> $$$$
> $$u'(X) = 100 \quad\text{(constant, linear function)}$$
> $$$$
> $$$$
> $$u(X) \approx 420 + 100(X - 1.0) = 100X + 320 \quad\text{(exact, linear function)}$$
> $$$$
>
> ---
>
> **示例 24b** — 非线性函数的泰勒展开
>
> $$f(x) = e^x, \quad x_0 = 0$$
>
> $$f(0) = 1, \quad f'(0) = 1, \quad f''(0) = 1$$
>
> 一阶:
> $$f(x) \approx 1 + x$$
> （在
> $$x=0$$
> 附近）
> 二阶:
> $$f(x) \approx 1 + x + \frac{1}{2}x^2$$
>
> 在
> $$x=0.5$$
> 处：
> - 真值:
> - $$e^{0.5} \approx 1.6487$$
> - 一阶近似:
> - $$1.5$$
> - （误差 9%）
> - 二阶近似:
> - $$1 + 0.5 + 0.125 = 1.625$$
> - （误差 1.4%）
>
> 这就是**高斯-牛顿法 vs 牛顿法**的区别：高斯-牛顿用一阶近似（省掉计算 Hessian 的成本），牛顿法用二阶近似（更精确但计算量大）。

---

## 1.3 微积分

### 1.3.1 导数与梯度

- 一阶导数 → 变化率
- 梯度
- $$\nabla f$$
- → 多变量函数的最速上升方向
- SLAM 中：**雅可比矩阵 J** 就是多维函数对所有变量的偏导数矩阵

> **示例 25** — 多变量函数的梯度
>
> $$f(x, y) = x^2 + 3xy + y^2$$
>
> 偏导:
> $$\frac{\partial f}{\partial x} = 2x + 3y$$,
> $$\frac{\partial f}{\partial y} = 3x + 2y$$
>
> 梯度:
> $$\nabla f = [2x + 3y,\ 3x + 2y]^\top$$
>
> 在点
> $$(x,y) = (1, 2)$$
> 处:
> $$\nabla f(1,2) = [2\times1 + 3\times2,\ 3\times1 + 2\times2]^\top = [8,\ 7]^\top$$
>
> 函数在
> $$(1,2)$$
> 处沿方向
> $$[8,7]^\top$$
> 增长最快。

> **示例 26** — 雅可比矩阵（SLAM 中最重要的导数）
>
> 重投影函数将 3D 点
> $$[X,Y,Z]^\top$$
> 映射到像素
> $$(u,v)$$:
> $$u = f_x\frac{X}{Z} + c_x, \quad v = f_y\frac{Y}{Z} + c_y$$
>
> 雅可比矩阵（对 3D 点求偏导）:
> $$J = \begin{bmatrix} \frac{\partial u}{\partial X} & \frac{\partial u}{\partial Y} & \frac{\partial u}{\partial Z} \\ \frac{\partial v}{\partial X} & \frac{\partial v}{\partial Y} & \frac{\partial v}{\partial Z} \end{bmatrix} = \begin{bmatrix} \frac{f_x}{Z} & 0 & -\frac{f_x X}{Z^2} \\ 0 & \frac{f_y}{Z} & -\frac{f_y Y}{Z^2} \end{bmatrix}$$
>
> 当
> $$f_x=f_y=500$$,
> $$[X,Y,Z]=[1, 0.5, 5]^\top$$:
> $$J = \begin{bmatrix} \frac{500}{5} & 0 & -\frac{500\times1}{25} \\ 0 & \frac{500}{5} & -\frac{500\times0.5}{25} \end{bmatrix} = \begin{bmatrix} 100 & 0 & -20 \\ 0 & 100 & -10 \end{bmatrix}$$
>
> J 告诉我们: 改变 X（3D）对 u（像素）的影响是 100 像素/米，改变 Z 对 u 的影响是 -20 像素/米。
> 离相机越远（Z 大），J 越小 → 远处点对位姿优化的贡献小。

### 1.3.2 对数与指数

- $\ln(e^x) = x$
- $\ln(ab) = \ln a + \ln b$

SLAM 中：
- 取对数把乘积变加和 → 负对数似然 → 最小二乘
- $$\exp$$
- 映射 se(3) → SE(3)（李代数到李群）
- $$\log$$
- 映射 SE(3) → se(3)（李群到李代数）

> **示例 27** — 对数变换在 SLAM 中的应用
>
> 似然函数（高斯噪声，多次观测）:
> $$L = \prod_{i=1}^n \exp\left(-\frac{(z_i - h(x))^2}{2\sigma^2}\right)$$
>
> 取对数变为求和:
> $$\log L = \sum_{i=1}^n -\frac{(z_i - h(x))^2}{2\sigma^2} = -\frac{1}{2\sigma^2}\sum_{i=1}^n (z_i - h(x))^2$$
>
> 最大化
> $$\log L$$
> = 最小化
> $$\sum (z_i - h(x))^2$$
> — 这就是**最小二乘**的由来。
>
> ---
>
> **示例 27b** — 指数映射（李代数）
>
> 旋转向量
> $$\phi = [0, 0, \frac{\pi}{4}]^\top$$
> （绕Z轴旋转45°）
> $$\phi^\wedge = \begin{bmatrix} 0 & -\frac{\pi}{4} & 0 \\ \frac{\pi}{4} & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$
>
> 指数映射（Rodrigues 公式）:
> $$R = \exp(\phi^\wedge) = I + \frac{\sin\|\phi\|}{\|\phi\|}\phi^\wedge + \frac{1-\cos\|\phi\|}{\|\phi\|^2}(\phi^\wedge)^2$$
>
> $$\|\phi\| = \frac{\pi}{4}, \quad \sin\frac{\pi}{4} = 0.7071, \quad 1-\cos\frac{\pi}{4} = 0.2929$$
> $$R = I + \frac{0.7071}{\pi/4}\phi^\wedge + \cdots = R_z(45^\circ) = \begin{bmatrix} 0.7071 & -0.7071 & 0 \\ 0.7071 & 0.7071 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$\exp$$
> 将 3 维 se(3) 映射到 4×4 SE(3) 矩阵 — 这是优化位姿的核心操作。

---

## 📝 自检清单

在进入 Level 2 之前，确保你能回答：

- [ ] 叉积在 SLAM 中的两个核心用途是什么？
- [ ] SVD 分解后如何求解
- $$Ax=0$$
- ？
- [ ] 为什么旋转矩阵的逆等于转置？
- [ ] 协方差矩阵的特征值告诉你什么？
- [ ] 为什么最小化重投影误差 = 最大似然估计？
- [ ] 多元高斯分布在 SLAM 中代表什么？
- [ ] 齐次变换矩阵 T 有哪四个部分？各有什么含义？

---

> **下一步**: 完成 `exercises/exercise_01_linear_algebra.py` 和 `exercises/exercise_02_probability.py` 来巩固这些数学概念。

