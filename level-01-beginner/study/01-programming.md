# 模块1: SLAM 编程基础

> 覆盖 visual-slam-roadmap Level 1 的「Programming」和后续各级的编程需求。
> 重点：不要求精通 C++，但要知道 SLAM 系统为什么用 C++，以及如何用 Python 学习原理。

---

## 1.1 为什么 SLAM 主要用 C++？

| 原因 | 说明 |
|------|------|
| **实时性** | SLAM 需要 30+ FPS，C++ 接近裸机性能 |
| **内存控制** | 手动管理内存，避免 GC 暂停 |
| **矩阵运算** | Eigen 库提供 SIMD 加速的向量化运算 |
| **GPU 加速** | CUDA 直接操作 GPU 内存（TSDF 融合等） |
| **嵌入式部署** | 机器人/无人机上跑 ARM 芯片，只有 C++ |

**但本学习课程用 Python！**

原因：
1. 学习 SLAM **原理**不需要 C++ 的性能
2. Python 的 numpy/scipy 是 Eigen 的 Python 等效
3. 可视化、调试、实验修改速度远快于 C++
4. 理解原理后，迁移到 C++ 只是重写 API 调用

**关键 Python → C++ 对应**：

| Python | C++ (Eigen) | 用途 |
|--------|-------------|------|
| `np.array([[1,2],[3,4]])` | `Matrix2d m; m << 1,2,3,4;` | 矩阵 |
| `np.linalg.svd(A)` | `JacobiSVD<MatrixXd> svd(A)` | SVD |
| `np.linalg.inv(A)` | `A.inverse()` | 求逆 |
| `A @ B` | `A * B` | 矩阵乘法 |
| `np.linalg.norm(v)` | `v.norm()` | 向量模长 |

---

## 1.2 Python 环境准备

```bash
# 必需库
pip install numpy scipy matplotlib

# 后续会用到的 (Level 2+)
pip install opencv-python    # 真正的 SIFT/ORB 等特征
pip install scikit-learn     # K-means 等聚类
pip install networkx          # 图可视化
```

**学习期间的代码规范**：
- 所有练习使用 `numpy` 而非列表（习惯向量化思维）
- 用 `np.linalg` 做矩阵分解（不要自己写 LU/SVD）
- 画图用 `matplotlib`（理解几何关系的关键）

---

## 1.3 NumPy 实战示例 — SLAM 中最常用的操作

### 1.3.1 向量和矩阵基本操作

```python
import numpy as np

# === 向量 ===
v = np.array([1, 2, 3])           # 创建向量
norm = np.linalg.norm(v)          # L2 模长 = sqrt(14) ≈ 3.742
v_norm = v / norm                  # 归一化 = [0.267, 0.535, 0.802]

# === 点积和叉积 ===
a = np.array([1, 2, 3])
b = np.array([4, 5, 6])
dot = np.dot(a, b)                # 点积 = 32
dot = a @ b                       # 等价写法
cross = np.cross(a, b)            # 叉积 = [-3, 6, -3]
# 验证: cross 垂直于 a 和 b
assert abs(np.dot(cross, a)) < 1e-10
assert abs(np.dot(cross, b)) < 1e-10

# === 矩阵乘法 ===
A = np.array([[1, 2], [3, 4]])
B = np.array([[5, 6], [7, 8]])
C = A @ B                         # = [[19, 22], [43, 50]]
C = np.matmul(A, B)               # 等价写法

# === 转置 ===
A_T = A.T                          # [[1, 3], [2, 4]]
```

> **示例 1** — 验证旋转矩阵的正交性
>
> ```python
> import numpy as np
>
> theta = np.radians(30)  # 30度转弧度
> R = np.array([
>     [np.cos(theta), -np.sin(theta), 0],
>     [np.sin(theta),  np.cos(theta), 0],
>     [0,              0,             1]
> ])
>
> # 验证 R^T R = I
> I_approx = R.T @ R
> print(I_approx)
> # [[1. 0. 0.]
> #  [0. 1. 0.]
> #  [0. 0. 1.]]
>
> # 验证 det(R) = 1
> print(np.linalg.det(R))  # 1.0
>
> # 验证逆等于转置
> print(np.allclose(np.linalg.inv(R), R.T))  # True
> ```

### 1.3.2 矩阵分解

```python
# === SVD ===
A = np.array([[3, 1], [1, 3]])
U, S, Vt = np.linalg.svd(A)
# U = [[ 0.707,  0.707],   S = [4, 2],
#      [ 0.707, -0.707]]    Vt = [[0.707, 0.707],
#                                  [0.707, -0.707]]
A_recon = U @ np.diag(S) @ Vt   # 重构 = A

# SLAM 核心用法: 求 Ax=0 的解 (八点法)
# 解 = Vt 的最后一行 (最小奇异值对应)
F_vec = Vt[-1]                   # 1×9 向量
F = F_vec.reshape(3, 3)          # 基础矩阵

# === 特征值分解 ===
eigenvalues, eigenvectors = np.linalg.eig(A)
# eigenvalues = [4, 2]
# eigenvectors[:,0] ≈ [0.707, 0.707] (对应 λ=4)

# === QR 分解 ===
Q, R = np.linalg.qr(A)
# Q ≈ [[-0.949, -0.316],  R ≈ [[-3.162, -2.530],
#      [-0.316,  0.949]]        [ 0.000,  1.265]]
```

> **示例 2** — 用 SVD 求解八点法（完整流程）
>
> ```python
> import numpy as np
>
> # 8 对匹配点的像素坐标
> pts1 = np.array([
>     [400, 300], [500, 300], [300, 400], [500, 500],
>     [350, 250], [450, 350], [250, 450], [550, 450]
> ])
> pts2 = np.array([
>     [380, 300], [480, 300], [280, 400], [480, 500],
>     [330, 250], [430, 350], [230, 450], [530, 450]
> ])
>
> # 构造 A 矩阵 (8×9)
> A = np.zeros((8, 9))
> for i in range(8):
>     u1, v1 = pts1[i]
>     u2, v2 = pts2[i]
>     A[i] = [u1*u2, v1*u2, u2, u1*v2, v1*v2, v2, u1, v1, 1]
>
> # SVD 求最小特征向量
> _, _, Vt = np.linalg.svd(A)
> F = Vt[-1].reshape(3, 3)
>
> # 强制 rank(F) = 2
> U, S, Vt = np.linalg.svd(F)
> S[2] = 0
> F_rank2 = U @ np.diag(S) @ Vt
>
> print("F =", F_rank2)
> # 验证: 对每对点 p2^T F p1 ≈ 0
> for i in range(8):
>     p1 = np.append(pts1[i], 1)
>     p2 = np.append(pts2[i], 1)
>     epipolar_error = p2 @ F_rank2 @ p1
>     print(f"点对{i}: 对极误差 = {epipolar_error:.6f}")
> ```

### 1.3.3 齐次坐标操作

```python
# === 世界坐标 → 相机坐标 → 像素 ===
P_world = np.array([2, 1.5, 10, 1])   # 齐次坐标

# 外参: 相机在原点
T_cw = np.eye(4)                        # 世界→相机

# 内参
fx, fy, cx, cy = 800, 800, 640, 480
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

# 投影
P_cam = T_cw @ P_world                  # 相机坐标 [2, 1.5, 10, 1]
x_n = P_cam[:2] / P_cam[2]              # 归一化 [0.2, 0.15]
pixel = K @ np.array([x_n[0], x_n[1], 1])  # 像素 [800, 600]

print(f"世界点 {P_world[:3]} → 像素 ({pixel[0]:.0f}, {pixel[1]:.0f})")
```

> **示例 3** — 连续帧的位姿合成
>
> ```python
> import numpy as np
>
> # 帧1 → 帧2: 绕Z轴旋转 30°, 平移 [0.5, 0, 0]
> theta = np.radians(30)
> R12 = np.array([
>     [np.cos(theta), -np.sin(theta), 0],
>     [np.sin(theta),  np.cos(theta), 0],
>     [0,              0,             1]
> ])
> t12 = np.array([0.5, 0, 0])
> T12 = np.eye(4)
> T12[:3, :3] = R12
> T12[:3, 3] = t12
>
> # 帧2 → 帧3: 同样旋转, 平移 [0, 0.3, 0]
> theta2 = np.radians(30)
> R23 = np.array([
>     [np.cos(theta2), -np.sin(theta2), 0],
>     [np.sin(theta2),  np.cos(theta2), 0],
>     [0,               0,              1]
> ])
> t23 = np.array([0, 0.3, 0])
> T23 = np.eye(4)
> T23[:3, :3] = R23
> T23[:3, 3] = t23
>
> # 合成: 帧1 → 帧3
> T13 = T12 @ T23
> print("从帧1到帧3的变换 T13 =")
> print(T13)
> # 旋转 = 60° 绕Z (30+30)
> # 平移 ≈ [0.5, 0.3, 0]
> ```

### 1.3.4 概率相关

```python
from scipy.stats import multivariate_normal

# === 一维高斯 ===
mu, sigma = 5.0, 0.1  # 深度 5.0m ± 0.1m
# 在 x=5.05 处的概率密度
from scipy.stats import norm
pdf_val = norm.pdf(5.05, loc=mu, scale=sigma)
# = 3.52 (如数学模块示例17)

# === 多元高斯: 二维位姿不确定性 ===
mu = np.array([2.0, 3.0])               # 估计位姿 (x,y)
Sigma = np.array([[0.04, 0.01],          # 协方差矩阵
                  [0.01, 0.09]])
rv = multivariate_normal(mu, Sigma)
# 在 (1.8, 2.7) 处的概率密度
pdf = rv.pdf([1.8, 2.7])
```

---

## 1.4 Bash/Linux 基础

SLAM 开发和部署几乎全在 Linux 上：

```bash
# 编译 C++ SLAM 系统
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 运行
./bin/orb_slam_rgbd Vocabulary/ORBvoc.txt config.yaml

# 下载数据集
wget https://example.com/sequence.zip

# 查看 GPU 使用
nvidia-smi

# ROS 启动
roslaunch orb_slam3 euroc.launch
```

学习期间用不到，但要知道这些是最终目标。

---

## 1.5 C++ 核心概念速览

如果你之后要读 ORB-SLAM 源码，需要理解这些 C++ 特性：

### 指针与引用
```cpp
cv::Mat* pImg = &image;    // 指针：指向内存地址
cv::Mat& refImg = image;   // 引用：别名，更安全
```

### 面向对象（SLAM 系统的组织方式）
```cpp
class Tracking {
public:
    cv::Mat GrabImageMonocular(const cv::Mat& im);
private:
    cv::Mat mCurrentFrame;
    std::vector<MapPoint*> mvpLocalMapPoints;
};
```

SLAM 系统 = 多个类协作：
- `Tracking` 类处理每帧的位姿估计
- `LocalMapping` 类管理关键帧和局部 BA
- `LoopClosing` 类检测回环

### 现代 C++ 特性
```cpp
auto ptr = std::make_shared<MapPoint>(pos);  // 智能指针（自动释放）
std::vector<MapPoint*> vpPoints;             // STL 容器
std::mutex mMutexMap;                        // 多线程互斥锁
```

### CMake（SLAM 项目的构建系统）
```cmake
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(my_slam main.cpp)
target_link_libraries(my_slam ${OpenCV_LIBS} Eigen3::Eigen)
```

---

## 1.6 编程学习建议

1. **Level 1-3 全用 Python**：理解原理，快速迭代
2. **Level 3 后期读 ORB-SLAM 源码**：对照 Python 理解，只看核心流程
3. **Level 5+ 学 Eigen + Ceres**：如果要做研究/开发

不需要现在就学 C++。当你能用 Python 写出一个简化版 SLAM 系统后，C++ 只是另一个工具。

---

> **下一步**: 直接开始学数学和几何模块，编程能力会在练习中自然提升。
