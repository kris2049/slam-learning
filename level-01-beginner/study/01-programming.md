# Module 1: SLAM Programming Fundamentals

> Covers "Programming" from visual-slam-roadmap Level 1 and programming needs for subsequent levels.
> Key point: You don't need to master C++, but you should know why SLAM systems use C++ and how to learn the principles with Python.

---

## 1.1 Why Does SLAM Mainly Use C++?

| Reason | Explanation |
|--------|-------------|
| **Real-time** | SLAM requires 30+ FPS; C++ delivers near-bare-metal performance |
| **Memory control** | Manual memory management avoids GC pauses |
| **Matrix operations** | Eigen library provides SIMD-accelerated vectorized operations |
| **GPU acceleration** | CUDA directly accesses GPU memory (TSDF fusion, etc.) |
| **Embedded deployment** | Robots/drones run on ARM chips — C++ only |

**But this course uses Python!**

Reasons:
1. Learning SLAM **principles** does not require C++ performance
2. Python's numpy/scipy is the Python equivalent of Eigen
3. Visualization, debugging, and experimentation are much faster than C++
4. Once you understand the principles, migrating to C++ is just rewriting API calls

**Key Python → C++ Mapping**:

| Python | C++ (Eigen) | Purpose |
|--------|-------------|---------|
| `np.array([[1,2],[3,4]])` | `Matrix2d m; m << 1,2,3,4;` | Matrix |
| `np.linalg.svd(A)` | `JacobiSVD<MatrixXd> svd(A)` | SVD |
| `np.linalg.inv(A)` | `A.inverse()` | Inversion |
| `A @ B` | `A * B` | Matrix multiplication |
| `np.linalg.norm(v)` | `v.norm()` | Vector norm |

---

## 1.2 Python Environment Setup

```bash
# Required libraries
pip install numpy scipy matplotlib

# Used later (Level 2+)
pip install opencv-python    # Real SIFT/ORB features
pip install scikit-learn     # K-means clustering
pip install networkx          # Graph visualization
```

**Coding conventions during learning**:
- Use `numpy` for all exercises, not lists (build vectorized thinking habits)
- Use `np.linalg` for matrix decomposition (don't write your own LU/SVD)
- Use `matplotlib` for plotting (key to understanding geometric relationships)

---

## 1.3 NumPy Hands-On Examples — Most Common SLAM Operations

### 1.3.1 Basic Vector and Matrix Operations

```python
import numpy as np

# === Vectors ===
v = np.array([1, 2, 3])           # Create a vector
norm = np.linalg.norm(v)          # L2 norm = sqrt(14) ≈ 3.742
v_norm = v / norm                  # Normalize = [0.267, 0.535, 0.802]

# === Dot product and cross product ===
a = np.array([1, 2, 3])
b = np.array([4, 5, 6])
dot = np.dot(a, b)                # Dot product = 32
dot = a @ b                       # Equivalent syntax
cross = np.cross(a, b)            # Cross product = [-3, 6, -3]
# Verify: cross is perpendicular to a and b
assert abs(np.dot(cross, a)) < 1e-10
assert abs(np.dot(cross, b)) < 1e-10

# === Matrix multiplication ===
A = np.array([[1, 2], [3, 4]])
B = np.array([[5, 6], [7, 8]])
C = A @ B                         # = [[19, 22], [43, 50]]
C = np.matmul(A, B)               # Equivalent syntax

# === Transpose ===
A_T = A.T                          # [[1, 3], [2, 4]]
```

> **Example 1** — Verify orthogonality of a rotation matrix
>
> ```python
> import numpy as np
>
> theta = np.radians(30)  # 30 degrees to radians
> R = np.array([
>     [np.cos(theta), -np.sin(theta), 0],
>     [np.sin(theta),  np.cos(theta), 0],
>     [0,              0,             1]
> ])
>
> # Verify R^T R = I
> I_approx = R.T @ R
> print(I_approx)
> # [[1. 0. 0.]
> #  [0. 1. 0.]
> #  [0. 0. 1.]]
>
> # Verify det(R) = 1
> print(np.linalg.det(R))  # 1.0
>
> # Verify inverse equals transpose
> print(np.allclose(np.linalg.inv(R), R.T))  # True
> ```

### 1.3.2 Matrix Decomposition

```python
# === SVD ===
A = np.array([[3, 1], [1, 3]])
U, S, Vt = np.linalg.svd(A)
# U = [[ 0.707,  0.707],   S = [4, 2],
#      [ 0.707, -0.707]]    Vt = [[0.707, 0.707],
#                                  [0.707, -0.707]]
A_recon = U @ np.diag(S) @ Vt   # Reconstruction = A

# Core SLAM usage: solve Ax=0 (8-point algorithm)
# Solution = last row of Vt (smallest singular value)
F_vec = Vt[-1]                   # 1×9 vector
F = F_vec.reshape(3, 3)          # Fundamental matrix

# === Eigenvalue decomposition ===
eigenvalues, eigenvectors = np.linalg.eig(A)
# eigenvalues = [4, 2]
# eigenvectors[:,0] ≈ [0.707, 0.707] (corresponds to λ=4)

# === QR decomposition ===
Q, R = np.linalg.qr(A)
# Q ≈ [[-0.949, -0.316],  R ≈ [[-3.162, -2.530],
#      [-0.316,  0.949]]        [ 0.000,  1.265]]
```

> **Example 2** — Use SVD for the 8-point algorithm (complete workflow)
>
> ```python
> import numpy as np
>
> # 8 pairs of matched pixel coordinates
> pts1 = np.array([
>     [400, 300], [500, 300], [300, 400], [500, 500],
>     [350, 250], [450, 350], [250, 450], [550, 450]
> ])
> pts2 = np.array([
>     [380, 300], [480, 300], [280, 400], [480, 500],
>     [330, 250], [430, 350], [230, 450], [530, 450]
> ])
>
> # Construct A matrix (8×9)
> A = np.zeros((8, 9))
> for i in range(8):
>     u1, v1 = pts1[i]
>     u2, v2 = pts2[i]
>     A[i] = [u1*u2, v1*u2, u2, u1*v2, v1*v2, v2, u1, v1, 1]
>
> # SVD to find minimum eigenvector
> _, _, Vt = np.linalg.svd(A)
> F = Vt[-1].reshape(3, 3)
>
> # Enforce rank(F) = 2
> U, S, Vt = np.linalg.svd(F)
> S[2] = 0
> F_rank2 = U @ np.diag(S) @ Vt
>
> print("F =", F_rank2)
> # Verify: for each pair, p2^T F p1 ≈ 0
> for i in range(8):
>     p1 = np.append(pts1[i], 1)
>     p2 = np.append(pts2[i], 1)
>     epipolar_error = p2 @ F_rank2 @ p1
>     print(f"Pair {i}: epipolar error = {epipolar_error:.6f}")
> ```

### 1.3.3 Homogeneous Coordinate Operations

```python
# === World coordinates → Camera coordinates → Pixel ===
P_world = np.array([2, 1.5, 10, 1])   # Homogeneous coordinates

# Extrinsics: camera at origin
T_cw = np.eye(4)                        # World → Camera

# Intrinsics
fx, fy, cx, cy = 800, 800, 640, 480
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

# Projection
P_cam = T_cw @ P_world                  # Camera coords [2, 1.5, 10, 1]
x_n = P_cam[:2] / P_cam[2]              # Normalized [0.2, 0.15]
pixel = K @ np.array([x_n[0], x_n[1], 1])  # Pixel [800, 600]

print(f"World point {P_world[:3]} → Pixel ({pixel[0]:.0f}, {pixel[1]:.0f})")
```

> **Example 3** — Pose composition across consecutive frames
>
> ```python
> import numpy as np
>
> # Frame 1 → Frame 2: rotate 30° around Z, translate [0.5, 0, 0]
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
> # Frame 2 → Frame 3: same rotation, translate [0, 0.3, 0]
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
> # Compose: Frame 1 → Frame 3
> T13 = T12 @ T23
> print("Transform from frame 1 to frame 3, T13 =")
> print(T13)
> # Rotation = 60° around Z (30+30)
> # Translation ≈ [0.5, 0.3, 0]
> ```

### 1.3.4 Probability-Related Operations

```python
from scipy.stats import multivariate_normal

# === 1D Gaussian ===
mu, sigma = 5.0, 0.1  # Depth 5.0m ± 0.1m
# Probability density at x=5.05
from scipy.stats import norm
pdf_val = norm.pdf(5.05, loc=mu, scale=sigma)
# = 3.52 (see Example 17 in the math module)

# === Multivariate Gaussian: 2D pose uncertainty ===
mu = np.array([2.0, 3.0])               # Estimated pose (x, y)
Sigma = np.array([[0.04, 0.01],          # Covariance matrix
                  [0.01, 0.09]])
rv = multivariate_normal(mu, Sigma)
# Probability density at (1.8, 2.7)
pdf = rv.pdf([1.8, 2.7])
```

---

## 1.4 Bash/Linux Basics

SLAM development and deployment is almost entirely on Linux:

```bash
# Build a C++ SLAM system
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run
./bin/orb_slam_rgbd Vocabulary/ORBvoc.txt config.yaml

# Download a dataset
wget https://example.com/sequence.zip

# Check GPU usage
nvidia-smi

# ROS launch
roslaunch orb_slam3 euroc.launch
```

You won't need these during the learning phase, but know they are the end goal.

---

## 1.5 C++ Core Concepts Overview

If you later read the ORB-SLAM source code, you'll need to understand these C++ features:

### Pointers and References
```cpp
cv::Mat* pImg = &image;    // Pointer: points to a memory address
cv::Mat& refImg = image;   // Reference: alias, safer
```

### Object-Oriented Programming (how SLAM systems are organized)
```cpp
class Tracking {
public:
    cv::Mat GrabImageMonocular(const cv::Mat& im);
private:
    cv::Mat mCurrentFrame;
    std::vector<MapPoint*> mvpLocalMapPoints;
};
```

SLAM system = multiple collaborating classes:
- `Tracking` class handles per-frame pose estimation
- `LocalMapping` class manages keyframes and local BA
- `LoopClosing` class detects loop closures

### Modern C++ Features
```cpp
auto ptr = std::make_shared<MapPoint>(pos);  // Smart pointer (automatic release)
std::vector<MapPoint*> vpPoints;             // STL container
std::mutex mMutexMap;                        // Multithreading mutex
```

### CMake (SLAM project build system)
```cmake
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(my_slam main.cpp)
target_link_libraries(my_slam ${OpenCV_LIBS} Eigen3::Eigen)
```

---

## 1.6 Programming Learning Advice

1. **Levels 1-3: use Python exclusively** — understand the principles, iterate quickly
2. **Late Level 3: read ORB-SLAM source** — cross-reference with Python, focus on core flow
3. **Level 5+: learn Eigen + Ceres** — if you plan to do research/development

You don't need to learn C++ right now. Once you can write a simplified SLAM system in Python, C++ is just another tool.

---

> **Next**: Dive straight into the math and geometry modules — programming skills will improve naturally through the exercises.
