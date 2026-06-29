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

## 1.3 Bash/Linux 基础

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

## 1.4 C++ 核心概念速览

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

## 1.5 编程学习建议

1. **Level 1-3 全用 Python**：理解原理，快速迭代
2. **Level 3 后期读 ORB-SLAM 源码**：对照 Python 理解，只看核心流程
3. **Level 5+ 学 Eigen + Ceres**：如果要做研究/开发

不需要现在就学 C++。当你能用 Python 写出一个简化版 SLAM 系统后，C++ 只是另一个工具。

---

> **下一步**: 直接开始学数学和几何模块，编程能力会在练习中自然提升。
