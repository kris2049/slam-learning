"""
练习 4: 图像处理 (交互式)
=============================
Harris 角点、Sobel 边缘、高斯模糊
"""

import numpy as np
from scipy.signal import convolve2d


def task_01_rgb_to_gray():
    """任务1: RGB → 灰度"""
    print("\n═══ 任务1: RGB→灰度 ═══")

    # 创建一个简单的 RGB 图像
    img = np.zeros((50, 50, 3))
    img[10:30, 10:30] = [1.0, 0.0, 0.0]  # 红色方块

    # TODO: RGB 转灰度: Gray = 0.299*R + 0.587*G + 0.114*B
    gray = np.zeros((50, 50))  # TODO

    red_val = gray[20, 20]
    print(f"  红色区域灰度值: {red_val:.3f} (应≈0.299)")

    ok = True
    if abs(red_val - 0.299) > 0.01:
        print("  ❌ gray = 0.299*img[:,:,0] + 0.587*img[:,:,1] + 0.114*img[:,:,2]"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_02_gaussian_blur():
    """任务2: 构造高斯核"""
    print("\n═══ 任务2: 高斯模糊 ═══")

    size, sigma = 5, 1.0

    # TODO: 构造 5x5 高斯核
    # 公式: G(x,y) = exp(-(x^2+y^2)/(2σ^2))
    ax = np.arange(-size//2 + 1, size//2 + 1)
    kernel = np.zeros((size, size))  # TODO: 用广播计算

    # TODO: 归一化（核之和=1）
    kernel = kernel  # TODO: kernel / kernel.sum()

    center_val = kernel[2, 2]
    print(f"  中心权重: {center_val:.3f} (应≈0.09-0.11)")
    print(f"  核之和: {kernel.sum():.4f} (应=1.0)")

    ok = True
    if abs(kernel.sum() - 1.0) > 0.01:
        print("  ❌ 核未归一化。提示: kernel = kernel / kernel.sum()"); ok = False
    if center_val < 0.05 or center_val > 0.15:
        print("  ❌ 公式: exp(-(ax^2[:,None]+ax^2[None,:])/(2*σ^2))"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_03_sobel_edge():
    """任务3: Sobel 边缘检测"""
    print("\n═══ 任务3: Sobel 边缘检测 ═══")

    # 创建一条垂直边缘
    img = np.zeros((30, 30))
    img[:, :15] = 50
    img[:, 15:] = 200

    # TODO: 定义 Sobel X 核 (检测垂直边缘)
    sobel_x = np.zeros((3, 3))  # TODO: [[-1,0,1],[-2,0,2],[-1,0,1]]

    # TODO: 应用卷积
    Gx = np.zeros((30, 30))  # TODO: convolve2d(img, sobel_x, mode='same')

    edge_response = abs(Gx[15, 15])
    print(f"  边缘处 |Gx| = {edge_response:.0f} (应 > 500 — 垂直边缘 → 水平梯度大)")

    ok = True
    if edge_response < 100:
        print("  ❌ sobel_x = [[-1,0,1],[-2,0,2],[-1,0,1]]; Gx=convolve2d(img,sobel_x,mode='same')"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_04_harris_corner():
    """任务4: Harris 角点响应

    Harris: R = det(M) - k * trace(M)^2
    其中 M = [[ΣIx², ΣIxIy], [ΣIxIy, ΣIy²]]
    """
    print("\n═══ 任务4: Harris 角点检测 ═══")

    # 一个简单的 L 型角点
    img = np.ones((20, 20)) * 100
    img[5:15, 5:15] = 200

    sobel_x = np.array([[-1,0,1],[-2,0,2],[-1,0,1]], dtype=float)
    sobel_y = sobel_x.T
    Ix = convolve2d(img, sobel_x, mode='same')
    Iy = convolve2d(img, sobel_y, mode='same')

    # 在角点 (5,5) 处
    y, x = 5, 5
    # TODO: 计算 M 矩阵的各个分量 (简单求和即可)
    Ix2 = 0  # TODO: Ix[y,x]**2
    Iy2 = 0  # TODO: Iy[y,x]**2
    Ixy = 0  # TODO: Ix[y,x] * Iy[y,x]

    # TODO: 计算 Harris 响应 R
    detM = 0    # TODO: Ix2*Iy2 - Ixy**2
    traceM = 0  # TODO: Ix2 + Iy2
    k = 0.04
    R = 0  # TODO: detM - k * traceM**2

    print(f"  角点处 R = {R:.0f} (应 > 0 — 角点)")
    print(f"  提示: 角点→R>0, 边缘→R<0, 平坦→R≈0")

    ok = True
    if R < 0:
        print("  ❌ R 应为正数。Ix2=Ix[y,x]**2, detM=Ix2*Iy2-Ixy**2, traceM=Ix2+Iy2"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_05_canny():
    """任务5: Canny 双阈值

    Canny 用双阈值: 强边缘(>高阈值) 连接 弱边缘(>低阈值)。
    """
    print("\n═══ 任务5: Canny 双阈值 ═══")

    # 模拟梯度幅值图
    magnitude = np.random.RandomState(0).randn(20, 20) * 30 + 100
    # 在第10列放一条强边缘
    magnitude[:, 10] = 255

    high_thresh = 150
    low_thresh = 50

    # TODO: 强边缘
    strong = np.zeros((20, 20))  # TODO: magnitude > high_thresh
    # TODO: 弱边缘
    weak = np.zeros((20, 20))    # TODO: (magnitude > low_thresh) & ~strong

    strong_pixels = np.sum(strong)
    weak_pixels = np.sum(weak)
    print(f"  强边缘: {strong_pixels} 像素, 弱边缘: {weak_pixels} 像素")

    ok = True
    if strong_pixels < 15:
        print("  ❌ strong = magnitude > high_thresh; weak = (magnitude>low_thresh) & ~strong"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


if __name__ == "__main__":
    results = [task_01_rgb_to_gray(), task_02_gaussian_blur(),
               task_03_sobel_edge(), task_04_harris_corner(), task_05_canny()]
    passed = sum(results)
    print(f"\n{'='*40}")
    print(f"  通过: {passed}/{len(results)}")
    if passed == len(results):
        print("  🎉 全部完成!")
    else:
        print(f"  还有 {len(results)-passed} 个任务，改完重跑。")
