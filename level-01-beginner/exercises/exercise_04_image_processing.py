"""
练习 4: 图像处理基础
========================
角点检测、边缘检测 — SLAM 前端特征提取的基础。
"""

import numpy as np
from scipy import ndimage
from scipy.signal import convolve2d

def task_01_color_and_grayscale():
    """任务1: 彩色图像与灰度转换 (5分钟)"""
    print("\n═══ 任务1: 彩色与灰度 ═══")

    # 创建一个简单的合成"图像" (RGB)
    image_rgb = np.zeros((100, 100, 3), dtype=np.float64)
    image_rgb[20:50, 30:70] = [1.0, 0.0, 0.0]   # 红色方块
    image_rgb[50:80, 40:80] = [0.0, 1.0, 0.0]   # 绿色方块
    image_rgb[10:40, 60:90] = [0.0, 0.0, 1.0]   # 蓝色方块

    # TODO: RGB → 灰度 (加权平均)
    gray = 0.299 * image_rgb[:,:,0] + 0.587 * image_rgb[:,:,1] + 0.114 * image_rgb[:,:,2]

    print(f"  RGB 形状: {image_rgb.shape}")
    print(f"  灰度形状: {gray.shape}")
    print(f"  红色区域灰度值: {gray[25, 50]:.3f} (≈0.299)")
    print(f"  绿色区域灰度值: {gray[65, 60]:.3f} (≈0.587)")

    assert gray.shape == (100, 100)
    assert abs(gray[25, 50] - 0.299) < 0.01
    print("  ✅ 通过!")

    return image_rgb, gray


def task_02_gaussian_blur():
    """任务2: 高斯模糊 (10分钟)

    图像去噪和平滑 — 特征提取前的必要步骤。
    """
    print("\n═══ 任务2: 高斯模糊 ═══")

    # 创建带噪声的测试图像 (一条边 + 噪声)
    image = np.zeros((50, 50), dtype=np.float64)
    image[:, :25] = 100   # 左半亮
    image[:, 25:] = 200   # 右半更亮

    # 加噪声
    noisy = image + np.random.normal(0, 15, image.shape)

    # TODO: 实现高斯核
    size, sigma = 5, 1.4
    ax = np.arange(-size//2 + 1, size//2 + 1)
    gauss_kernel = np.exp(-(ax**2)[:, None] / (2*sigma**2) - (ax**2)[None, :] / (2*sigma**2))
    gauss_kernel /= gauss_kernel.sum()

    print(f"  高斯核 (σ={sigma}, {size}x{size}):")
    print(f"  中心权重: {gauss_kernel[2,2]:.3f}")
    print(f"  角权重:   {gauss_kernel[0,0]:.3f}")
    print(f"  核之和:   {gauss_kernel.sum():.6f} (应为1)")

    # 应用模糊
    blurred = convolve2d(noisy, gauss_kernel, mode='same', boundary='symm')

    # 验证: 降噪效果
    noise_before = np.std(noisy[10:20, 10:20])
    noise_after = np.std(blurred[10:20, 10:20])
    print(f"  模糊前局部标准差: {noise_before:.1f}")
    print(f"  模糊后局部标准差: {noise_after:.1f} (应减少)")
    assert noise_after < noise_before, "模糊应该降低噪声"
    print("  ✅ 通过!")

    return noisy, blurred


def task_03_sobel_edge():
    """任务3: Sobel 边缘检测 (10分钟)

    边缘是图像中强度变化剧烈的地方 — 视觉 SLAM 的基础特征。
    """
    print("\n═══ 任务3: Sobel 边缘检测 ═══")

    # 创建一条明显的垂直边缘
    image = np.zeros((50, 50), dtype=np.float64)
    image[:, :25] = 50
    image[:, 25:] = 200

    # Sobel 核 (水平梯度 Gx)
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float64)
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=np.float64)

    # 应用
    Gx = convolve2d(image, sobel_x, mode='same', boundary='symm')
    Gy = convolve2d(image, sobel_y, mode='same', boundary='symm')

    # TODO: 梯度幅值和方向
    gradient_magnitude = np.sqrt(Gx**2 + Gy**2)
    gradient_direction = np.arctan2(Gy, Gx)

    print(f"  垂直边缘处 Gx 值: {Gx[25, 25]:.0f} (应很大 — 水平方向变化剧烈)")
    print(f"  垂直边缘处 Gy 值: {Gy[25, 25]:.0f} (应≈0 — 垂直方向无变化)")
    print(f"  平坦区域梯度幅值: {gradient_magnitude[10, 10]:.0f} (应≈0)")

    # 验证边缘检测原理
    assert abs(Gx[25, 25]) > 200, "边缘处水平梯度应很大"
    assert abs(Gy[25, 25]) < 50, "垂直边缘处垂直梯度应≈0"
    print("  ✅ 通过!")

    return image, gradient_magnitude


def task_04_harris_corner():
    """任务4: Harris 角点检测 (15分钟)

    角点: 两个方向都有大梯度的地方。SLAM 中最常用的特征点。
    """
    print("\n═══ 任务4: Harris 角点检测 ═══")

    # 创建测试图像: 包含角点、边缘、平坦区域
    image = np.ones((60, 60), dtype=np.float64) * 100  # 平坦背景

    # 画一个矩形（创建4个角点）
    image[15:45, 15:45] = 200  # 亮矩形

    # 计算梯度
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float64)
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=np.float64)
    Ix = convolve2d(image, sobel_x, mode='same', boundary='symm')
    Iy = convolve2d(image, sobel_y, mode='same', boundary='symm')

    # 构造 M 矩阵的元素 (每个像素的局部窗口内求和)
    # M = [[ΣIx², ΣIxIy], [ΣIxIy, ΣIy²]]
    window_size = 3
    Ix2 = Ix**2
    Iy2 = Iy**2
    Ixy = Ix * Iy

    # 高斯加权窗口
    ax = np.arange(-1, 2)
    w = np.exp(-(ax**2)[:, None] / 2 - (ax**2)[None, :] / 2)
    w /= w.sum()

    Sxx = convolve2d(Ix2, w, mode='same', boundary='symm')
    Syy = convolve2d(Iy2, w, mode='same', boundary='symm')
    Sxy = convolve2d(Ixy, w, mode='same', boundary='symm')

    # TODO: Harris 响应 R = det(M) - k * trace(M)²
    k = 0.04
    det_M = Sxx * Syy - Sxy**2
    trace_M = Sxx + Syy
    R = det_M - k * trace_M**2

    # 找到响应最大的点（角点）
    # 排除边界
    R_padded = R.copy()
    R_padded[:2, :] = -np.inf; R_padded[-2:, :] = -np.inf
    R_padded[:, :2] = -np.inf; R_padded[:, -2:] = -np.inf

    flat_indices = np.argsort(R_padded.ravel())[::-1]
    top_corners = []
    for idx in flat_indices[:20]:
        y, x = divmod(idx, R.shape[1])
        if R[y, x] > 0:  # R > 0 才是角点
            top_corners.append((y, x))

    print(f"  检测到的角点数 (R>0): {len(top_corners)}")
    print(f"  最强角点位置: {top_corners[:4]}")
    print(f"  应该检测到矩形的4个角点")

    # 验证: 矩形角点处 R 应该 > 其他位置
    # 角点 (15,15) 处
    R_corner = R[15, 15]
    # 边缘中心处 (不应是角点)
    R_edge = R[30, 15]
    # 平坦区域
    R_flat = R[5, 5]

    print(f"  R(角点): {R_corner:.0f}")
    print(f"  R(边缘): {R_edge:.0f}")
    print(f"  R(平坦): {R_flat:.0f}")
    print(f"  角点R > 边缘R? {R_corner > R_edge}")
    print(f"  角点R > 平坦R? {R_corner > R_flat}")

    assert R_corner > 0, "角点应该有正响应"
    print("  ✅ 通过!")

    return image, R


def task_05_canny_edge():
    """任务5: Canny 边缘检测 (10分钟)

    多阶段边缘检测 — 比 Sobel 更精确。
    """
    print("\n═══ 任务5: Canny 边缘检测 ═══")

    # 使用之前的图像
    image = np.zeros((50, 50), dtype=np.float64)
    image[:, :25] = 50
    image[:, 25:] = 200

    # 步骤1: 高斯平滑
    sigma = 1.0
    image_blurred = ndimage.gaussian_filter(image, sigma)

    # 步骤2: 计算梯度
    Gx = ndimage.sobel(image_blurred, axis=1)
    Gy = ndimage.sobel(image_blurred, axis=0)
    magnitude = np.hypot(Gx, Gy)
    direction = np.arctan2(Gy, Gx)

    # 步骤3: 非极大值抑制 (简化版)
    suppressed = np.zeros_like(magnitude)
    for i in range(1, magnitude.shape[0]-1):
        for j in range(1, magnitude.shape[1]-1):
            if magnitude[i, j] > 50:  # 简化的阈值判断
                # 检查是否为局部最大值
                local_max = True
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        if magnitude[i+di, j+dj] > magnitude[i, j]:
                            local_max = False
                if local_max:
                    suppressed[i, j] = magnitude[i, j]

    # 步骤4: 双阈值 (简化)
    edges = suppressed > 100

    edge_count = np.sum(edges)
    print(f"  检测到的边缘像素: {edge_count}")
    print(f"  边缘像素占总像素: {edge_count / (50*50) * 100:.1f}%")
    print(f"  边缘位置: 列{np.where(edges.any(axis=0))[0]} (应接近25)")

    # 验证边缘位置
    edge_cols = np.where(edges.any(axis=0))[0]
    assert len(edge_cols) > 0, "应该检测到边缘"
    assert abs(np.median(edge_cols) - 25) < 5, "边缘应在第25列附近"
    print("  ✅ 通过!")

    return magnitude, edges


if __name__ == "__main__":
    task_01_color_and_grayscale()
    task_02_gaussian_blur()
    task_03_sobel_edge()
    task_04_harris_corner()
    task_05_canny_edge()
    print("\n🎉 所有图像处理练习完成!")
