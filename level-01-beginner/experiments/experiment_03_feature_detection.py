"""
实验 3: 特征检测对比 — Harris vs SIFT-like
================================================
目标: 理解不同特征检测器的行为差异。

运行: python3 experiment_03_feature_detection.py
"""

import numpy as np
from scipy import ndimage
from scipy.signal import convolve2d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def create_test_image():
    """创建包含角点、边缘和纹理的测试图像"""
    img = np.ones((200, 300), dtype=np.float64) * 128

    # 矩形 (4个角点)
    img[40:100, 50:150] = 200
    # 圆形
    cy, cx = 150, 200
    Y, X = np.ogrid[:200, :300]
    mask = (X - cx)**2 + (Y - cy)**2 < 30**2
    img[mask] = 50
    # 三角形
    for y in range(120, 180):
        for x in range(20, 80):
            if x >= 20 + (y-120)*0.5 and x <= 80 - (y-120)*0.5:
                img[y, x] = 180
    # 加纹理 (正弦波)
    img += 20 * np.sin(X/10) * np.sin(Y/10)

    return img


def harris_corner_detector(image, k=0.04, threshold=0.01):
    """Harris 角点检测器"""
    # 梯度
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float64)
    sobel_y = sobel_x.T
    Ix = convolve2d(image, sobel_x, mode='same', boundary='symm')
    Iy = convolve2d(image, sobel_y, mode='same', boundary='symm')

    # 高斯加权
    sigma = 2.0
    Ix2 = ndimage.gaussian_filter(Ix**2, sigma)
    Iy2 = ndimage.gaussian_filter(Iy**2, sigma)
    Ixy = ndimage.gaussian_filter(Ix*Iy, sigma)

    # Harris 响应
    det = Ix2 * Iy2 - Ixy**2
    trace = Ix2 + Iy2
    R = det - k * trace**2

    # 非极大值抑制
    from scipy.ndimage import maximum_filter
    local_max = (R == maximum_filter(R, size=5))
    threshold_abs = threshold * R.max()
    corners = np.where(local_max & (R > threshold_abs))

    return np.column_stack([corners[1], corners[0]]), R


def fast_corner_detector(image, n=12, threshold=30):
    """简化版 FAST 角点检测器

    FAST: 如果圆周上有12个连续像素都比中心亮/暗，就是角点。
    """
    h, w = image.shape
    corners = []

    # 圆周上16个点的偏移 (Bresenham 圆, 半径3)
    circle_offsets = [
        (0, 3), (1, 3), (2, 2), (3, 1),
        (3, 0), (3, -1), (2, -2), (1, -3),
        (0, -3), (-1, -3), (-2, -2), (-3, -1),
        (-3, 0), (-3, 1), (-2, 2), (-1, 3)
    ]

    for y in range(3, h-3):
        for x in range(3, w-3):
            p_center = image[y, x]

            # 快速排除: 检查第1,5,9,13点
            test_indices = [0, 4, 8, 12]
            brighter = 0
            darker = 0
            for idx in test_indices:
                dx, dy = circle_offsets[idx]
                p_val = image[y+dy, x+dx]
                if p_val > p_center + threshold:
                    brighter += 1
                elif p_val < p_center - threshold:
                    darker += 1

            if brighter < 3 and darker < 3:
                continue  # 快速排除

            # 完整检查
            circle_vals = [image[y+dy, x+dx] for dx, dy in circle_offsets]

            brighter_count = 0
            darker_count = 0

            for i in range(16):
                if circle_vals[i] > p_center + threshold:
                    brighter_count += 1
                    if brighter_count >= n:
                        corners.append((x, y))
                        break
                else:
                    brighter_count = 0

                if circle_vals[i] < p_center - threshold:
                    darker_count += 1
                    if darker_count >= n:
                        corners.append((x, y))
                        break
                else:
                    darker_count = 0

    return np.array(corners) if corners else np.zeros((0, 2), dtype=int)


def experiment():
    print("═" * 60)
    print("  实验: 角点检测器对比")
    print("═" * 60)

    image = create_test_image()

    # 检测角点
    harris_corners, harris_response = harris_corner_detector(image)
    fast_corners = fast_corner_detector(image, n=10, threshold=25)

    print(f"  Harris 检测到: {len(harris_corners)} 个角点")
    print(f"  FAST 检测到:   {len(fast_corners)} 个角点")

    # 可视化
    fig, axes = plt.subplots(1, 4, figsize=(18, 5))

    # 1. 原图
    axes[0].imshow(image, cmap='gray')
    axes[0].set_title('测试图像', fontsize=12)
    axes[0].axis('off')

    # 2. Harris 响应图
    im2 = axes[1].imshow(harris_response, cmap='hot')
    axes[1].set_title('Harris 响应 (R)', fontsize=12)
    axes[1].axis('off')
    plt.colorbar(im2, ax=axes[1], fraction=0.046)

    # 3. Harris 角点
    axes[2].imshow(image, cmap='gray')
    for x, y in harris_corners:
        axes[2].plot(x, y, 'r+', markersize=8, markeredgewidth=1.5)
    axes[2].set_title(f'Harris: {len(harris_corners)} 角点', fontsize=12)
    axes[2].axis('off')

    # 4. FAST 角点
    axes[3].imshow(image, cmap='gray')
    if len(fast_corners) > 0:
        axes[3].scatter(fast_corners[:, 0], fast_corners[:, 1],
                       c='cyan', marker='+', s=60, linewidth=1.5)
    axes[3].set_title(f'FAST: {len(fast_corners)} 角点', fontsize=12)
    axes[3].axis('off')

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-01-beginner/experiments/feature_detection.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  📊 保存到 feature_detection.png")

    # 分析
    print(f"\n  🔍 分析:")
    print(f"  1. Harris: 对L型和T型角点敏感，重复性好")
    print(f"  2. FAST: 速度快(只比较亮度)，适合实时SLAM前端")
    print(f"  3. Harris响应图：角点处R值为正且大，边缘处为负，平坦区≈0")
    print(f"  4. 实际SLAM中 ORB-SLAM 使用 oFAST (带方向) + rBRIEF (描述子)")
    print(f"  ✅ 实验完成!")


if __name__ == "__main__":
    experiment()
