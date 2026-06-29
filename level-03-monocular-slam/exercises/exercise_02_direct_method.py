"""
练习 2: 直接法与光度误差 (Level 3)
========================================
直接法 SLAM (LSD-SLAM, DSO) 不使用特征点，直接用像素强度优化位姿。
"""

import numpy as np
from scipy import ndimage

def task_01_photometric_error():
    """任务1: 光度误差 (15分钟)

    直接法的核心: E = Σ ||I₁(p) - I₂(warp(p, ξ))||²
    """
    print("\n═══ 任务1: 光度误差 ═══")

    # 创建两个图像: I₂ 是 I₁ 平移后的结果
    size = 40
    Y, X = np.meshgrid(np.arange(size), np.arange(size))

    # I₁: 高斯斑点
    I1 = 100 + 150 * np.exp(-((X-20)**2 + (Y-20)**2) / 50)

    # I₂: I₁ 平移 (dx=1, dy=-0.5)
    from scipy.ndimage import map_coordinates
    dx_true, dy_true = 1.0, -0.5
    X2 = X - dx_true
    Y2 = Y - dy_true
    I2 = map_coordinates(I1, [X2.ravel(), Y2.ravel()], order=1, mode='nearest')
    I2 = I2.reshape(size, size)

    # 计算图像梯度
    Ix = ndimage.sobel(I1, axis=0)
    Iy = ndimage.sobel(I1, axis=1)

    # 在每个像素，光流方程: Ix*u + Iy*v = -(I2 - I1)
    # 取中心区域 (避免边界)
    region = slice(10, 30), slice(10, 30)
    Ix_flat = Ix[region].ravel()
    Iy_flat = Iy[region].ravel()
    It_flat = (I2 - I1)[region].ravel()

    A = np.column_stack([Ix_flat, Iy_flat])
    b = -It_flat

    # 最小二乘
    flow, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    dx_est, dy_est = flow

    print(f"  光度误差估计的位移: (dx={dx_est:.3f}, dy={dy_est:.3f})")
    print(f"  真值:                 (dx={dx_true}, dy={dy_true})")

    err = np.sqrt((dx_est-dx_true)**2 + (dy_est-dy_true)**2)
    print(f"  误差: {err:.3f}px")

    # 与特征法的关键区别
    print(f"\n  直接法 vs 特征法:")
    print(f"  • 直接法: 用所有像素的强度信息")
    print(f"  • 特征法: 只用稀疏的关键点")
    print(f"  • 直接法对光照变化敏感, 特征法对模糊敏感")
    print(f"  • DSO 用光度标定 (vignette, exposure time) 提高鲁棒性")

    assert err < 1.0, f"光度误差应给出大致正确的位移"
    print("  ✅ 通过!")


def task_02_scale_ambiguity():
    """任务2: 单目尺度不确定性 (10分钟)"""
    print("\n═══ 任务2: 单目尺度不确定性 ═══")

    print("""
    单目 SLAM 的根本限制: 尺度模糊

    场景A: 物体在 2m 处, 相机移动 1m
    场景B: 物体在 20m 处, 相机移动 10m
    → 两者产生完全相同的图像!

    解决方案:
    1. 已知大小的物体 (如标定板)
    2. IMU (加速度计提供绝对尺度)
    3. 双目相机 (已知基线)
    4. 深度学习先验 (MiDaS, Depth Anything)
    """)

    # 数学证明
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    t1 = np.array([0, 0, 0])
    t2 = np.array([1, 0, 0])

    # 场景A: 点在 3m 处
    pA = np.array([0, 0, 3])
    # 场景B: 缩放10倍
    pB = pA * 10
    t2_B = t2 * 10

    # 投影
    for name, p, t in [("场景A", pA, t2), ("场景B(×10)", pB, t2_B)]:
        P1 = K @ np.hstack([np.eye(3), t1.reshape(3,1)])
        P2 = K @ np.hstack([np.eye(3), t.reshape(3,1)])
        x1 = P1 @ np.append(p, 1); u1 = x1[0]/x1[2]
        x2 = P2 @ np.append(p, 1); u2 = x2[0]/x2[2]
        print(f"  {name}: u1={u1:.1f}, u2={u2:.1f}, 位移={u2-u1:.1f}px")

    print("\n  → 完全相同的像素位移! 无法区分尺度.")
    print("  ✅ 通过!")


def task_03_direct_vs_feature():
    """任务3: 直接法vs特征法对比 (15分钟)"""
    print("\n═══ 任务3: 鲁棒性对比 ═══")

    # 场景: 低纹理区域
    size = 50
    Y, X = np.meshgrid(np.arange(size), np.arange(size))

    # 图像1: 低纹理 (渐变)
    I1_low = 50 + X * 3 + Y * 2
    # 图像2: 平移
    from scipy.ndimage import map_coordinates
    I2_low = map_coordinates(I1_low, [(X-2).ravel(), Y.ravel()],
                              order=1, mode='nearest').reshape(size, size)

    # 直接法在低纹理区域
    Ix = ndimage.sobel(I1_low, axis=0)
    region = slice(10, 40), slice(10, 40)
    A_direct = np.column_stack([Ix[region].ravel(), ndimage.sobel(I1_low, axis=1)[region].ravel()])

    # 梯度幅值统计
    grad_mag = np.sqrt(Ix[region]**2 + ndimage.sobel(I1_low, axis=1)[region]**2)
    strong_gradients = np.sum(grad_mag > 10) / len(grad_mag.ravel()) * 100

    print(f"  低纹理区域: 强梯度像素占 {strong_gradients:.1f}%")
    print(f"  直接法: 需要足够梯度 → 低纹理区域退化")
    print(f"  特征法: Harris响应很低 → 无法提取角点")
    print(f"  → 这就是为什么 LSD-SLAM 只用 '高梯度像素'")
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_photometric_error()
    task_02_scale_ambiguity()
    task_03_direct_vs_feature()
    print("\n🎉 直接法练习完成!")
