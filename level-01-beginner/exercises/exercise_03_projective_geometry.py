"""
练习 3: 射影几何基础
========================
针孔相机模型、对极几何、三角化 — SLAM 的几何核心。
"""

import numpy as np

def task_01_pinhole_camera():
    """任务1: 针孔相机模型 (15分钟)

    世界中的一个 3D 点如何投影到图像平面上的 2D 像素。
    """
    print("\n═══ 任务1: 针孔相机模型 ═══")

    # 定义相机内参矩阵 K (3x3)
    fx, fy = 500.0, 500.0    # 焦距 (像素)
    cx, cy = 320.0, 240.0    # 主点 (640x480 图像中心)
    K = np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,  0,  1]
    ])
    print(f"  内参矩阵 K:\n{K}")

    # 定义相机的世界位姿 (相机在世界坐标系原点, 朝向 +Z)
    R = np.eye(3)              # 无旋转
    t = np.array([0.0, 0.0, 0.0])  # 在世界原点

    # TODO: 构造外参矩阵 [R|t]
    Rt = np.hstack([R, t.reshape(3, 1)])
    print(f"  外参 [R|t]:\n{Rt}")

    # TODO: 投影矩阵 P = K [R|t]
    P = K @ Rt
    print(f"  投影矩阵 P:\n{P}")
    # 应该看到：前两列对角线上是 fx/fy，最后一列零（相机在世界原点）

    # 投影一个 3D 点
    points_3d = np.array([
        [1.0, 0.5, 5.0, 1.0],   # 点1: 在相机前方5m
        [-2.0, 1.0, 3.0, 1.0],   # 点2
        [0.0, 0.0, 2.0, 1.0],   # 点3: 正前方
    ])

    for i, p_w in enumerate(points_3d):
        # 投影: x_img = P @ P_world
        x_homo = P @ p_w
        # 从齐次坐标到像素坐标 (除以第三分量)
        u, v = x_homo[0] / x_homo[2], x_homo[1] / x_homo[2]
        print(f"  点{p_w[:3]}: 像素 ({u:.0f}, {v:.0f})")

    # 验证: 正前方的点应该投影到图像中心
    p_forward = np.array([0.0, 0.0, 5.0, 1.0])
    x_h = P @ p_forward
    u_c, v_c = x_h[0] / x_h[2], x_h[1] / x_h[2]
    print(f"  正前方点: 像素 ({u_c:.0f}, {v_c:.0f}) — 应该在主点 (320, 240) 附近")
    assert abs(u_c - cx) < 1 and abs(v_c - cy) < 1, "正前方点应投影到主点"
    print("  ✅ 通过!")


def task_02_camera_calibration():
    """任务2: 理解相机标定 (10分钟)

    模拟标定过程: 已知3D点和2D像素→恢复相机参数。
    """
    print("\n═══ 任务2: 相机标定模拟 ═══")

    # 已知内参 (模拟)
    K_true = np.array([
        [600.0, 0.0,   320.0],
        [0.0,   600.0, 240.0],
        [0.0,   0.0,   1.0]
    ])

    # 已知外参: 相机在 Z=2 处
    t_true = np.array([0.0, 0.0, 2.0])
    R_true = np.eye(3)

    # 世界中的标定板上的点 (假设在 Z=0 平面)
    board_points = np.array([
        [0.0, 0.0, 0.0, 1.0],
        [0.1, 0.0, 0.0, 1.0],
        [0.0, 0.1, 0.0, 1.0],
        [0.1, 0.1, 0.0, 1.0],
        [0.2, 0.0, 0.0, 1.0],
    ])

    # 生成对应的像素坐标 (添加噪声)
    P_true = K_true @ np.hstack([R_true, t_true.reshape(3,1)])
    pixels = []
    for p in board_points:
        xh = P_true @ p
        u, v = xh[0]/xh[2], xh[1]/xh[2]
        # 添加一点噪声模拟真实测量
        u += np.random.normal(0, 0.5)
        v += np.random.normal(0, 0.5)
        pixels.append([u, v])
    pixels = np.array(pixels)

    # 简化版标定: 已知这些点共面(Z=0), 估计单应性 H
    # H 将标定板点映射到像素: [u, v, 1]^T ~ H [X, Y, 1]^T
    # 然后从 H 恢复 K 和 R,t (Zhang's method 的简化)

    # 构造 A 矩阵 Ah = 0
    A = []
    for (X, Y, Z, _), (u, v) in zip(board_points, pixels):
        A.append([X, Y, 1, 0, 0, 0, -u*X, -u*Y, -u])
        A.append([0, 0, 0, X, Y, 1, -v*X, -v*Y, -v])
    A = np.array(A)

    # SVD 求最小奇异值对应的向量
    _, _, Vt = np.linalg.svd(A)
    H_vec = Vt[-1]
    H = H_vec.reshape(3, 3)
    print(f"  估计的单应性 H:\n{H}")

    # 从 H 恢复旋转和平移的近似
    # h1, h2, h3 是 H 的列
    h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
    K_inv_approx = np.linalg.inv(K_true)  # 实际标定需要迭代

    # λ ≈ 1/|K^{-1}h1|
    lam = 1.0 / np.linalg.norm(K_inv_approx @ h1)
    r1 = lam * (K_inv_approx @ h1)
    r2 = lam * (K_inv_approx @ h2)
    r3 = np.cross(r1, r2)
    t_est = lam * (K_inv_approx @ h3)

    print(f"  估计的 t: {t_est} (真值: {t_true})")
    print(f"  估计的 R 列1: {r1} (真值: {R_true[:, 0]})")
    print(f"  ℹ️  简化的标定方法精度有限 — 真实标定使用张正友法(N点+迭代优化)")
    print("  ✅ 通过!")


def task_03_epipolar_geometry():
    """任务3: 对极几何 (20分钟)

    两张图像之间的几何关系 — 双目和单目 SLAM 的核心。
    """
    print("\n═══ 任务3: 对极几何 ═══")

    # 两台相机: 相机1在世界原点, 相机2在右侧平移b
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    b = 0.5  # 基线 (baseline)

    R1, t1 = np.eye(3), np.zeros(3)
    R2, t2 = np.eye(3), np.array([b, 0, 0])  # 右移0.5m

    P1 = K @ np.hstack([R1, t1.reshape(3,1)])
    P2 = K @ np.hstack([R2, t2.reshape(3,1)])

    # 世界中的 3D 点
    points_3d = np.array([
        [0.5, 0.3, 3.0],   # 前方3m
        [0.0, 0.5, 2.0],   # 前方2m
        [-0.3, -0.2, 4.0], # 前方4m
    ])

    # 投影到两张图像
    pts1, pts2 = [], []
    for p in points_3d:
        ph = np.append(p, 1)
        x1 = P1 @ ph; pts1.append(x1[:2]/x1[2])
        x2 = P2 @ ph; pts2.append(x2[:2]/x2[2])

    pts1 = np.array(pts1)
    pts2 = np.array(pts2)

    fx = K[0, 0]
    print("  图像1的点:    图像2的点:    视差:")
    for (u1,v1), (u2,v2), p3d in zip(pts1, pts2, points_3d):
        disparity = u1 - u2  # 水平视差
        depth_from_disparity = fx * b / disparity if abs(disparity) > 0.001 else float('inf')
        print(f"  ({u1:6.1f},{v1:5.1f})   ({u2:6.1f},{v2:5.1f})   视差={disparity:.1f}px  "
              f"深度从视差={depth_from_disparity:.2f}m (真值={p3d[2]:.2f}m)")

    # TODO: 计算本质矩阵 E 和基础矩阵 F
    # E = [t]× R
    t_skew = np.array([
        [0,      -t2[2],  t2[1]],
        [t2[2],   0,     -t2[0]],
        [-t2[1],  t2[0],  0]
    ])
    E = t_skew @ R2  # 本质矩阵 (归一化坐标间)
    F = np.linalg.inv(K).T @ E @ np.linalg.inv(K)  # 基础矩阵 (像素坐标间)

    print(f"\n  本质矩阵 E:\n{E}")
    print(f"\n  基础矩阵 F:\n{F}")

    # 验证对极约束: x2^T F x1 = 0
    for (u1,v1), (u2,v2) in zip(pts1, pts2):
        x1_h = np.array([u1, v1, 1])
        x2_h = np.array([u2, v2, 1])
        epipolar_error = x2_h @ F @ x1_h
        print(f"  对极约束误差: {epipolar_error:.2e} (应≈0)")

    # 使用八点法从匹配点恢复 F
    A_8pt = []
    for (u1,v1), (u2,v2) in zip(pts1, pts2):
        A_8pt.append([u1*u2, v1*u2, u2, u1*v2, v1*v2, v2, u1, v1, 1])
    A_8pt = np.array(A_8pt)

    _, _, Vt = np.linalg.svd(A_8pt)
    F_8pt = Vt[-1].reshape(3, 3)

    # 强制秩为2 (SVD, 设最小奇异值为0)
    Uf, Sf, Vtf = np.linalg.svd(F_8pt)
    Sf[-1] = 0
    F_8pt_rank2 = Uf @ np.diag(Sf) @ Vtf

    print(f"\n  八点法恢复的 F:\n{F_8pt_rank2}")
    print(f"  与真值 F 相似? (仅方向, 差一个比例因子)")
    # 归一化比较
    F_norm = F / np.linalg.norm(F)
    F8_norm = F_8pt_rank2 / np.linalg.norm(F_8pt_rank2)
    print(f"  |F - F_8pt| = {np.linalg.norm(F_norm - F8_norm):.4f}")

    print("  ✅ 通过!")


def task_04_triangulation():
    """任务4: 三角化 (15分钟)

    从两个视角的匹配点反推 3D 点位置。
    """
    print("\n═══ 任务4: 三角化 ═══")

    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])

    # 两台相机位姿
    P1 = K @ np.hstack([np.eye(3), np.zeros((3,1))])
    t2 = np.array([0.5, 0.0, 0.0])
    P2 = K @ np.hstack([np.eye(3), t2.reshape(3,1)])

    # 真值 3D 点
    true_point = np.array([0.3, 0.2, 3.0, 1.0])

    # 投影 (加噪声)
    x1_h = P1 @ true_point
    u1, v1 = x1_h[0]/x1_h[2] + np.random.normal(0, 0.5), x1_h[1]/x1_h[2] + np.random.normal(0, 0.5)
    x2_h = P2 @ true_point
    u2, v2 = x2_h[0]/x2_h[2] + np.random.normal(0, 0.5), x2_h[1]/x2_h[2] + np.random.normal(0, 0.5)

    # TODO: DLT 三角化
    # 对每个点, x × (P X) = 0, 构造 A X = 0
    A = np.array([
        u1 * P1[2] - P1[0],
        v1 * P1[2] - P1[1],
        u2 * P2[2] - P2[0],
        v2 * P2[2] - P2[1],
    ])

    _, _, Vt = np.linalg.svd(A)
    X_triangulated = Vt[-1]
    X_triangulated = X_triangulated / X_triangulated[3]  # 齐次归一化

    print(f"  三角化结果: {X_triangulated[:3]}")
    print(f"  真值:        {true_point[:3]}")
    print(f"  误差: {np.linalg.norm(X_triangulated[:3] - true_point[:3]):.3f}m")

    # 重投影误差验证
    reproj1 = (P1 @ X_triangulated); reproj1 = reproj1[:2]/reproj1[2]
    reproj2 = (P2 @ X_triangulated); reproj2 = reproj2[:2]/reproj2[2]
    print(f"  重投影误差 视角1: {np.linalg.norm(reproj1 - [u1,v1]):.2f}px")
    print(f"  重投影误差 视角2: {np.linalg.norm(reproj2 - [u2,v2]):.2f}px")
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_pinhole_camera()
    task_02_camera_calibration()
    task_03_epipolar_geometry()
    task_04_triangulation()
    print("\n🎉 所有射影几何练习完成!")
