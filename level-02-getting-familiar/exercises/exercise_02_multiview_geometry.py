"""
练习 2: 多视图几何 — RANSAC, PnP, ICP (Level 2)
======================================================
"""

import numpy as np

def task_01_ransac():
    """任务1: RANSAC — 外点剔除 (15分钟)

    RANSAC 通过随机采样一致集来估计模型，过滤外点。
    """
    print("\n═══ 任务1: RANSAC 直线拟合 ═══")

    np.random.seed(42)
    n_total = 100
    n_inliers = 70

    # 真实直线: y = 2x + 1
    true_slope, true_intercept = 2.0, 1.0

    # 生成内点 (在直线附近)
    x_inliers = np.random.uniform(0, 10, n_inliers)
    y_inliers = true_slope * x_inliers + true_intercept + np.random.normal(0, 0.5, n_inliers)

    # 生成外点 (随机散布)
    x_outliers = np.random.uniform(0, 10, n_total - n_inliers)
    y_outliers = np.random.uniform(0, 25, n_total - n_inliers)

    X = np.concatenate([x_inliers, x_outliers])
    Y = np.concatenate([y_inliers, y_outliers])

    # RANSAC
    n_iterations = 100
    threshold = 1.0
    best_inliers = 0
    best_slope, best_intercept = 0, 0

    for _ in range(n_iterations):
        # 随机采样2个点
        idx = np.random.choice(len(X), 2, replace=False)
        x1, x2 = X[idx[0]], X[idx[1]]
        y1, y2 = Y[idx[0]], Y[idx[1]]

        if abs(x2 - x1) < 0.01:
            continue

        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1

        # 计算一致集
        residuals = np.abs(Y - (slope * X + intercept))
        inliers = np.sum(residuals < threshold)

        if inliers > best_inliers:
            best_inliers = inliers
            best_slope, best_intercept = slope, intercept

    print(f"  RANSAC 估计: y = {best_slope:.3f}x + {best_intercept:.3f}")
    print(f"  真值:       y = {true_slope}x + {true_intercept}")
    print(f"  内点数: {best_inliers}/{n_inliers} (真实内点)")
    print(f"  斜率误差: {abs(best_slope - true_slope):.4f}")

    assert abs(best_slope - true_slope) < 0.3
    assert best_inliers >= n_inliers * 0.8
    print("  ✅ 通过!")


def task_02_pnp():
    """任务2: PnP — 2D-3D对应求解相机位姿 (20分钟)

    Perspective-n-Point: 已知n个3D点和对应2D投影，求相机位姿。
    """
    print("\n═══ 任务2: PnP 求解 ═══")

    # 相机内参
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])

    # 真实相机位姿
    theta = np.radians(20)
    R_true = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    t_true = np.array([1.0, 0.5, 3.0])

    # 6个3D点
    points_3d = np.array([
        [0, 0, 0], [1, 0, 0], [0, 1, 0],
        [0, 0, 1], [1, 1, 0], [1, 0, 1]
    ])

    # 投影
    P_true = K @ np.hstack([R_true, t_true.reshape(3,1)])
    pixels = []
    for p in points_3d:
        xh = P_true @ np.append(p, 1)
        pixels.append([xh[0]/xh[2], xh[1]/xh[2]])
    pixels = np.array(pixels)

    # PnP 求解 (DLT + 非线性优化简化版)
    # 先用 DLT 估计初始位姿
    A = []
    for (X, Y, Z), (u, v) in zip(points_3d, pixels):
        A.append([X, Y, Z, 1, 0, 0, 0, 0, -u*X, -u*Y, -u*Z, -u])
        A.append([0, 0, 0, 0, X, Y, Z, 1, -v*X, -v*Y, -v*Z, -v])
    A = np.array(A)

    _, _, Vt = np.linalg.svd(A)
    P_vec = Vt[-1]
    P_est = P_vec.reshape(3, 4)

    # 从 P 恢复 R, t
    # P = K [R|t] → [R|t] = K^{-1} P
    Rt_est = np.linalg.inv(K) @ P_est

    # R 应满足 det=1, R^T R = I
    R_est_raw = Rt_est[:, :3]
    U, _, Vt = np.linalg.svd(R_est_raw)
    R_est = U @ Vt
    # 确保 det = 1
    if np.linalg.det(R_est) < 0:
        Vt[-1] *= -1
        R_est = U @ Vt

    # 恢复尺度
    lam = np.linalg.norm(P_est[:, :3]) / np.linalg.norm(K)
    t_est = Rt_est[:, 3] / lam

    print(f"  估计的 R:\n{R_est}")
    print(f"  真值 R:\n{R_true}")
    print(f"  旋转误差: {np.linalg.norm(R_est - R_true):.4f}")
    print(f"  估计的 t: {t_est} (真值: {t_true})")

    # 重投影误差
    P_final = K @ np.hstack([R_est, t_est.reshape(3,1)])
    reproj_errors = []
    for p_3d, p_2d in zip(points_3d, pixels):
        xh = P_final @ np.append(p_3d, 1)
        u_est, v_est = xh[0]/xh[2], xh[1]/xh[2]
        reproj_errors.append(np.sqrt((u_est-p_2d[0])**2 + (v_est-p_2d[1])**2))
    print(f"  平均重投影误差: {np.mean(reproj_errors):.2f}px")
    assert np.mean(reproj_errors) < 5
    print("  ✅ 通过!")


def task_03_icp():
    """任务3: ICP — 3D-3D点云配准 (20分钟)

    Iterative Closest Point: 对齐两个点云。
    """
    print("\n═══ 任务3: ICP 点云配准 ═══")

    # 创建源点云
    np.random.seed(0)
    src_points = np.random.randn(30, 3)

    # 应用真实变换到目标点云
    theta = np.radians(30)
    R_true = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])
    t_true = np.array([2.0, 1.0, 0.5])
    dst_points = (R_true @ src_points.T).T + t_true

    # 加噪声
    dst_points += np.random.normal(0, 0.02, dst_points.shape)

    # ICP 迭代
    R_est = np.eye(3)
    t_est = np.zeros(3)
    src_aligned = src_points.copy()

    for iteration in range(20):
        # 步骤1: 找最近邻 (假设 known correspondence, 简化版)
        # 实际ICP需要找最近邻，这里已知对应关系
        # 步骤2: 计算最优变换 (SVD)
        centroid_src = src_aligned.mean(axis=0)
        centroid_dst = dst_points.mean(axis=0)

        H = (src_aligned - centroid_src).T @ (dst_points - centroid_dst)
        U, _, Vt = np.linalg.svd(H)
        R_update = Vt.T @ U.T
        if np.linalg.det(R_update) < 0:
            Vt[-1] *= -1
            R_update = Vt.T @ U.T

        t_update = centroid_dst - R_update @ centroid_src

        # 应用变换
        src_aligned = (R_update @ src_aligned.T).T + t_update
        R_est = R_update @ R_est
        t_est = R_update @ t_est + t_update

        # 计算误差
        error = np.mean(np.linalg.norm(src_aligned - dst_points, axis=1))
        if error < 1e-6:
            break

    print(f"  ICP 迭代次数: {iteration+1}")
    print(f"  最终误差: {error:.6f}")
    print(f"  估计的 t: {t_est} (真值: {t_true})")
    print(f"  旋转误差: {np.linalg.norm(R_est - R_true):.4f}")

    assert np.linalg.norm(t_est - t_true) < 0.1
    assert np.linalg.norm(R_est - R_true) < 0.1
    print("  ✅ 通过!")


def task_04_ransac_fundamental():
    """任务4: RANSAC + 基础矩阵估计 (15分钟)

    结合 RANSAC 和八点法估计包含外点的匹配对的基础矩阵。
    """
    print("\n═══ 任务4: RANSAC + F 矩阵估计 ═══")

    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])

    # 真实相机运动
    R1 = np.eye(3); t1 = np.zeros(3)
    R2 = np.array([[0.9848, 0, 0.1736], [0, 1, 0], [-0.1736, 0, 0.9848]])
    t2 = np.array([0.5, 0, 0])
    t2_skew = np.array([[0, -t2[2], t2[1]], [t2[2], 0, -t2[0]], [-t2[1], t2[0], 0]])
    E_true = t2_skew @ R2
    F_true = np.linalg.inv(K).T @ E_true @ np.linalg.inv(K)

    # 生成3D点和匹配
    np.random.seed(1)
    n_total = 50
    n_inliers = 35
    pts1_list, pts2_list = [], []

    for i in range(n_total):
        p_3d = np.random.uniform(-1, 1, 3)
        p_3d[2] += 3  # 深度3-5m

        ph = np.append(p_3d, 1)
        x1h = K @ np.hstack([R1, t1.reshape(3,1)]) @ ph
        u1, v1 = x1h[0]/x1h[2], x1h[1]/x1h[2]

        x2h = K @ np.hstack([R2, t2.reshape(3,1)]) @ ph
        u2, v2 = x2h[0]/x2h[2], x2h[1]/x2h[2]

        if i >= n_inliers:
            # 外点: 随机像素
            u2 = np.random.uniform(0, 640)
            v2 = np.random.uniform(0, 480)

        pts1_list.append([u1, v1])
        pts2_list.append([u2, v2])

    pts1 = np.array(pts1_list); pts2 = np.array(pts2_list)

    # RANSAC + 八点法
    best_inliers_count = 0
    best_F = None
    threshold = 3.0  # 像素

    for _ in range(200):
        idx = np.random.choice(n_total, 8, replace=False)
        A = []
        for k in idx:
            u1, v1 = pts1[k]; u2, v2 = pts2[k]
            A.append([u1*u2, v1*u2, u2, u1*v2, v1*v2, v2, u1, v1, 1])
        A = np.array(A)

        if np.linalg.matrix_rank(A) < 8:
            continue

        _, _, Vt = np.linalg.svd(A)
        F_cand = Vt[-1].reshape(3, 3)
        Uf, Sf, Vtf = np.linalg.svd(F_cand)
        Sf[-1] = 0
        F_cand = Uf @ np.diag(Sf) @ Vtf

        # 计算内点
        inliers = 0
        for k in range(n_total):
            x1h = np.array([pts1[k,0], pts1[k,1], 1])
            x2h = np.array([pts2[k,0], pts2[k,1], 1])
            err = abs(x2h @ F_cand @ x1h)
            if err < threshold:
                inliers += 1

        if inliers > best_inliers_count:
            best_inliers_count = inliers
            best_F = F_cand

    print(f"  RANSAC 内点数: {best_inliers_count}/{n_inliers} (真实内点)")
    print(f"  估计的 F (归一化):")
    F_norm = best_F / np.linalg.norm(best_F)
    print(F_norm.round(3))
    print(f"  真值 F (归一化):")
    F_true_norm = F_true / np.linalg.norm(F_true)
    print(F_true_norm.round(3))

    assert best_inliers_count >= n_inliers * 0.7
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_ransac()
    task_02_pnp()
    task_03_icp()
    task_04_ransac_fundamental()
    print("\n🎉 所有多视图几何练习完成!")
