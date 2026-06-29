"""
实验 2: 对极几何 — 从两张图像恢复3D结构
============================================
目标: 给定两张图像中的匹配点，恢复相机相对运动并三角化3D点。

运行: python3 experiment_02_epipolar_geometry.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def experiment():
    print("═" * 60)
    print("  实验: 双视图重建 — 对极几何实战")
    print("═" * 60)

    # ── 1. 设置 ──
    np.random.seed(42)
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)

    # 创建一些三维点（场景）
    n_points = 20
    true_points = np.random.uniform(-2, 2, (n_points, 3))
    true_points[:, 2] = np.abs(true_points[:, 2]) + 3  # 深度3-5m

    print(f"  场景: {n_points}个3D点, 深度范围 {true_points[:,2].min():.1f}-{true_points[:,2].max():.1f}m")

    # 相机1: 世界原点
    R1, t1 = np.eye(3), np.zeros(3)

    # 相机2: 在X轴方向平移0.5m, 绕Y轴微转10°
    theta = np.radians(10)
    R2 = np.array([
        [np.cos(theta),  0, np.sin(theta)],
        [0,              1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    t2 = np.array([0.5, 0.1, 0.05])

    print(f"  相机2位移: {t2}, 旋转: {np.degrees(theta):.1f}° 绕Y轴")

    # 投影矩阵
    P1 = K @ np.hstack([R1, t1.reshape(3,1)])
    P2 = K @ np.hstack([R2, t2.reshape(3,1)])

    # ── 2. 生成带噪声的匹配点 ──
    pts1, pts2 = [], []
    for p in true_points:
        ph = np.append(p, 1)
        x1 = P1 @ ph; u1, v1 = x1[0]/x1[2], x1[1]/x1[2]
        x2 = P2 @ ph; u2, v2 = x2[0]/x2[2], x2[1]/x2[2]
        # 加噪声 (1像素标准差)
        u1 += np.random.normal(0, 1.0); v1 += np.random.normal(0, 1.0)
        u2 += np.random.normal(0, 1.0); v2 += np.random.normal(0, 1.0)
        pts1.append([u1, v1]); pts2.append([u2, v2])
    pts1 = np.array(pts1); pts2 = np.array(pts2)

    # ── 3. 用八点法估计基础矩阵 F ──
    # 先归一化提高数值稳定性
    def normalize_points(pts):
        centroid = pts.mean(axis=0)
        shifted = pts - centroid
        mean_dist = np.mean(np.linalg.norm(shifted, axis=1))
        scale = np.sqrt(2) / mean_dist
        T = np.array([[scale, 0, -scale*centroid[0]],
                       [0, scale, -scale*centroid[1]],
                       [0, 0, 1]])
        normed = (T @ np.column_stack([pts, np.ones(len(pts))]).T).T
        return normed[:, :2], T

    pts1_n, T1 = normalize_points(pts1)
    pts2_n, T2 = normalize_points(pts2)

    A = np.zeros((n_points, 9))
    for i in range(n_points):
        u1, v1 = pts1_n[i]; u2, v2 = pts2_n[i]
        A[i] = [u1*u2, v1*u2, u2, u1*v2, v1*v2, v2, u1, v1, 1]

    _, _, Vt = np.linalg.svd(A)
    F_norm = Vt[-1].reshape(3, 3)

    # 强制秩为2
    Uf, Sf, Vtf = np.linalg.svd(F_norm)
    Sf[-1] = 0
    F_norm = Uf @ np.diag(Sf) @ Vtf

    # 反归一化
    F = T2.T @ F_norm @ T1

    print(f"\n  八点法估计的 F:\n{F}")

    # ── 4. 从 F 恢复 E, 再恢复 R,t ──
    E = K.T @ F @ K
    print(f"\n  本质矩阵 E:\n{E}")

    # 从 E 分解 R,t
    Ue, _, Vte = np.linalg.svd(E)
    # 确保 det(U)=det(V)=1 (旋转矩阵)
    if np.linalg.det(Ue) < 0: Ue[:, -1] *= -1
    if np.linalg.det(Vte) < 0: Vte[-1, :] *= -1

    W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    R_candidates = [Ue @ W @ Vte, Ue @ W.T @ Vte]
    t_candidates = [Ue[:, 2], -Ue[:, 2]]

    # 选择正确的 (R,t): 三角化后所有点都在相机前方 (正深度)
    best_R, best_t, max_front = None, None, 0
    for R_cand, t_cand in zip(R_candidates, t_candidates):
        P2_cand = K @ np.hstack([R_cand, t_cand.reshape(3,1)])
        # 三角化一个点测试
        front_count = 0
        for i in range(min(10, n_points)):
            u1, v1 = pts1[i]; u2, v2 = pts2[i]
            A_tri = np.array([
                u1*P1[2] - P1[0], v1*P1[2] - P1[1],
                u2*P2_cand[2] - P2_cand[0], v2*P2_cand[2] - P2_cand[1]
            ])
            _, _, Vt_tri = np.linalg.svd(A_tri)
            X = Vt_tri[-1]; X = X / X[3]
            if X[2] > 0: front_count += 1
        if front_count > max_front:
            max_front = front_count
            best_R = R_cand; best_t = t_cand

    print(f"\n  恢复的 R:\n{best_R}")
    print(f"  真值 R:\n{R2}")
    print(f"  恢复的 t: {best_t / np.linalg.norm(best_t)} (归一化)")
    print(f"  真值 t:   {t2 / np.linalg.norm(t2)} (归一化)")

    # ── 5. 三角化所有点 ──
    P2_est = K @ np.hstack([best_R, best_t.reshape(3,1)])
    triangulated = []
    for i in range(n_points):
        u1, v1 = pts1[i]; u2, v2 = pts2[i]
        A_tri = np.array([
            u1*P1[2] - P1[0], v1*P1[2] - P1[1],
            u2*P2_est[2] - P2_est[0], v2*P2_est[2] - P2_est[1]
        ])
        _, _, Vt_tri = np.linalg.svd(A_tri)
        X = Vt_tri[-1]; X = X / X[3]
        triangulated.append(X[:3])
    triangulated = np.array(triangulated)

    # 对齐恢复的点云到真值 (相似变换)
    # 简化: 直接比较相对结构
    errors = np.linalg.norm(triangulated - true_points, axis=1)
    print(f"\n  三角化误差: 中位数 {np.median(errors):.3f}m, 均值 {np.mean(errors):.3f}m")

    # ── 6. 可视化 ──
    fig = plt.figure(figsize=(12, 5))

    # 左: 两张图像的匹配点 (显示像素坐标)
    ax1 = fig.add_subplot(131)
    ax1.scatter(pts1[:,0], pts1[:,1], c='blue', s=20, label='图像1')
    ax1.scatter(pts2[:,0], pts2[:,1], c='red', s=20, marker='x', label='图像2')
    for i in range(min(n_points, 10)):
        ax1.plot([pts1[i,0], pts2[i,0]], [pts1[i,1], pts2[i,1]],
                'gray', alpha=0.3, linewidth=0.5)
    ax1.set_title('匹配点 (像素坐标)')
    ax1.set_xlabel('u (像素)'); ax1.set_ylabel('v (像素)')
    ax1.legend()
    ax1.invert_yaxis()

    # 中: 三角化点 vs 真值
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(true_points[:,2], true_points[:,1], true_points[:,0],
               c='green', s=30, marker='o', label='真值')
    ax2.scatter(triangulated[:,2], triangulated[:,1], triangulated[:,0],
               c='orange', s=30, marker='^', label='三角化')
    for i in range(n_points):
        ax2.plot([true_points[i,2], triangulated[i,2]],
                [true_points[i,1], triangulated[i,1]],
                [true_points[i,0], triangulated[i,0]],
                'gray', alpha=0.2, linewidth=0.5)
    ax2.set_title('3D 重建 vs 真值')
    ax2.set_xlabel('Z'); ax2.set_ylabel('Y'); ax2.set_zlabel('X')
    ax2.legend()

    # 右: 误差分布
    ax3 = fig.add_subplot(133)
    ax3.hist(errors, bins=10, edgecolor='black', alpha=0.7)
    ax3.axvline(np.median(errors), color='red', linestyle='--', label=f'中位数={np.median(errors):.3f}m')
    ax3.set_xlabel('三角化误差 (m)')
    ax3.set_ylabel('点数')
    ax3.set_title('误差分布')
    ax3.legend()

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-01-beginner/experiments/epipolar_geometry.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  📊 保存到 epipolar_geometry.png")
    print(f"  ✅ 实验完成!")


if __name__ == "__main__":
    experiment()
