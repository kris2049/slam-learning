"""
实验 1: 完整 Bundle Adjustment (Level 2)
=============================================
从噪声初始化同时优化 5 个相机位姿和 20 个 3D 点。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def experiment():
    print("═" * 60)
    print("  实验: Bundle Adjustment — 联合优化位姿和地图点")
    print("═" * 60)

    np.random.seed(42)

    # ── 相机 ──
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])

    # ── 真实场景 ──
    n_pts = 20
    pts_true = np.random.uniform(-2, 2, (n_pts, 3))
    pts_true[:, 2] = np.abs(pts_true[:, 2]) + 3

    n_cams = 5
    poses_true = []
    for i in range(n_cams):
        angle = np.radians(i * 20)
        R = np.array([[np.cos(angle), 0, np.sin(angle)],
                      [0, 1, 0],
                      [-np.sin(angle), 0, np.cos(angle)]])
        t = np.array([i * 0.5, 0.2 * i, 0])
        poses_true.append((R, t))

    # ── 生成观测 ──
    observations = []
    for cam in range(n_cams):
        R, t = poses_true[cam]
        for pt in range(n_pts):
            p_cam = R @ pts_true[pt] + t
            if p_cam[2] <= 0.1:
                continue
            u = K[0,0] * p_cam[0] / p_cam[2] + K[0,2]
            v = K[1,1] * p_cam[1] / p_cam[2] + K[1,2]
            if 0 <= u < 640 and 0 <= v < 480:
                u += np.random.normal(0, 1.0)
                v += np.random.normal(0, 1.0)
                observations.append((cam, pt, u, v))

    print(f"  观测数: {len(observations)}")
    print(f"  3D点数: {n_pts}, 相机数: {n_cams}")

    # ── 初始化 (大噪声) ──
    pts_est = pts_true + np.random.normal(0, 0.3, pts_true.shape)
    poses_est = []
    for i, (R_true, t_true) in enumerate(poses_true):
        angle_noise = np.random.normal(0, 0.1, 3)
        R_noise = R_true
        for _ in range(3):
            R_noise = R_noise @ np.array([
                [1, -angle_noise[2], angle_noise[1]],
                [angle_noise[2], 1, -angle_noise[0]],
                [-angle_noise[1], angle_noise[0], 1]
            ])
        t_noise = t_true + np.random.normal(0, 0.3, 3)
        poses_est.append((R_noise, t_noise))

    # ── 记录初始误差 ──
    def compute_errors():
        pt_err = np.median(np.linalg.norm(pts_est - pts_true, axis=1))
        pose_err = np.mean([np.linalg.norm(t_est - t_true)
                           for (_, t_est), (_, t_true) in zip(poses_est, poses_true)])
        return pt_err, pose_err

    pt_err_0, pose_err_0 = compute_errors()
    print(f"\n  BA前: 点误差={pt_err_0:.3f}m, 位姿误差={pose_err_0:.3f}m")

    # ── BA 优化 ──
    n_iters = 8
    errors_history = []

    for iteration in range(n_iters):
        n_params = 3 * n_pts + 6 * n_cams
        H = np.zeros((n_params, n_params))
        b = np.zeros(n_params)

        for cam, pt, u_obs, v_obs in observations:
            R, t = poses_est[cam]
            p = pts_est[pt]
            p_cam = R @ p + t

            if abs(p_cam[2]) < 0.01:
                continue

            u_proj = K[0,0] * p_cam[0] / p_cam[2] + K[0,2]
            v_proj = K[1,1] * p_cam[1] / p_cam[2] + K[1,2]
            r = np.array([u_proj - u_obs, v_proj - v_obs])

            # Jacobian: d(proj)/d(p_cam)
            dp_duv = np.array([
                [K[0,0]/p_cam[2], 0, -K[0,0]*p_cam[0]/p_cam[2]**2],
                [0, K[1,1]/p_cam[2], -K[1,1]*p_cam[1]/p_cam[2]**2]
            ])

            # d(p_cam)/d(p_world) = R
            J_pt = dp_duv @ R

            # d(p_cam)/d(xi) 简化: 只平移部分
            J_cam_trans = dp_duv
            J_cam_rot = np.zeros((2, 3))  # 简化

            pt_off = 3 * pt
            cam_off = 3 * n_pts + 6 * cam

            # 3D point contribution
            H_pt = J_pt.T @ J_pt
            H[pt_off:pt_off+3, pt_off:pt_off+3] += H_pt
            b[pt_off:pt_off+3] -= J_pt.T @ r

            # Camera translation contribution
            for k in range(3):
                for l in range(3):
                    H[cam_off+3+k, cam_off+3+l] += (J_cam_trans.T @ J_cam_trans)[k, l]
            b[cam_off+3:cam_off+6] -= J_cam_trans.T @ r

        # 固定第一个相机
        H[:6, :] = 0; H[:, :6] = 0
        H[:6, :6] = np.eye(6)

        lam = max(0.01, 1.0 / (iteration + 1))
        try:
            dx = np.linalg.solve(H + lam * np.eye(n_params), b)

            for i in range(n_pts):
                pts_est[i] += dx[3*i:3*i+3]

            for i in range(1, n_cams):
                dxi = dx[3*n_pts+6*i:3*n_pts+6*i+6]
                R, t = poses_est[i]
                poses_est[i] = (R, t + dxi[:3])

        except np.linalg.LinAlgError:
            print(f"    迭代{iteration}: 奇异矩阵, 跳过")

        pt_err, pose_err = compute_errors()
        errors_history.append((pt_err, pose_err))
        if iteration < 4 or iteration == n_iters-1:
            print(f"  迭代{iteration}: 点误差={pt_err:.3f}m, 位姿误差={pose_err:.3f}m")

    # ── 结果 ──
    pt_err_final, pose_err_final = compute_errors()
    print(f"\n  BA后: 点误差={pt_err_final:.3f}m (改善 {pt_err_0/pt_err_final:.1f}x)")
    print(f"        位姿误差={pose_err_final:.3f}m (改善 {pose_err_0/pose_err_final:.1f}x)")

    # ── 可视化 ──
    fig = plt.figure(figsize=(14, 5))

    # 3D 视图
    ax1 = fig.add_subplot(131, projection='3d')
    for pt_idx in range(n_pts):
        ax1.plot([pts_true[pt_idx,0], pts_est[pt_idx,0]],
                 [pts_true[pt_idx,1], pts_est[pt_idx,1]],
                 [pts_true[pt_idx,2], pts_est[pt_idx,2]],
                 'gray', alpha=0.3)
    ax1.scatter(pts_true[:,0], pts_true[:,1], pts_true[:,2], c='green', s=10, label='True')
    ax1.scatter(pts_est[:,0], pts_est[:,1], pts_est[:,2], c='red', s=10, label='BA est')

    # 画相机
    for (Rt, tt), (Re, te) in zip(poses_true, poses_est):
        ax1.scatter(*tt, c='green', marker='^', s=40)
        ax1.scatter(*te, c='red', marker='^', s=40)

    ax1.set_title('3D Reconstruction')
    ax1.legend()

    # 误差收敛
    ax2 = fig.add_subplot(132)
    iters = range(len(errors_history))
    ax2.plot(iters, [e[0] for e in errors_history], 'b-o', label='Point Error')
    ax2.plot(iters, [e[1] for e in errors_history], 'r-s', label='Pose Error')
    ax2.set_xlabel('Iteration'); ax2.set_ylabel('Error (m)')
    ax2.set_title('BA Convergence')
    ax2.legend(); ax2.grid(True, alpha=0.3)

    # 重投影误差分布
    ax3 = fig.add_subplot(133)
    reproj_errors = []
    for cam, pt, u_obs, v_obs in observations:
        R, t = poses_est[cam]; p = pts_est[pt]
        p_cam = R @ p + t
        u_proj = K[0,0]*p_cam[0]/p_cam[2] + K[0,2]
        v_proj = K[1,1]*p_cam[1]/p_cam[2] + K[1,2]
        reproj_errors.append(np.sqrt((u_proj-u_obs)**2 + (v_proj-v_obs)**2))
    ax3.hist(reproj_errors, bins=20, edgecolor='black', alpha=0.7)
    ax3.axvline(np.median(reproj_errors), color='red', linestyle='--',
                label=f'median={np.median(reproj_errors):.1f}px')
    ax3.set_xlabel('Reprojection Error (px)')
    ax3.set_title('Final Reprojection Errors')
    ax3.legend()

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-02-getting-familiar/experiments/ba_result.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  📊 保存到 ba_result.png")
    print(f"  ✅ 实验完成!")


if __name__ == "__main__":
    experiment()
