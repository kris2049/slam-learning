"""
实验 2: 位姿图优化 — 回环检测 (Level 2)
============================================
模拟一个带漂移的轨迹 + 回环边，用位姿图优化纠正。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def experiment():
    print("═" * 60)
    print("  实验: 位姿图优化 — 回环闭合")
    print("═" * 60)

    np.random.seed(42)

    # ── 真实轨迹 (圆形) ──
    n_poses = 30
    true_poses = []
    for i in range(n_poses):
        angle = 2 * np.pi * i / n_poses
        R = np.array([[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle),  np.cos(angle), 0],
                      [0, 0, 1]])
        radius = 5.0
        t = np.array([radius * np.cos(angle), radius * np.sin(angle), 0])
        true_poses.append((R, t))

    # ── 模拟噪声里程计 (漂移) ──
    drift_per_step = 0.3
    noisy_poses = [(np.eye(3), np.array([5.0, 0.0, 0.0]))]  # 起点

    for i in range(1, n_poses):
        # 真实相对运动 + 漂移
        R_prev, t_prev = true_poses[i-1]
        R_curr, t_curr = true_poses[i]

        dR_true = R_prev.T @ R_curr
        dt_true = R_prev.T @ (t_curr - t_prev)

        # 加噪声
        angle_noise = np.random.normal(0, 0.02, 3)
        K_noise = np.array([[0, -angle_noise[2], angle_noise[1]],
                            [angle_noise[2], 0, -angle_noise[0]],
                            [-angle_noise[1], angle_noise[0], 0]])
        dR_noisy = dR_true @ (np.eye(3) + K_noise)
        dt_noisy = dt_true + np.random.normal(0, drift_per_step, 3)

        R_noisy = noisy_poses[i-1][0] @ dR_noisy
        t_noisy = noisy_poses[i-1][0] @ dt_noisy + noisy_poses[i-1][1]
        noisy_poses.append((R_noisy, t_noisy))

    print(f"  轨迹长度: {n_poses} 个位姿")
    print(f"  终点真值: ({true_poses[-1][1][0]:.1f}, {true_poses[-1][1][1]:.1f})")
    print(f"  终点漂移: ({noisy_poses[-1][1][0]:.1f}, {noisy_poses[-1][1][1]:.1f})")

    # ── 位姿图优化 (加回环约束) ──
    # 简化: 只优化平移
    xis = np.array([t for _, t in noisy_poses])

    for iteration in range(10):
        H = np.zeros((n_poses, n_poses))
        b = np.zeros(n_poses)

        # 里程计边
        for i in range(n_poses - 1):
            dt_true = true_poses[i+1][1] - true_poses[i][1]
            dt_curr = xis[i+1] - xis[i]
            r = dt_curr - dt_true  # 3D残差

            for dim in range(2):  # 只优化x,y
                H[i, i] += 1; H[i+1, i+1] += 1
                H[i, i+1] -= 1; H[i+1, i] -= 1
                b[i] -= r[dim]; b[i+1] += r[dim]

        # 回环边: 最后一个位姿应回到起点
        dt_true = np.array([0, 0, 0])
        dt_curr = xis[-1] - xis[0]
        r = dt_curr - dt_true
        weight_loop = 10.0  # 回环权重较高

        for dim in range(2):
            H[0, 0] += weight_loop; H[-1, -1] += weight_loop
            H[0, -1] -= weight_loop; H[-1, 0] -= weight_loop
            b[0] -= weight_loop * r[dim]; b[-1] += weight_loop * r[dim]

        # 固定第一个位姿
        H[0, 0] += 100
        # H[:] 不动, 只加正则化

        dx = np.linalg.solve(H + 0.1 * np.eye(n_poses), b)
        xis[:, 0] -= dx
        xis[:, 1] -= dx

        if np.linalg.norm(dx) < 1e-4:
            break

    optimized_poses = [(noisy_poses[i][0], xis[i]) for i in range(n_poses)]

    # ── 结果 ──
    true_t = np.array([t for _, t in true_poses])
    noisy_t = np.array([t for _, t in noisy_poses])
    opt_t = np.array([t for _, t in optimized_poses])

    drift_before = np.linalg.norm(noisy_t[-1] - true_t[-1])
    drift_after = np.linalg.norm(opt_t[-1] - true_t[-1])
    print(f"  漂移改善: {drift_before:.2f}m → {drift_after:.2f}m")

    # ── 可视化 ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # 轨迹对比
    ax1.plot(true_t[:, 0], true_t[:, 1], 'g-', linewidth=2, label='True')
    ax1.plot(noisy_t[:, 0], noisy_t[:, 1], 'r--', linewidth=1.5, alpha=0.7, label='Noisy Odom')
    ax1.plot(opt_t[:, 0], opt_t[:, 1], 'b-', linewidth=2, label='Optimized')
    ax1.scatter(true_t[0, 0], true_t[0, 1], c='green', s=80, marker='*', label='Start')
    ax1.scatter(true_t[-1, 0], true_t[-1, 1], c='green', s=80, marker='X', label='End (True)')
    ax1.scatter(noisy_t[-1, 0], noisy_t[-1, 1], c='red', s=80, marker='X', label='End (Noisy)')
    ax1.set_aspect('equal')
    ax1.set_title('Trajectory with Loop Closure')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 漂移对比 (每个点的误差)
    errors_before = np.linalg.norm(noisy_t - true_t, axis=1)
    errors_after = np.linalg.norm(opt_t - true_t, axis=1)
    ax2.plot(errors_before, 'r-', label='Before PGO')
    ax2.plot(errors_after, 'b-', label='After PGO')
    ax2.set_xlabel('Pose Index'); ax2.set_ylabel('Position Error (m)')
    ax2.set_title('Error Reduction')
    ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-02-getting-familiar/experiments/pose_graph.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  📊 保存到 pose_graph.png")
    print(f"  ✅ 实验完成!")


if __name__ == "__main__":
    experiment()
