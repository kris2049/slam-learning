"""
实验 3: 稀疏性与 Schur 补 (Level 2)
=========================================
展示 BA 的 H 矩阵稀疏结构和 Schur 消元加速。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def experiment():
    print("═" * 60)
    print("  实验: Schur 补 — BA 的稀疏结构")
    print("═" * 60)

    n_cams, n_pts = 3, 10

    # 构造简化的 H 矩阵
    # H = [[B  E],
    #      [E^T C]]
    # B: 相机-相机 (对角块), C: 点-点 (对角块), E: 相机-点

    dim_cam = 6; dim_pt = 3
    H_size = n_cams * dim_cam + n_pts * dim_pt
    H = np.zeros((H_size, H_size))

    # 填充: 每个点被每个相机观测到
    for cam in range(n_cams):
        cam_start = cam * dim_cam
        for pt in range(n_pts):
            pt_start = n_cams * dim_cam + pt * dim_pt

            # B: 相机-相机块 (6x6)
            H[cam_start:cam_start+dim_cam, cam_start:cam_start+dim_cam] += np.eye(dim_cam)

            # C: 点-点块 (3x3)
            H[pt_start:pt_start+dim_pt, pt_start:pt_start+dim_pt] += np.eye(dim_pt)

            # E: 相机-点交叉块
            E = np.random.randn(dim_cam, dim_pt) * 0.1
            H[cam_start:cam_start+dim_cam, pt_start:pt_start+dim_pt] += E
            H[pt_start:pt_start+dim_pt, cam_start:cam_start+dim_cam] += E.T

    # ── 展示稀疏性 ──
    print(f"  H矩阵大小: {H_size}x{H_size}")
    print(f"  非零元素: {np.count_nonzero(H)}/{H_size*H_size} "
          f"({np.count_nonzero(H)/(H_size*H_size)*100:.1f}%)")

    # ── Schur 补 ──
    cam_dim_total = n_cams * dim_cam
    pt_dim_total = n_pts * dim_pt

    B = H[:cam_dim_total, :cam_dim_total]
    E = H[:cam_dim_total, cam_dim_total:]
    C = H[cam_dim_total:, cam_dim_total:]

    # C的逆(对角块易求逆)
    C_inv = np.zeros_like(C)
    for pt in range(n_pts):
        s = pt * dim_pt; e = s + dim_pt
        C_inv[s:e, s:e] = np.linalg.inv(C[s:e, s:e] + 0.01*np.eye(dim_pt))

    # Schur 补: S = B - E C^{-1} E^T
    S = B - E @ C_inv @ E.T
    print(f"\n  Schur 补 S 大小: {S.shape} (比原始H小 {H_size - S.shape[0]} 维)")
    print(f"  S 非零元素: {np.count_nonzero(S > 0.01)}/{S.shape[0]*S.shape[1]}")

    # ── 可视化 ──
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    axes[0].spy(H, markersize=3, aspect='auto')
    axes[0].set_title(f'Full H ({H_size}x{H_size})\nSparsity: {np.count_nonzero(H)/(H_size*H_size)*100:.1f}%')

    axes[1].spy(S, markersize=5, aspect='auto')
    axes[1].set_title(f'Schur Complement S ({S.shape[0]}x{S.shape[1]})\n(Reduced system)')

    # 结构示意
    axes[2].text(0.1, 0.9, 'H = [[B  E]', fontsize=11, fontfamily='monospace')
    axes[2].text(0.1, 0.8, '     [E^T C]]', fontsize=11, fontfamily='monospace')
    axes[2].text(0.1, 0.6, 'S = B - E C^{-1} E^T', fontsize=11, fontfamily='monospace')
    axes[2].text(0.1, 0.5, '(Schur complement)', fontsize=9)
    axes[2].text(0.1, 0.3, f'Size: {H_size}x{H_size} → {S.shape[0]}x{S.shape[1]}', fontsize=10)
    axes[2].text(0.1, 0.15, 'Marginalization in', fontsize=9, fontfamily='monospace')
    axes[2].text(0.1, 0.05, 'sliding-window BA', fontsize=9, fontfamily='monospace')
    axes[2].axis('off')

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-02-getting-familiar/experiments/schur_complement.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  📊 保存到 schur_complement.png")
    print(f"  ✅ 实验完成!")

    print(f"\n  🔍 关键理解:")
    print(f"  1. BA 的 H 矩阵高度稀疏 (本例 {np.count_nonzero(H)/(H_size*H_size)*100:.1f}% 非零)")
    print(f"  2. Schur 补将大系统缩为更小的相机系统")
    print(f"  3. 这是滑动窗口 BA 和边缘化的数学基础")
    print(f"  4. ORB-SLAM 用这一步大幅加速优化")


if __name__ == "__main__":
    experiment()
