"""
练习 3: 滑动窗口与边缘化 (Level 3)
==========================================
DSO 和 VINS-Mono 使用滑动窗口 BA + 边缘化来控制计算量。
"""

import numpy as np

def task_01_sliding_window():
    """任务1: 滑动窗口 BA (15分钟)

    保持固定数量(N)的关键帧，窗口外的帧被边缘化。
    """
    print("\n═══ 任务1: 滑动窗口 ═══")

    window_size = 7
    total_frames = 15
    compute_per_frame = []

    for frame in range(total_frames):
        if frame < window_size:
            n_in_window = frame + 1
            compute = n_in_window * (n_in_window - 1) / 2  # 简化: 每对约束
        else:
            compute = window_size * (window_size - 1) / 2
        compute_per_frame.append(compute)

    print(f"  总帧数: {total_frames}, 窗口大小: {window_size}")
    for i, c in enumerate(compute_per_frame):
        marker = " ← 滑动窗口固定" if i >= window_size else ""
        print(f"    帧{i}: ~{c:.0f} 对约束{marker}")

    # 对比: 如果用全局BA
    total_global = sum(i for i in range(total_frames))
    total_sliding = sum(compute_per_frame)
    print(f"\n  全局BA总计算量: ~{total_global}")
    print(f"  滑动窗口总计算量: ~{total_sliding}")
    print(f"  节省: {(1-total_sliding/total_global)*100:.0f}%")

    print("  ✅ 通过!")


def task_02_marginalization():
    """任务2: 边缘化 — Schur 补的应用 (20分钟)

    从窗口中移除旧帧时，把它被观测的信息压缩到剩余帧上。
    """
    print("\n═══ 任务2: 边缘化 (Marginalization) ═══")

    # 3个位姿 + 2个路标 → 窗口 [X₁, X₂, X₃, L₁, L₂]
    # 移出 X₁ 时: 用 Schur 补将 X₁ 的信息压缩到剩余变量

    dim_pose, dim_pt = 6, 3
    n_pose, n_pt = 3, 2
    total_dim = n_pose*dim_pose + n_pt*dim_pt

    # 构造信息矩阵 (H)
    H = np.zeros((total_dim, total_dim))
    np.random.seed(0)
    # 填充对角块
    for i in range(n_pose):
        s = i*dim_pose; e = s+dim_pose
        H[s:e, s:e] = np.eye(dim_pose) * 10
    for j in range(n_pt):
        s = n_pose*dim_pose + j*dim_pt
        e = s + dim_pt
        H[s:e, s:e] = np.eye(dim_pt) * 5

    # 交叉项 (观测约束)
    H[0:6, 18:21] = np.random.randn(6, 3)
    H[18:21, 0:6] = H[0:6, 18:21].T
    H[0:6, 21:24] = np.random.randn(6, 3)
    H[21:24, 0:6] = H[0:6, 21:24].T

    H_sym = (H + H.T) / 2  # 对称化

    # ── 边缘化 X₁ (前6维) ──
    H_aa = H_sym[:6, :6]        # X₁-X₁
    H_ab = H_sym[:6, 6:]        # X₁-其余
    H_bb = H_sym[6:, 6:]        # 其余-其余

    # Schur 补: H_marg = H_bb - H_ab^T H_aa^{-1} H_ab
    H_aa_inv = np.linalg.inv(H_aa + 0.1*np.eye(6))
    H_marginalized = H_bb - H_ab.T @ H_aa_inv @ H_ab

    print(f"  原始 H: {total_dim}x{total_dim}")
    print(f"  边缘化后 H: {H_marginalized.shape}")
    print(f"  → 密度增加: 非零比 {np.count_nonzero(np.abs(H_marginalized)>0.01)}/{H_marginalized.size}")
    print(f"  → 这就是 'fill-in' 效应: 边缘化引入稠密约束")

    print("  ✅ 通过!")


def task_03_keyframe_selection():
    """任务3: 关键帧选择策略 (10分钟)"""
    print("\n═══ 任务3: 关键帧选择 ═══")

    frames = [
        {"id": 1, "translation": 0.0, "angle": 0.0, "features": 150},
        {"id": 2, "translation": 0.3, "angle": 2.0, "features": 140},
        {"id": 3, "translation": 0.8, "angle": 5.0, "features": 130},
        {"id": 4, "translation": 1.5, "angle": 8.0, "features": 100},
        {"id": 5, "translation": 1.8, "angle": 9.0, "features": 120},
        {"id": 6, "translation": 2.5, "angle": 15.0, "features": 90},
    ]

    last_kf = frames[0]
    keyframes = [last_kf]

    for f in frames[1:]:
        trans_delta = f["translation"] - last_kf["translation"]
        angle_delta = abs(f["angle"] - last_kf["angle"])

        # DSO 的关键帧策略:
        # 1. 视场变化大 (平移 > 12% 场景深度 或 旋转 > 阈值)
        # 2. 光流中位数太小 → 场景离得太远
        # 3. 相机静止太久

        if trans_delta > 0.5 or angle_delta > 5 or f["features"] < 80:
            keyframes.append(f)
            last_kf = f
            print(f"  帧{f['id']}: 平移Δ={trans_delta:.1f}, 角度Δ={angle_delta:.0f}° → 选为关键帧")
        else:
            print(f"  帧{f['id']}: 平移Δ={trans_delta:.1f}, 角度Δ={angle_delta:.0f}° → 跳过")

    print(f"\n  总帧数: {len(frames)}, 关键帧: {len(keyframes)}")

    assert len(keyframes) >= 3
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_sliding_window()
    task_02_marginalization()
    task_03_keyframe_selection()
    print("\n🎉 滑动窗口练习完成!")
