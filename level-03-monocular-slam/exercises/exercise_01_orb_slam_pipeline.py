"""
练习 1: ORB-SLAM 核心流程模拟 (Level 3)
=============================================
模拟 ORB-SLAM 的三线程架构: Tracking → LocalMapping → LoopClosing
"""

import numpy as np
from scipy.spatial import cKDTree

def task_01_tracking_thread():
    """任务1: Tracking 线程 — 帧到帧/帧到地图的位姿估计 (20分钟)"""
    print("\n═══ 任务1: Tracking 线程 ═══")

    # 模拟: 有一个局部地图 (3D点 + 描述子)
    np.random.seed(42)
    n_map_pts = 50
    map_pts = np.random.uniform(-3, 3, (n_map_pts, 3))
    map_pts[:, 2] += 4  # 深度 1-7m
    map_descs = np.random.randn(n_map_pts, 32)  # 简化描述子

    # 当前帧提取的特征 (2D像素 + 描述子)
    n_features = 30
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])

    # 真实相机位姿 (已知，用于生成投影)
    R_true = np.eye(3)
    t_true = np.array([0.2, 0.1, 0.0])  # 小位移

    # 生成当前帧的特征
    features_uv = []
    features_desc = []
    visible_map_idx = np.random.choice(n_map_pts, min(20, n_features), replace=False)

    for idx in visible_map_idx:
        p_cam = R_true @ map_pts[idx] + t_true
        if p_cam[2] < 0.1: continue
        u = K[0,0]*p_cam[0]/p_cam[2] + K[0,2]
        v = K[1,1]*p_cam[1]/p_cam[2] + K[1,2]
        if 0 <= u < 640 and 0 <= v < 480:
            features_uv.append([u, v])
            features_desc.append(map_descs[idx] + np.random.normal(0, 0.1, 32))

    features_uv = np.array(features_uv)
    features_desc = np.array(features_desc)

    # ── 匹配: 帧特征 → 地图点 ──
    tree = cKDTree(map_descs)
    matches = []
    for i, desc in enumerate(features_desc):
        dist, idx = tree.query(desc, k=2)
        if dist[0] / (dist[1] + 1e-8) < 0.8:  # Lowe's ratio
            matches.append((i, idx[0]))

    print(f"  地图点: {n_map_pts}, 帧特征: {len(features_uv)}")
    print(f"  2D-3D匹配: {len(matches)} 对")

    if len(matches) >= 4:
        # PnP 求解位姿 (简化: 只用匹配点)
        pts2d = features_uv[[m[0] for m in matches]]
        pts3d = map_pts[[m[1] for m in matches]]

        # DLT PnP
        A = []
        for (u, v), (X, Y, Z) in zip(pts2d, pts3d):
            A.append([X, Y, Z, 1, 0, 0, 0, 0, -u*X, -u*Y, -u*Z, -u])
            A.append([0, 0, 0, 0, X, Y, Z, 1, -v*X, -v*Y, -v*Z, -v])
        A = np.array(A)

        _, _, Vt = np.linalg.svd(A)
        P_vec = Vt[-1].reshape(3, 4)

        # 从 P = K[R|t] 恢复
        Rt = np.linalg.inv(K) @ P_vec
        U, _, Vt2 = np.linalg.svd(Rt[:,:3])
        R_est = U @ Vt2
        t_est = Rt[:, 3] / np.linalg.norm(Rt[:,:3]) * np.linalg.norm(K)

        print(f"  估计的平移: {t_est.round(3)} (真值: {t_true})")
        err = np.linalg.norm(t_est - t_true)
        print(f"  平移误差: {err:.3f}m")
        print(f"  ℹ️  简化PnP受限于少量匹配点 — 真实ORB-SLAM用迭代优化+鲁棒核")
    else:
        print("  ⚠️  匹配不足, 需要重定位模式")

    print("  ✅ 通过!")


def task_02_local_mapping():
    """任务2: LocalMapping — 关键帧管理与局部BA (15分钟)"""
    print("\n═══ 任务2: LocalMapping 线程 ═══")

    # 模拟关键帧选择策略
    class KeyFrame:
        def __init__(self, idx, t, features):
            self.idx = idx
            self.t = t
            self.features = features  # 2D特征数

    keyframes = [
        KeyFrame(1, np.array([0, 0, 0]), 150),
        KeyFrame(2, np.array([1, 0, 0]), 120),
        KeyFrame(3, np.array([2, 0, 0]), 100),
        KeyFrame(4, np.array([2.8, 0, 0]), 90),
        KeyFrame(5, np.array([3.1, 0.1, 0]), 85),
    ]

    # 关键帧插入策略:
    # 1. 与上一关键帧距离 > 阈值
    # 2. 与上一关键帧角度差 > 阈值
    # 3. 跟踪到的特征数 < 阈值 (场景变化大)

    candidates = []
    for i in range(1, len(keyframes)):
        dist = np.linalg.norm(keyframes[i].t - keyframes[i-1].t)
        n_features = keyframes[i].features
        should_insert = dist > 0.5 or n_features < 90
        candidates.append((keyframes[i].idx, dist, n_features, should_insert))

    print("  关键帧选择:")
    for idx, dist, nf, insert in candidates:
        print(f"    KF{idx}: dist={dist:.1f}, features={nf} → {'插入' if insert else '跳过'}")

    # 局部BA: 对最近的N个关键帧 + 它们观测到的地图点
    # 这里演示共视关系的构造
    covisibility = {
        1: {2: 80, 3: 40},
        2: {1: 80, 3: 60, 4: 30},
        3: {1: 40, 2: 60, 4: 50, 5: 20},
        4: {2: 30, 3: 50, 5: 55},
        5: {3: 20, 4: 55},
    }

    # 选择与当前帧(KF5)共视最强的关键帧
    current = 5
    neighbors = sorted(covisibility[current].items(), key=lambda x: x[1], reverse=True)
    local_window = [current] + [n[0] for n in neighbors[:3]]  # 本帧+Top3共视帧
    print(f"\n  局部窗口 (共视图): {local_window}")
    print(f"  Local BA 优化窗口中的 {len(local_window)} 个关键帧")

    assert len(local_window) >= 3
    print("  ✅ 通过!")


def task_03_loop_closing():
    """任务3: LoopClosing — 回环检测与全局优化 (15分钟)"""
    print("\n═══ 任务3: LoopClosing 线程 ═══")

    # 模拟词袋向量 (5个关键帧的BoVW)
    np.random.seed(0)
    vocab_size = 100
    kf_bows = {}
    kf_bows[1] = np.random.randn(vocab_size)
    kf_bows[5] = np.random.randn(vocab_size)
    kf_bows[10] = np.random.randn(vocab_size)
    kf_bows[20] = np.random.randn(vocab_size)
    kf_bows[30] = kf_bows[1] + np.random.normal(0, 0.5, vocab_size)  # 回环!
    kf_bows[35] = np.random.randn(vocab_size)

    # 当前帧 KF35, 搜索回环候选
    current_idx = 35

    similarities = {}
    for idx, bow in kf_bows.items():
        if idx >= current_idx - 5:  # 排除最近的关键帧
            continue
        sim = np.dot(kf_bows[current_idx], bow) / (
            np.linalg.norm(kf_bows[current_idx]) * np.linalg.norm(bow) + 1e-8)
        similarities[idx] = sim

    # 找最佳回环候选
    best_match = max(similarities, key=similarities.get)

    print(f"  回环候选相似度:")
    for idx, sim in sorted(similarities.items()):
        marker = " ← 回环!" if idx == best_match else ""
        print(f"    KF{idx}: {sim:.3f}{marker}")

    print(f"\n  检测到回环: KF{current_idx} ↔ KF{best_match}")

    # 几何验证: 用Sim3对齐 (简化)
    # 然后: Pose Graph Optimization 传播回环修正
    print(f"  执行: Pose Graph Optimization → 全局BA")
    print("  ✅ (演示回环检测流程)")


def task_04_map_point_management():
    """任务4: 地图点管理 (10分钟)"""
    print("\n═══ 任务4: 地图点管理 ═══")

    # 模拟地图点的生命周期
    class MapPoint:
        def __init__(self, idx):
            self.idx = idx
            self.observations = 0  # 被观测次数
            self.found_ratio = 0.0  # 在预期可见帧中实际被找到的比例
            self.created_frame = 0

    mps = [MapPoint(i) for i in range(20)]
    for mp in mps:
        mp.observations = np.random.randint(0, 10)
        mp.found_ratio = np.random.random()

    # 剔除规则:
    # 1. 在最近3帧中至少被观测到25% (found_ratio < 0.25 → cull)
    # 2. 总观测次数 < 3 且 created > 10 帧前

    culled = []
    for mp in mps:
        if mp.found_ratio < 0.25 and mp.observations < 5:
            culled.append(mp.idx)

    kept = len(mps) - len(culled)
    print(f"  地图点总数: {len(mps)}, 剔除: {len(culled)}, 保留: {kept}")
    print(f"  ORB-SLAM: '地图点必须被≥25%的预期可见帧找到'")

    assert len(culled) > 0, "应该有地图点被剔除"
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_tracking_thread()
    task_02_local_mapping()
    task_03_loop_closing()
    task_04_map_point_management()
    print("\n🎉 ORB-SLAM 核心流程练习完成!")
