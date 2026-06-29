"""
实验: 共视图 (Covisibility Graph) 构建 (Level 3)
=====================================================
ORB-SLAM 用共视图来管理关键帧关系，是局部 BA 的基础。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def experiment():
    print("═" * 60)
    print("  实验: 共视图构建")
    print("═" * 60)

    n_kfs = 8
    np.random.seed(42)

    # 每个关键帧观测到一些地图点
    n_mps = 30
    kf_observations = {}
    for kf in range(n_kfs):
        n_obs = np.random.randint(8, 20)
        obs = np.random.choice(n_mps, n_obs, replace=False)
        kf_observations[kf] = set(obs)

    # 构建共视图: 如果两个KF共享≥15个地图点，就有边
    covis_graph = {}
    edges = []
    for i in range(n_kfs):
        covis_graph[i] = {}
        for j in range(i+1, n_kfs):
            shared = len(kf_observations[i] & kf_observations[j])
            if shared >= 3:  # ORB-SLAM 阈值
                covis_graph[i][j] = shared
                covis_graph[j][i] = shared
                edges.append((i, j, shared))

    print(f"  关键帧数: {n_kfs}")
    print(f"  共视图边数: {len(edges)}")

    # 找出与 KF0 共视最强的帧
    if 0 in covis_graph:
        neighbors = sorted(covis_graph[0].items(), key=lambda x: x[1], reverse=True)
        print(f"  KF0 的共视邻居: {[(k,v) for k,v in neighbors[:4]]}")

    # ── 可视化 ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # 共视图网络
    import networkx as nx
    G = nx.Graph()
    for i in range(n_kfs):
        G.add_node(i, size=len(kf_observations[i]))
    for i, j, w in edges:
        G.add_edge(i, j, weight=w)

    pos = nx.spring_layout(G, seed=42)
    node_sizes = [G.nodes[n]['size']*30 for n in G.nodes]
    edge_widths = [G[u][v]['weight']/5 for u, v in G.edges]

    nx.draw_networkx_nodes(G, pos, ax=ax1, node_size=node_sizes,
                           node_color='lightblue', edgecolors='navy')
    nx.draw_networkx_edges(G, pos, ax=ax1, width=edge_widths,
                          alpha=0.6, edge_color='gray')
    nx.draw_networkx_labels(G, pos, ax=ax1, font_size=10)
    ax1.set_title('Covisibility Graph\n(node size = #observed points, edge width = #shared)')
    ax1.axis('off')

    # 观测矩阵
    obs_matrix = np.zeros((n_kfs, n_mps))
    for kf, mps in kf_observations.items():
        for mp in mps:
            obs_matrix[kf, mp] = 1

    im = ax2.imshow(obs_matrix, cmap='Blues', aspect='auto')
    ax2.set_xlabel('Map Point ID'); ax2.set_ylabel('KeyFrame ID')
    ax2.set_title('Observation Matrix\n(white = observed)')
    plt.colorbar(im, ax=ax2)

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-03-monocular-slam/experiments/covisibility.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  📊 保存到 covisibility.png")
    print(f"  ✅ 实验完成!")

    print(f"\n  🔍 关键理解:")
    print(f"  1. 共视图是 ORB-SLAM 的核心数据结构")
    print(f"  2. Local BA 只优化当前帧+共视最强的邻居 ≈ 20-30帧")
    print(f"  3. 稀疏性来自: 每个KF只观测地图点的一小部分")
    print(f"  4. 共视图让Schur补高效: 不共享地图点的KF在H矩阵中无交叉项")


if __name__ == "__main__":
    experiment()
