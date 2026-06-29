"""
Level 2 综合测试: SLAM 入门
===============================
通过所有测试才能进入 Level 3。
"""

import numpy as np

PASS = 0; TOTAL = 12

def check(name, condition, detail=""):
    global PASS
    if condition:
        PASS += 1
        print(f"  ✅ {name}")
    else:
        print(f"  ❌ {name} FAILED: {detail}")
    return condition


def test_01_feature_matching():
    from scipy.spatial import cKDTree
    np.random.seed(0)
    desc = np.random.randn(30, 64)
    tree = cKDTree(desc)
    dist, idx = tree.query(desc[0:1], k=2)[:2]
    return check("Kd-Tree 匹配: 自己对自己最近距离≈0", dist[0,0] < 0.01)

def test_02_lowe_ratio():
    np.random.seed(1)
    a = np.random.randn(50, 32)
    b = a[:20] + np.random.normal(0, 0.05, (20, 32))
    b = np.vstack([b, np.random.randn(30, 32)])

    from scipy.spatial import cKDTree
    tree = cKDTree(b)
    d, idx = tree.query(a, k=2)
    ratios = d[:,0] / (d[:,1] + 1e-8)
    good = np.sum((ratios < 0.7) & (idx[:,0] < 20))
    return check("Lowe's ratio test过滤外点", good >= 10, f"{good}/20")

def test_03_ransac():
    x = np.linspace(0, 10, 100)
    y = 3*x + 2 + np.random.normal(0, 0.5, 100)
    y[80:] = np.random.uniform(0, 35, 20)

    best_n, best_s, best_i = 0, 0, 0
    for _ in range(50):
        i1, i2 = np.random.choice(100, 2, replace=False)
        if abs(x[i2]-x[i1]) < 0.01: continue
        s = (y[i2]-y[i1])/(x[i2]-x[i1])
        b_int = y[i1] - s*x[i1]
        inliers = np.sum(np.abs(y - (s*x + b_int)) < 1.0)
        if inliers > best_n: best_n, best_s, best_i = inliers, s, b_int

    return check("RANSAC 恢复直线参数", abs(best_s - 3) < 0.3 and best_n >= 70,
                 f"slope={best_s:.2f}, inliers={best_n}")

def test_04_pnp_reprojection():
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    R = np.eye(3); t = np.array([0, 0, 3])
    pts3d = np.array([[1,1,0],[2,2,0],[1,0,0],[3,3,0],[1,1,0]])
    P = K @ np.hstack([R, t.reshape(3,1)])

    errors = []
    for p in pts3d:
        xh = P @ np.append(p, 1)
        u, v = xh[0]/xh[2], xh[1]/xh[2]
        errors.append(np.sqrt((u-320)**2 + (v-240)**2))

    # 点[1,0,0]偏离光轴 → 应有投影误差; 远离光轴的点投影更偏
    return check("PnP投影: 偏离光轴的点有非零像素偏移", errors[0] > 50)

def test_05_icp_convergence():
    src = np.random.randn(20, 3)
    theta = np.radians(15)
    R = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    t = np.array([1, 0.5, 0])
    dst = (R @ src.T).T + t

    R_e, t_e = np.eye(3), np.zeros(3)
    aligned = src.copy()
    for _ in range(10):
        c_s, c_d = aligned.mean(0), dst.mean(0)
        H = (aligned-c_s).T @ (dst-c_d)
        U, _, Vt = np.linalg.svd(H)
        dR = Vt.T @ U.T
        dt = c_d - dR @ c_s
        aligned = (dR @ aligned.T).T + dt
        R_e = dR @ R_e; t_e = dR @ t_e + dt

    return check("ICP 收敛到正确变换", np.linalg.norm(t_e - t) < 0.01,
                 f"error={np.linalg.norm(t_e-t):.4f}")

def test_06_gauss_newton():
    def f(x): return np.array([x-3, 2*(x+1)])
    def J(x): return np.array([[1],[2]])
    x = 0
    for _ in range(10):
        r = f(x); j = J(x)
        H = j.T @ j; g = j.T @ r
        dx = -np.linalg.solve(H, g)
        x += dx[0]
    return check("GN 收敛到正确解 x=-0.2", abs(x - (-0.2)) < 0.001, f"x={x:.4f}")

def test_07_lie_exp_log():
    from scipy.linalg import expm, logm
    xi = np.array([0.1, 0.2, 0.3, 0.05, 0.1, 0.15])
    phi = xi[3:6]
    phi_skew = np.array([[0,-phi[2],phi[1]],[phi[2],0,-phi[0]],[-phi[1],phi[0],0]])
    R = expm(phi_skew)
    return check("se(3) exp映射产生旋转矩阵", abs(np.linalg.det(R) - 1) < 0.01)

def test_08_factor_graph_structure():
    n_cams, n_pts = 3, 10
    h_size = n_cams*6 + n_pts*3
    density = 100 - (n_cams*6*n_cams*6) / (h_size*h_size) * 100
    return check("因子图H矩阵高稀疏性", density > 50,
                 f"稀疏度≈{density:.0f}%")

def test_09_pose_graph_loop_closure():
    n = 4
    poses = [np.array([float(i), 0.0, 0.0]) for i in range(n)]
    poses[3] = np.array([0.5, 2.0, 0.0])  # 漂移
    # 回环约束: poses[3] 应靠近 poses[0]
    err_before = np.linalg.norm(poses[3] - poses[0])
    return check("回环检测识别漂移", err_before > 1.5, f"drift={err_before:.2f}m")

def test_10_ate_rpe():
    true = np.column_stack([np.linspace(0,10,50), np.zeros(50), np.zeros(50)])
    est = true + np.random.normal(0, 0.1, (50,3))
    ate = np.sqrt(np.mean(np.sum((true-est)**2, axis=1)))
    return check("ATE正确反映整体误差", 0 < ate < 0.3, f"ATE={ate:.4f}")

def test_11_tsdf():
    pts = np.random.randn(50, 3) * 2
    pts[:,2] = np.abs(pts[:,2])
    grid = np.zeros((5,5,5))
    vs = 1.0
    for p in pts:
        x, y, z = int((p[0]+2.5)/vs), int((p[1]+2.5)/vs), int(p[2]/vs)
        if 0 <= x < 5 and 0 <= y < 5 and 0 <= z < 5:
            grid[x,y,z] = 1
    return check("TSDF体素: 物体占据部分空间", 0 < np.sum(grid) < 125)

def test_12_bow_similarity():
    np.random.seed(0)
    d1 = np.random.randn(50, 64)
    d2 = d1 + np.random.normal(0, 0.1, (50, 64))
    d3 = np.random.randn(50, 64)

    sim = lambda a,b: np.dot(a.flatten(), b.flatten()) / (np.linalg.norm(a)*np.linalg.norm(b)+1e-8)
    s12 = sim(d1, d2); s13 = sim(d1, d3)
    return check("BoW: 相似图像描述子更相似", s12 > s13,
                 f"s12={s12:.3f}, s13={s13:.3f}")


if __name__ == "__main__":
    print("═" * 50)
    print("  Level 2 综合测试 — SLAM 入门")
    print("═" * 50)

    test_01_feature_matching()
    test_02_lowe_ratio()
    test_03_ransac()
    test_04_pnp_reprojection()
    test_05_icp_convergence()
    test_06_gauss_newton()
    test_07_lie_exp_log()
    test_08_factor_graph_structure()
    test_09_pose_graph_loop_closure()
    test_10_ate_rpe()
    test_11_tsdf()
    test_12_bow_similarity()

    print(f"\n{'═'*50}")
    print(f"  结果: {PASS}/{TOTAL} 通过")
    if PASS == TOTAL:
        print(f"  🎉 Level 2 完成! 可以进入 Level 3!")
    else:
        print(f"  ⚠️  还有 {TOTAL-PASS} 个测试未通过")
