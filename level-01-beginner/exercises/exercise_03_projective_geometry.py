"""
练习 3: 射影几何 (交互式)
=============================
针孔相机 → 对极几何 → 三角化
"""

import numpy as np

def task_01_pinhole_camera():
    """任务1: 针孔相机投影

    K = diag(500, 500), 主点 (320, 240)
    相机在世界原点，朝向 +Z。
    点 P_world = [0, 0, 5]，投影到哪个像素？
    """
    print("\n═══ 任务1: 针孔相机投影 ═══")

    fx, fy = 500.0, 500.0
    cx, cy = 320.0, 240.0

    # TODO: 构造内参矩阵 K (3x3)
    K = np.zeros((3, 3))  # TODO: K[0,0]=fx, K[0,2]=cx, K[1,1]=fy, K[1,2]=cy, K[2,2]=1

    # 相机在世界原点
    Rt = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0]])

    # TODO: 投影矩阵 P = K @ Rt
    P = np.zeros((3, 4))  # TODO

    # TODO: 投影点 [0, 0, 5, 1]
    P_world = np.array([0, 0, 5, 1.0])
    x_homo = np.zeros(3)     # TODO: P @ P_world
    u = 0  # TODO: x_homo[0] / x_homo[2]
    v = 0  # TODO: x_homo[1] / x_homo[2]

    print(f"  像素: ({u:.0f}, {v:.0f})  (应≈(320, 240) — 正前方点投影到主点)")

    ok = True
    if abs(u - 320) > 1 or abs(v - 240) > 1:
        print("  ❌ 投影不对。步骤: K = [[fx,0,cx],[0,fy,cy],[0,0,1]]; P = K @ Rt; x = P @ Pw; u=x[0]/x[2]"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_02_epipolar_constraint():
    """任务2: 验证对极约束 x2^T F x1 = 0

    相机2 在相机1 右侧 0.5m。构造 F 矩阵并验证约束。
    """
    print("\n═══ 任务2: 对极约束 ═══")

    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    b = 0.5  # 基线

    # 相机1: 原点, 相机2: 在 X 轴方向平移 b
    # E = [t]× R (其中 R=I, t=[b,0,0])
    t = np.array([b, 0, 0])

    # TODO: 构造 t 的反对称矩阵
    t_skew = np.zeros((3, 3))  # TODO: [[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]]

    # TODO: E = t_skew @ R (R=I)
    E = np.zeros((3, 3))  # TODO

    # TODO: F = K^{-T} @ E @ K^{-1}
    K_inv = np.linalg.inv(K)
    F = np.zeros((3, 3))  # TODO: K_inv.T @ E @ K_inv

    # 验证: 两个匹配点应满足 x2^T F x1 ≈ 0
    x1 = np.array([400, 300, 1.0])  # 图像1中的点
    x2 = np.array([380, 300, 1.0])  # 图像2中对应的点

    # TODO: 计算对极约束误差
    error = 999  # TODO: x2 @ F @ x1

    print(f"  E = \n{E}")
    print(f"  F = \n{F}")
    print(f"  对极约束误差: {error:.6f} (应≈0)")

    ok = True
    if abs(error) > 0.1:
        print("  ❌ t_skew = [[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]]; E=t_skew; F=K_inv.T@E@K_inv"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_03_triangulation():
    """任务3: DLT 三角化

    已知两相机投影矩阵和匹配点，恢复 3D 点。
    """
    print("\n═══ 任务3: 三角化 ═══")

    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    t2 = np.array([0.5, 0.0, 0.0])
    P2 = K @ np.hstack([np.eye(3), t2.reshape(3, 1)])

    # 真实 3D 点
    true_pt = np.array([0.3, 0.2, 3.0])

    # 投影 (无噪声)
    x1h = P1 @ np.append(true_pt, 1); u1, v1 = x1h[0]/x1h[2], x1h[1]/x1h[2]
    x2h = P2 @ np.append(true_pt, 1); u2, v2 = x2h[0]/x2h[2], x2h[1]/x2h[2]

    # TODO: 构造 DLT 矩阵 A (4x4)
    # 每台相机贡献两行: u*P[2] - P[0], v*P[2] - P[1]
    A = np.zeros((4, 4))  # TODO: 填入正确值

    # TODO: SVD 分解 A, 取最小奇异值向量
    X = np.zeros(4)  # TODO: Vt[-1]; X = X / X[3]

    err = np.linalg.norm(X[:3] - true_pt)
    print(f"  三角化: {X[:3].round(3)}")
    print(f"  真值:   {true_pt}")
    print(f"  误差:   {err:.3f}m (应≈0)")

    ok = True
    if err > 0.1:
        print("  ❌ 提示:"); ok = False
        print("    A[0] = u1*P1[2] - P1[0]")
        print("    A[1] = v1*P1[2] - P1[1]")
        print("    A[2] = u2*P2[2] - P2[0]")
        print("    A[3] = v2*P2[2] - P2[1]")
        print("    _,_,Vt = np.linalg.svd(A); X = Vt[-1]; X /= X[3]")

    if ok: print("  ✅ 通过!")
    return ok


def task_04_disparity_depth():
    """任务4: 视差 → 深度

    双目相机: 焦距 f=500px, 基线 B=0.5m。
    某点在左右图中的视差 d=50px。求深度 Z。
    """
    print("\n═══ 任务4: 视差→深度 ═══")

    f = 500.0  # 焦距 (像素)
    B = 0.5    # 基线 (米)
    d = 50.0   # 视差 (像素)

    # TODO: Z = f * B / d
    Z = 0  # TODO

    print(f"  深度 Z = {Z:.2f}m (应=5.0m)")

    ok = True
    if abs(Z - 5.0) > 0.1:
        print("  ❌ 公式: Z = f * B / d"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


if __name__ == "__main__":
    results = [task_01_pinhole_camera(), task_02_epipolar_constraint(),
               task_03_triangulation(), task_04_disparity_depth()]
    passed = sum(results)
    print(f"\n{'='*40}")
    print(f"  通过: {passed}/{len(results)}")
    if passed == len(results):
        print("  🎉 全部完成!")
    else:
        print(f"  还有 {len(results)-passed} 个任务，改完重跑。")
