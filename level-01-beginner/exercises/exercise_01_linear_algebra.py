"""
练习 1: 线性代数 (交互式)
===========================
把 `# TODO` 标记的行改成正确代码，然后运行。
检查通过才算完成。
"""

import numpy as np

def task_01_vectors():
    """任务1: 向量运算

            a = [3, 0, 0],  b = [0, 4, 0]
    """
    print("\n═══ 任务1: 向量运算 ═══")
    a = np.array([3.0, 0.0, 0.0])
    b = np.array([0.0, 4.0, 0.0])

    # ─── 改下面的 TODO ───
    # TODO 1: 计算 a 的 L2 范数
    norm_a = 0  # TODO: 改成 np.linalg.norm(a)

    # TODO 2: 计算 a 和 b 的点积
    dot = 0  # TODO: 改成 np.dot(a, b)

    # TODO 3: 计算叉积
    cross = np.zeros(3)  # TODO: 改成 np.cross(a, b)

    # TODO 4: 计算夹角 (度数)
    angle_deg = 0  # TODO: 用点积+arccos+degrees

    # ─── 验证 ───
    print(f"  |a| = {norm_a}     (应≈3.0)")
    print(f"  a·b = {dot}        (应≈0.0)")
    print(f"  a×b = {cross}")
    print(f"  夹角 = {angle_deg:.1f}° (应≈90°)")

    ok = True
    if abs(norm_a - 3.0) > 0.1 or norm_a == 0:
        print("  ❌ norm_a 不对。提示: np.linalg.norm(a)"); ok = False
    if abs(dot) > 0.1 or dot == 0:
        print("  ❌ dot 不对。提示: np.dot(a, b)"); ok = False
    if not np.allclose(cross, [0, 0, 12]):
        print("  ❌ cross 不对。提示: np.cross(a, b)"); ok = False
    if abs(angle_deg - 90) > 1:
        print("  ❌ angle_deg 不对。步骤: cos = dot/(|a|*|b|) → arccos → degrees"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_02_matrices():
    """任务2: 旋转矩阵

    绕 Z 轴旋转 30° 的 2D 矩阵。
    """
    print("\n═══ 任务2: 旋转矩阵 ═══")

    theta = np.radians(30)
    v = np.array([1.0, 0.0])

    # TODO: 构造 2x2 旋转矩阵 (绕Z轴转 theta 度)
    R = np.zeros((2, 2))  # TODO: 改成正确的旋转矩阵

    # TODO: 用 R 旋转 v
    rotated = np.zeros(2)  # TODO: 改成 R @ v

    # TODO: 计算行列式
    det_R = 0  # TODO: 改成 np.linalg.det(R)

    print(f"  R = \n{R}")
    print(f"  rotated = {rotated}  (应≈[0.866, 0.5])")
    print(f"  det(R) = {det_R}  (应≈1.0)")

    ok = True
    if abs(det_R - 1.0) > 0.01:
        print("  ❌ det(R)≠1。旋转矩阵: R[0,0]=cosθ, R[0,1]=-sinθ, R[1,0]=sinθ, R[1,1]=cosθ"); ok = False
    if abs(rotated[0] - 0.866) > 0.01:
        print("  ❌ rotated 不对。提示: R @ v"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_03_svd():
    """任务3: SVD 解 Ax=0

    八点法求基础矩阵时，解 = V 的最后一列。
    """
    print("\n═══ 任务3: SVD 解 Ax=0 ═══")

    A = np.random.RandomState(42).randn(9, 9)
    A[:, -1] = A[:, 0] * 0.5 + A[:, 1] * 0.3

    # TODO: SVD 分解
    U = np.zeros((9, 9))       # TODO: np.linalg.svd(A) 返回 U, S, Vt
    S = np.zeros(9)
    Vt = np.zeros((9, 9))

    # TODO: 取最小奇异值对应的向量
    x = np.zeros(9)  # TODO: x = Vt[-1] (最后一列)

    # TODO: 验证 Ax ≈ 0
    residual = 999  # TODO: np.linalg.norm(A @ x)

    print(f"  奇异值: {S.round(3)}")
    print(f"  ||Ax|| = {residual:.2e}  (应≈0)")

    ok = True
    if residual > 0.1:
        print("  ❌ residual 太大。提示:"); ok = False
        print("    U, S, Vt = np.linalg.svd(A)")
        print("    x = Vt[-1]")
        print("    residual = np.linalg.norm(A @ x)")

    if ok: print("  ✅ 通过!")
    return ok


def task_04_rigid_transform():
    """任务4: 齐次变换矩阵

    绕 Z 轴转 45°，平移 [1, 2, 3]。
    """
    print("\n═══ 任务4: 齐次变换 ═══")

    theta = np.radians(45)

    # TODO: 绕 Z 轴的 3x3 旋转矩阵
    R = np.zeros((3, 3))  # TODO: 填入正确值

    # TODO: 4x4 齐次变换矩阵
    t = np.array([1.0, 2.0, 3.0])
    T = np.eye(4)  # TODO: T[:3,:3] = R; T[:3,3] = t

    # TODO: T 的逆
    T_inv = np.eye(4)  # TODO: np.linalg.inv(T)

    # 验证
    p = np.array([1, 1, 1, 1.0])
    p_trans = np.zeros(4)  # TODO: T @ p
    p_back = np.zeros(4)    # TODO: T_inv @ p_trans

    diff = np.linalg.norm(p_back - p)
    print(f"  T = \n{T}")
    print(f"  p_trans = {p_trans[:3]}")
    print(f"  逆回去误差 = {diff:.4f} (应≈0)")

    ok = True
    if diff > 0.01:
        print("  ❌ 变换不对。提示:"); ok = False
        print("    R[0,0]=cosθ, R[0,1]=-sinθ")
        print("    R[1,0]=sinθ, R[1,1]=cosθ, R[2,2]=1")
        print("    T[:3,:3]=R; T[:3,3]=t")
        print("    T_inv = np.linalg.inv(T)")

    if ok: print("  ✅ 通过!")
    return ok


if __name__ == "__main__":
    results = [task_01_vectors(), task_02_matrices(),
               task_03_svd(), task_04_rigid_transform()]
    passed = sum(results)
    print(f"\n{'='*40}")
    print(f"  通过: {passed}/{len(results)}")
    if passed == len(results):
        print("  🎉 全部完成!")
    else:
        print(f"  还有 {len(results)-passed} 个任务，改完重跑。")
