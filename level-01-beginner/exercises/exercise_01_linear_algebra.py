"""
练习 1: SLAM 所需的线性代数基础
===================================
完成以下任务来巩固你的线性代数知识。
每个任务都有 ✅ 验证 — 运行脚本即自动检查。
"""

import numpy as np

def task_01_vectors():
    """任务1: 向量基本运算 (5分钟)
    
    SLAM 中处处是向量：3D点、速度、平移都是向量。
    """
    print("\n═══ 任务1: 向量运算 ═══")
    
    # 定义两个3D向量（SLAM中代表空间点或方向）
    a = np.array([3.0, 0.0, 0.0])   # 沿X轴
    b = np.array([0.0, 4.0, 0.0])   # 沿Y轴
    
    # TODO: 计算以下内容
    # 1. 向量的模 (L2范数)
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    print(f"  |a| = {norm_a:.1f}  (应该是 3.0)")
    print(f"  |b| = {norm_b:.1f}  (应该是 4.0)")
    
    # 2. 点积 (dot product) - SLAM中用于计算角度
    dot = np.dot(a, b)
    print(f"  a·b = {dot:.1f}  (应该是 0.0 — 正交)")
    
    # 3. 叉积 (cross product) - SLAM中用于计算法向量
    cross = np.cross(a, b)
    print(f"  a×b = {cross}  (应该是 [0, 0, 12] — 沿Z轴)")
    
    # 4. 夹角（用点积推导）
    cos_angle = dot / (norm_a * norm_b)
    angle = np.arccos(np.clip(cos_angle, -1, 1))
    print(f"  夹角 = {np.degrees(angle):.1f}°  (应该是 90°)")
    
    # 验证点积和叉积的关系
    assert abs(dot) < 0.01, "点积应为0（正交向量）"
    assert np.allclose(cross, [0, 0, 12]), "叉积应为 [0, 0, 12]"
    print("  ✅ 通过!")


def task_02_matrices():
    """任务2: 矩阵运算 (10分钟)
    
    旋转矩阵和变换矩阵是 SLAM 的核心。
    """
    print("\n═══ 任务2: 矩阵运算 ═══")
    
    # 2D 旋转矩阵 (绕 Z 轴旋转 θ)
    theta = np.radians(30)  # 30度
    R_2d = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    print(f"  2D旋转矩阵(30°):\n{R_2d}")
    
    # 旋转一个向量
    v = np.array([1.0, 0.0])
    rotated = R_2d @ v
    print(f"  旋转 [1,0] 后: {rotated}  (应该是 [cos30, sin30])")
    
    # TODO: 计算行列式 (必须是1，旋转矩阵保持体积)
    det = np.linalg.det(R_2d)
    print(f"  det(R) = {det:.6f}  (旋转矩阵行列式应为 1)")
    
    # TODO: 计算逆矩阵 (旋转矩阵的逆 = 转置)
    R_inv = np.linalg.inv(R_2d)
    print(f"  R 的逆等于转置? {np.allclose(R_inv, R_2d.T)}  (应该是 True)")
    
    # 验证: R * R^T = I
    I_check = R_2d @ R_2d.T
    print(f"  R*R^T = I? {np.allclose(I_check, np.eye(2))}")
    
    assert abs(det - 1.0) < 0.01
    assert np.allclose(R_inv @ rotated, v)
    print("  ✅ 通过!")


def task_03_eigen_and_svd():
    """任务3: 特征值分解 与 SVD (15分钟)
    
    SVD 是对极几何和 PnP 求解的核心工具。
    """
    print("\n═══ 任务3: 特征值与 SVD ═══")
    
    # 创建一个对称矩阵（类似 SLAM 中的 Hessian 矩阵）
    A = np.array([
        [3.0, 1.0, 0.0],
        [1.0, 3.0, 1.0],
        [0.0, 1.0, 3.0]
    ])
    
    # TODO: 特征值分解
    eigvals, eigvecs = np.linalg.eig(A)
    print(f"  特征值: {eigvals}")
    print(f"  特征向量:\n{eigvecs}")
    
    # 验证: A * v = λ * v
    for i in range(3):
        lhs = A @ eigvecs[:, i]
        rhs = eigvals[i] * eigvecs[:, i]
        assert np.allclose(lhs, rhs, rtol=1e-8), f"特征值{i}验证失败"
    print("  ✅ 特征值分解验证通过")
    
    # TODO: SVD 分解 (A = U Σ V^T)
    U, S, Vt = np.linalg.svd(A)
    print(f"  奇异值: {S}")
    
    # 验证: U @ diag(S) @ Vt ≈ A
    A_reconstructed = U @ np.diag(S) @ Vt
    print(f"  重构误差: {np.max(np.abs(A - A_reconstructed)):.2e}")
    assert np.allclose(A, A_reconstructed)
    
    # SVD 在 SLAM 中的应用: 求解超定线性方程组的最小二乘解
    # Ax = b 的最小二乘解: x = V @ diag(1/s) @ U^T @ b
    b = np.array([1.0, 2.0, 3.0])
    x_svd = Vt.T @ np.diag(1/S) @ U.T @ b
    x_np = np.linalg.lstsq(A, b, rcond=None)[0]
    print(f"  SVD 求解: {x_svd}")
    print(f"  NumPy 求解: {x_np}")
    assert np.allclose(x_svd, x_np)
    print("  ✅ SVD 分解验证通过")


def task_04_rigid_transforms():
    """任务4: 刚体变换与齐次坐标 (15分钟)
    
    SLAM 中相机位姿用 SE(3) 变换表示。
    """
    print("\n═══ 任务4: 刚体变换 ═══")
    
    # 3D 旋转矩阵 (绕 Z 轴 45°)
    theta = np.radians(45)
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,             0,             1]
    ])
    
    # 平移向量
    t = np.array([1.0, 2.0, 3.0])
    
    # TODO: 构造 4x4 齐次变换矩阵 T ∈ SE(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    print(f"  齐次变换矩阵 T:\n{T}")
    
    # 变换一个 3D 点 (齐次坐标)
    p_3d = np.array([1.0, 1.0, 1.0, 1.0])  # 齐次坐标
    p_transformed = T @ p_3d
    print(f"  点 [1,1,1] 变换后: {p_transformed[:3]}")
    
    # TODO: 计算 T 的逆（把点变回去）
    T_inv = np.linalg.inv(T)
    p_back = T_inv @ p_transformed
    print(f"  逆变换回去: {p_back[:3]}")
    assert np.allclose(p_back[:3], [1, 1, 1])
    
    # 验证 SE(3) 性质: T_inv 应该等于 [R^T | -R^T * t]
    T_inv_expected = np.eye(4)
    T_inv_expected[:3, :3] = R.T
    T_inv_expected[:3, 3] = -R.T @ t
    print(f"  T^{-1} = [R^T | -R^T t]? {np.allclose(T_inv, T_inv_expected)}")
    
    print("  ✅ 通过!")


def task_05_transformation_chains():
    """任务5: 变换链 (10分钟)
    
    SLAM 中相机在世界坐标系中的位姿通过变换链计算。
    """
    print("\n═══ 任务5: 变换链 ═══")
    
    # 世界坐标系中有一个点
    P_world = np.array([5.0, 0.0, 0.0, 1.0])
    
    # 相机在世界坐标系中的位姿 T_wc (camera from world)
    theta = np.radians(30)
    R_wc = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,             0,             1]
    ])
    t_wc = np.array([2.0, 1.0, 0.0])
    T_wc = np.eye(4)
    T_wc[:3, :3] = R_wc
    T_wc[:3, 3] = t_wc
    
    # TODO: 世界点变换到相机坐标系
    # P_camera = T_cw * P_world = T_wc^{-1} * P_world
    T_cw = np.linalg.inv(T_wc)
    P_camera = T_cw @ P_world
    print(f"  世界点 {P_world[:3]} → 相机坐标: {P_camera[:3]}")
    
    # TODO: 相机点再变回世界坐标
    P_world_back = T_wc @ P_camera
    print(f"  变回世界: {P_world_back[:3]}")
    assert np.allclose(P_world_back[:3], P_world[:3])
    
    # 再引入一个物体坐标系 (object frame)
    T_wo = np.eye(4)  # 物体在世界中的位姿
    T_wo[:3, 3] = [3.0, 0.0, 0.0]
    
    # TODO: 物体上的点在相机坐标系中的位置
    # P_cam = T_cw * T_wo * P_obj
    P_obj = np.array([0.5, 0.5, 0.0, 1.0])  # 物体上的点
    P_cam_from_obj = T_cw @ T_wo @ P_obj
    print(f"  物体点 {P_obj[:3]} → 相机坐标: {P_cam_from_obj[:3]}")
    
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_vectors()
    task_02_matrices()
    task_03_eigen_and_svd()
    task_04_rigid_transforms()
    task_05_transformation_chains()
    print("\n🎉 所有线性代数练习完成!")
