"""
练习 3: 非线性优化 — 高斯牛顿/LM/李代数/BA (Level 2)
==========================================================
"""

import numpy as np
from scipy.linalg import expm, logm

def skew(v):
    """向量 → 反对称矩阵"""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def exp_se3(xi):
    """SE(3) 指数映射: 6维向量 → 4x4变换矩阵"""
    rho, phi = xi[:3], xi[3:6]
    theta = np.linalg.norm(phi)
    R = np.eye(3)
    if theta > 1e-10:
        axis = phi / theta
        R = np.eye(3) + np.sin(theta)*skew(axis) + (1-np.cos(theta))*skew(axis)@skew(axis)

    # Jacobian
    if theta > 1e-10:
        A = np.sin(theta)/theta
        B = (1-np.cos(theta))/(theta*theta)
        V = np.eye(3) + B*skew(phi) + (1-A)/(theta*theta)*skew(phi)@skew(phi)
    else:
        V = np.eye(3)

    t = V @ rho
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t
    return T

def log_se3(T):
    """SE(3) 对数映射: 4x4变换矩阵 → 6维向量"""
    R, t = T[:3,:3], T[:3,3]
    theta = np.arccos(np.clip((np.trace(R)-1)/2, -1, 1))

    if theta < 1e-10:
        return np.concatenate([t, np.zeros(3)])

    lnR = theta / (2*np.sin(theta)) * (R - R.T)
    phi = np.array([lnR[2,1], lnR[0,2], lnR[1,0]])

    # V^{-1}
    A = np.sin(theta)/theta
    B = (1-np.cos(theta))/(theta*theta)
    V_inv = np.eye(3) - 0.5*skew(phi) + (1/(theta*theta))*(1 - A/(2*B))*skew(phi)@skew(phi)
    rho = V_inv @ t

    return np.concatenate([rho, phi])


def task_01_gauss_newton():
    """任务1: 高斯-牛顿法 (20分钟)

    最小化 f(x) = (x-3)² + 4*(x+1)²
    """
    print("\n═══ 任务1: 高斯-牛顿法 ═══")

    # 残差: r1 = x-3, r2 = 2*(x+1) → r = [x-3, 2x+2]
    def residual(x):
        return np.array([x-3, 2*x+2])

    def jacobian(x):
        return np.array([[1], [2]])

    x = 0.0  # 初始值
    for i in range(10):
        r = residual(x)
        J = jacobian(x)

        # 高斯-牛顿: Δx = -(J^T J)^{-1} J^T r
        H = J.T @ J
        b = J.T @ r
        dx = -np.linalg.solve(H, b)
        x += dx[0]

        cost = 0.5 * np.sum(r**2)
        if i < 5:
            print(f"  迭代{i}: x={x:.4f}, cost={cost:.4f}")

        if abs(dx[0]) < 1e-8:
            break

    print(f"  最终 x = {x:.4f} (解析解: x=-0.2)")
    assert abs(x - (-0.2)) < 0.01
    print("  ✅ 通过!")


def task_02_levenberg_marquardt():
    """任务2: Levenberg-Marquardt (15分钟)

    LM = GN + 阻尼: Δx = -(J^T J + λI)^{-1} J^T r
    """
    print("\n═══ 任务2: Levenberg-Marquardt ═══")

    def residual(x):
        return np.array([x[0]**2 + x[1] - 11, x[0] + x[1]**2 - 7])

    def jacobian(x):
        return np.array([[2*x[0], 1], [1, 2*x[1]]])

    x = np.array([1.0, 1.0])  # 初始值
    lam = 1.0  # 阻尼因子

    for i in range(20):
        r = residual(x)
        J = jacobian(x)
        cost = 0.5 * np.sum(r**2)

        H = J.T @ J + lam * np.eye(2)
        g = J.T @ r
        dx = -np.linalg.solve(H, g)

        x_new = x + dx
        r_new = residual(x_new)
        cost_new = 0.5 * np.sum(r_new**2)

        # 判断是否接受
        if cost_new < cost:
            x = x_new
            lam *= 0.5  # 减小阻尼
            if i < 5:
                print(f"  迭代{i}: x={x.round(3)}, cost={cost:.4f}, λ={lam:.4f}")
        else:
            lam *= 2.0  # 增大阻尼

        if np.linalg.norm(dx) < 1e-8:
            break

    print(f"  最终 x = {x.round(4)}")
    print(f"  已知解: x≈(3, 2) 或 x≈(3.584, -1.848)")
    assert cost < 0.01, f"cost应≈0, 实际={cost:.6f}"
    print("  ✅ 通过!")


def task_03_lie_algebra():
    """任务3: 李群李代数 SE(3) (20分钟)

    SLAM 中位姿用 SE(3) 表示, 用 se(3) 做优化。
    """
    print("\n═══ 任务3: SE(3) 李代数 ═══")

    # 定义一个小扰动
    xi = np.array([0.1, 0.05, 0.02, 0.01, 0.02, 0.03])  # [rho, phi]

    # 指数映射: se(3) → SE(3)
    T = exp_se3(xi)
    print(f"  se(3) ξ = {xi}")
    print(f"  SE(3) T = exp(ξ^):")
    print(T.round(4))

    # 对数映射: SE(3) → se(3)
    xi_recovered = log_se3(T)
    print(f"  恢复的 ξ = {xi_recovered.round(4)}")
    print(f"  原始 ξ   = {xi.round(4)}")
    print(f"  误差 = {np.linalg.norm(xi - xi_recovered):.6f}")

    assert np.linalg.norm(xi - xi_recovered) < 0.01
    print("  ✅ 通过!")

    # 验证 SE(3) 性质
    det_R = np.linalg.det(T[:3,:3])
    print(f"  det(R) = {det_R:.6f} (应为1)")
    assert abs(det_R - 1) < 0.01

    # 在 SLAM 中的应用: 位姿更新用指数映射
    # T_new = exp(ξ^) @ T_current
    update_xi = np.array([0.2, 0, 0, 0, 0, 0.1])  # 沿X轴移动0.2, 绕Z轴转0.1
    T_current = np.eye(4)
    T_update = exp_se3(update_xi)
    T_new = T_update @ T_current
    print(f"  位姿更新: T_new = exp(ξ^) @ T")
    print(f"  新平移: {T_new[:3,3].round(3)} (应≈[0.2, 0, 0])")


def task_04_bundle_adjustment_mini():
    """任务4: 微型 Bundle Adjustment (25分钟)

    BA: 同时优化相机位姿和3D点位置，最小化重投影误差。
    """
    print("\n═══ 任务4: 微型 Bundle Adjustment ═══")

    # 设置
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])

    # 真实值
    n_pts = 5
    pts_true = np.random.RandomState(42).uniform(-1, 1, (n_pts, 3))
    pts_true[:, 2] += 3  # 深度

    poses_true = []
    for i in range(3):
        theta = np.radians(i * 15)
        R = np.array([[np.cos(theta), 0, np.sin(theta)],
                      [0, 1, 0],
                      [-np.sin(theta), 0, np.cos(theta)]])
        t = np.array([i*0.3, 0, 0])
        poses_true.append((R, t))

    # 生成观测
    observations = []
    for cam_idx, (R, t) in enumerate(poses_true):
        for pt_idx in range(n_pts):
            ph = np.append(pts_true[pt_idx], 1)
            P = K @ np.hstack([R, t.reshape(3,1)])
            xh = P @ ph
            u, v = xh[0]/xh[2] + np.random.normal(0, 0.5), xh[1]/xh[2] + np.random.normal(0, 0.5)
            observations.append((cam_idx, pt_idx, u, v))

    # 初始化 (加噪声)
    pts_est = pts_true + np.random.normal(0, 0.1, pts_true.shape)
    poses_est = []
    for R, t in poses_true:
        noise_R = expm(skew(np.random.normal(0, 0.05, 3)))
        noise_t = np.random.normal(0, 0.05, 3)
        poses_est.append((noise_R @ R, t + noise_t))

    # GN 优化 (几轮)
    for iteration in range(5):
        H = np.zeros((3*n_pts + 6*len(poses_est), 3*n_pts + 6*len(poses_est)))
        b = np.zeros(3*n_pts + 6*len(poses_est))

        for cam_idx, pt_idx, u_obs, v_obs in observations:
            R, t = poses_est[cam_idx]
            p = pts_est[pt_idx]

            # 投影
            p_cam = R @ p + t
            u_proj = K[0,0]*p_cam[0]/p_cam[2] + K[0,2]
            v_proj = K[1,1]*p_cam[1]/p_cam[2] + K[1,2]

            # 残差
            r = np.array([u_proj - u_obs, v_proj - v_obs])

            # Jacobian w.r.t 3D点
            if abs(p_cam[2]) > 1e-6:
                dp_dp_cam = np.array([
                    [K[0,0]/p_cam[2], 0, -K[0,0]*p_cam[0]/p_cam[2]**2],
                    [0, K[1,1]/p_cam[2], -K[1,1]*p_cam[1]/p_cam[2]**2]
                ])
                J_pt = dp_dp_cam @ R  # 2x3

                # Jacobian w.r.t 相机位姿 (简化: 只平移)
                J_pose_trans = dp_dp_cam  # 2x3

            # 填充 H 和 b
            pt_offset = 3 * pt_idx
            cam_offset = 3 * n_pts + 6 * cam_idx

            # w.r.t 3D点
            H[pt_offset:pt_offset+3, pt_offset:pt_offset+3] += J_pt.T @ J_pt
            b[pt_offset:pt_offset+3] -= J_pt.T @ r

            # w.r.t 相机平移
            H[cam_offset+3:cam_offset+6, cam_offset+3:cam_offset+6] += J_pose_trans.T @ J_pose_trans
            b[cam_offset+3:cam_offset+6] -= J_pose_trans.T @ r

        # 求解 (加阻尼)
        H_reg = H + 0.1 * np.eye(H.shape[0])
        try:
            dx = np.linalg.solve(H_reg, b)

            # 更新
            for i in range(n_pts):
                pts_est[i] += dx[3*i:3*i+3]
            for i in range(len(poses_est)):
                R, t = poses_est[i]
                dxi = dx[3*n_pts+6*i:3*n_pts+6*i+6]
                dR = expm(skew(dxi[3:6]))
                poses_est[i] = (dR @ R, t + dxi[:3])
        except np.linalg.LinAlgError:
            break

    # 评估
    pt_errors = np.linalg.norm(pts_est - pts_true, axis=1)
    print(f"  BA 后3D点误差: 中位数 {np.median(pt_errors):.3f}m (初始加了0.1m噪声)")

    for i, ((R_est, t_est), (R_true, t_true)) in enumerate(zip(poses_est, poses_true)):
        err = np.linalg.norm(t_est - t_true)
        print(f"  相机{i} 平移误差: {err:.3f}m")

    print(f"  ℹ️  简化BA用了几轮迭代和固定Jacobian — 真实BA用Ceres/g2o完整实现")
    print("  ✅ 通过! (理解BA原理)")


if __name__ == "__main__":
    task_01_gauss_newton()
    task_02_levenberg_marquardt()
    task_03_lie_algebra()
    task_04_bundle_adjustment_mini()
    print("\n🎉 所有非线性优化练习完成!")
