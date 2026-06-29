"""
Level 1 综合测试: 基础篇
============================
通过所有测试才能进入 Level 2。

运行: python3 test_01_basics.py
"""

import numpy as np
import sys

PASS_COUNT = 0
TOTAL_TESTS = 15

def check(name, condition, detail=""):
    global PASS_COUNT
    if condition:
        PASS_COUNT += 1
        print(f"  ✅ {name}")
    else:
        print(f"  ❌ {name} FAILED: {detail}")
    return condition


def test_01_vector_norm():
    """向量的L2范数"""
    v = np.array([3.0, 4.0, 0.0])
    n = np.linalg.norm(v)
    return check("向量范数 |[3,4,0]| = 5", abs(n - 5.0) < 0.01)

def test_02_dot_product():
    """点积验证"""
    a = np.array([1, 0, 0])
    b = np.array([0, 1, 0])
    return check("正交向量点积=0", abs(np.dot(a, b)) < 0.01)

def test_03_cross_product():
    """叉积验证"""
    a = np.array([1, 0, 0])
    b = np.array([0, 1, 0])
    c = np.cross(a, b)
    return check("叉积 a×b = [0,0,1]", np.allclose(c, [0, 0, 1]))

def test_04_rotation_matrix():
    """旋转矩阵性质"""
    theta = np.radians(45)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    det = np.linalg.det(R)
    R_inv = np.linalg.inv(R)
    return check("旋转矩阵 det=1, R^{-1}=R^T",
                 abs(det - 1.0) < 0.01 and np.allclose(R_inv, R.T))

def test_05_svd_solve():
    """SVD 求解超定方程"""
    A = np.array([[1, 2], [3, 4], [5, 6]])
    b = np.array([7, 8, 9])
    x_lstsq = np.linalg.lstsq(A, b, rcond=None)[0]

    U, S, Vt = np.linalg.svd(A, full_matrices=False)
    x_svd = Vt.T @ np.diag(1/S) @ U.T @ b
    return check("SVD 最小二乘解正确", np.allclose(x_lstsq, x_svd, rtol=1e-6))

def test_06_eigen_decomposition():
    """特征值分解验证"""
    A = np.array([[2, 1], [1, 2]])
    eigvals, eigvecs = np.linalg.eig(A)
    for i in range(2):
        if not np.allclose(A @ eigvecs[:, i], eigvals[i] * eigvecs[:, i]):
            return check("A v = λ v", False, f"特征值{i}不满足")
    return check("特征值分解 A v = λ v", True)

def test_07_homogeneous_transform():
    """齐次变换"""
    T = np.eye(4)
    T[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # 绕Z轴90°
    T[:3, 3] = [1, 2, 3]
    p = np.array([1, 0, 0, 1])
    p_trans = T @ p
    # 旋转90°: (1,0,0) → (0,1,0), 再平移(1,2,3) → (1,3,3)
    return check("齐次变换正确", np.allclose(p_trans, [1, 3, 3, 1]))

def test_08_gaussian_68_95_99():
    """高斯分布 68-95-99.7 规则"""
    samples = np.random.normal(0, 1, 100000)
    w1 = np.mean(np.abs(samples) < 1)
    w2 = np.mean(np.abs(samples) < 2)
    return check("高斯68-95规则",
                 w1 > 0.67 and w1 < 0.70 and w2 > 0.94)

def test_09_bayes():
    """贝叶斯定理"""
    prior = 0.01  # P(疾病)
    sensitivity = 0.99  # P(阳性|疾病)
    specificity = 0.95  # P(阴性|健康)
    # P(疾病|阳性) = sensitivity*prior / (sensitivity*prior + (1-specificity)*(1-prior))
    false_pos = 1 - specificity
    p_pos = sensitivity * prior + false_pos * (1-prior)
    posterior = sensitivity * prior / p_pos
    return check("贝叶斯后验概率 ≈ 0.167",
                 abs(posterior - 0.1667) < 0.02, f"got {posterior:.4f}")

def test_10_mle():
    """最大似然估计"""
    true_mean = 5.0
    measurements = true_mean + np.random.normal(0, 0.5, 1000)
    mle = np.mean(measurements)
    return check("MLE 估计接近真值", abs(mle - true_mean) < 0.1, f"mle={mle:.3f}")

def test_11_pinhole_projection():
    """针孔相机投影"""
    fx, fy = 600, 600
    cx, cy = 320, 240
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    P = K @ np.hstack([np.eye(3), np.zeros((3,1))])
    X = np.array([0, 0, 5, 1])
    xh = P @ X
    u, v = xh[0]/xh[2], xh[1]/xh[2]
    return check("正前方点投影到主点 (320,240)",
                 abs(u - 320) < 0.1 and abs(v - 240) < 0.1,
                 f"got ({u:.1f},{v:.1f})")

def test_12_epipolar_constraint():
    """对极约束验证"""
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    t = np.array([0.5, 0, 0])
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx  # R=I 所以 E = [t]×
    pt1 = np.array([300, 240, 1.0])
    pt2 = np.array([280, 240, 1.0])
    err = pt2 @ E @ pt1
    return check("对极约束 x2^T E x1 ≈ 0", abs(err) < 0.01, f"error={err:.4f}")

def test_13_triangulation():
    """三角化验证"""
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])
    P1 = K @ np.hstack([np.eye(3), np.zeros((3,1))])
    t2 = np.array([1.0, 0.0, 0.0])
    P2 = K @ np.hstack([np.eye(3), t2.reshape(3,1)])

    true_pt = np.array([0.5, 0.5, 3.0])

    # 无噪声投影
    x1h = P1 @ np.append(true_pt, 1)
    u1, v1 = x1h[0]/x1h[2], x1h[1]/x1h[2]
    x2h = P2 @ np.append(true_pt, 1)
    u2, v2 = x2h[0]/x2h[2], x2h[1]/x2h[2]

    # DLT 三角化
    A = np.array([u1*P1[2]-P1[0], v1*P1[2]-P1[1], u2*P2[2]-P2[0], v2*P2[2]-P2[1]])
    _, _, Vt = np.linalg.svd(A)
    X = Vt[-1]; X = X / X[3]
    return check("三角化恢复3D点 (误差<0.01m)",
                 np.linalg.norm(X[:3] - true_pt) < 0.01,
                 f"error={np.linalg.norm(X[:3]-true_pt):.4f}")

def test_14_sobel_gradient():
    """Sobel 梯度方向"""
    from scipy.signal import convolve2d
    # 垂直线
    img = np.zeros((10, 10))
    img[:, :5] = 0; img[:, 5:] = 255
    Gx = convolve2d(img, np.array([[-1,0,1],[-2,0,2],[-1,0,1]]), mode='same')
    Gy = convolve2d(img, np.array([[-1,-2,-1],[0,0,0],[1,2,1]]), mode='same')
    # 垂直边缘处 Gx 应 >> Gy
    return check("垂直边缘处 |Gx| >> |Gy|",
                 abs(Gx[5,5]) > abs(Gy[5,5]) * 5,
                 f"Gx={Gx[5,5]:.0f}, Gy={Gy[5,5]:.0f}")

def test_15_harris_corner_response():
    """Harris 角点响应"""
    from scipy.signal import convolve2d
    # L型角点
    img = np.ones((20, 20)) * 100
    img[5:15, 5:15] = 200  # 正方形，角点在(5,5),(5,14),(14,5),(14,14)

    sx = np.array([[-1,0,1],[-2,0,2],[-1,0,1]], dtype=np.float64)
    Ix = convolve2d(img, sx, mode='same')
    Iy = convolve2d(img, sx.T, mode='same')
    Ix2, Iy2, Ixy = Ix**2, Iy**2, Ix*Iy

    from scipy.ndimage import gaussian_filter
    Sxx = gaussian_filter(Ix2, 1.0)
    Syy = gaussian_filter(Iy2, 1.0)
    Sxy = gaussian_filter(Ixy, 1.0)

    k = 0.04
    detM = Sxx*Syy - Sxy**2
    traceM = Sxx + Syy
    R = detM - k * traceM**2

    return check("角点处 Harris R > 0, 平坦处 R ≈ 0",
                 R[5,5] > 10 and abs(R[8,8]) < R[5,5] * 0.01,
                 f"R(角点)={R[5,5]:.0f}, R(平坦)={R[8,8]:.0f}")


if __name__ == "__main__":
    print("═" * 50)
    print("  Level 1 综合测试 — 基础篇")
    print("═" * 50)

    results = []
    results.append(test_01_vector_norm())
    results.append(test_02_dot_product())
    results.append(test_03_cross_product())
    results.append(test_04_rotation_matrix())
    results.append(test_05_svd_solve())
    results.append(test_06_eigen_decomposition())
    results.append(test_07_homogeneous_transform())
    results.append(test_08_gaussian_68_95_99())
    results.append(test_09_bayes())
    results.append(test_10_mle())
    results.append(test_11_pinhole_projection())
    results.append(test_12_epipolar_constraint())
    results.append(test_13_triangulation())
    results.append(test_14_sobel_gradient())
    results.append(test_15_harris_corner_response())

    print(f"\n{'═'*50}")
    print(f"  结果: {PASS_COUNT}/{TOTAL_TESTS} 通过")
    if PASS_COUNT == TOTAL_TESTS:
        print(f"  🎉 Level 1 完成! 可以进入 Level 2!")
        sys.exit(0)
    else:
        failed = TOTAL_TESTS - PASS_COUNT
        print(f"  ⚠️  还有 {failed} 个测试未通过，继续加油!")
        sys.exit(1)
