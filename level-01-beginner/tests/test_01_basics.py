"""
Level 1 综合测试 (交互式)
=============================
每道题补全代码或回答，15题全对进入 Level 2。
"""

import numpy as np

PASS = 0; TOTAL = 15

def check(name, ok, hint=""):
    global PASS
    if ok: PASS += 1; print(f"  ✅ {name}")
    else: print(f"  ❌ {name} | 提示: {hint}")


def t01():
    """向量的 L2 范数"""
    v = np.array([3.0, 4.0, 0.0])
    n = 0  # TODO: 填正确代码
    check("向量 |[3,4,0]| = 5", abs(n - 5.0) < 0.01,
          "np.linalg.norm(v)")

def t02():
    """点积与正交"""
    a = np.array([1, 0, 0]); b = np.array([0, 1, 0])
    d = 0  # TODO
    check("正交向量点积=0", abs(d) < 0.01, "np.dot(a, b)")

def t03():
    """叉积"""
    a = np.array([1, 0, 0]); b = np.array([0, 1, 0])
    c = np.zeros(3)  # TODO
    check("叉积 a×b=[0,0,1]", np.allclose(c, [0,0,1]),
          "np.cross(a, b)")

def t04():
    """旋转矩阵性质"""
    theta = np.radians(45)
    R = np.zeros((2,2))  # TODO: 构造旋转矩阵
    detR = np.linalg.det(R)
    check("det(R)=1", abs(detR-1)<0.01,
          "R=[[cosθ,-sinθ],[sinθ,cosθ]]")

def t05():
    """SVD 解方程"""
    A = np.array([[1,2],[3,4],[5,6]]); b = np.array([7,8,9])
    x = np.zeros(2)  # TODO: 最小二乘解
    x_np = np.linalg.lstsq(A, b, rcond=None)[0]
    check("SVD最小二乘", np.allclose(x, x_np, rtol=1e-4),
          "U,S,Vt=np.linalg.svd(A); x=Vt.T@np.diag(1/S)@U.T@b")

def t06():
    """特征值验证"""
    A = np.array([[2,1],[1,2]])
    eigvals = np.zeros(2)  # TODO
    check("特征值之和=迹", abs(sum(eigvals)-np.trace(A))<0.01,
          "np.linalg.eigvals(A)")

def t07():
    """齐次变换"""
    T = np.eye(4)
    T[:3, :3] = np.array([[0,-1,0],[1,0,0],[0,0,1]])  # 绕Z 90°
    T[:3, 3] = [1,2,3]
    p = np.array([1,0,0,1.0])
    pt = np.zeros(4)  # TODO: T @ p
    check("齐次变换 [1,0,0]→[1,3,3]", np.allclose(pt[:3], [1,3,3]),
          "pt = T @ p")

def t08():
    """68-95规则"""
    s = np.random.normal(0,1,50000)
    w1 = 0  # TODO: np.mean(np.abs(s) < 1)
    check("1σ≈68%", w1 > 0.67 and w1 < 0.70,
          "np.mean(np.abs(s) < 1)")

def t09():
    """贝叶斯"""
    prior, sens, spec = 0.01, 0.99, 0.95
    post = 0  # TODO: P(病|+) = sens*prior/(sens*prior+(1-spec)*(1-prior))
    check("贝叶斯后验≈0.166", abs(post-0.166)<0.02,
          "post = sens*prior / (sens*prior + (1-spec)*(1-prior))")

def t10():
    """MLE"""
    measurements = 5.0 + np.random.normal(0,0.5,1000)
    mle = 0  # TODO
    check("MLE≈5.0", abs(mle-5)<0.1, "np.mean(measurements)")

def t11():
    """针孔投影"""
    K = np.array([[600,0,320],[0,600,240],[0,0,1]])
    P = K @ np.hstack([np.eye(3), np.zeros((3,1))])
    X = np.array([0,0,5,1.0]); xh = P @ X
    u = 0  # TODO: xh[0]/xh[2]
    v = 0  # TODO: xh[1]/xh[2]
    check("正前方→主点", abs(u-320)<1 and abs(v-240)<1,
          "u=xh[0]/xh[2]; v=xh[1]/xh[2]")

def t12():
    """对极约束"""
    t = np.array([0.5, 0, 0])
    tx = np.array([[0,-t[2],t[1]],[t[2],0,-t[0]],[-t[1],t[0],0]])
    p1 = np.array([300,240,1.0]); p2 = np.array([280,240,1.0])
    err = 999  # TODO: p2 @ tx @ p1
    check("对极约束≈0", abs(err)<0.01, "err = p2 @ tx @ p1")

def t13():
    """三角化"""
    K = np.array([[500,0,320],[0,500,240],[0,0,1]])
    P1 = K @ np.hstack([np.eye(3), np.zeros((3,1))])
    P2 = K @ np.hstack([np.eye(3), np.array([[1],[0],[0]])])
    true_pt = np.array([0.5, 0.5, 3.0])
    x1h = P1 @ np.append(true_pt,1); u1=x1h[0]/x1h[2]; v1=x1h[1]/x1h[2]
    x2h = P2 @ np.append(true_pt,1); u2=x2h[0]/x2h[2]; v2=x2h[1]/x2h[2]
    A = np.array([u1*P1[2]-P1[0], v1*P1[2]-P1[1],
                  u2*P2[2]-P2[0], v2*P2[2]-P2[1]])
    X = np.zeros(4)  # TODO: SVD取最后一列, X/=X[3]
    check("三角化误差<0.01", np.linalg.norm(X[:3]-true_pt)<0.01,
          "_,_,Vt=np.linalg.svd(A); X=Vt[-1]; X/=X[3]")

def t14():
    """Sobel梯度"""
    img = np.zeros((10,10)); img[:,5:] = 255
    from scipy.signal import convolve2d
    sx = np.array([[-1,0,1],[-2,0,2],[-1,0,1]])
    Gx = convolve2d(img, sx, mode='same')
    check("垂直边缘|Gx|>|Gy|", abs(Gx[5,5]) > 10 and abs(Gx[5,5]) > abs(Gx[5,5]*0.1),
          "边缘处水平梯度应远大于垂直梯度")

def t15():
    """Harris判据"""
    from scipy.signal import convolve2d
    from scipy.ndimage import gaussian_filter
    img = np.ones((20,20))*100; img[5:15,5:15]=200
    sx = np.array([[-1,0,1],[-2,0,2],[-1,0,1]], dtype=float)
    Ix = convolve2d(img, sx, mode='same')
    Iy = convolve2d(img, sx.T, mode='same')
    Sxx = gaussian_filter(Ix**2, 1.0); Syy = gaussian_filter(Iy**2, 1.0)
    Sxy = gaussian_filter(Ix*Iy, 1.0)
    detM = 0   # TODO: Sxx*Syy - Sxy**2
    traceM = 0  # TODO: Sxx + Syy
    R = detM - 0.04*traceM**2
    check("角点R>0", R > 0,
          "detM=Sxx*Syy-Sxy**2; traceM=Sxx+Syy; 角点处Ix,Iy都大→detM>>0")


if __name__ == "__main__":
    print("═" * 50)
    print("  Level 1 综合测试 — 15 题，改 TODO 直到全对")
    print("═" * 50)
    t01(); t02(); t03(); t04(); t05(); t06(); t07(); t08()
    t09(); t10(); t11(); t12(); t13(); t14(); t15()
    print(f"\n{'═'*50}")
    print(f"  结果: {PASS}/{TOTAL} 通过")
    if PASS == TOTAL:
        print(f"  🎉 Level 1 完成! 可以进入 Level 2!")
    else:
        print(f"  ⚠️  还有 {TOTAL-PASS} 题未通过")
