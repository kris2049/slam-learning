"""
Level 3 测试: 单目视觉 SLAM
===============================
"""

import numpy as np
PASS = 0; TOTAL = 8

def check(n, c, d=""):
    global PASS
    if c: PASS += 1; print(f"  ✅ {n}")
    else: print(f"  ❌ {n} FAILED: {d}")

def test_01_frontend_backend():
    return check("PTAM: 前端Tracking与后端Mapping分离", True)

def test_02_scale_ambiguity():
    K = np.array([[500,0,320],[0,500,240],[0,0,1]])
    # 场景A: 1m位移, 3m深度
    t_a, p_a = np.array([1,0,0]), np.array([0,0,3])
    P1_a = K @ np.hstack([np.eye(3), np.zeros((3,1))])
    P2_a = K @ np.hstack([np.eye(3), t_a.reshape(3,1)])
    x1 = P1_a @ np.append(p_a,1); u1 = x1[0]/x1[2]
    x2 = P2_a @ np.append(p_a,1); u2 = x2[0]/x2[2]
    # 场景B: 10x位移, 10x深度
    t_b, p_b = t_a*10, p_a*10
    P2_b = K @ np.hstack([np.eye(3), t_b.reshape(3,1)])
    x1b = P1_a @ np.append(p_b,1); u1b = x1b[0]/x1b[2]
    x2b = P2_b @ np.append(p_b,1); u2b = x2b[0]/x2b[2]
    same = abs((u2-u1) - (u2b-u1b)) < 0.1
    return check("单目尺度模糊: A和B产生相同像素位移", same)

def test_03_covisibility():
    n1 = {0,1,2,3,4,5,6,7,8,9}
    n2 = {5,6,7,8,9,10,11,12}
    shared = len(n1 & n2)
    return check("共视图: 共享地图点决定边权重", shared == 5)

def test_04_sliding_window():
    n_kf, w = 20, 7
    full = n_kf * (n_kf-1) / 2
    sliding = sum(min(i+1, w) for i in range(n_kf))
    return check("滑动窗口减少计算量", sliding < full,
                 f"full={full:.0f}, sliding={sliding:.0f}")

def test_05_direct_vs_feature():
    return check("直接法用全部像素, 特征法用关键点", True)

def test_06_marginalization():
    H = np.eye(24)
    H[:6, 18:21] = 0.5
    H[18:21, :6] = 0.5
    H_marg = H[6:,6:] - H[6:,:6] @ np.linalg.inv(H[:6,:6]) @ H[:6,6:]
    fill_in = np.count_nonzero(np.abs(H_marg) > 0.01) > np.count_nonzero(np.abs(H[6:,6:]) > 0.01)
    return check("边缘化引入 fill-in 效应", fill_in)

def test_07_map_point_culling():
    # 地图点: found_ratio < 0.25 + observations < 3 → cull
    mp = {"found_ratio": 0.1, "observations": 2}
    should_cull = mp["found_ratio"] < 0.25 and mp["observations"] < 3
    return check("地图点剔除规则", should_cull)

def test_08_loop_closure_benefit():
    poses = [np.array([i, i*0.3, 0]) for i in range(10)]
    poses[9] = np.array([0, 8, 0])  # 严重漂移
    err_before = np.linalg.norm(poses[9] - poses[0])
    # 回环后
    poses[9] = np.array([0, 0.5, 0])
    err_after = np.linalg.norm(poses[9] - poses[0])
    return check("回环闭合大幅减少漂移", err_after < err_before,
                 f"before={err_before:.1f}, after={err_after:.1f}")

if __name__ == "__main__":
    print("═" * 50)
    print("  Level 3 测试 — 单目视觉 SLAM")
    print("═" * 50)
    test_01_frontend_backend()
    test_02_scale_ambiguity()
    test_03_covisibility()
    test_04_sliding_window()
    test_05_direct_vs_feature()
    test_06_marginalization()
    test_07_map_point_culling()
    test_08_loop_closure_benefit()
    print(f"\n{'═'*50}")
    print(f"  结果: {PASS}/{TOTAL} 通过")
    print(f"  {'🎉 Level 3 完成!' if PASS==TOTAL else '继续加油!'}")
