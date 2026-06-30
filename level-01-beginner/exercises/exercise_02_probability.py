"""
练习 2: 概率与统计 (交互式)
=============================
把 # TODO 改成正确代码，跑通为止。
"""

import numpy as np

def task_01_gaussian():
    """任务1: 验证高斯分布的 68-95 规则"""
    print("\n═══ 任务1: 高斯分布 ═══")

    mu, sigma = 0.0, 1.0
    samples = np.random.normal(mu, sigma, 10000)

    # TODO: 计算 1σ, 2σ, 3σ 范围内的样本比例
    within_1sigma = 0  # TODO: np.mean(np.abs(samples - mu) < sigma)
    within_2sigma = 0  # TODO: np.mean(np.abs(samples - mu) < 2*sigma)

    print(f"  1σ 内: {within_1sigma:.3f} (理论 0.683)")
    print(f"  2σ 内: {within_2sigma:.3f} (理论 0.954)")

    ok = True
    if within_1sigma < 0.65 or within_1sigma > 0.72:
        print("  ❌ 1σ 不对。提示: np.mean(np.abs(samples) < sigma)"); ok = False
    if within_2sigma < 0.93:
        print("  ❌ 2σ 不对。"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_02_multivariate_gaussian():
    """任务2: 多元高斯分布"""
    print("\n═══ 任务2: 多元高斯 ═══")

    mean = np.array([0.0, 0.0])
    cov = np.array([[2.0, 1.0], [1.0, 1.5]])

    samples = np.random.multivariate_normal(mean, cov, 5000)

    # TODO: 计算样本均值和协方差矩阵
    sample_mean = np.zeros(2)    # TODO: np.mean(samples, axis=0)
    sample_cov  = np.zeros((2,2))  # TODO: np.cov(samples.T)

    print(f"  样本均值: {sample_mean}")
    print(f"  样本协方差:\n{sample_cov}")

    ok = True
    if np.linalg.norm(sample_mean - mean) > 0.1:
        print("  ❌ 均值不对。提示: np.mean(samples, axis=0)"); ok = False
    if abs(sample_cov[0, 0] - 2.0) > 0.5:
        print("  ❌ 协方差不对。提示: np.cov(samples.T)"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_03_bayes():
    """任务3: 贝叶斯定理

    机器人检测门。已知:
      P(门前) = 0.3         (先验)
      P(检测到|门前) = 0.9   (真阳性率)
      P(检测到|非门前) = 0.1 (假阳性率)
    求: 检测到一次后，门存在的后验概率。
    """
    print("\n═══ 任务3: 贝叶斯定理 ═══")

    prior = 0.3
    tp_rate = 0.9
    fp_rate = 0.1

    # TODO: P(检测到) = P(检测|门)*P(门) + P(检测|非门)*P(非门)
    p_evidence = 0  # TODO: tp_rate * prior + fp_rate * (1-prior)

    # TODO: 后验 = P(检测|门)*P(门) / P(检测)
    posterior = 0  # TODO: tp_rate * prior / p_evidence

    print(f"  后验 P(门|检测) = {posterior:.4f} (应≈0.794)")

    ok = True
    if abs(posterior - 0.794) > 0.02:
        print("  ❌ 贝叶斯公式: P(A|B) = P(B|A)*P(A) / P(B)"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


def task_04_mle():
    """任务4: 最大似然估计

    传感器测量真实距离 10m，噪声标准差 0.3m。
    求 10m 的 MLE 估计。
    """
    print("\n═══ 任务4: MLE ═══")

    true_distance = 10.0
    measurements = true_distance + np.random.normal(0, 0.3, 50)

    # TODO: 高斯噪声下的 MLE 就是样本均值
    mu_mle = 0  # TODO: np.mean(measurements)

    print(f"  真值: {true_distance}m")
    print(f"  MLE:  {mu_mle:.3f}m")
    print(f"  误差: {abs(mu_mle - true_distance):.3f}m")

    ok = True
    if abs(mu_mle - true_distance) > 0.3:
        print("  ❌ MLE 估计偏差太大。提示: np.mean(measurements)"); ok = False

    if ok: print("  ✅ 通过!")
    return ok


if __name__ == "__main__":
    results = [task_01_gaussian(), task_02_multivariate_gaussian(),
               task_03_bayes(), task_04_mle()]
    passed = sum(results)
    print(f"\n{'='*40}")
    print(f"  通过: {passed}/{len(results)}")
    if passed == len(results):
        print("  🎉 全部完成!")
    else:
        print(f"  还有 {len(results)-passed} 个任务，改完重跑。")
