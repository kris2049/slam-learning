"""
练习 2: 概率与统计基础
=========================
SLAM 中的状态估计、不确定性量化都依赖概率论。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def task_01_gaussian():
    """任务1: 高斯分布 (10分钟)

    SLAM 中最核心的概率分布。测量噪声、状态不确定性都建模为高斯分布。
    """
    print("\n═══ 任务1: 高斯分布 ═══")

    # 一维高斯: p(x) = 1/(σ√(2π)) * exp(-(x-μ)²/(2σ²))
    mu, sigma = 0.0, 1.0

    # 采样
    samples = np.random.normal(mu, sigma, 10000)

    # 验证 68-95-99.7 规则
    within_1sigma = np.mean(np.abs(samples - mu) < sigma)
    within_2sigma = np.mean(np.abs(samples - mu) < 2 * sigma)
    within_3sigma = np.mean(np.abs(samples - mu) < 3 * sigma)

    print(f"  1σ 内的样本比例: {within_1sigma:.3f} (理论: 0.683)")
    print(f"  2σ 内的样本比例: {within_2sigma:.3f} (理论: 0.954)")
    print(f"  3σ 内的样本比例: {within_3sigma:.3f} (理论: 0.997)")

    assert within_1sigma > 0.65, "68% 应在 1σ 内"
    assert within_2sigma > 0.93, "95% 应在 2σ 内"
    print("  ✅ 通过!")

    # 绘制概率密度
    x = np.linspace(-4, 4, 200)
    pdf = 1/(sigma * np.sqrt(2*np.pi)) * np.exp(-(x-mu)**2 / (2*sigma**2))

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(x, pdf, 'b-', linewidth=2, label=f'𝒩({mu}, {sigma}²)')
    ax.hist(samples, bins=50, density=True, alpha=0.4, color='blue')
    ax.axvline(mu, color='red', linestyle='--', label=f'μ={mu}')
    ax.axvspan(mu-sigma, mu+sigma, alpha=0.15, color='green', label='±1σ')
    ax.legend()
    ax.set_title('高斯分布 𝒩(0,1) — SLAM 的数学基石')
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-01-beginner/exercises/gaussian.png')
    plt.close()
    print("  📊 保存到 gaussian.png")


def task_02_multivariate_gaussian():
    """任务2: 多元高斯分布 (10分钟)

    SLAM 中的状态向量是多维的，协方差矩阵描述各维度间的关系。
    """
    print("\n═══ 任务2: 多元高斯分布 ═══")

    mean = np.array([0.0, 0.0])
    # 确保协方差矩阵正定: 使用 Cholesky 友好的形式
    cov = np.array([
        [2.0, 1.0],
        [1.0, 1.5]
    ])

    samples = np.random.multivariate_normal(mean, cov, 5000)

    # 验证样本均值和协方差
    sample_mean = np.mean(samples, axis=0)
    sample_cov = np.cov(samples.T)

    print(f"  样本均值: {sample_mean} (理论: [0, 0])")
    print(f"  样本协方差:\n{sample_cov}")
    print(f"  理论协方差:\n{cov}")

    assert np.allclose(sample_mean, mean, atol=0.05), "均值偏差过大"
    print("  ✅ 通过!")

    # 绘制2D高斯椭圆
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(samples[:, 0], samples[:, 1], alpha=0.15, s=2)

    # 绘制1σ和2σ置信椭圆
    from numpy.linalg import eigh
    for n_std in [1, 2]:
        eigvals, eigvecs = eigh(cov * n_std**2)
        angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
        width, height = 2 * np.sqrt(eigvals)
        from matplotlib.patches import Ellipse
        ellipse = Ellipse(xy=mean, width=width, height=height,
                          angle=angle, facecolor='none', edgecolor='red',
                          linewidth=1.5, alpha=0.5)
        ax.add_patch(ellipse)
        ax.text(mean[0] + width/2 + 0.2, mean[1], f'{n_std}σ', color='red')

    ax.set_aspect('equal')
    ax.set_title('多元高斯 — SLAM 不确定性的可视化')
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-01-beginner/exercises/multivariate_gaussian.png')
    plt.close()
    print("  📊 保存到 multivariate_gaussian.png")


def task_03_bayes():
    """任务3: 贝叶斯定理 (15分钟)

    SLAM 本质上是一个贝叶斯推断问题: P(地图, 位姿 | 观测)
    """
    print("\n═══ 任务3: 贝叶斯定理 ═══")

    # 场景：机器人检测门。已知:
    # P(门前|门前真实) = 0.9 (true positive rate)
    # P(门前|非门前) = 0.1 (false positive rate)
    # P(门前真实) = 0.3 (先验 — 约30%的场景有门)

    prior = 0.3
    tp_rate = 0.9
    fp_rate = 0.1

    # TODO: 用贝叶斯定理计算 P(门前真实 | 检测到门前)
    # P(A|B) = P(B|A) * P(A) / P(B)
    # P(B) = P(B|A)*P(A) + P(B|¬A)*P(¬A)
    p_B_given_A = tp_rate
    p_B_given_not_A = fp_rate
    p_not_A = 1 - prior

    p_B = p_B_given_A * prior + p_B_given_not_A * p_not_A
    posterior = p_B_given_A * prior / p_B

    print(f"  先验 P(门) = {prior:.2f}")
    print(f"  P(检测|门) = {tp_rate:.2f}")
    print(f"  P(检测|非门) = {fp_rate:.2f}")
    print(f"  后验 P(门|检测) = {posterior:.4f}")
    print(f"  解释: 检测到「门」后，门存在的概率从 30% 上升到 {posterior*100:.1f}%")

    # SLAM 中: 随着更多观测，后验不断更新
    # 第二次独立检测也显示「门」
    prior2 = posterior
    p_B2 = tp_rate * prior2 + fp_rate * (1 - prior2)
    posterior2 = tp_rate * prior2 / p_B2
    print(f"  第二次检测后: P(门|两次检测) = {posterior2:.4f}")

    assert posterior > prior, "后验应该比先验更确定"
    assert posterior2 > posterior, "更多观测应该更确定"
    print("  ✅ 通过!")


def task_04_maximum_likelihood():
    """任务4: 最大似然估计 MLE (15分钟)

    SLAM 中，Bundle Adjustment 就是在求最大似然(或最大后验)。
    """
    print("\n═══ 任务4: 最大似然估计 ═══")

    # 模拟: 用带噪声的传感器测量一个固定距离（真值=10m）
    true_distance = 10.0
    noise_std = 0.3
    measurements = true_distance + np.random.normal(0, noise_std, 50)

    # TODO: MLE 估计 — 使似然最大的 μ 就是样本均值
    mu_mle = np.mean(measurements)
    sigma_mle = np.std(measurements, ddof=0)

    print(f"  真值: {true_distance}m")
    print(f"  MLE 估计 μ: {mu_mle:.3f}m")
    print(f"  MLE 估计 σ: {sigma_mle:.3f}m")
    print(f"  误差: {abs(mu_mle - true_distance):.3f}m")

    # 似然函数可视化
    mu_range = np.linspace(9.0, 11.0, 200)
    log_likelihood = [-0.5 * np.sum((measurements - mu)**2) / noise_std**2 for mu in mu_range]

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(mu_range, log_likelihood, 'b-')
    ax.axvline(mu_mle, color='red', linestyle='--', label=f'MLE μ={mu_mle:.3f}')
    ax.axvline(true_distance, color='green', linestyle=':', label=f'真值={true_distance}')
    ax.set_xlabel('μ (距离估计)')
    ax.set_ylabel('对数似然')
    ax.legend()
    ax.set_title('MLE 示例 — 类似 SLAM 中的 Bundle Adjustment')
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-01-beginner/exercises/mle.png')
    plt.close()
    print("  📊 保存到 mle.png")
    print("  ✅ 通过!")


if __name__ == "__main__":
    task_01_gaussian()
    task_02_multivariate_gaussian()
    task_03_bayes()
    task_04_maximum_likelihood()
    print("\n🎉 所有概率练习完成!")
