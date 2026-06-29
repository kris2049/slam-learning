
"""深度学习特征 (Level 5)

理解 SuperPoint/SuperGlue 如何替代 SIFT/ORB。
"""
import numpy as np

def task_01_self_supervised_features():
    """自监督特征学习的概念"""
    print("\n═══ 深度学习特征 ═══")

    # SuperPoint 的核心思想: Homographic Adaptation
    # 对同一场景应用多个单应变换，要求特征检测器一致
    print("SuperPoint 训练流程:")
    print("  1. 在合成数据集上预训练 (MagicPoint)")
    print("  2. 真实图像 + 随机单应变换 → 自监督标签")
    print("  3. 共享编码器 + 检测/描述两个分支")
    print("")
    print("SuperGlue 匹配:")
    print("  • 图神经网络: 特征点=节点, 边=空间关系")
    print("  • Self-Attention: 增强特征 (上下文感知)")
    print("  • Cross-Attention: 寻找对应关系")
    print("  • Sinkhorn算法: 最优传输 → 软分配矩阵")
    print("")
    print("LightGlue (2023) 改进:")
    print("  • 自适应深度/宽度: 简单场景提前停止")
    print("  • 5-10x 加速, 精度持平")

    # 模拟 attention 匹配
    n_src, n_dst = 5, 7
    scores = np.random.randn(n_src, n_dst)
    scores = np.exp(scores) / np.exp(scores).sum(axis=1, keepdims=True)
    matches = np.argmax(scores, axis=1)
    print(f"\n  模拟匹配: {n_src}→{n_dst} 点, 分配: {matches}")

    assert len(matches) == n_src
    print("  ✅ 通过!")

if __name__ == "__main__":
    task_01_self_supervised_features()
    print("\n🎉 深度学习特征练习完成!")
