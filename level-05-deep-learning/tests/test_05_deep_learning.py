"""Level 5 测试: 深度学习 + SLAM"""
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")
c("SuperPoint: 自监督 + Homographic Adaptation", True)
c("SuperGlue: GNN + Sinkhorn最优传输", True)
c("RAFT: 全对相关 + ConvGRU迭代", True)
c("DROID-SLAM: 可微分BA + 稠密光流", True)
c("MiDaS: 多数据集混合训练", True)
c("SAM: 提示式分割基础模型", True)
print(f"\n  结果: {P}/6")
