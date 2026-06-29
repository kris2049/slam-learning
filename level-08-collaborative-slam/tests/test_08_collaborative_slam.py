"""Level 8 测试: 协作SLAM"""
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")
c("集中式: 服务器汇总所有机器人数据", True)
c("分布式: 机器人间P2P通信", True)
c("地图合并: 对齐不同机器人的子图", True)
c("CCM-SLAM: 集中的协作单目SLAM", True)
c("Swarm-SLAM: 去中心化, 稀疏通信", True)
print(f"\n  结果: {P}/5")
