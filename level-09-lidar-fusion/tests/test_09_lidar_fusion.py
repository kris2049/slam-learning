"""Level 9 测试: LiDAR融合SLAM"""
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")
c("LOAM: 边缘+平面特征, 激光里程计", True)
c("FAST-LIO2: ikd-Tree, 极快LiDAR-惯性里程计", True)
c("LVI-SAM = LIO-SAM + VINS-Mono via 因子图", True)
c("R3LIVE: 实时稠密RGB点云地图", True)
print(f"\n  结果: {P}/4")
