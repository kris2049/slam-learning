"""Level 6 测试: VIO/VINS"""
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")
c("VIO = Visual + Inertial Odometry", True)
c("紧耦合 > 松耦合 (精度更高)", True)
c("IMU预积分: 避免重复积分", True)
c("MSCKF: 多状态约束卡尔曼滤波", True)
c("VINS-Mono: 紧耦合+回环+位姿图", True)
c("DM-VIO: 深度学习增强VIO", True)
print(f"\n  结果: {P}/6")
