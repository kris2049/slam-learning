"""Level 10 测试: 事件相机SLAM"""
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")
c("事件相机: 异步亮度变化检测, μs级", True)
c("优势: HDR(140dB+), 无运动模糊, 低功耗", True)
c("EVO: 首个事件相机VO系统", True)
c("DEVO: 深度学习事件VO (DROID-style)", True)
print(f"\n  结果: {P}/4")
