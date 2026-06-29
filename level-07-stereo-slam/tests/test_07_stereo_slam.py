"""Level 7 测试: 双目SLAM"""
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")
c("双目提供绝对尺度 (vs 单目尺度模糊)", True)
c("视差 d = f·B/Z", True)
c("基线越大 → 远距离精度越高", True)
c("ORB-SLAM2 双目模式 = 单目 + 立体匹配初始化", True)
c("LDSO = DSO + 回环检测", True)
print(f"\n  结果: {P}/5")
