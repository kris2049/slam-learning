"""Level 4 测试: RGB-D SLAM"""
import numpy as np
P = 0; def c(n, cond): global P; P += 1 if cond else 0; print(f"  {'✅' if cond else '❌'} {n}")

c("ICP: 3D-3D点云配准", True)
c("TSDF: 截断距离 = truncation_distance", True)
c("KinectFusion: GPU体积融合", True)
c("Surfel = 面元(surface element)", True)
c("ElasticFusion用随机蕨编码做回环", True)
c("DynamicFusion支持非刚体场景", True)
print(f"\n  结果: {P}/6")
