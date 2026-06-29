
"""TSDF 体积融合 (Level 4)

KinectFusion 的核心: 将多帧深度图融合到统一的 TSDF 体积中。
"""
import numpy as np

def task_01_tsdf_basics():
    """TSDF: 截断符号距离函数"""
    print("\n═══ TSDF 体素融合 ═══")

    # 体素网格
    grid_size = 10
    voxel_size = 0.1
    tsdf = np.ones((grid_size, grid_size, grid_size))
    weight = np.zeros((grid_size, grid_size, grid_size))

    # 模拟传感器在 (0,0,0) 观测到前方 0.5m 处的平面
    sensor = np.array([0, 0, 0])
    surface_z = 0.5  # 平面在 z=0.5

    for z in range(grid_size):
        point_z = (z + 0.5) * voxel_size
        sdf = point_z - surface_z  # 有符号距离

        # 截断
        trunc_dist = 0.2
        if sdf > trunc_dist:
            tsdf[:, :, z] = 1.0
        elif sdf < -trunc_dist:
            tsdf[:, :, z] = -1.0
        else:
            tsdf[:, :, z] = sdf / trunc_dist

    surface_cells = np.sum(np.abs(tsdf) < 0.99)
    print(f"  体素总数: {grid_size**3}")
    print(f"  表面附近体素: {surface_cells} (零交叉面)")
    print(f"  TSDF 零交叉处 = 物体表面")

    # 体积融合 (多帧观测的加权平均)
    tsdf_new = np.random.randn(*tsdf.shape) * 0.1 + tsdf
    w_new = np.ones_like(weight)
    tsdf_fused = (tsdf * weight + tsdf_new * w_new) / (weight + w_new + 1e-8)

    print(f"  融合后 TSDF 均值: {tsdf_fused.mean():.3f}")
    assert surface_cells > 0
    print("  ✅ 通过!")

if __name__ == "__main__":
    task_01_tsdf_basics()
    print("\n🎉 TSDF 练习完成!")
