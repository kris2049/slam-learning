"""
实验 1: 从零实现针孔相机
==========================
目标: 理解 3D→2D 投影的完整流程，可视化相机模型。

运行: python3 experiment_01_camera_model.py
输出: camera_model.png — 3D场景投影到2D图像的示意图
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def experiment():
    print("═" * 60)
    print("  实验: 针孔相机模型 — 从3D世界到2D图像")
    print("═" * 60)

    # ── 1. 定义相机 ──
    fx, fy = 500, 500
    cx, cy = 320, 240
    img_w, img_h = 640, 480
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # 相机在世界坐标系中的位姿
    # 相机位置: [0, -1, 3], 朝向原点
    cam_pos = np.array([0.0, -1.0, 3.0])
    # 构造旋转: 相机朝前看
    forward = -cam_pos / np.linalg.norm(cam_pos)  # 朝向原点
    up_world = np.array([0.0, 0.0, 1.0])
    right = np.cross(up_world, forward)
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)

    R_cw = np.column_stack([right, up, forward])  # world → camera
    t_cw = -R_cw @ cam_pos
    Rt = np.hstack([R_cw, t_cw.reshape(3,1)])
    P = K @ Rt

    print(f"  相机位置: {cam_pos}")
    print(f"  相机朝向: {forward}")
    print(f"  投影矩阵 P = K[R|t]")
    print(f"  K = diag({fx}, {fy}), 主点({cx}, {cy})")

    # ── 2. 定义3D场景 ──
    # 一些3D点: 房间的8个角 + 几个物体
    room_size = 2.0
    corners = np.array([
        [-room_size, -room_size, 0, 1],
        [ room_size, -room_size, 0, 1],
        [ room_size,  room_size, 0, 1],
        [-room_size,  room_size, 0, 1],
        [-room_size, -room_size, room_size, 1],
        [ room_size, -room_size, room_size, 1],
        [ room_size,  room_size, room_size, 1],
        [-room_size,  room_size, room_size, 1],
    ])

    # 物体点
    object_points = np.array([
        [0.5, 0.5, 1.0, 1],   # 物体中心
        [0.5, 0.0, 1.0, 1],   # 物体底部
        [1.0, 0.5, 1.0, 1],   # 物体右侧
    ])

    all_3d = np.vstack([corners, object_points])
    labels = ['c0','c1','c2','c3','c4','c5','c6','c7','obj','obj_b','obj_r']

    # ── 3. 投影 ──
    pixels = []
    depths = []
    for p in all_3d:
        xh = P @ p
        u, v = xh[0]/xh[2], xh[1]/xh[2]
        pixels.append([u, v])
        depths.append(p[2])  # Z in world
    pixels = np.array(pixels)

    # ── 4. 可视化 ──
    fig = plt.figure(figsize=(14, 6))

    # 左图: 3D场景
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('3D 世界场景', fontsize=14)

    # 绘制房间
    room_edges = [
        (0,1),(1,2),(2,3),(3,0),  # 底面
        (4,5),(5,6),(6,7),(7,4),  # 顶面
        (0,4),(1,5),(2,6),(3,7),  # 垂直边
    ]
    for i, j in room_edges:
        ax1.plot([corners[i][0], corners[j][0]],
                 [corners[i][1], corners[j][1]],
                 [corners[i][2], corners[j][2]], 'gray', alpha=0.3)

    # 绘制所有点
    ax1.scatter(corners[:,0], corners[:,1], corners[:,2], c='blue', s=30, label='房间角')
    ax1.scatter(object_points[:,0], object_points[:,1], object_points[:,2],
               c='red', s=50, marker='^', label='物体')

    # 绘制相机
    ax1.scatter(*cam_pos, c='green', s=150, marker='*', label='相机')
    # 光轴
    axis_len = 1.5
    ax1.quiver(*cam_pos, *(-forward*axis_len), color='green', alpha=0.5, label='光轴')

    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.legend(loc='upper left')
    ax1.set_xlim([-3, 3]); ax1.set_ylim([-3, 3]); ax1.set_zlim([0, 4])

    # 右图: 2D图像 + 投影点
    ax2 = fig.add_subplot(122)
    ax2.set_title(f'2D 图像 ({img_w}×{img_h})', fontsize=14)
    ax2.set_xlim(0, img_w); ax2.set_ylim(img_h, 0)
    ax2.set_aspect('equal')
    ax2.set_facecolor('#f0f0f0')

    # 画图像边框
    ax2.add_patch(plt.Rectangle((0,0), img_w, img_h, fill=False, edgecolor='black'))

    # 画主点
    ax2.axvline(cx, color='gray', linestyle=':', alpha=0.3)
    ax2.axhline(cy, color='gray', linestyle=':', alpha=0.3)

    # 投影点
    for i, (u, v) in enumerate(pixels):
        color = 'blue' if i < 8 else 'red'
        marker = 'o' if i < 8 else '^'
        ax2.scatter(u, v, c=color, s=50, marker=marker, zorder=3)
        ax2.annotate(labels[i], (u, v), xytext=(5, 5), textcoords='offset points',
                    fontsize=8, alpha=0.7)

    # 连接房间在图像中的投影
    for i, j in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
        ax2.plot([pixels[i,0], pixels[j,0]], [pixels[i,1], pixels[j,1]],
                'gray', alpha=0.3, linewidth=0.5)

    plt.tight_layout()
    plt.savefig('/home/ubuntu/workspace/slam-learning/level-01-beginner/experiments/camera_model.png',
                dpi=120, bbox_inches='tight')
    plt.close()
    print(f"\n  📊 保存到 camera_model.png")
    print(f"  ✅ 实验完成!")

    # ── 5. 关键观察 ──
    print(f"\n  🔍 关键观察:")
    print(f"  1. 3D房间的8个角在图像中仍然可见")
    print(f"  2. 平行的3D线在图像中会汇聚（透视效应）")
    print(f"  3. 远的物体投影更靠近图像中心")
    print(f"  4. 相机前方无限远处的点投影到主点({cx},{cy})")
    print(f"  5. 物体在3D中的大小和它在图像中的大小成反比（近大远小）")


if __name__ == "__main__":
    experiment()
