import math
import json
import numpy as np
def generate_figure_eight(radius=5, height=3, resolution=0.2):
    # 原始采样 - 高密度，用于计算路径长度
    t_dense = np.linspace(0, 2 * np.pi, 10000)
    x_dense = radius * np.sin(t_dense)
    y_dense = radius * np.sin(t_dense) * np.cos(t_dense)
    z_dense = np.full_like(t_dense, height)

    # 计算累积路径长度
    dx = np.diff(x_dense)
    dy = np.diff(y_dense)
    dz = np.diff(z_dense)
    segment_lengths = np.sqrt(dx**2 + dy**2 + dz**2)
    arc_lengths = np.concatenate(([0], np.cumsum(segment_lengths)))

    # 按照固定间距采样
    total_length = arc_lengths[-1]
    num_points = int(total_length // resolution) + 1
    target_lengths = np.linspace(0, total_length, num_points)

    # 使用插值找到均匀间距对应的坐标点
    x_uniform = np.interp(target_lengths, arc_lengths, x_dense)
    y_uniform = np.interp(target_lengths, arc_lengths, y_dense)
    z_uniform = np.interp(target_lengths, arc_lengths, z_dense)

    path = [[float(x), float(y), float(z)] for x, y, z in zip(x_uniform, y_uniform, z_uniform)]

    return {
        "resolution": resolution,
        "path": path
    }
def generate_two_circles(radius=3, height=3, resolution=0.15):
    # 点的数量（按弧长决定）
    num_points = int(2 * np.pi * radius / resolution)

    # 左圆：从 (0,0) 逆时针一圈，圆心在 (-radius, 0)
    theta_left = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x_left = -radius + radius * np.cos(theta_left)
    y_left = radius * np.sin(theta_left)
    z_left = np.full_like(x_left, height)

    # 右圆：从 (0,0) 顺时针一圈，圆心在 (radius, 0)
    theta_right = np.linspace(0, -2 * np.pi, num_points, endpoint=False)
    x_right = radius - radius * np.cos(theta_right)
    y_right = -radius * np.sin(theta_right)
    z_right = np.full_like(x_right, height)

    # 拼接路径：左圆 + 右圆（从切点(0,0)开始并连续）
    x = np.concatenate([x_left, x_right])
    y = np.concatenate([y_left, y_right])
    z = np.concatenate([z_left, z_right])

    path = [[float(xi), float(yi), float(zi)] for xi, yi, zi in zip(x, y, z)]

    return {
        "resolution": resolution,
        "path": path
    }
# 生成路径数据
figure_eight_data = generate_two_circles()

# 保存为JSON文件
with open("map/figure_eight_path.json", "w") as f:
    json.dump(figure_eight_data, f, indent=2)

# 可选：打印输出以查看
