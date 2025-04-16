import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['font.sans-serif'] = ['Times New Roman']  # 设置字体为 Times New Roman
plt.rcParams.update({
    'font.size': 18,        # 全局字体
    'axes.titlesize': 20,   # 标题字体
    'axes.labelsize': 18,   # 坐标轴标签
    'xtick.labelsize': 16,  # X轴刻度
    'ytick.labelsize': 16,  # Y轴刻度
    'legend.fontsize': 18,  # 图例
})

# 从 JSON 读取地图
import json
with open("map/voxel_map.json", "r") as f:
    voxel_data = json.load(f)

resolution = voxel_data["resolution"]
size = voxel_data["size"]
voxel_array = np.array(voxel_data["data"])  # shape: [z][y][x]

# 获取所有被占用的 voxel 的坐标
occupied = np.argwhere(voxel_array == 1)  # 返回 (z, y, x) 坐标

# 分离成 x, y, z 坐标，注意转换成真实米制单位
zs, ys, xs = occupied[:, 0], occupied[:, 1], occupied[:, 2]
xs = xs * resolution + resolution / 2
ys = ys * resolution + resolution / 2
zs = zs * resolution + resolution / 2

# === 绘图 ===
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# ax.scatter(xs, ys, zs, c='red', marker='s', s=10)

# 所有方块的边长
dx = dy = dz = resolution

# bar3d 需要 x, y, z 是底部角落的位置（不是中心）
xs_bar = xs - resolution / 2
ys_bar = ys - resolution / 2
zs_bar = zs - resolution / 2

ax.bar3d(xs_bar, ys_bar, zs_bar, dx, dy, dz, color='gray', shade=True, alpha=0.8)





ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
# ax.set_title('3D Voxel Map from Webots Walls')

ax.set_xlim(0, size[0] * resolution)
ax.set_ylim(0, size[1] * resolution)
ax.set_zlim(0, size[2] * resolution)

# 设置 XYZ 等比例缩放
def set_axes_equal(ax):
    '''设置xyz轴等比例，同时Z轴从0开始'''
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]
    z_range = z_limits[1] - z_limits[0]

    max_range = max(x_range, y_range, z_range)

    # 保持 z 轴从 0 开始
    z_min = 0
    z_max = max_range

    # 以中心对称方式扩展 x 和 y
    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    x_min = x_middle - max_range / 2
    x_max = x_middle + max_range / 2
    y_min = y_middle - max_range / 2
    y_max = y_middle + max_range / 2

    ax.set_xlim3d(x_min, x_max)
    ax.set_ylim3d(y_min, y_max)
    ax.set_zlim3d(z_min, z_max)

# 添加到显示前

plt.tight_layout()
set_axes_equal(ax)

ax.view_init(elev=90, azim=-90)

plt.show()
