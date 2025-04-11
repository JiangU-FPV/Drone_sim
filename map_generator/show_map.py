import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

ax.scatter(xs, ys, zs, c='red', marker='s', s=10)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Voxel Map from Webots Walls')

ax.set_xlim(0, size[0] * resolution)
ax.set_ylim(0, size[1] * resolution)
ax.set_zlim(0, size[2] * resolution)
plt.tight_layout()
plt.show()
