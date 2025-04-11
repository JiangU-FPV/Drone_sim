import re
import numpy as np
import json

# === 参数设置 ===
wbt_file = "D:\Final_ws\Drone_sim\worlds\Drone_sim.wbt"     # 你的 .wbt 文件路径
map_resolution = 0.5
map_x, map_y, map_z = 30, 30, 10  # 米
grid_x = int(map_x / map_resolution)
grid_y = int(map_y / map_resolution)
grid_z = int(map_z / map_resolution)

# 初始化体素地图
voxel_map = np.zeros((grid_z, grid_y, grid_x), dtype=int)

# 读取 .wbt 文件
with open(wbt_file, 'r') as f:
    content = f.read()

# 提取 Wall 的 translation 和 size
wall_pattern = re.compile(
    r'Wall\s*{[^}]*?translation\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)[^}]*?size\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)[^}]*?}', 
    re.MULTILINE
)
walls = wall_pattern.findall(content)
print(f"[INFO] 提取到 {len(walls)} 个 Wall.")

for idx, wall in enumerate(walls):
    cx, cy, cz = map(float, wall[:3])
    sx, sy, sz = map(float, wall[3:])

    x_min, x_max = cx - sx / 2, cx + sx / 2
    y_min, y_max = cy - sy / 2, cy + sy / 2
    z_min, z_max = cz, cz + sz

    ix_min = int(x_min / map_resolution)
    ix_max = int(x_max / map_resolution)
    iy_min = int(y_min / map_resolution)
    iy_max = int(y_max / map_resolution)
    iz_min = int(z_min / map_resolution)
    iz_max = int(z_max / map_resolution)

    for i in range(ix_min, ix_max + 1):
        for j in range(iy_min, iy_max + 1):
            for k in range(iz_min, iz_max + 1):
                if 0 <= i < grid_x and 0 <= j < grid_y and 0 <= k < grid_z:
                    voxel_map[k][j][i] = 1

# 转换为 JSON 格式
voxel_dict = {
    "resolution": map_resolution,
    "size": [grid_x, grid_y, grid_z],
    "data": voxel_map.tolist()
}

# 写入 JSON 文件
with open("map/voxel_map.json", "w") as f:
    json.dump(voxel_dict, f, indent=2)

print("[OK] 已生成 voxel_map.json ✅")

