import re
import numpy as np
import json


wbt_file = "D:\Final_ws\Drone_sim\worlds\Drone_sim.wbt"     # 你的 .wbt 文件路径
# 读取 .wbt 文件
with open(wbt_file, 'r') as f:
    content = f.read()

# === 参数设置 ===
map_resolution = 0.5
map_x, map_y, map_z = 30, 30, 10  # 米


# 提取 Floor 的 size 用于地图尺寸
floor_pattern = re.compile(
    r'Floor\s*{[^}]*?size\s+([^\s]+)\s+([^\s]+)', 
    re.MULTILINE
)
floor_match = floor_pattern.search(content)
if floor_match:
    map_x = float(floor_match.group(1))
    map_y = float(floor_match.group(2))
    grid_x = int(map_x / map_resolution)
    grid_y = int(map_y / map_resolution)
    print(f"[INFO] Floor 尺寸提取成功: map_x={map_x}, map_y={map_y}")
else:
    print("[WARN] 未找到 Floor size，使用默认值")


grid_x = int(map_x / map_resolution)
grid_y = int(map_y / map_resolution)
grid_z = int(map_z / map_resolution)

# 初始化体素地图
voxel_map = np.zeros((grid_z, grid_y, grid_x), dtype=int)



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

