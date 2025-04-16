from vedo import Points, show, Plotter, Text3D
import numpy as np
import json
font_name = "SimSun"  # 宋体的字体名称（Windows 内置）
with open("map/voxel_map.json", "r") as f:
    data = json.load(f)

resolution = data["resolution"]
vox = np.array(data["data"])
occupied = np.argwhere(vox == 1)

# 转成真实坐标
coords = occupied[:, [2, 1, 0]] * resolution + resolution / 2
pts = Points(coords).c("red")
show(pts, axes=1)
