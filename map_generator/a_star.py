import numpy as np
import heapq
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import binary_dilation
from scipy.interpolate import CubicSpline
from scipy.interpolate import BSpline
plt.rcParams['font.sans-serif'] = ['SimSun']
# 全局字体大小设置
plt.rcParams.update({
    'font.size': 18,        # 全局字体
    'axes.titlesize': 20,   # 标题字体
    'axes.labelsize': 18,   # 坐标轴标签
    'xtick.labelsize': 16,  # X轴刻度
    'ytick.labelsize': 16,  # Y轴刻度
    'legend.fontsize': 18,  # 图例
})
def smooth_path_with_bspline(path_voxels, resolution, degree=3):
    # 将路径点转为列表
    path_voxels = np.array(path_voxels)
    path_x, path_y, path_z = path_voxels[:, 0], path_voxels[:, 1], path_voxels[:, 2]
    
    # 创建节点向量
    n = len(path_voxels)
    knot_vector = np.concatenate(([0] * degree, np.linspace(0, 1, n - degree + 1), [1] * degree))

    # 构造B样条
    spline_x = BSpline(knot_vector, path_x, degree)
    spline_y = BSpline(knot_vector, path_y, degree)
    spline_z = BSpline(knot_vector, path_z, degree)

    # 生成更多插值点
    t_new = np.linspace(0, 1, 10 * len(path_x))
    smooth_x = spline_x(t_new)
    smooth_y = spline_y(t_new)
    smooth_z = spline_z(t_new)

    # 转换为世界坐标（米）
    smooth_x = smooth_x * resolution + resolution / 2
    smooth_y = smooth_y * resolution + resolution / 2
    smooth_z = smooth_z * resolution + resolution / 2

    return smooth_x, smooth_y, smooth_z

def smooth_path(path_voxels, resolution):
    if len(path_voxels) < 4:
        return path_voxels  # 太短就不处理了

    path_voxels = np.array(path_voxels)
    t = np.arange(len(path_voxels))  # 参数化

    # 分别对x、y、z插值
    cs_x = CubicSpline(t, path_voxels[:, 0])
    cs_y = CubicSpline(t, path_voxels[:, 1])
    cs_z = CubicSpline(t, path_voxels[:, 2])

    # 更细的插值点
    t_new = np.linspace(0, len(path_voxels) - 1, 5 * len(path_voxels))
    x_new = cs_x(t_new) * resolution + resolution / 2
    y_new = cs_y(t_new) * resolution + resolution / 2
    z_new = cs_z(t_new) * resolution + resolution / 2

    return x_new, y_new, z_new

def smooth_path_with_spline(path_voxels, resolution):
    # 将路径点转为列表
    path_voxels = np.array(path_voxels)
    path_x, path_y, path_z = path_voxels[:, 0], path_voxels[:, 1], path_voxels[:, 2]
    
    # 插值计算
    spline_x = CubicSpline(range(len(path_x)), path_x, bc_type='clamped')  # 使用clamped边界条件
    spline_y = CubicSpline(range(len(path_y)), path_y, bc_type='clamped')
    spline_z = CubicSpline(range(len(path_z)), path_z, bc_type='clamped')

    # 生成更多插值点（曲线平滑）
    t_new = np.linspace(0, len(path_x) - 1, 10 * len(path_x))
    smooth_x = spline_x(t_new)
    smooth_y = spline_y(t_new)
    smooth_z = spline_z(t_new)

    # 转换为世界坐标（米）
    smooth_x = smooth_x * resolution + resolution / 2
    smooth_y = smooth_y * resolution + resolution / 2
    smooth_z = smooth_z * resolution + resolution / 2

    return smooth_x, smooth_y, smooth_z

def smooth_path_with_fewer_points(path_voxels, resolution):
    # 将路径点转为列表
    path_voxels = np.array(path_voxels)
    path_x, path_y, path_z = path_voxels[:, 0], path_voxels[:, 1], path_voxels[:, 2]

    # 减少插值点的数量（改变`n_points`）
    n_points = 10  # 插值点数，减小这个值，转弯会更圆滑

    # 使用CubicSpline进行插值
    spline_x = CubicSpline(range(len(path_x)), path_x, bc_type='clamped')
    spline_y = CubicSpline(range(len(path_y)), path_y, bc_type='clamped')
    spline_z = CubicSpline(range(len(path_z)), path_z, bc_type='clamped')

    # 生成更多插值点
    t_new = np.linspace(0, len(path_x) - 1, n_points * len(path_x))
    smooth_x = spline_x(t_new)
    smooth_y = spline_y(t_new)
    smooth_z = spline_z(t_new)

    # 转换为世界坐标（米）
    smooth_x = smooth_x * resolution + resolution / 2
    smooth_y = smooth_y * resolution + resolution / 2
    smooth_z = smooth_z * resolution + resolution / 2

    return smooth_x, smooth_y, smooth_z

# def smooth_path_with_catmull_rom(path_voxels, resolution, samples_per_segment=20):
#     path_voxels = np.array(path_voxels)
#     n = len(path_voxels)

#     if n < 4:
#         raise ValueError("路径点数量太少，至少需要4个点用于 Catmull-Rom 插值")

#     def catmull_rom(p0, p1, p2, p3, n=20):
#         t = np.linspace(0, 1, int(n))[:, None]
#         M = 0.5 * np.array([
#             [-1, 3, -3, 1],
#             [ 2, -5, 4, -1],
#             [-1, 0, 1, 0],
#             [ 0, 2, 0, 0]
#         ])
#         G = np.vstack([p0, p1, p2, p3])
#         T = np.hstack([t**3, t**2, t, np.ones_like(t)])
#         return (T @ M @ G).astype(float)

#     smooth_points = []
#     for i in range(1, n - 2):
#         segment = catmull_rom(path_voxels[i-1], path_voxels[i], path_voxels[i+1], path_voxels[i+2], n=samples_per_segment)
#         smooth_points.append(segment)

#     smooth_points = np.vstack(smooth_points)

#     # 转换为世界坐标（米）
#     smooth_x = smooth_points[:, 0] * resolution + resolution / 2
#     smooth_y = smooth_points[:, 1] * resolution + resolution / 2
#     smooth_z = smooth_points[:, 2] * resolution + resolution / 2

#     return smooth_x, smooth_y, smooth_z

def smooth_path_with_catmull_rom(path_voxels, resolution, samples_per_segment=20):
    path_voxels = np.array(path_voxels)
    n = len(path_voxels)

    if n < 4:
        raise ValueError("路径点数量太少，至少需要4个点用于 Catmull-Rom 插值")

    def catmull_rom(p0, p1, p2, p3, n=20):
        t = np.linspace(0, 1, int(n))[:, None]
        M = 0.5 * np.array([
            [-1, 3, -3, 1],
            [ 2, -5, 4, -1],
            [-1, 0, 1, 0],
            [ 0, 2, 0, 0]
        ])
        G = np.vstack([p0, p1, p2, p3])
        T = np.hstack([t**3, t**2, t, np.ones_like(t)])
        return (T @ M @ G).astype(float)

    smooth_points = []

    # 首段（使用前两个点复制填充）
    smooth_points.append(
        catmull_rom(path_voxels[0], path_voxels[0], path_voxels[1], path_voxels[2], n=samples_per_segment)
    )

    # 中间段
    for i in range(1, n - 2):
        segment = catmull_rom(path_voxels[i-1], path_voxels[i], path_voxels[i+1], path_voxels[i+2], n=samples_per_segment)
        smooth_points.append(segment)

    # 尾段（使用最后两个点复制填充）
    smooth_points.append(
        catmull_rom(path_voxels[-3], path_voxels[-2], path_voxels[-1], path_voxels[-1], n=samples_per_segment)
    )

    # 合并并转换为世界坐标
    smooth_points = np.vstack(smooth_points)
    smooth_x = smooth_points[:, 0] * resolution + resolution / 2
    smooth_y = smooth_points[:, 1] * resolution + resolution / 2
    smooth_z = smooth_points[:, 2] * resolution + resolution / 2

    return smooth_x, smooth_y, smooth_z


def rdp_simplify(points, epsilon):
    def point_line_distance(point, start, end):
        if np.all(start == end):
            return np.linalg.norm(point - start)
        return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

    def rdp(points, epsilon):
        if len(points) < 3:
            return points
        start, end = points[0], points[-1]
        dmax, index = 0, -1
        for i in range(1, len(points) - 1):
            d = point_line_distance(points[i], start, end)
            if d > dmax:
                index, dmax = i, d
        if dmax > epsilon:
            left = rdp(points[:index + 1], epsilon)
            right = rdp(points[index:], epsilon)
            return np.vstack((left[:-1], right))
        else:
            return np.array([start, end])

    return rdp(np.array(points), epsilon)


# 创建球形结构核
def create_spherical_struct(radius):
    """生成球形结构核"""
    L = 2 * radius + 1
    center = radius
    struct = np.zeros((L, L, L), dtype=bool)
    for x in range(L):
        for y in range(L):
            for z in range(L):
                if np.linalg.norm(np.array([x, y, z]) - center) <= radius:
                    struct[x, y, z] = True
    return struct


# 从 JSON 读取地图
with open("map/voxel_map.json", "r") as f:
    voxel_data = json.load(f)

# 获取解析数据
map_resolution = voxel_data["resolution"]
size = voxel_data["size"]
voxel_array = np.array(voxel_data["data"])

# === Step 1: 膨胀障碍物 ===
inflate_distance = 1  # 单位：米
inflate_radius = int(np.ceil(inflate_distance / map_resolution))  # 转为体素单位
structure = np.ones((2 * inflate_radius + 1,) * 3)  # 立方体结构

sphere_struct = create_spherical_struct(inflate_radius) # 球体结构

obstacle_mask = voxel_array == 1
inflated_mask = binary_dilation(obstacle_mask, structure=sphere_struct)
inflated_voxel_array = np.where(inflated_mask, 1, 0)  # 新地图

# 起点和终点（这里使用的是 voxel 坐标）
start_world = (1.0, 1.0, 2.0)
goal_world = (29.0, 1.0, 2.0)
start = tuple(int(coord / map_resolution) for coord in start_world)
goal = tuple(int(coord / map_resolution) for coord in goal_world)

z_weight = 20  # 垂直惩罚因子

# 6方向邻居（在3D空间里）
# neighbors = [(0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0)]
# 26邻域（8个邻居）
neighbors_3d = [
    (0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0),
    (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0), (0, 1, 1), (0, -1, 1),
    (0, 1, -1), (0, -1, -1), (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
    (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1), (-1, 1, 1), (-1, 1, -1),
    (-1, -1, 1), (-1, -1, -1)
]

neighbors_3d_g = [np.sqrt(dx**2 + dy**2 + dz**2) for dx, dy, dz in neighbors_3d]

# A* 算法实现
# def a_star(voxel_map, start, goal):
#     def heuristic(a, b):
#         dz = abs(a[2] - b[2])
#         dx_dy = abs(a[0] - b[0]) + abs(a[1] - b[1])
#         return dx_dy + dz * z_weight
#     # A* 算法中的欧几里得启发式函数
#     def euclidean_heuristic(a, b):
#         dx = a[0] - b[0]
#         dy = a[1] - b[1]
#         dz = (a[2] - b[2]) * z_weight
#         return np.sqrt(dx**2 + dy**2 + dz**2)  # 欧几里得距离
    
#     grid_z, grid_x, grid_y = voxel_map.shape

#         # 初始化开放列表和闭合列表
#     open_list = []
#     heapq.heappush(open_list, (0 + euclidean_heuristic(start, goal), 0, start))  # (f, g, node)
#     came_from = {}  # 记录路径
#     g_score = {start: 0}  # 到起点的代价
#     f_score = {start: euclidean_heuristic(start, goal)}  # 启发式代价 f = g + h

#     while open_list:
#         _, current_g, current = heapq.heappop(open_list)

#         # 如果到达终点
#         if current == goal:
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.append(start)
#             path.reverse()
#             return path

#         # 遍历邻居
#         for dx, dy, dz in neighbors_3d:
#             neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)

#             # 检查邻居是否在地图内，并且不是障碍物
#             if 0 <= neighbor[0] < grid_x and 0 <= neighbor[1] < grid_y and 0 <= neighbor[2] < grid_z:
#                 if voxel_map[neighbor[2], neighbor[1], neighbor[0]] == 0:  # 0是空地
#                     tentative_g_score = current_g + 1  # 代价假设每步为1
#                     if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                         came_from[neighbor] = current
#                         g_score[neighbor] = tentative_g_score
#                         f_score[neighbor] = tentative_g_score + euclidean_heuristic(neighbor, goal)
#                         heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))

#     return None  # 如果没有路径

    # open_list = []
    # heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))
    # came_from = {}
    # g_score = {start: 0}
    # f_score = {start: heuristic(start, goal)}

    # while open_list:
    #     _, current_g, current = heapq.heappop(open_list)

    #     if current == goal:
    #         path = []
    #         while current in came_from:
    #             path.append(current)
    #             current = came_from[current]
    #         path.append(start)
    #         path.reverse()
    #         return path

    #     for dx, dy, dz in neighbors_3d:
    #         neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)

    #         if 0 <= neighbor[0] < grid_x and 0 <= neighbor[1] < grid_y and 0 <= neighbor[2] < grid_z:
    #             if voxel_map[neighbor[2], neighbor[1], neighbor[0]] == 0:
    #                 step_cost = z_weight if dz != 0 else 1
    #                 tentative_g_score = current_g + step_cost
    #                 if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
    #                     came_from[neighbor] = current
    #                     g_score[neighbor] = tentative_g_score
    #                     f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
    #                     heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))

    # return None  # 无路径
def a_star(voxel_map, start, goal):
    def euclidean_heuristic(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = (a[2] - b[2]) * z_weight
        return np.sqrt(dx**2 + dy**2 + dz**2)

    grid_z, grid_x, grid_y = voxel_map.shape

    open_list = []
    heapq.heappush(open_list, (euclidean_heuristic(start, goal), 0, start))  # (f, g, node)
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: euclidean_heuristic(start, goal)}
    closed_set = set()

    while open_list:
        _, current_g, current = heapq.heappop(open_list)

        if current in closed_set:
            continue
        closed_set.add(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for (dx, dy, dz), move_cost in zip(neighbors_3d, neighbors_3d_g):
            neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)

            if 0 <= neighbor[0] < grid_x and 0 <= neighbor[1] < grid_y and 0 <= neighbor[2] < grid_z:
                if voxel_map[neighbor[2], neighbor[1], neighbor[0]] != 0:
                    continue  # 避开障碍

                if neighbor in closed_set:
                    continue
                
                tentative_g_score = current_g + move_cost# 可根据方向调整更精细的代价
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f = tentative_g_score + euclidean_heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f, tentative_g_score, neighbor))

    return None  # 没有路径

# === Step 2: 计算路径 ===
path = a_star(inflated_voxel_array, start, goal)
print(path)
# === Step 3: 可视化 ===
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# === 可视化原始障碍和膨胀区域 ===
original_occupied = np.argwhere(voxel_array == 1)
inflated_occupied = np.argwhere((inflated_voxel_array == 1) & (voxel_array == 0))  # 仅膨胀出来的体素

# 原始障碍：红色
zs, ys, xs = original_occupied[:, 0], original_occupied[:, 1], original_occupied[:, 2]
xs = xs * map_resolution + map_resolution / 2
ys = ys * map_resolution + map_resolution / 2
zs = zs * map_resolution + map_resolution / 2
ax.scatter(xs, ys, zs, c='red', marker='s', s=10, label="障碍物")

# 膨胀区域：橙色 + 半透明
zs, ys, xs = inflated_occupied[:, 0], inflated_occupied[:, 1], inflated_occupied[:, 2]
xs = xs * map_resolution + map_resolution / 2
ys = ys * map_resolution + map_resolution / 2
zs = zs * map_resolution + map_resolution / 2
ax.scatter(xs, ys, zs, c='green', alpha=0.2, marker='s', s=10, label="膨胀区域")

# 可视化路径
if path:
    # 原始路径（折线）
    path_x, path_y, path_z = zip(*path)
    path_x = np.array(path_x) * map_resolution + map_resolution / 2
    path_y = np.array(path_y) * map_resolution + map_resolution / 2
    path_z = np.array(path_z) * map_resolution + map_resolution / 2
    ax.plot(path_x, path_y, path_z, color='blue', marker='o', markersize=3, linestyle='--', label="A*原始路径")
    # 路径简化（黑色关键点）
    simplified = rdp_simplify(path, epsilon=1.5)
    simp_x = np.array([p[0] for p in simplified]) * map_resolution + map_resolution / 2
    simp_y = np.array([p[1] for p in simplified]) * map_resolution + map_resolution / 2
    simp_z = np.array([p[2] for p in simplified]) * map_resolution + map_resolution / 2
    ax.plot(simp_x, simp_y, simp_z, color='black', marker='x', markersize=5, linestyle='-', label="RDP简化后路径")
    # 平滑路径（曲线）
    smoothed_x, smoothed_y, smoothed_z = smooth_path_with_catmull_rom(simplified, map_resolution)
    ax.plot(smoothed_x, smoothed_y, smoothed_z, color='deepskyblue', linewidth=2, label="平滑后路径")
    # === 导出平滑路径到 JSON ===
    smoothed_path_world = list(zip(smoothed_x.tolist(), smoothed_y.tolist(), smoothed_z.tolist()))
    output_data = {
        "resolution": map_resolution,
        "path": smoothed_path_world
    }

with open("map/smoothed_path.json", "w") as f:
    json.dump(output_data, f, indent=2)

print("平滑路径已导出到 smoothed_path.json ✅")

# 起点终点
start_x, start_y, start_z = start[0] * map_resolution + map_resolution / 2, start[1] * map_resolution + map_resolution / 2, start[2] * map_resolution + map_resolution / 2
goal_x, goal_y, goal_z = goal[0] * map_resolution + map_resolution / 2, goal[1] * map_resolution + map_resolution / 2, goal[2] * map_resolution + map_resolution / 2
ax.scatter(start_x, start_y, start_z, color='green', s=100, label="起点")
ax.scatter(goal_x, goal_y, goal_z, color='purple', s=100, label="目标点")

# 轴标签与范围
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
# ax.set_title('3D Path Planning with A* + Obstacle Inflation')
ax.set_xlim(0, size[0] * map_resolution)
ax.set_ylim(0, size[1] * map_resolution)
ax.set_zlim(0, size[2] * map_resolution)
ax.legend()

plt.tight_layout()
ax.view_init(elev=45, azim=-135)
plt.show()
