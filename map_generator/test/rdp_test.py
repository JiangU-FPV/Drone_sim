import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.sans-serif'] = ['SimSun']
# 计算点到线段的垂直距离
def perpendicular_distance(point, start, end):
    if np.all(start == end):
        return np.linalg.norm(point - start)
    return np.abs(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

# 递归记录每一步画图
def rdp(points, epsilon, ax=None, depth=0):
    if len(points) < 3:
        if ax:
            ax.plot(points[:, 0], points[:, 1], 'g--', alpha=0.5)
        return points

    start, end = points[0], points[-1]
    dmax = 0
    index = -1

    for i in range(1, len(points) - 1):
        d = perpendicular_distance(points[i], start, end)
        if d > dmax:
            index = i
            dmax = d

    if ax:
        # 绘制当前线段和最大偏差点
        ax.plot([start[0], end[0]], [start[1], end[1]], 'b--', alpha=0.5)
        ax.plot(points[index][0], points[index][1], 'bo')
        ax.set_title(f'epsilon = {epsilon:.2f}', fontsize=16)

    if dmax > epsilon:
        left = rdp(points[:index+1], epsilon, ax, depth+1)
        right = rdp(points[index:], epsilon, ax, depth+1)
        return np.vstack((left[:-1], right))
    else:
        return np.array([start, end])

# 示例路径（模拟曲线）
points = np.array([
    [0, 0], [1, 0.2], [2, 0.4], [3, 0.3],
    [4, 0.1], [5, -0.2], [6, -0.3], [7, 0],
    [8, 0.5], [9, 1], [10, 1.2]
])

# 绘图流程
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(points[:, 0], points[:, 1], 'r-', label='原始路径')
simplified = rdp(points, epsilon=0.1, ax=ax)
ax.plot(simplified[:, 0], simplified[:, 1], 'go-', label='简化路径')
ax.legend()
ax.grid(True)
ax.tick_params(axis='both', labelsize=14)
plt.legend(fontsize=18)
plt.tight_layout()
plt.show()
