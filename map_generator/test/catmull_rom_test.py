import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimSun']
def catmull_rom(p0, p1, p2, p3, num_points=20):
    """
    计算一段 Catmull-Rom 样条上的插值点
    """
    t = np.linspace(0, 1, num_points)
    t = t.reshape(len(t), 1)

    # 参数形式公式
    a = 2 * p1
    b = -p0 + p2
    c = 2*p0 - 5*p1 + 4*p2 - p3
    d = -p0 + 3*p1 - 3*p2 + p3

    curve = 0.5 * (a + b*t + c*t**2 + d*t**3)
    return curve

def generate_catmull_rom_chain(points, num_points_per_segment=20):
    """
    给定一组控制点，生成完整的 Catmull-Rom 曲线
    """
    points = np.array(points)
    curve = []

    # 为首尾补点处理边界
    extended = np.vstack([points[0], points, points[-1]])

    for i in range(1, len(extended) - 2):
        p0, p1, p2, p3 = extended[i-1], extended[i], extended[i+1], extended[i+2]
        segment = catmull_rom(p0, p1, p2, p3, num_points=num_points_per_segment)
        curve.extend(segment)

    return np.array(curve)

# 示例控制点（模拟 A* 输出路径）
control_points = [
    [0, 0],
    [1, 2],
    [3, 5],
    [6, 5],
    [8, 2],
    [10, 0]
]

# 生成样条曲线
smooth_curve = generate_catmull_rom_chain(control_points, num_points_per_segment=30)

# 可视化
plt.figure(figsize=(8, 6))
control_points = np.array(control_points)
plt.plot(control_points[:, 0], control_points[:, 1], 'ro--', label='原始路径')
plt.plot(smooth_curve[:, 0], smooth_curve[:, 1], 'b-', label='Catmull-Rom 平滑路径')
plt.title('Catmull-Rom 样条路径平滑')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
