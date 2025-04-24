import pandas as pd
import matplotlib.pyplot as plt

import os
plt.rcParams["font.family"] = ["Times New Roman", "SimSun"]
# 全局字体大小设置
plt.rcParams.update({
    'font.size': 20,        # 全局字体
    'axes.titlesize': 18,   # 标题字体
    'axes.labelsize': 18,   # 坐标轴标签
    'xtick.labelsize': 18,  # X轴刻度
    'ytick.labelsize': 18,  # Y轴刻度
    'legend.fontsize': 20,  # 图例
})

# === 修改这里 ===
current_dir = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(current_dir, 'astar_test_log.csv')

FIELDS_TO_PLOT = [
    # "target_angle_roll",
    # "measure_angle_roll"
    # "target_angle_yaw",
    # "measure_angle_yaw"
    # "target_speed_x",
    # "measure_speed_x"

    # "target_speed_z",
    # "measure_speed_z"
    "measure_liner_speed"
]
# =================

def load_csv(filename):
    try:
        df = pd.read_csv(filename)
        return df
    except Exception as e:
        print(f"❌ 读取文件出错: {e}")
        return None

def plot_data(df, fields):
    plt.figure(figsize=(12, 6))
    for field in fields:
        if field in df.columns:
            plt.plot(df["timestamp"], df[field], label=field)

            # 👈 添加：计算并打印平均值
            mean_value = df[field].mean()
            print(f"✅ {field} 的平均值为: {mean_value:.3f}")

            # 👈 可选：将平均值画在图上
            plt.axhline(y=mean_value, color='r', linestyle='--', linewidth=1.5, label=f'{field} 平均值')

        else:
            print(f"⚠️ 字段不存在: {field}")
    plt.xlabel("Timestamp(s)")
    plt.ylabel("Speed(m/s)")
    # plt.title("Roll轴姿态控制器阶跃响应")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    df = load_csv(CSV_FILE)
    if df is not None and "timestamp" in df.columns:
        plot_data(df, FIELDS_TO_PLOT)
    else:
        print("❌ 缺少 timestamp 字段，无法绘图")

if __name__ == "__main__":
    main()
