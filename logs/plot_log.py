import pandas as pd
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimSun']
# === 修改这里 ===
CSV_FILE = "D:/Final_ws/Drone_sim/logs/ang_test_log.csv"
FIELDS_TO_PLOT = [
    "target_angle_roll",
    "measure_angle_roll"
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
        else:
            print(f"⚠️ 字段不存在: {field}")
    plt.xlabel("Timestamp")
    plt.ylabel("Value")
    plt.title("角度控制器")
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
