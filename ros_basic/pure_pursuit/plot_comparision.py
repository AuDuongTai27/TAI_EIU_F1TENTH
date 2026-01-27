import matplotlib.pyplot as plt
import numpy as np

# --- DỮ LIỆU ---
configs = ['Short\n(Ld=0.4)', 'Tuned\n(Ld=0.8)', 'Long\n(Ld=1.5)']
rmse_vals = [0.5814, 0.4512, 0.8230]      
max_cte_vals = [1.2540, 1.1557, 1.9500]  
time_vals = [72.30, 65.49, 68.15]        

# --- CẤU HÌNH VẼ (ĐÃ SỬA LỖI STYLE) ---
# Dùng try-except để tự động chọn style phù hợp với phiên bản máy bạn
try:
    plt.style.use('seaborn-v0_8-whitegrid')
except OSError:
    # Fallback cho các máy dùng matplotlib bản cũ (Ubuntu 20.04/22.04)
    try:
        plt.style.use('seaborn-whitegrid')
    except:
        plt.style.use('ggplot') # Style kinh điển, chắc chắn chạy được

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5)) 

# MÀU SẮC
colors = ['#ff9999', '#66b3ff', '#99ff99'] 
bar_width = 0.35
index = np.arange(len(configs))

# === BIỂU ĐỒ 1: SAI SỐ ===
rects1 = ax1.bar(index, rmse_vals, bar_width, label='RMSE (Avg Error)', color='tab:blue', alpha=0.8)
rects2 = ax1.bar(index + bar_width, max_cte_vals, bar_width, label='Max CTE (Worst Case)', color='tab:red', alpha=0.8)

ax1.set_ylabel('Error [meters]', fontsize=11, fontweight='bold')
ax1.set_title('Tracking Accuracy Comparison', fontsize=13, fontweight='bold')
ax1.set_xticks(index + bar_width / 2)
ax1.set_xticklabels(configs, fontsize=11)
ax1.legend()
ax1.grid(axis='y', linestyle='--', alpha=0.7)

def autolabel(rects, ax):
    for rect in rects:
        height = rect.get_height()
        ax.annotate(f'{height:.2f}',
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3), 
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

autolabel(rects1, ax1)
autolabel(rects2, ax1)

# === BIỂU ĐỒ 2: THỜI GIAN ===
rects3 = ax2.bar(configs, time_vals, color=['gray', 'tab:green', 'gray'], alpha=0.8)

ax2.set_ylabel('Time [seconds]', fontsize=11, fontweight='bold')
ax2.set_title('Lap Time Comparison (Lower is Better)', fontsize=13, fontweight='bold')
ax2.set_ylim(60, 80) 
ax2.grid(axis='y', linestyle='--', alpha=0.7)

autolabel(rects3, ax2)

# --- LƯU ẢNH ---
plt.tight_layout()
plt.savefig('comparison_result.png', dpi=300)
print("Đã lưu ảnh: comparison_result.png")
# plt.show() # Bạn có thể comment dòng này nếu chạy trên server không có màn hình