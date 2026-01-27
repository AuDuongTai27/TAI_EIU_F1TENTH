import pandas as pd
import numpy as np

# ĐỔI TÊN FILE CSV CỦA BẠN VÀO ĐÂY
file_path = '/home/adt/f1tenth_log_2026-01-22_18-56-57.csv' 

df = pd.read_csv(file_path)

# 1. Tính RMSE
rmse = np.sqrt(np.mean(df['cte']**2))

# 2. Tính Max CTE
max_cte = np.max(df['cte'])

# 3. Tính Lap Time (Sơ bộ)
# Lấy thời gian cuối - thời gian đầu
total_time = df['time'].iloc[-1] - df['time'].iloc[0]

print("="*30)
print(f"KẾT QUẢ PHÂN TÍCH CHO BÁO CÁO")
print("="*30)
print(f"RMSE (Accuracy): {rmse:.4f} m")
print(f"Max CTE:         {max_cte:.4f} m")
print(f"Total Time:      {total_time:.2f} s")
print("="*30)