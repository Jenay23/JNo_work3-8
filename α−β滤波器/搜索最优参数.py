"""
α-β 滤波器最优参数搜索

通过两级网格搜索（粗搜 + 精搜），寻找使估计位置均方误差 (MSE) 最小的 α, β 参数。
粗搜步长 0.01，在最优值附近精搜步长 0.00001。
"""

import numpy as np

Δt = 5.0
z = np.array([30171, 30353, 30756, 30799, 31018,
              31278, 31276, 31379, 31748, 32175])
true = np.array([30200, 30400, 30600, 30800, 31000,
                 31200, 31400, 31600, 31800, 32000])
N = len(z)


def run_filter(α, β):
    x_est = np.zeros(N); v_est = np.zeros(N)
    x_pred = np.zeros(N); v_pred = np.zeros(N)
    for n in range(N):
        if n == 0:
            x_est[n] = z[n]; v_est[n] = 0.0
        else:
            r_n = z[n] - x_pred[n-1]
            x_est[n] = x_pred[n-1] + α * r_n
            v_est[n] = v_pred[n-1] + (β / Δt) * r_n
        x_pred[n] = x_est[n] + Δt * v_est[n]
        v_pred[n] = v_est[n]
    return np.mean((x_est - true) ** 2)


# 第一轮：粗搜
#   α 范围 [0.001, 0.999]，步长 0.01
#   β 范围 [0.001, 0.499]，步长 0.01
print("第一轮粗搜中...")
best_mse = float('inf'); best_α = 0; best_β = 0
for α in np.arange(0.001, 1.0, 0.01):
    for β in np.arange(0.001, 0.5, 0.01):
        mse = run_filter(α, β)
        if mse < best_mse:
            best_mse = mse; best_α = α; best_β = β
print(f"粗搜结果: α={best_α:.3f}, β={best_β:.3f}, MSE={best_mse:.2f}")

# 第二轮：在粗搜最优值附近精搜，步长 0.00001
print("第二轮精搜中...")
α_start = max(0.001, best_α - 0.01)
α_end = min(0.999, best_α + 0.01)
β_start = max(0.001, best_β - 0.01)
β_end = min(0.499, best_β + 0.01)

best_mse2 = float('inf'); best_α2 = 0; best_β2 = 0
for α in np.arange(α_start, α_end, 0.00001):
    for β in np.arange(β_start, β_end, 0.00001):
        mse = run_filter(α, β)
        if mse < best_mse2:
            best_mse2 = mse; best_α2 = α; best_β2 = β

print(f"精搜结果: α={best_α2:.5f}, β={best_β2:.5f}, MSE={best_mse2:.2f}")
