"""
α-β 滤波器 —— 跟踪匀速飞行器

基于 α-β 滤波器，根据雷达测量值估计目标距离和速度。
适用于一维匀速运动目标，雷达采样周期 Δt = 5s。

滤波器方程（n ≥ 2）：
  残差（新息）     r_n = z_n - x̂_{n,n-1}
  校正（测量更新） x̂_{n,n} = x̂_{n,n-1} + α·r_n
                   v̂_{n,n} = v̂_{n,n-1} + (β/Δt)·r_n
  预测（时间更新） x̂_{n+1,n} = x̂_{n,n} + Δt·v̂_{n,n}
                   v̂_{n+1,n} = v̂_{n,n}
"""

import numpy as np
import matplotlib.pyplot as plt

# ==================== 参数设置 ====================
Δt = 5.0          # 雷达采样周期 (s)
α = 0.50908       # 位置增益 (0 < α < 1)
β = 0.28501       # 速度增益 (0 < β < 1, 稳态下 β=α²/(2-α) ≈ 0.174)

# 测量数据 z_n 与真实值（用于对比验证）
z = np.array([30171, 30353, 30756, 30799, 31018,
              31278, 31276, 31379, 31748, 32175])
true = np.array([30200, 30400, 30600, 30800, 31000,
                 31200, 31400, 31600, 31800, 32000])
N = len(z)

# ==================== α-β 滤波器 ====================
x_est = np.zeros(N)   # x̂_{n,n}   第 n 时刻位置估计
v_est = np.zeros(N)   # v̂_{n,n}   第 n 时刻速度估计
x_pred = np.zeros(N)  # x̂_{n+1,n} 第 n 时刻对 n+1 时刻的位置预测
v_pred = np.zeros(N)  # v̂_{n+1,n} 第 n 时刻对 n+1 时刻的速度预测

for n in range(N):
    if n == 0:
        # 初始化：使用第一次测量值作为初始位置估计，初始速度设为 0
        x_est[n] = z[n]
        v_est[n] = 0.0
    else:
        # 残差（新息）：测量值与第 n-1 步预测值之差
        r_n = z[n] - x_pred[n-1]

        # 校正步骤（测量更新）：
        #   x̂_{n,n} = x̂_{n,n-1} + α·r_n
        #   v̂_{n,n} = v̂_{n,n-1} + (β/Δt)·r_n
        x_est[n] = x_pred[n-1] + α * r_n
        v_est[n] = v_pred[n-1] + (β / Δt) * r_n

    # 预测步骤（时间更新）：
    #   x̂_{n+1,n} = x̂_{n,n} + Δt·v̂_{n,n}
    #   v̂_{n+1,n} = v̂_{n,n}（匀速模型恒速假设）
    x_pred[n] = x_est[n] + Δt * v_est[n]
    v_pred[n] = v_est[n]

# ==================== 打印结果表 ====================
print(" n    z_n      True    x̂_{n,n}   v̂_{n,n}   x̂_{n+1,n}  v̂_{n+1,n}")
print("-" * 70)
for n in range(N):
    print(f"{n+1:2d}  {z[n]:5.0f}   {true[n]:5.0f}  {x_est[n]:8.2f}  {v_est[n]:8.2f}  {x_pred[n]:8.2f}  {v_pred[n]:8.2f}")

# ==================== 绘图 ====================
plt.figure(figsize=(10, 5))
plt.plot(range(1, N+1), true,  'ko-', label='True value',  linewidth=2)
plt.plot(range(1, N+1), z,     'rs--', label='Measurement', alpha=0.6)
plt.plot(range(1, N+1), x_est, 'b^-',  label='Estimate',   linewidth=2)
plt.xlabel('Time step n')
plt.ylabel('Distance')
plt.legend()
plt.grid(True)
plt.title(r'$\alpha$-$\beta$ Filter Tracking ($\alpha=$' + f'{α}, $\\beta=${β})')
plt.tight_layout()
plt.savefig('/home/jno/Documents/GitHub/JNo_work3-8/α−β滤波器/alpha_beta_filter.png', dpi=150)
print("\nChart saved to current folder.")
