import numpy as np
import matplotlib.pyplot as plt

# ==================== 参数设置 ====================
Δt = 5.0          # 雷达采样周期 (s)
α = 0.50908           # 位置增益
β = 0.28501           # 速度增益

# 测量数据 和 真实值（用于对比）
z = np.array([30171, 30353, 30756, 30799, 31018,
              31278, 31276, 31379, 31748, 32175])
true = np.array([30200, 30400, 30600, 30800, 31000,
                 31200, 31400, 31600, 31800, 32000])
N = len(z)

# ==================== α-β 滤波器 ====================
x_est = np.zeros(N)   # x̂_{n,n}   位置估计
v_est = np.zeros(N)   # v̂_{n,n}   速度估计
x_pred = np.zeros(N)  # x̂_{n+1,n} 位置预测
v_pred = np.zeros(N)  # v̂_{n+1,n} 速度预测

for n in range(N):
    if n == 0:
        # 初始化：用第一次测量作为初始估计，速度设为 0
        x_est[n] = z[n]
        v_est[n] = 0.0
    else:
        # 残差（新息）：测量值 - 预测值
        r = z[n] - x_pred[n-1]

        # 更新（修正）步骤
        x_est[n] = x_pred[n-1] + α * r
        v_est[n] = v_pred[n-1] + (β / Δt) * r

    # 预测下一步
    x_pred[n] = x_est[n] + Δt * v_est[n]
    v_pred[n] = v_est[n]

# ==================== 打印结果表 ====================
print("n   z_n      True    x̂_n,n     v̂_n,n     x̂_n+1,n   v̂_n+1,n")
print("-" * 68)
for n in range(N):
    print(f"{n+1:2d}  {z[n]:5.0f}   {true[n]:5.0f}  {x_est[n]:8.2f}  {v_est[n]:8.2f}  {x_pred[n]:8.2f}  {v_pred[n]:8.2f}")

# ==================== 绘图 ====================
plt.figure(figsize=(10, 5))
plt.plot(range(1, N+1), true, 'ko-', label='True value',  linewidth=2)
plt.plot(range(1, N+1), z,    'rs--', label='Measurement', alpha=0.6)
plt.plot(range(1, N+1), x_est, 'b^-', label='Estimate',    linewidth=2)
plt.xlabel('Time step n')
plt.ylabel('Distance')
plt.legend()
plt.grid(True)
plt.title(r'$\alpha$-$\beta$ Filter Tracking ($\alpha=$' + f'{α}, $\\beta=${β})')
plt.tight_layout()
plt.savefig('/home/jno/Desktop/BKD2026培训/α−β滤波器/alpha_beta_filter.png', dpi=150)
print("\nChart saved.")
