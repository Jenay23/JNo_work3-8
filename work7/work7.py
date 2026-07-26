import math

v0 = 17.0
k1 = 0.0089
g = 9.8
x = 3.0
y = 4.0
s = math.sqrt(x * x + y * y)
z0 = 0.25

E = math.exp(k1 * s) - 1
A = E / k1
a = 0.5 * g * A * A / (v0 * v0)

D = A * A - 4 * a * (a + z0)

print(f"E = {E:.6f}, A = {A:.6f}, a = {a:.6f}, D = {D:.6f}\n")

if D < 0:
    print("判别式 D < 0，无实数解")
elif abs(D) < 1e-12:
    u = A / (2 * a)
    theta = math.degrees(math.atan(u))
    print(f"判别式 D ≈ 0，唯一解: θ = {theta:.6f}°")
else:
    u1 = (A + math.sqrt(D)) / (2 * a)
    u2 = (A - math.sqrt(D)) / (2 * a)
    theta1 = math.degrees(math.atan(u1))
    theta2 = math.degrees(math.atan(u2))

    print(f"高抛解 (u = {u1:.6f}): θ₁ = {theta1:.6f}°")
    print(f"低抛解 (u = {u2:.6f}): θ₂ = {theta2:.6f}°")
