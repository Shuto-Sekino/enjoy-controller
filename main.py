import numpy as np
from scipy.linalg import solve_continuous_are

# システムパラメータ
M, m, L, g, b = 1.0, 0.1, 1.0, 9.81, 0.1
p = 4 * M + m  # = 4.1

# 状態空間行列
A = np.array(
    [
        [0, 1, 0, 0],
        [0, -4 * b / p, -3 * m * g / p, 0],
        [0, 0, 0, 1],
        [0, 3 * b / (L * p), 3 * g * (M + m) / (L * p), 0],
    ]
)

B = np.array([[0], [4 / p], [0], [-3 / (L * p)]])

# 重み行列（チューニングパラメータ）
# 状態: [x, x_dot, theta, theta_dot]
Q = np.diag([1.0, 1.0, 100.0, 10.0])  # theta を重視
R = np.array([[1.0]])

# 代数リカッチ方程式を解く
P = solve_continuous_are(A, B, Q, R)

# 最適ゲイン
K = np.linalg.inv(R) @ B.T @ P
print(f"LQR gains: K = {K.flatten()}")
# 例: K ≈ [-1.00, -2.40, 47.1, 9.81]

# 安定性確認（閉ループ固有値）
eigenvalues = np.linalg.eigvals(A - B @ K)
print(f"Closed-loop eigenvalues: {eigenvalues}")
# 全て実部が負なら安定
