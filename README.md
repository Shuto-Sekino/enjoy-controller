# Enjoy Controller - 制御系を楽しく学ぼう

インタラクティブなシミュレーターを通じて、制御工学を楽しく学べるコンテンツ集です。

## デモ

GitHub Pagesで公開中: [デモを見る](https://shuto-sekino.github.io/enjoy-controller/)

---

## コンテンツ

### 1. 倒立振子シミュレーター

PID制御を使った倒立振子のシミュレーターです。

**機能:**
- リアルタイムアニメーション
- PIDゲイン（Kp, Ki, Kd）の調整
- マウス/タッチで振子を揺らす
- 外乱を加えるボタン
- 状態のリアルタイム表示

---

## 数学モデル

### システムの定義

カートに乗った倒立振子（cart-pole）システムです。

| 記号 | 意味 | デフォルト値 |
|------|------|-------------|
| $M$ | カートの質量 | 1.0 kg |
| $m$ | 振子の質量 | 0.1 kg |
| $L$ | 振子の長さ | 1.0 m |
| $g$ | 重力加速度 | 9.81 m/s² |
| $b$ | カートの摩擦係数 | 0.1 N·s/m |
| $F$ | カートへの制御入力（力）| — N |

**状態変数:**

| 記号 | 意味 |
|------|------|
| $x$ | カートの位置 (m) |
| $\dot{x}$ | カートの速度 (m/s) |
| $\theta$ | 振子の傾き角 (rad)、$\theta=0$ が直立 |
| $\dot{\theta}$ | 振子の角速度 (rad/s) |

---

### 非線形運動方程式

Euler-Lagrange法により導出した方程式です（振子を一様棒として扱い、重心周りの回転慣性を考慮）。

**振子の角加速度:**

$$\ddot{\theta} = \frac{g\sin\theta + \cos\theta \cdot \dfrac{-F - mL\dot{\theta}^2\sin\theta + b\dot{x}}{M+m}}{L\!\left(\dfrac{4}{3} - \dfrac{m\cos^2\theta}{M+m}\right)}$$

**カートの加速度:**

$$\ddot{x} = \frac{F + mL\!\left(\dot{\theta}^2\sin\theta - \ddot{\theta}\cos\theta\right) - b\dot{x}}{M+m}$$

> 分母の $4/3$ は振子を一様棒として扱ったときの慣性モーメント $I = \frac{1}{3}mL^2$（支点周り）に由来します。

---

### 線形化

直立平衡点 $(\theta, \dot{\theta}, \dot{x}) = (0, 0, 0)$ 周りで線形化します。

$$\sin\theta \approx \theta, \quad \cos\theta \approx 1, \quad \dot{\theta}^2\sin\theta \approx 0$$

変数 $p = 4M + m$ と置くと：

**線形化された振子の角加速度:**

$$\ddot{\theta} = \frac{3g(M+m)}{Lp}\,\theta + \frac{3b}{Lp}\,\dot{x} - \frac{3}{Lp}\,F$$

**線形化されたカートの加速度:**

$$\ddot{x} = -\frac{3mg}{p}\,\theta - \frac{4b}{p}\,\dot{x} + \frac{4}{p}\,F$$

---

### 状態空間表現

$$\dot{\mathbf{x}} = A\mathbf{x} + Bu, \quad \mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}, \quad u = F$$

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & -\dfrac{4b}{p} & -\dfrac{3mg}{p} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & \dfrac{3b}{Lp} & \dfrac{3g(M+m)}{Lp} & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ \dfrac{4}{p} \\ 0 \\ -\dfrac{3}{Lp} \end{bmatrix}$$

**デフォルトパラメータでの数値:**

$$A \approx \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & -0.098 & -0.718 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0.073 & 7.896 & 0 \end{bmatrix}, \quad B \approx \begin{bmatrix} 0 \\ 0.976 \\ 0 \\ -0.732 \end{bmatrix}$$

$A$ の固有値は $\{0,\; 0,\; +2.81,\; -2.81\}$ 程度であり、$+2.81$ の不安定極が存在するため制御なしでは転倒します。

---

## 現代制御による最適ゲイン計算（LQR）

### 状態フィードバック制御則

$$u = -K\mathbf{x} = -\begin{bmatrix} k_1 & k_2 & k_3 & k_4 \end{bmatrix} \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}$$

閉ループ系は $\dot{\mathbf{x}} = (A - BK)\mathbf{x}$ となり、$K$ を適切に設計することで全ての固有値を左半平面に配置できます。

### LQR（Linear Quadratic Regulator）

LQRは以下の二次形式コスト関数を最小化するゲイン $K$ を求めます。

$$J = \int_0^\infty \!\left(\mathbf{x}^T Q\,\mathbf{x} + u^T R\,u\right) dt$$

- $Q \succeq 0$：状態への重み行列（大きいほど状態偏差を小さくしようとする）
- $R \succ 0$：制御入力への重み（大きいほど省エネな制御になる）

最適ゲインは**代数リカッチ方程式 (Algebraic Riccati Equation, ARE)** の解 $P$ から得られます。

$$A^T P + PA - PBR^{-1}B^T P + Q = 0$$

$$\boxed{K = R^{-1}B^T P}$$

### 重み行列の設計指針

| 設計方針 | $Q$ の設定 | 結果 |
|----------|-----------|------|
| 角度を最優先 | $Q_{33}$ を大きく | 振子が素早く直立するが、カートが動く |
| 位置も制御 | $Q_{11}$ も加える | カートを原点近傍に保つ |
| 省エネ重視 | $R$ を大きく | 小さい力でゆっくり安定化 |
| 応答性重視 | $R$ を小さく | 大きい力で素早く安定化 |

### Python実装例

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# システムパラメータ
M, m, L, g, b = 1.0, 0.1, 1.0, 9.81, 0.1
p = 4 * M + m  # = 4.1

# 状態空間行列
A = np.array([
    [0,          1,              0,                    0],
    [0, -4*b/p,     -3*m*g/p,          0],
    [0,          0,              0,                    1],
    [0,  3*b/(L*p),  3*g*(M+m)/(L*p),  0],
])

B = np.array([[0], [4/p], [0], [-3/(L*p)]])

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
```

### 可制御性の確認

LQRを適用する前に、システムが可制御（controllable）であることを確認します。

```python
# 可制御性行列 C = [B, AB, A²B, A³B]
n = A.shape[0]
C = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
rank = np.linalg.matrix_rank(C)
print(f"Controllability rank: {rank} / {n}")  # n と一致すれば可制御
```

このシステムはランク4（フルランク）であり、完全可制御です。

---

### PID制御との比較

| 観点 | PID制御 | LQR（状態フィードバック） |
|------|---------|--------------------------|
| 設計情報 | 不要 | 数学モデルが必要 |
| 調整パラメータ | $K_p, K_i, K_d$ | 重み行列 $Q, R$ |
| 使用する状態量 | $\theta$（角度のみ） | $x, \dot{x}, \theta, \dot{\theta}$（全状態） |
| 最適性 | 保証なし | コスト関数に対して最適 |
| 外乱への応答 | 積分項で定常偏差除去 | 積分器を別途追加する必要あり |
| 実装の簡便さ | シンプル | 全状態の計測/推定が必要 |

> 現在のシミュレーターはPID制御を採用しています。LQRゲインを実装する場合は `pidControl()` を状態フィードバック則 $u = -K\mathbf{x}$ で置き換えてください。

---

## 技術スタック

- Pure HTML/CSS/JavaScript
- Canvas API for animations
- GitHub Pages for hosting

## 使い方

1. [デモサイト](https://shuto-sekino.github.io/enjoy-controller/)にアクセス
2. サイドバーのスライダーでPIDゲインを調整
3. 振子をクリック/ドラッグして揺らしてみる
4. 最適なゲインを見つけて安定化させよう

## 今後の予定

- [ ] LQR状態フィードバック制御の実装
- [ ] 極配置法（Pole Placement）の実装
- [ ] オブザーバー（カルマンフィルタ）による状態推定
- [ ] プリセットゲイン設定（PID / LQR）
- [ ] 他の制御システム（モーター制御、温度制御など）

## ライセンス

MIT License

## コントリビューション

Issues、Pull Requestsは大歓迎です！
