# Enjoy Controller - 制御系を楽しく学ぼう

インタラクティブなシミュレーターを通じて、制御工学を楽しく学べるコンテンツ集です。

## デモ

GitHub Pagesで公開中: [デモを見る](https://shuto-sekino.github.io/enjoy-controller/)

---

## コンテンツ

### 1. 倒立振子シミュレーター

LQR状態フィードバック制御を使った倒立振子のシミュレーターです。

**機能:**
- リアルタイムアニメーション
- 状態フィードバックゲイン（$k_x, k_{\dot{x}}, k_\theta, k_{\dot{\theta}}$）の調整
- 目標位置 $x_\mathrm{ref}$ のスライダー操作
- マウス/タッチで振子を揺らす
- 外乱を加えるボタン
- 状態のリアルタイム表示

---

## 数学モデル

### システムの定義

カートに乗った倒立振子（cart-pole）システムです。

| 記号 | 意味 | 値 | 単位 |
|------|------|-----|------|
| $M$ | カートの質量 | 1.0 | kg |
| $m$ | 振子の質量 | 0.1 | kg |
| $L$ | 振子の長さ | 1.0 | m |
| $g$ | 重力加速度 | 9.81 | m/s² |
| $b$ | カートの摩擦係数 | 0.1 | N·s/m |

**状態変数と制御入力:**

| 記号 | 意味 |
|------|------|
| $x$ | カートの位置 (m) |
| $\dot{x}$ | カートの速度 (m/s) |
| $\theta$ | 振子の傾き角 (rad)、$\theta=0$ が直立 |
| $\dot{\theta}$ | 振子の角速度 (rad/s) |
| $u = F$ | カートへの制御入力 (N) |

---

### 非線形運動方程式（Euler-Lagrange）

振子を一様棒（質量 $m$、長さ $L$）としてラグランジアン法で導出します。

**第1式（カートの並進）:**

$$( M + m)\ddot{x} + \frac{mL}{2}\!\left(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta\right) = F - b\dot{x}$$

左辺 = カート全体の慣性力 + 振子がカートに及ぼす反力、右辺 = 外力 $F$ − 摩擦 $b\dot{x}$

**第2式（振子の回転）:**

$$\frac{mL^2}{3}\,\ddot{\theta} + \frac{mL}{2}\,\ddot{x}\cos\theta = \frac{mgL}{2}\sin\theta$$

左辺 = 支点周りの回転慣性（$I = \tfrac{1}{3}mL^2$）+ カート加速度による慣性結合、右辺 = 重力トルク

---

### 線形化

直立平衡点 $(\theta, \dot{\theta}, \dot{x}) = (0, 0, 0)$ 周りで線形化します。

$$\sin\theta \approx \theta, \quad \cos\theta \approx 1, \quad \dot{\theta}^2\sin\theta \approx 0$$

変数 $p = 4M + m$ と置くと：

**線形化された第1式:**

$$( M + m)\ddot{x} + \frac{mL}{2}\ddot{\theta} = F - b\dot{x}$$

**線形化された第2式:**

$$\frac{mL^2}{3}\,\ddot{\theta} + \frac{mL}{2}\,\ddot{x} = \frac{mgL}{2}\theta$$

これら2式を $\ddot{x}$、$\ddot{\theta}$ について解くと：

$$\ddot{x} = -\frac{3mg}{p}\,\theta - \frac{4b}{p}\,\dot{x} + \frac{4}{p}\,F$$

$$\ddot{\theta} = \frac{3g(M+m)}{Lp}\,\theta + \frac{3b}{Lp}\,\dot{x} - \frac{3}{Lp}\,F$$

---

### 状態空間表現

$$\dot{\mathbf{x}} = A\mathbf{x} + Bu, \quad \mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}, \quad u = F$$

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & -\dfrac{4b}{p} & -\dfrac{3mg}{p} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & \dfrac{3b}{Lp} & \dfrac{3g(M+m)}{Lp} & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ \dfrac{4}{p} \\ 0 \\ -\dfrac{3}{Lp} \end{bmatrix}$$

**デフォルトパラメータでの数値（$p = 4.1$）:**

$$A \approx \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & -0.10 & -0.72 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0.07 & 7.90 & 0 \end{bmatrix}, \quad B \approx \begin{bmatrix} 0 \\ 0.98 \\ 0 \\ -0.73 \end{bmatrix}$$

$A$ の固有値は $\{0,\; 0,\; +2.81,\; -2.81\}$ 程度であり、$+2.81$ の不安定極が存在するため制御なしでは転倒します。

---

## 現代制御による最適ゲイン計算（LQR）

### 状態フィードバック制御則

目標位置 $x_\mathrm{ref}$ への追従を含む制御則：

$$u = -K(\mathbf{x} - \mathbf{x}_\mathrm{ref}), \quad \mathbf{x}_\mathrm{ref} = \begin{bmatrix} x_\mathrm{ref} \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

$$F = k_x(x - x_\mathrm{ref}) + k_{\dot{x}}\dot{x} + k_\theta\,\theta + k_{\dot{\theta}}\,\dot{\theta}$$

閉ループ系は $\dot{\mathbf{e}} = (A - BK)\mathbf{e}$（$\mathbf{e} = \mathbf{x} - \mathbf{x}_\mathrm{ref}$）となり、$K$ を適切に設計することで全ての固有値を左半平面に配置できます。

### LQR（Linear Quadratic Regulator）

LQRは以下の二次形式コスト関数を最小化するゲイン $K$ を求めます。

$$J = \int_0^\infty \!\left(\mathbf{e}^T Q\,\mathbf{e} + u^T R\,u\right) dt$$

- $Q \succeq 0$：状態偏差への重み行列（大きいほど偏差を小さくしようとする）
- $R \succ 0$：制御入力への重み（大きいほど省エネな制御になる）

最適ゲインは**代数リカッチ方程式 (Algebraic Riccati Equation, ARE)** の解 $P$ から得られます。

$$A^T P + PA - PBR^{-1}B^T P + Q = 0$$

$$\boxed{K = R^{-1}B^T P}$$

### 重み行列の設計指針

| 設計方針 | $Q$ の設定 | 結果 |
|----------|-----------|------|
| 角度を最優先 | $Q_{33}$ を大きく | 振子が素早く直立するが、カートが動く |
| 位置も制御 | $Q_{11}$ も加える | カートを目標位置近傍に保つ |
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
    [0,          1,              0,                   0],
    [0, -4*b/p,     -3*m*g/p,         0],
    [0,          0,              0,                   1],
    [0,  3*b/(L*p),  3*g*(M+m)/(L*p), 0],
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
# K = [-1.00, -2.74, -40.05, -14.37]

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

## 技術スタック

- Pure HTML/CSS/JavaScript
- Canvas API for animations
- KaTeX for math rendering
- GitHub Pages for hosting

## 使い方

1. [デモサイト](https://shuto-sekino.github.io/enjoy-controller/)にアクセス
2. 「目標位置」スライダーでカートの目標位置を設定
3. 振子をクリック/ドラッグして揺らしてみる
4. 「外乱を加える」で動作を確認
5. 「LQR 最適ゲイン設定」ボタンで最適ゲインを適用

## 今後の予定

- [x] LQR状態フィードバック制御の実装
- [ ] 極配置法（Pole Placement）の実装
- [ ] オブザーバー（カルマンフィルタ）による状態推定
- [ ] 他の制御システム（モーター制御、温度制御など）

## ライセンス

MIT License

## コントリビューション

Issues、Pull Requestsは大歓迎です！
