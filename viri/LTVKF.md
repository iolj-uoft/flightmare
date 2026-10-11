# Relative LTVKF

Math behind [`src/interceptor/relative_ltvkf_node.cpp`](src/interceptor/relative_ltvkf_node.cpp): a linear time-varying Kalman filter that estimates the target drone's position and velocity relative to the chaser. It runs in the chaser's gravity-aligned (GA) frame and corrects for delayed YOLO measurements.

---

## 1. Frames

| Symbol | Frame | Definition |
|---|---|---|
| $W$ | World | Inertial, z up |
| $B$ | Body | `chaser_drone/base_link` (x forward, y left, z up, REP-103) |
| $C$ | Camera | `chaser_drone/camera_left_optical` (z forward, x right, y down) |
| $G$ | Gravity-aligned | `ga_body_frame`: same origin as $B$, rotated only by the chaser's yaw $\psi$ |

The chaser's attitude is $R_{WB} = R_z(\psi)\,R_y(\theta)\,R_x(\phi)$ (ZYX, from odometry). The GA frame drops roll and pitch:

$$
R_{WG} = R_z(\psi), \qquad
R_{GB} = R_{WG}^\top R_{WB} = R_y(\theta)\,R_x(\phi)
$$

$R_{GB}$ is `getDerotationMatrix()`. It takes body-frame vectors into the GA frame.

**Why the GA frame:** the GA frame rotates only about the gravity axis, at the yaw rate, so the kinematics stay simple. Roll and pitch oscillations from the attitude controller never enter the state.

---

## 2. State

$$
\mathbf{x} =
\begin{bmatrix} \mathbf{p} \\ \mathbf{v}_t \end{bmatrix} \in \mathbb{R}^6,
\qquad
\mathbf{p} = {}^{G}\!\left(\mathbf{r}_{t} - \mathbf{r}_{c}\right),
\quad
\mathbf{v}_t = {}^{G}\dot{\mathbf{r}}_{t}\big|_W
$$

- $\mathbf{p}$: the target's position relative to the chaser's base_link origin, expressed in $G$.
- $\mathbf{v}_t$: the target's **inertial** velocity, expressed in $G$.

---

## 3. Inputs (from `noisy_odometry`)

The odometry twist is in the body frame (`velocity_estimate_in_world_frame: false`):

$$
\mathbf{v}_c^B = [u, v, w]^\top, \qquad \boldsymbol{\omega}^B = [p, q, r]^\top
$$

**Chaser linear velocity in GA:**

$$
\mathbf{v}_c = R_{GB}\,\mathbf{v}_c^B
$$

**GA-frame angular velocity.** The GA frame rotates only about $z$, at the Euler yaw rate (ZYX kinematics):

$$
\dot\psi = \frac{q\sin\phi + r\cos\phi}{\cos\theta},
\qquad
\boldsymbol{\omega}_G = [0,\;0,\;\dot\psi]^\top
$$

When $|\cos\theta| < 10^{-3}$ the code falls back to $\dot\psi \approx r$.

---

## 4. Process model (continuous time)

Take the derivative of a vector expressed in a frame that rotates at $\boldsymbol{\omega}_G$ (transport theorem):

$$
\dot{\mathbf{p}} = \mathbf{v}_t - \mathbf{v}_c - \boldsymbol{\omega}_G \times \mathbf{p}
$$

Assume the target has constant inertial velocity, with unmodelled acceleration treated as noise. Its GA-frame coordinates still change because the frame rotates:

$$
\dot{\mathbf{v}}_t = -\boldsymbol{\omega}_G \times \mathbf{v}_t + \mathbf{w}_a
$$

With $[\boldsymbol{\omega}]_\times$ the skew-symmetric matrix (`skew()`), the model in state-space form is

$$
\boxed{
\dot{\mathbf{x}} =
\underbrace{\begin{bmatrix}
-[\boldsymbol{\omega}_G(t)]_\times & I_3 \\
0 & -[\boldsymbol{\omega}_G(t)]_\times
\end{bmatrix}}_{A(t)}
\mathbf{x}
+
\underbrace{\begin{bmatrix} -I_3 \\ 0 \end{bmatrix}}_{B}
\mathbf{v}_c(t)
+ \mathbf{w},
\qquad \mathbf{w} \sim \mathcal{N}(0, Q_c)
}
$$

### Why this is an LTV KF and not an EKF

The model is **linear in $\mathbf{x}$**. The measured odometry enters only through the time-varying matrix $A(t)$ and the input term $B\,\mathbf{v}_c(t)$. The Jacobian $\partial f/\partial \mathbf{x}$ equals $A(t)$ exactly and does not depend on the estimate, so there is no linearization error. That is the definition of a linear time-varying Kalman filter. The measurement model in §6 is also linear, which settles it.

(The odometry is treated as a known, noise-free input. Its noise is absorbed into $Q_c$.)

---

## 5. Prediction (discrete time)

Prediction runs on every odometry message, with $\Delta t = t_k - t_{k-1}$. Steps where $\Delta t \le 0$ or $\Delta t > 0.1\,$s are skipped.

### 5.1 Mean: RK4

$$
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_{k-1}), &
\mathbf{k}_2 &= f(\mathbf{x}_{k-1} + \tfrac{\Delta t}{2}\mathbf{k}_1), \\
\mathbf{k}_3 &= f(\mathbf{x}_{k-1} + \tfrac{\Delta t}{2}\mathbf{k}_2), &
\mathbf{k}_4 &= f(\mathbf{x}_{k-1} + \Delta t\,\mathbf{k}_3), \\
\hat{\mathbf{x}}_k^- &= \hat{\mathbf{x}}_{k-1} + \tfrac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
\end{aligned}
$$

Here $f(\mathbf{x}) = A\mathbf{x} + B\mathbf{v}_c$ (`computeDynamics()`), and $\mathbf{v}_c$, $\boldsymbol{\omega}_G$ are held constant over the step.

### 5.2 Covariance

State transition, second-order Taylor:

$$
F = A(t_k), \qquad
\Phi_k = I_6 + F\,\Delta t + \tfrac{1}{2}F^2\,\Delta t^2
$$

$$
P_k^- = \Phi_k\,P_{k-1}\,\Phi_k^\top + Q_c\,\Delta t
$$

$Q_c\,\Delta t$ is the first-order approximation of the discrete process noise.

> **Exact alternative.** Over one step $A$ is constant and $[\boldsymbol\omega]_\times$ commutes with itself, so the matrix exponential has a closed form:
> $$
> \Phi_k = e^{A\Delta t} =
> \begin{bmatrix} R_\Delta & \Delta t\,R_\Delta \\ 0 & R_\Delta \end{bmatrix},
> \qquad R_\Delta = R_z(-\dot\psi\,\Delta t)
> $$
> At 100 Hz the Taylor form is accurate enough. The closed form would also give the mean exactly, without RK4.

### 5.3 Tuning

| | Code | Value |
|---|---|---|
| $P_0$ | `P_` | $\mathrm{diag}(5\,I_3,\;100\,I_3)$ |
| $Q_c$ | `Q_` | $\mathrm{diag}(0.05\,I_3,\;10\,I_3)$ |
| $\hat{\mathbf{x}}_0$ | `x_` | $[\mathbf{z}_0^G,\;\mathbf{0}]$, from the first measurement |

The large velocity block in $Q_c$ lets the filter follow a maneuvering target, at the cost of a noisier velocity estimate.

---

## 6. Measurement model

### 6.1 Stereo triangulation (`depth_estimation.cpp`)

The YOLO node gives the target's horizontal pixel coordinate in the left and right images, $u_L$ and $u_R$, and the vertical coordinates $v_L$ and $v_R$. The pipeline uses intrinsics $f$, $(c_x, c_y)$ and baseline $b$. Frames are rejected when the target is near the image edge, when $|v_L - v_R| > 15\,$px, or when the disparity $d \le 0.5$.

$$
d = u_L - u_R, \qquad
Z_C = \frac{f\,b}{d}, \qquad
X_C = \frac{(u_L - c_x)\,Z_C}{f}, \qquad
Y_C = \frac{(\bar v - c_y)\,Z_C}{f},\quad \bar v = \tfrac{v_L + v_R}{2}
$$

This produces ${}^{C}\mathbf{z} = [X_C, Y_C, Z_C]^\top$ in `camera_left_optical`, stamped with the **image** timestamp $t_m$.

### 6.2 Camera → body → GA

TF looks up the camera extrinsics at time $t_m$ and moves the point into the body frame:

$$
{}^{B}\mathbf{z} = R_{BC}\,{}^{C}\mathbf{z} + \mathbf{t}_{BC}
$$

The node then rotates it into the GA frame:

$$
\mathbf{z} = R_{GB}\,{}^{B}\mathbf{z}
$$

### 6.3 Linear measurement equation

After this preprocessing, the measurement observes the relative position directly:

$$
\boxed{
\mathbf{z} = H\,\mathbf{x} + \boldsymbol{\nu},
\qquad
H = \begin{bmatrix} I_3 & 0_{3\times3} \end{bmatrix},
\qquad
\boldsymbol{\nu} \sim \mathcal{N}(0, R_G)
}
$$

Velocity is never measured directly. It becomes observable through the coupling term $\dot{\mathbf{p}} = \mathbf{v}_t - \cdots$.

### 6.4 Range-dependent measurement noise (`computeDynamicCovariance()`)

Propagating pixel noise through the triangulation gives

$$
\sigma_{\text{depth}} \approx \frac{Z^2}{f\,b}\,\sigma_d,
\qquad
\sigma_{\text{lat}} \approx \frac{Z}{f}\,\sigma_u
$$

Depth error grows **quadratically** with range and lateral error **linearly**. The code models this in the body frame, using range $\rho = \max(\lVert{}^{B}\mathbf{z}\rVert,\,0.5)$ in place of $Z$:

$$
\sigma_x = c_d\,\rho^2, \quad \sigma_y = \sigma_z = c_\ell\,\rho,
\qquad c_d = 0.05,\; c_\ell = 0.02
$$

$$
R_B = \mathrm{diag}(\sigma_x^2,\,\sigma_y^2,\,\sigma_z^2) + 0.01\,I_3
$$

Body $x$ is the depth axis because the cameras face forward. The $0.01\,I_3$ floor keeps $R_B$ well-conditioned at close range.

Rotated into the GA frame:

$$
R_G = R_{GB}\,R_B\,R_{GB}^\top
$$

For the launch-file values $f = 311.77$ and $b = 0.2$ (so $fb \approx 62.4$), the coefficients correspond to pixel noise of roughly $\sigma_d \approx c_d\,fb \approx 3.1\,$px on disparity and $\sigma_u \approx c_\ell f \approx 6.2\,$px laterally. These are useful sanity checks when tuning.

(`R_` = $0.2\,I_3$ is still declared in the class, but nothing uses it.)

---

## 7. Update with delayed measurements (rewind and replay)

YOLO and stereo processing take time, so the measurement time $t_m$ is earlier than the current filter time. Every prediction step stores
$(t_k,\ \hat{\mathbf{x}}_k,\ P_k,\ \mathbf{v}_{c,k},\ \boldsymbol{\omega}_{G,k},\ \Delta t_k)$ in a ring buffer holding the last 100 steps (≈ 1 s at 100 Hz).

When a measurement arrives:

1. **Rewind.** Find the last buffered index $j$ with $t_j < t_m$. Abort if $t_m$ falls outside the buffer.

2. **Update at $j$.** This is the standard Kalman update:

$$
\begin{aligned}
\mathbf{y} &= \mathbf{z} - H\hat{\mathbf{x}}_j &&\text{(innovation)} \\
S &= H P_j H^\top + R_G &&\text{(innovation covariance)} \\
K &= P_j H^\top S^{-1} &&\text{(gain)} \\
\hat{\mathbf{x}}_j^+ &= \hat{\mathbf{x}}_j + K\mathbf{y} \\
P_j^+ &= (I_6 - KH)\,P_j
\end{aligned}
$$

   Because $H = [I\;0]$, writing $P_j = \begin{bmatrix} P_{pp} & P_{pv} \\ P_{vp} & P_{vv}\end{bmatrix}$ gives
   $S = P_{pp} + R_G$ and
   $K = \begin{bmatrix} P_{pp} \\ P_{vp} \end{bmatrix} S^{-1}$.
   The velocity correction comes entirely from the cross-covariance $P_{vp}$ that prediction has built up.

3. **Replay.** For $i = j+1, \dots, N$, re-run §5 from the corrected state, using the stored inputs $(\mathbf{v}_{c,i}, \boldsymbol{\omega}_{G,i}, \Delta t_i)$, and overwrite the buffer.

4. **Finalize.** Set $\hat{\mathbf{x}} \leftarrow \hat{\mathbf{x}}_N$ and $P \leftarrow P_N$.

Because the system is linear, this produces the same estimate as a filter that had received the measurement on time, up to the timing offset noted below.

---

## 8. Outputs

| Topic | Type | Content |
|---|---|---|
| `filtered_relative_state` | `nav_msgs/Odometry` | pose.position = $\mathbf{p}$, twist.linear = $\mathbf{v}_t$, frame `ga_body_frame` |
| `filtered_target_position` | `geometry_msgs/PointStamped` | $\mathbf{p}$, for RViz |
| TF `base_link → ga_body_frame` | | rotation $R_{BG} = R_{GB}^\top$, zero translation |

Note that `twist.linear` holds the **target's** inertial velocity. It is not the relative velocity, which is $\mathbf{v}_t - \mathbf{v}_c$.

---

## 9. Known approximations / possible improvements

| # | Approximation | Effect | Possible fix |
|---|---|---|---|
| 1 | The derotation $R_{GB}$ for a measurement uses the **latest** attitude, not the attitude at $t_m$ | Rotation error of roughly $\dot\phi\,\Delta t_{\text{delay}}$ in the measurement | Store `q` in `HistoryElement` and use $R_{GB}(t_j)$ |
| 2 | The update is applied at $t_j \le t_m$, not exactly at $t_m$ | Timing error of up to one odometry step (≤ 10 ms) | Propagate from $t_j$ to $t_m$ before updating |
| 3 | $\Phi$ uses second-order Taylor while the mean uses RK4 | Negligible at 100 Hz | Closed-form $\Phi$ (§5.2) |
| 4 | $Q_d \approx Q_c\,\Delta t$ | Ignores how position and velocity noise couple within a step | Van Loan discretization, or $Q_d$ for a white-noise-acceleration model |
| 5 | Covariance update is $(I-KH)P$ | Can lose symmetry or positive-definiteness numerically | Joseph form $(I-KH)P(I-KH)^\top + KRK^\top$ |
| 6 | No outlier gating | One bad stereo match can corrupt the estimate | Reject when $\mathbf{y}^\top S^{-1}\mathbf{y} > \chi^2_{3,0.99} \approx 11.3$ |
| 7 | Odometry noise is not propagated through $\mathbf{v}_c$ | Prediction can be overconfident with noisy odometry | Add $B\,\Sigma_{v_c}B^\top\Delta t$ to the position block of $Q$ |
