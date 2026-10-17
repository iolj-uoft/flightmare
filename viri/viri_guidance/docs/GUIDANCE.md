# Guidance

Math behind [`src/pro_nav.cpp`](../src/pro_nav.cpp), [`src/hybrid_pro_nav.cpp`](../src/hybrid_pro_nav.cpp) and [`src/attitude_mapping.cpp`](../src/attitude_mapping.cpp). The guidance node turns the LTVKF relative state into attitude and thrust commands at 50 Hz (paper §III-G).

---

## 1. Inputs

Everything is in the chaser's gravity-aligned (GA) frame $G$. Its yaw is the drone heading, and roll and pitch are removed (see [`viri_estimation/docs/LTVKF.md`](../../viri_estimation/docs/LTVKF.md)). The inputs come from `viri_msgs/RelativeState`:

| Symbol | Field | Meaning |
|---|---|---|
| $\mathbf{r}$ | `p_rel` | target position − chaser position |
| $\mathbf{v}_t$ | `v_tgt` | target inertial velocity |
| $\mathbf{v}_c$ | `v_chaser` | chaser inertial velocity (odometry) |
| $R_{GB}$ | from odometry | current body → GA rotation (tilt only) |

## 2. Pure ProNav (eq. 25–29)

$$
\mathbf{v}_{rel} = \mathbf{v}_t - \mathbf{v}_c,\qquad
V_c = -\hat{\mathbf{r}}\cdot\mathbf{v}_{rel},\qquad
\boldsymbol\Omega = \frac{\mathbf{r}\times\mathbf{v}_{rel}}{\lVert\mathbf{r}\rVert^2}
$$

$$
\mathbf{a}_{PN} = N\,|V_c|\,(\boldsymbol\Omega\times\hat{\mathbf{r}}),\qquad
\mathbf{a}_{fwd} = K_p\,(V_d - V_c)\,\hat{\mathbf{r}},\qquad
\mathbf{a} = \mathbf{a}_{PN} + \mathbf{a}_{fwd} + [0,0,g]^\top
$$

Defaults are $N = 4$, $K_p = 1.5$ and $V_d = 1.5$ m/s. When $\lVert\mathbf{r}\rVert$ is below `min_range` the LOS is undefined and the law commands hover. ProNav does not control the heading: its yaw rate is 0.

## 3. Hybrid heading / ProNav (eq. 30–31)

The paper writes the azimuth error as $\epsilon_\psi = \mathrm{atan2}(r_y, r_x) - \psi_{drone}$ with $\mathbf{r}$ in the world frame. In GA the heading is already subtracted, so:

$$
\epsilon_\psi = \mathrm{atan2}(r_y^G,\ r_x^G) \in (-\pi, \pi]
$$

| Condition | Mode | Command |
|---|---|---|
| $\lvert\epsilon_\psi\rvert \ge 15^\circ$ | HEADING | roll = pitch = 0, $\dot\psi = \mathrm{clamp}(K_\psi\epsilon_\psi, \pm\pi)$, thrust = $g$ |
| $\lvert\epsilon_\psi\rvert < 15^\circ - h$ | PRONAV | §2 |

$K_\psi = 4\ \mathrm{s^{-1}}$. The hysteresis $h$ (`hysteresis_deg`) is 0 by default, which matches the paper.

HEADING mode keeps the target inside the 60° stereo FOV, so the filter keeps receiving updates while the drone turns.

## 4. Acceleration → attitude and thrust

The desired body z-axis is aligned with $\mathbf{a}$ while the current heading is held (yaw 0 in GA). For $R = R_y(\theta)R_x(\phi)$:

$$
\hat{\mathbf{z}}_B = [\cos\phi\sin\theta,\ -\sin\phi,\ \cos\phi\cos\theta]^\top
\;\Rightarrow\;
\theta = \mathrm{atan2}(a_x, a_z),\quad
\phi = \mathrm{atan2}\!\left(-a_y,\ \sqrt{a_x^2 + a_z^2}\right)
$$

Roll and pitch are clamped to ±40°. Collective thrust is the projection onto the **current** body z-axis, which stops the thrust from over-inflating during banked turns. It is clamped to [9, 14] m/s²:

$$
T = \mathbf{a}\cdot(R_{GB}\,\mathbf{e}_z)
$$

The world orientation command is $q = R_z(\psi_{current})R_y(\theta)R_x(\phi)$.

## 5. Command interface

The node publishes `quadrotor_msgs/ControlCommand` on `autopilot/control_command_input`. The RPG autopilot forwards it in COMMAND_FEEDTHROUGH mode, which requires `enable_command_feedthrough: true`.

- `command_mode: body_rates` (default) uses a P attitude controller on the quaternion error, $q_e = q^{-1}q_{des}$, with $\boldsymbol\omega = 2\,\mathrm{sgn}(q_{e,w})[k_{rp}q_{e,x},\ k_{rp}q_{e,y},\ k_{yaw}q_{e,z}] + [0,0,\dot\psi_{cmd}]$. This is the BODY_RATES path that `rpg_rotors_interface` already accepted from the RL agent.
- `command_mode: attitude` sends the orientation and thrust, with $\dot\psi_{cmd}$ as a body-rate feed-forward. Only use it if your `rpg_rotors_interface` build handles ATTITUDE commands.

## 6. Engagement sequencing

```
            auto_takeoff: arm + autopilot/start
IDLE ──(autopilot HOVER ∧ LTVKF initialized ∧ auto_engage)──▶ ENGAGED ──▶ PRONAV / HEADING
  ▲                                                    │
  │                         no stereo update > 1 s ────┴──▶ STALE (coasts on LTVKF prediction)
  └── ~engage service                    /engagement/intercepted ──▶ INTERCEPTED (latched)
```

`~engage` re-engages and clears the intercept latch. `~disengage` stops commanding. Whenever the node stops publishing, the autopilot falls back to HOVER after its control-command input timeout.
