# Multi-Node Sync & Reconciliation — Logic and Math

Companion to `MULTINODE_SYNC_DESIGN.md` (architecture/decisions). This document
records the **math and algorithm** behind time-aligning multiple IMU nodes, as
implemented in `tools/reconcile_nodes.py`. It is written so the method can be
reviewed, reproduced, and improved independently of the code.

> Equations use LaTeX (`$…$` inline, ` ```math ` display blocks), which renders
> on GitHub and in most Markdown viewers. Where a viewer does not render math,
> the symbols are still readable in source form.

---

## 1. The problem

Each node timestamps its samples with its own free-running `millis()` counter,
which starts at the node's power-on and ticks on its own crystal. For a physical
event at real time $\tau$, node $X$ records

```math
t_X(\tau) = \bigl(\tau - \text{boot}_X\bigr)\cdot 1000 \quad[\text{ms since node } X \text{ booted}]
```

Two nodes therefore disagree in two ways:

1. **Offset** — $\text{boot}_A \neq \text{boot}_B$, so their clocks have
   different origins.
2. **Drift (skew)** — their crystals tick at slightly different rates
   (tens of ppm), so the offset changes slowly over time.

"Reconciling" means finding, for each non-reference node $B$, a mapping onto the
reference node $A$'s timeline:

```math
t_{\text{on }A} = t_B + \text{offset} + \text{drift}\cdot\bigl(t_B - t_{B,0}\bigr) \tag{1}
```

where $\text{offset}$ (ms) and $\text{drift}$ (unitless rate, reported in ppm)
are what we must estimate. This is the **standard two-parameter clock model**
used by NTP, PTP, and distributed-systems clock synchronization.

The design constraint (see `MULTINODE_SYNC_DESIGN.md`): nodes are wireless and
analysis is offline, so we recover $\text{offset}$/$\text{drift}$ **from the
recorded motion itself**, independent of BLE latency.

---

## 2. The motion feature: angular speed (mounting-invariant)

Each node logs unit quaternions $q_i$ (orientation). Raw quaternions are **not**
comparable between nodes because each sensor is mounted at a different, unknown
orientation. So we reduce each stream to a scalar invariant to mounting:
**angular speed**, the magnitude of the rotation rate.

The rotation between consecutive orientations $q_i, q_{i+1}$ has angle

```math
\theta_i = 2\,\arccos\!\bigl(\lvert\, q_i \cdot q_{i+1}\,\rvert\bigr) \tag{2}
```

(the dot product is the quaternion inner product; the absolute value takes the
shorter arc, since $q$ and $-q$ are the same rotation). Angular speed is

```math
\omega_i = \frac{\theta_i}{\Delta t_i}, \qquad \Delta t_i = \frac{t_{i+1}-t_i}{1000} \tag{3}
```

**Why this works:** for a rigid body, angular velocity is a property of the body
(the same for every frame on it), so its magnitude $\lvert\omega\rvert$ is
identical regardless of how each sensor is bolted on. Two sensors on the same
moving segment therefore produce *correlated* angular-speed signals even though
their raw quaternions differ. (Implementation: `angular_speed()`.)

---

## 3. Estimating the offset by cross-correlation

We have two angular-speed signals sampled at slightly different, jittered times.
Resample each onto a uniform grid at $f_s$ (default 100 Hz) in **relative** time
($t - t_0$), giving sequences $a[n]$ (node $A$) and $b[n]$ (node $B$) plus each
node's first timestamp $A_0, B_0$. (Implementation: `resample_uniform()`.)

The lag between the two sequences is found by **cross-correlation** (the classic
time-delay-estimation method):

```math
C[k] = \sum_n \bigl(a[n]-\bar a\bigr)\,\bigl(b[n-k]-\bar b\bigr) \tag{4}
```

```math
\hat k = \arg\max_k C[k], \qquad L = \frac{\hat k}{f_s}\ \ [\text{s}] \tag{5}
```

Positive $L$ means $a$ is delayed relative to $b$. (Implementation:
`_xcorr_full()` via FFT for $O(N\log N)$; `best_lag_seconds()` /
`lag_and_confidence()`.)

### 3.1 From relative-time lag to an absolute clock offset

The correlation is computed on **relative-time** signals, so $L$ alone is not
the clock offset — it must be combined with the difference of the nodes' start
timestamps. The two signals match when they sample the same real motion, i.e.

```math
\text{boot}_A + \frac{A_0 + s}{1000} \;=\; \text{boot}_B + \frac{B_0 + s - 1000L}{1000}
\quad \forall s
```

The $s$ terms cancel (a constant offset — the "no drift" case), leaving

```math
\bigl(\text{boot}_B - \text{boot}_A\bigr)\cdot 1000 \;=\; (A_0 - B_0) + 1000L
```

The left side is exactly the offset that maps $B$'s timestamps onto $A$'s clock
($t_A = t_B + \text{offset}$). Hence

```math
\boxed{\ \text{offset} = (A_0 - B_0) + 1000L \quad[\text{ms}]\ } \tag{6}
```

- $A_0 - B_0$ converts the relative-time result back to absolute clocks
  (bookkeeping).
- $L$ is the *measured* correction, read from the shared motion.

Equivalently, cross-correlating the two signals in **absolute** timestamps
directly yields $\text{offset}$; the code works in relative time only for
numerical convenience and adds $A_0 - B_0$ back.
(Implementation: `estimate_offset_drift()`.)

### 3.2 Sub-sample refinement (parabolic interpolation)

$C[k]$ is only evaluated at integer lags (10 ms apart at $f_s = 100$ Hz). The
true peak usually lies between samples, so we fit a parabola through the peak
sample and its two neighbours $y_{-1}, y_0, y_{+1}$
($= C[\hat k -1], C[\hat k], C[\hat k +1]$) and take its vertex:

```math
\delta = \tfrac{1}{2}\,\frac{y_{-1}-y_{+1}}{\,y_{-1}-2y_0+y_{+1}\,}, \qquad \delta\in[-\tfrac12,\tfrac12] \tag{7}
```

```math
L = \frac{\hat k + \delta}{f_s}
```

Near a smooth peak any function is locally quadratic (Taylor), so this is a
standard, well-justified refinement — it takes lag resolution from ~10 ms down
to ~1 ms. (Textbook DSP; used in spectral-peak and pitch estimation.)

---

## 4. Confidence — was there real shared motion?

$\arg\max$ always returns *a* peak, even for unrelated signals. So we separately
score whether the peak reflects genuine shared motion, using the **Pearson
correlation** of the two signals overlapped at the best lag:

```math
r = \operatorname{corr}\bigl(a_{\text{overlap}},\, b_{\text{overlap}}\bigr)\ \text{at lag }\hat k, \qquad r\in[-1,1] \tag{8}
```

Pearson is normalized (divided by each signal's standard deviation), so it
measures *shape* agreement independent of amplitude — a differently-swinging node
still scores high if it moved *together*. We accept the offset only when

```math
r \ge r_{\min} \quad (r_{\min} = 0.40) \tag{9}
```

Below that, the tool flags the result unreliable and the caller falls back to the
BLE clock-offset read and/or a start-of-session sync gesture.
(Implementation: `_pearson_at_lag()`.)

**Limitation this guards:** cross-correlation needs shared motion. Two nodes on
*independently* moving limbs (one arm swings, the other still) share nothing to
lock onto, and $r$ correctly drops toward 0. This is why a shared sync gesture is
the *primary* anchor and cross-correlation is a *refinement* (see
`MULTINODE_SYNC_DESIGN.md`).

---

## 5. Drift (clock skew) estimation

Drift is **off by default**: over a few-minute record, relative crystal drift is
only a few ms — below the measurement noise and negligible against the 25–50 ms
target — so fitting it adds noise. It is opt-in for long records.

When enabled, the lag is measured in $K$ windows spanning the record, giving
pairs $(t_j, L_j)$. A least-squares line $L = m\,t + c$ is fit; the slope $m$ is
the drift. It is accepted only if it is both statistically significant and large
enough to matter:

```math
\lvert m\rvert > 3\,\operatorname{SE}(m) \quad\text{AND}\quad \lvert m\rvert\cdot\bigl(t_{\text{last}}-t_{\text{first}}\bigr) > \text{DRIFT\_RESOLVE\_MS}\ (=15\text{ ms}) \tag{10}
```

Otherwise $\text{drift}=0$. Reported as $\text{drift\_ppm} = m\times 10^6$.
(Implementation: `_windowed_lags()`, `_lsq_slope()`.)

---

## 6. Applying the mapping and emitting aligned streams

With $\text{offset}$ (and optional $\text{drift}$), node $B$'s timestamps are
mapped onto $A$'s clock via Eq. (1) (`to_A_clock()`). A common uniform time grid
is built over the mutual overlap, and each node's **quaternions** are
interpolated onto it with normalized-linear interpolation (nlerp) along the
shorter arc:

```math
q(t) = \operatorname{normalize}\!\bigl((1-f)\,q_0 + f\,q_1\bigr), \qquad f = \frac{t - t_0}{t_1 - t_0} \tag{11}
```

(with a sign flip when $q_0\cdot q_1 < 0$). For 10 Hz human motion nlerp is
adequate; true SLERP could be substituted. The output is a CSV: one
`t_common_ms` column plus each node's $q_w,q_x,q_y,q_z$, ready for analysis.
(Implementation: `nlerp()`, `align_and_emit()`.)

---

## 7. Validation

`reconcile_nodes.py --selftest` generates two synthetic nodes from a shared
random motion with a **known** injected offset and drift, different mounting
rotations, sensor noise, and timing jitter, then checks recovery:

- recovered offset vs. the injected truth (target: within ~25–50 ms), and
- a **negative control** — a second node given *independent* motion must come
  back with low confidence and be flagged.

Current result: offset recovered to ~15–18 ms; shared-motion confidence
$r\approx 0.93$, independent-motion $r\approx 0.07$. This validates the *math* on
synthetic data; it is **not** a validation against hardware ground truth (§9).

---

## 8. Known limitations

- **Needs shared motion** (§4). Independent-limb captures require the sync
  gesture / BLE offset instead.
- **Linear clock model** (Eq. 1) — valid over minutes; a long session with
  temperature-driven crystal changes could need piecewise / periodic re-sync.
- **Dropouts/gaps** in the logs degrade the correlation and inflate the residual
  (observed on real captures with intermittent recording).
- **Feature is a magnitude** — angular speed discards the rotation axis; two
  motions with identical speed profiles but different axes are indistinguishable
  to the aligner (rare in practice; the confidence check still applies).

---

## 9. Academic basis and honest status

The method is an assembly of **standard, well-established techniques**, not a
novel algorithm:

- **Cross-correlation time-delay estimation** — the core. Classic reference:
  Knapp & Carter, *"The Generalized Correlation Method for Estimation of Time
  Delay,"* IEEE Trans. ASSP, 1976 (the GCC / GCC-PHAT foundation).
- **Aligning unsynchronized sensors by a shared signal** — standard practice in
  multi-camera/mocap/audio sync and multi-IMU synchronization (cross-correlate a
  common motion feature such as angular rate).
- **Offset + linear-skew clock model** — the standard model in NTP/PTP and
  sensor-network clock synchronization.
- **Parabolic peak interpolation** — textbook sub-sample estimation.

**Status:** sound engineering built from reliable components, validated on
synthetic data. It has **not** been benchmarked against a hardware ground truth.
Trust the *approach*; do not treat a specific recovered *number* as calibrated
until §10 is done.

---

## 10. Upgrades for a rigorous / publishable result

1. **Ground-truth it** — feed a shared electrical sync pulse to a GPIO on all
   nodes, timestamp it in `millis()`, and report the *actual* alignment error in
   ms rather than a confidence score.
2. **GCC-PHAT** — whiten the cross-spectrum before the inverse FFT for a much
   sharper, more robust delay peak under noise and gaps.
3. **Robust fitting** — RANSAC or weighted least squares for offset/drift so
   dropouts don't skew the estimate.
4. **Richer feature** — correlate the full angular-velocity vector (rotated into
   a common frame after a coarse alignment) instead of only its magnitude, to
   break the axis-ambiguity of §8.
