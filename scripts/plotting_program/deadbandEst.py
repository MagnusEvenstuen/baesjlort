import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.ndimage import uniform_filter1d

#This file is created by Claude, and validated by me.

# ─────────────────────────────────────────────
# Load data
# ─────────────────────────────────────────────
file_path = 'data_files/sensor_data.csv'   # ← adjust path if needed
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

timestamp = data[:, 0]
acc_x     = data[:, 1]
acc_y     = data[:, 2]
acc_z     = data[:, 3]

t = {i + 1: data[:, 29 + i] for i in range(8)}

dt = np.median(np.diff(timestamp))
fs = 1.0 / dt
print(f"Sampling interval: {dt*1000:.2f} ms  →  fs ≈ {fs:.1f} Hz\n")

# ─────────────────────────────────────────────
# Group commands and acceleration axes
#   Group A (T1–T4) → XY plane
#   Group B (T5–T8) → Z axis
# ─────────────────────────────────────────────
cmd_A  = np.sum([np.abs(t[i]) for i in [1, 2, 3, 4]], axis=0)
cmd_B  = np.sum([np.abs(t[i]) for i in [5, 6, 7, 8]], axis=0)
acc_xy = np.sqrt(acc_x**2 + acc_y**2)
acc_z_abs = np.abs(acc_z)

# ── Data-informed thresholds ──────────────────────────────────────────
# Measured from CSV (cmd ~ 0 periods):
#   acc_xy noise std ≈ 3.69 m/s²,   acc_z noise std ≈ 5.71 m/s²
# Thruster commands:
#   cmd_A: max ≈ 3.27,  75th-pct step ≈ 2.54
#   cmd_B: max ≈ 4.00,  75th-pct step ≈ 3.40
#
# Strategy:
#   • Only use LARGE steps (≥ 70 % of group max) → clean, detectable events
#   • Detect acc onset when signal exceeds 2× measured noise std
#   • Smooth acc with longer window (15 samples ≈ 190 ms) to reduce noise
#   • Require pre-step stability: std of acc in pre-window < 1.5× noise floor
# ─────────────────────────────────────────────
SMOOTH_SAMPLES        = 3      # samples for acc smoothing (~190 ms at 78 Hz)
MIN_STEP_SPACING_S    = 0.5     # ignore steps closer than 1 s

# Group A
MIN_STEP_ABS_A        = 2.0     # ≈ 61 % of cmd_A max — keeps large steps only
ACC_NOISE_FLOOR_A     = 1.5    # measured acc_xy std when cmd_A ≈ 0  (m/s²)
ACC_ONSET_MULT_A      = 1.5     # onset threshold = MULT × noise floor
PRE_STABILITY_MULT_A  = 1.5     # reject step if pre-window acc std > MULT × noise floor

# Group B
MIN_STEP_ABS_B        = 2.0     # ≈ 70 % of cmd_B max
ACC_NOISE_FLOOR_B     = 1.5     # measured |acc_z| std when cmd_B ≈ 0  (m/s²)
ACC_ONSET_MULT_B      = 1.5
PRE_STABILITY_MULT_B  = 1.5

SEARCH_WINDOW_S       = 2.0
PRE_WINDOW_SAMPLES    = 50      # ≈ 640 ms of pre-step history

groups = {
    'A (T1-T4, XY)': {
        'cmd':            cmd_A,
        'acc':            acc_xy,
        'acc_label':      'Acc XY magnitude (m/s²)',
        'min_step':       MIN_STEP_ABS_A,
        'noise_floor':    ACC_NOISE_FLOOR_A,
        'onset_mult':     ACC_ONSET_MULT_A,
        'stability_mult': PRE_STABILITY_MULT_A,
    },
    'B (T5-T8, Z)': {
        'cmd':            cmd_B,
        'acc':            acc_z_abs,
        'acc_label':      '|Acc Z| (m/s²)',
        'min_step':       MIN_STEP_ABS_B,
        'noise_floor':    ACC_NOISE_FLOOR_B,
        'onset_mult':     ACC_ONSET_MULT_B,
        'stability_mult': PRE_STABILITY_MULT_B,
    },
}

for g in groups.values():
    g['acc_smooth'] = uniform_filter1d(g['acc'], size=SMOOTH_SAMPLES)


def standardise(x):
    s = x.std()
    return (x - x.mean()) / s if s > 1e-12 else x - x.mean()


# ═══════════════════════════════════════════════════
# METHOD 1 – Cross-correlation
# ═══════════════════════════════════════════════════
print("=" * 60)
print("METHOD 1 – Cross-correlation dead-time estimates")
print("=" * 60)

max_lag_s = 2.0

fig_cc, axes_cc = plt.subplots(1, 2, figsize=(13, 5))
fig_cc.suptitle("Method 1 – Cross-correlation per thruster group", fontsize=13)

cc_results = {}

for ax, (name, g) in zip(axes_cc, groups.items()):
    cmd_deriv = np.gradient(g['cmd'], timestamp)
    acc_deriv = np.gradient(g['acc_smooth'], timestamp)

    x  = standardise(cmd_deriv)
    y  = standardise(acc_deriv)
    cc = signal.correlate(y, x, mode='full')

    lags  = signal.correlation_lags(len(y), len(x), mode='full')
    lag_s = lags * dt

    mask             = (lag_s >= 0) & (lag_s <= max_lag_s)
    cc_masked        = cc.copy()
    cc_masked[~mask] = -np.inf
    best_lag_s       = lag_s[np.argmax(cc_masked)]
    cc_results[name] = best_lag_s

    print(f"  Group {name}: dead time ≈ {best_lag_s*1000:.1f} ms")

    cc_norm = cc / np.abs(cc).max()
    ax.plot(lag_s, cc_norm, color='steelblue', lw=0.9)
    ax.axvline(best_lag_s, color='red', linestyle='--',
               label=f"Peak = {best_lag_s*1000:.1f} ms")
    ax.axvline(0, color='gray', lw=0.7, linestyle=':')
    ax.set_xlim(-0.3, max_lag_s)
    ax.set_title(f"Group {name}")
    ax.set_xlabel("Lag (s)")
    ax.set_ylabel("Normalised cross-correlation")
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
print()


# ═══════════════════════════════════════════════════
# METHOD 2 – Step response analysis
# ═══════════════════════════════════════════════════
print("=" * 60)
print("Step-response dead-time estimates")
print("=" * 60)


def detect_steps(cmd, dt, min_abs, min_spacing_s):
    """Keep only large, well-spaced steps."""
    diff    = np.abs(np.diff(cmd))
    raw     = np.where(diff >= min_abs)[0]
    min_gap = int(min_spacing_s / dt)
    filtered, last = [], -min_gap - 1
    for i in raw:
        if i - last >= min_gap:
            filtered.append(i)
            last = i
    return np.array(filtered, dtype=int)


def find_response_onset(acc_smooth, step_idx, search_samples,
                        noise_floor, onset_mult, stability_mult,
                        pre_window):
    """
    Find first sample after step_idx where smoothed acc deviates from
    its pre-step mean by more than onset_mult × noise_floor.
    Rejects steps where pre-step acc is unstable (> stability_mult × noise_floor).
    Returns delay in samples, or None.
    """
    start = max(0, step_idx - pre_window)
    pre   = acc_smooth[start:step_idx]
    if len(pre) < 10:
        return None

    pre_mean = pre.mean()
    pre_std = pre.std()

    # Reject if pre-step is too noisy
    if pre_std > 2 * noise_floor:
        return None

    end = min(len(acc_smooth), step_idx + search_samples)

    # Derivative-based detection
    acc_diff = np.gradient(acc_smooth)

    pre_diff = acc_diff[start:step_idx]
    diff_std = np.std(pre_diff)

    onset_threshold = max(
        onset_mult * diff_std,
        1e-3
    )

    for k in range(step_idx, end):

        # Require some persistence (avoid spikes)
        window = acc_diff[k:k + 5]
        if len(window) < 5:
            continue
        if np.mean(window) < 0.5 * onset_threshold:
            continue

        if acc_diff[k] > onset_threshold:
            delay = (k - step_idx) * dt

            # Reject impossible early detections (< 2 samples ≈ 25 ms)
            if delay < 2 * dt:
                continue

            # Reject unrealistically late detections
            if delay > 0.6:
                return None

            return k - step_idx

    return None


search_samples = int(SEARCH_WINDOW_S / dt)

fig_sr, axes_sr = plt.subplots(2, 1, figsize=(14, 10))
fig_sr.suptitle("Method 2 – Step-response per thruster group", fontsize=13)

sr_results = {}

for ax, (name, g) in zip(axes_sr, groups.items()):
    steps    = detect_steps(g['cmd'], dt, g['min_step'], MIN_STEP_SPACING_S)
    delays_s = []

    for s_idx in steps:
        d = find_response_onset(
            g['acc_smooth'], s_idx, search_samples,
            g['noise_floor'], g['onset_mult'], g['stability_mult'],
            PRE_WINDOW_SAMPLES
        )
        if d is not None:
            delays_s.append(d * dt)

    sr_results[name] = delays_s

    d_ms = np.array(delays_s) * 1000
    med  = np.median(d_ms) if len(d_ms) else float('nan')
    std  = np.std(d_ms)    if len(d_ms) else float('nan')
    p25, p75 = (np.percentile(d_ms, [25, 75]) if len(d_ms) else (np.nan, np.nan))

    print(f"  Group {name}:")
    print(f"    Steps detected: {len(steps)},  responses found: {len(delays_s)}")
    print(f"    Dead time  median = {med:.1f} ms,  std = {std:.1f} ms")
    print(f"               25th pct = {p25:.1f} ms,  75th pct = {p75:.1f} ms\n")

    ax2 = ax.twinx()
    ax.plot(timestamp,  g['cmd'],        color='royalblue',  lw=0.8, label='Group cmd (sum |T|)')
    ax2.plot(timestamp, g['acc_smooth'], color='darkorange', lw=0.8, alpha=0.8, label=g['acc_label'])

    for s_idx in steps:
        ax.axvline(timestamp[s_idx], color='blue', alpha=0.25, lw=1.0, linestyle='--')

    ax.set_title(f"Group {name}  |  dead time median = {med:.1f} ms  (std = {std:.1f} ms)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Command (sum |T|)", color='royalblue')
    ax2.set_ylabel(g['acc_label'],      color='darkorange')
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()


# ═══════════════════════════════════════════════════
# Histogram
# ═══════════════════════════════════════════════════
fig_hist, axes_hist = plt.subplots(1, 2, figsize=(12, 5))
fig_hist.suptitle("Distribution of dead times", fontsize=13)

for ax, (name, delays_s) in zip(axes_hist, sr_results.items()):
    if not delays_s:
        ax.text(0.5, 0.5, 'No valid responses found\n(try lowering thresholds)',
                ha='center', va='center', fontsize=11)
        ax.set_title(f"Group {name}")
        continue
    d_ms = np.array(delays_s) * 1000
    p5, p95   = np.percentile(d_ms, [5, 95])
    d_clipped = d_ms[(d_ms >= p5) & (d_ms <= p95)]

    ax.hist(d_clipped, bins=40, color='steelblue', edgecolor='white', alpha=0.85)

    ax.axvline(np.median(d_ms), color='red',    linestyle='--', lw=1.5,
               label=f"Median = {np.median(d_ms):.1f} ms")
    ax.axvline(np.mean(d_ms),   color='orange', linestyle='--', lw=1.5,
               label=f"Mean   = {np.mean(d_ms):.1f} ms")
    ax.set_title(f"Group {name}")
    ax.set_xlabel("Dead time (ms)")
    ax.set_ylabel("Count")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.text(0.97, 0.92, f"5–95th pct shown\nn={len(d_clipped)}/{len(d_ms)}",
            transform=ax.transAxes, ha='right', va='top', fontsize=8, color='gray')

plt.tight_layout()

# ═══════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════
print("=" * 60)
print("SUMMARY")
print(f"{'Group':<22} {'CrossCorr (ms)':>16} {'Median (ms)':>13} {'Std (ms)':>10}")
print("-" * 64)
for name in groups:
    cc_ms  = cc_results[name] * 1000
    d_ms   = np.array(sr_results[name]) * 1000
    med_ms = np.median(d_ms) if len(d_ms) else float('nan')
    std_ms = np.std(d_ms)    if len(d_ms) else float('nan')
    print(f"  {name:<20} {cc_ms:>15.1f}   {med_ms:>11.1f}   {std_ms:>8.1f}")
print("=" * 60)

plt.show()