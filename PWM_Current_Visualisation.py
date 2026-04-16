"""
H-Bridge PWM Simulator — DC Motor RL-Load
==========================================
Unipolar PWM: during ON phase V_out = +V_DC, during OFF phase V_out = 0V
(freewheeling through diodes, slow decay mode).
Current through the RL load follows exponential charge/discharge with τ = L/R.

Interactive GUI with matplotlib text input fields.
All parameters are configurable:
  - V_DC [V], Frequency [Hz], Duty Cycle [%], Resistance [Ω], Inductance [mH]

Usage:
    python h_bridge_pwm_simulator.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button

# ── Simulation ──────────────────────────────────────────────────────────────

SAMPLES = 2000
CYCLES_SHOWN = 5
SETTLING_CYCLES = 30  # pre-run to reach steady state


def simulate(vdc, freq, duty, R, L_mH):
    """Solve RL circuit under unipolar H-Bridge PWM switching.

    Unipolar PWM: V_out = +V_DC during ON phase, V_out = 0 during OFF phase.
    During OFF, the freewheeling diodes keep current circulating through the
    motor at 0V (slow decay mode). Current always stays >= 0.
    """
    L = L_mH / 1000.0  # mH → H
    T = 1.0 / freq
    dt = (T * CYCLES_SHOWN) / SAMPLES
    tau = L / R
    t_on = duty * T

    # ── Settle to steady state ──
    current = 0.0
    n_settle = SETTLING_CYCLES * int(SAMPLES / CYCLES_SHOWN)
    for n in range(n_settle):
        t_in_cycle = (n * dt) % T
        v = vdc if t_in_cycle < t_on else 0.0
        i_ss = v / R
        current = i_ss + (current - i_ss) * np.exp(-dt / tau)
        current = max(current, 0.0)  # diode clamp: no negative current

    # ── Record cycles ──
    time = np.zeros(SAMPLES)
    voltage = np.zeros(SAMPLES)
    current_arr = np.zeros(SAMPLES)

    for i in range(SAMPLES):
        t = i * dt
        t_in_cycle = t % T
        v = vdc if t_in_cycle < t_on else 0.0
        i_ss = v / R
        current = i_ss + (current - i_ss) * np.exp(-dt / tau)
        current = max(current, 0.0)  # diode clamp

        time[i] = t * 1000.0  # → ms
        voltage[i] = v
        current_arr[i] = current

    stats = {
        "I_avg": np.mean(current_arr),
        "I_min": np.min(current_arr),
        "I_max": np.max(current_arr),
        "I_ripple": np.max(current_arr) - np.min(current_arr),
        "V_avg": vdc * duty,
        "tau_us": tau * 1e6,
    }
    return time, voltage, current_arr, stats


# ── Default parameters ──────────────────────────────────────────────────────

params = {
    "V_DC [V]":         12.0,
    "Freq [Hz]":        25000.0,
    "Duty [%]":         50.0,
    "R [Ω]":            40.0,
    "L [mH]":           1.5,
}

# ── Initial simulation ──────────────────────────────────────────────────────

time, voltage, current, stats = simulate(
    params["V_DC [V]"], params["Freq [Hz]"],
    params["Duty [%]"] / 100.0, params["R [Ω]"], params["L [mH]"]
)

# ── Figure setup ────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(13, 7.5), facecolor="#0f172a")
fig.canvas.manager.set_window_title("H-Bridge PWM Simulator")

# Main axes: voltage (top) and current (bottom)
ax_v = fig.add_axes([0.32, 0.55, 0.63, 0.38])
ax_i = fig.add_axes([0.32, 0.10, 0.63, 0.38])

# Shared styling
for ax in (ax_v, ax_i):
    ax.set_facecolor("#1e293b")
    ax.tick_params(colors="#94a3b8", labelsize=9)
    ax.spines["bottom"].set_color("#334155")
    ax.spines["left"].set_color("#334155")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, color="#334155", linewidth=0.5, linestyle="--", alpha=0.6)

# ── Plot lines ──────────────────────────────────────────────────────────────

COLOR_V = "#f59e0b"
COLOR_I = "#22d3ee"

line_v, = ax_v.step(time, voltage, where="post", color=COLOR_V, linewidth=1.5)
ax_v.set_ylabel("Voltage [V]", color=COLOR_V, fontsize=11, fontweight="bold")
ax_v.set_title("H-Bridge Output Voltage (Unipolar PWM)", color="#e2e8f0",
               fontsize=12, fontweight="bold", pad=8)
ax_v.set_xlabel("")
ax_v.axhline(0, color="#64748b", linewidth=0.8)

line_i, = ax_i.plot(time, current, color=COLOR_I, linewidth=1.5)
fill_i = ax_i.fill_between(time, current, alpha=0.12, color=COLOR_I)
ax_i.set_ylabel("Current [A]", color=COLOR_I, fontsize=11, fontweight="bold")
ax_i.set_xlabel("Time [ms]", color="#94a3b8", fontsize=10)
ax_i.set_title("Motor Current (RL Load)", color="#e2e8f0",
               fontsize=12, fontweight="bold", pad=8)
ax_i.axhline(0, color="#64748b", linewidth=0.8)

# ── Stats text ──────────────────────────────────────────────────────────────

stats_text = fig.text(
    0.03, 0.08, "", fontsize=9, color="#94a3b8",
    fontfamily="monospace", verticalalignment="bottom",
    linespacing=1.8
)


def format_stats(s):
    return (
        f"── Steady-State ──\n"
        f"  I avg   = {s['I_avg']*1000:+8.2f} mA\n"
        f"  I min   = {s['I_min']*1000:+8.2f} mA\n"
        f"  I max   = {s['I_max']*1000:+8.2f} mA\n"
        f"  Ripple  = {s['I_ripple']*1000:8.2f} mA\n"
        f"  V avg   = {s['V_avg']:+8.2f} V\n"
        f"  τ (L/R) = {s['tau_us']:8.1f} μs"
    )


stats_text.set_text(format_stats(stats))

# ── Title ───────────────────────────────────────────────────────────────────

fig.text(0.03, 0.97, "H-Bridge PWM", fontsize=16, color="#f59e0b",
         fontweight="bold", verticalalignment="top", fontfamily="monospace")
fig.text(0.03, 0.935, "DC Motor RL-Load Simulator", fontsize=10,
         color="#64748b", verticalalignment="top", fontfamily="monospace")

# ── Input fields (left panel) ──────────────────────────────────────────────

textboxes = {}
box_positions = {
    "V_DC [V]":   0.85,
    "Freq [Hz]":  0.76,
    "Duty [%]":   0.67,
    "R [Ω]":      0.58,
    "L [mH]":     0.49,
}

textbox_style = dict(
    color="#1e293b",
    hovercolor="#334155",
)

for name, y in box_positions.items():
    # Label
    fig.text(0.02, y + 0.025, name, fontsize=10, color="#cbd5e1",
             fontweight="bold", fontfamily="monospace",
             verticalalignment="center")
    # Text input box
    ax_box = fig.add_axes([0.14, y, 0.12, 0.04])
    tb = TextBox(ax_box, "", initial=str(params[name]), **textbox_style)
    tb.label.set_color("#f1f5f9")
    tb.label.set_fontfamily("monospace")
    # Style the text
    for child in ax_box.get_children():
        if hasattr(child, "set_color"):
            child.set_color("#f1f5f9")
        if hasattr(child, "set_fontfamily"):
            child.set_fontfamily("monospace")
    ax_box.set_facecolor("#1e293b")
    for spine in ax_box.spines.values():
        spine.set_color("#475569")
    textboxes[name] = tb

# ── Update button ───────────────────────────────────────────────────────────

ax_btn = fig.add_axes([0.04, 0.40, 0.20, 0.05])
btn = Button(ax_btn, "▶  SIMULATE", color="#0f766e", hovercolor="#14b8a6")
btn.label.set_color("#f1f5f9")
btn.label.set_fontweight("bold")
btn.label.set_fontfamily("monospace")
btn.label.set_fontsize(11)
for spine in ax_btn.spines.values():
    spine.set_color("#14b8a6")


def update(event=None):
    """Read all text fields, re-simulate, update plots."""
    global fill_i

    try:
        vdc  = float(textboxes["V_DC [V]"].text)
        freq = float(textboxes["Freq [Hz]"].text)
        duty = float(textboxes["Duty [%]"].text) / 100.0
        R    = float(textboxes["R [Ω]"].text)
        L_mH = float(textboxes["L [mH]"].text)
    except ValueError:
        return  # silently ignore bad input

    # Clamp values
    vdc  = max(0.1, vdc)
    freq = max(10, freq)
    duty = np.clip(duty, 0.01, 0.99)
    R    = max(0.01, R)
    L_mH = max(0.001, L_mH)

    time_new, voltage_new, current_new, stats_new = simulate(vdc, freq, duty, R, L_mH)

    # Update voltage plot
    line_v.set_data(time_new, voltage_new)
    ax_v.set_xlim(time_new[0], time_new[-1])
    ax_v.set_ylim(-vdc * 0.1, vdc * 1.2)

    # Update current plot
    line_i.set_data(time_new, current_new)
    fill_i.remove()
    fill_i = ax_i.fill_between(time_new, current_new, alpha=0.12, color=COLOR_I)
    ax_i.set_xlim(time_new[0], time_new[-1])
    i_max = stats_new["I_max"] * 1.3
    if i_max < 1e-9:
        i_max = 0.01
    ax_i.set_ylim(-i_max * 0.05, i_max)

    # Update stats
    stats_text.set_text(format_stats(stats_new))

    fig.canvas.draw_idle()


btn.on_clicked(update)

# Also allow Enter key in any text box to trigger update
for tb in textboxes.values():
    tb.on_submit(update)

# ── Show ────────────────────────────────────────────────────────────────────

plt.show()