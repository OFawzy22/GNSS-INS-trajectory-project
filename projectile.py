"""
SN307: GNSS INS Integration
Assignment 1 — Projectile Motion Trajectory

Inputs : V0 (initial speed), theta (angle), ax, ay (accelerations)
Outputs: Vx, Vy at every point + x-y trajectory plot

Equations used (from Lecture 3 — Dr. Mohamed Elfarran):
    Vx = Vox + ax * t
    Vy = Voy + ay * t
    x  = Vox*t + 0.5*ax*t^2
    y  = Voy*t + 0.5*ay*t^2

Team:
    - Omar  Mohamed Fawzy  Abou El-Roos
    - Abdel Hamid Ahmed    Abdel Hamid
    - Ali   Mohamed Hamed  Amin
    - Afnan Mohamed Fawzy  Ahmed
    - Yara  Mohamed Sayed  Kamel
    - Basmala Mohamed Owes Mahmoud
    - Sama  Ahmed  Mohamed Ali
"""

import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

matplotlib.rcParams['font.family'] = 'DejaVu Sans'


# ════════════════════════════════════════════════════════════════
#  CORE PHYSICS
# ════════════════════════════════════════════════════════════════

def compute_trajectory(v0: float, theta_deg: float,
                       ax: float, ay: float,
                       dt: float = 0.001):
    """
    Compute projectile trajectory step-by-step.

    Parameters
    ----------
    v0        : initial speed (m/s)
    theta_deg : launch angle (degrees)
    ax        : horizontal acceleration (m/s^2)  — 0 for standard projectile
    ay        : vertical   acceleration (m/s^2)  — negative for gravity
    dt        : time step (s)

    Returns
    -------
    dict with keys: t, x, y, vx, vy  (all numpy arrays)
    """
    theta = math.radians(theta_deg)

    # Initial velocity components
    vox = v0 * math.cos(theta)   # Vox = V0 * cos(theta)
    voy = v0 * math.sin(theta)   # Voy = V0 * sin(theta)

    # ── Find flight duration ──────────────────────────────────────
    # Solve  y = Voy*t + 0.5*ay*t^2 = 0  for t > 0
    # Case 1: ay == 0  →  t = -Voy / ... infinite if Voy=0, else no landing
    # Case 2: ay != 0  →  quadratic  0.5*ay*t^2 + Voy*t = 0
    #           t*(0.5*ay*t + Voy) = 0  →  t = -Voy / (0.5*ay) = -2*Voy/ay
    if ay == 0:
        if voy == 0:
            t_total = 1.0   # horizontal motion only, show 1 s
        else:
            t_total = abs(10 * voy / (abs(ax) + 0.01))  # fallback
    else:
        t_land = -2.0 * voy / ay   # landing time (positive root)
        if t_land <= 0:
            t_land = abs(2.0 * voy / ay)
        t_total = t_land

    # ── Build time array ─────────────────────────────────────────
    n = max(int(t_total / dt) + 2, 100)
    t_arr = np.linspace(0, t_total, n)

    # ── Equations of kinematics (applied at each time point) ─────
    vx_arr = vox + ax * t_arr          # Vx = Vox + ax*t
    vy_arr = voy + ay * t_arr          # Vy = Voy + ay*t
    x_arr  = vox * t_arr + 0.5 * ax * t_arr**2   # x = Vox*t + 0.5*ax*t^2
    y_arr  = voy * t_arr + 0.5 * ay * t_arr**2   # y = Voy*t + 0.5*ay*t^2

    # Clip y at ground (y = 0) after landing
    for i in range(len(y_arr)):
        if y_arr[i] < 0 and t_arr[i] > dt:
            y_arr[i:] = 0.0
            x_arr[i:] = x_arr[i]
            vy_arr[i:] = 0.0
            t_arr = t_arr[:i+1]
            x_arr  = x_arr[:i+1]
            y_arr  = y_arr[:i+1]
            vx_arr = vx_arr[:i+1]
            vy_arr = vy_arr[:i+1]
            break

    return {
        "t": t_arr, "x": x_arr, "y": y_arr,
        "vx": vx_arr, "vy": vy_arr,
        "vox": vox, "voy": voy,
        "t_total": t_arr[-1],
    }


def key_points_table(result: dict, ax: float, ay: float, n_steps: int = 10):
    """
    Return a list of dicts — one row per sampled time point.
    Includes: t, x, y, vx, vy, speed, angle
    """
    t_vals = np.linspace(0, result["t_total"], n_steps + 1)
    rows = []
    vox, voy = result["vox"], result["voy"]
    for t in t_vals:
        vx    = vox + ax * t
        vy    = voy + ay * t
        x     = vox * t + 0.5 * ax * t**2
        y     = voy * t + 0.5 * ay * t**2
        y     = max(y, 0.0)
        speed = math.hypot(vx, vy)
        angle = math.degrees(math.atan2(vy, vx))
        rows.append(dict(t=t, x=x, y=y, vx=vx, vy=vy, speed=speed, angle=angle))
    return rows


# ════════════════════════════════════════════════════════════════
#  PLOTTING
# ════════════════════════════════════════════════════════════════

def plot_results(v0, theta_deg, ax_input, ay_input, result, rows):
    """Create a 2×2 figure with trajectory, velocity components, speed, and table."""

    t  = result["t"]
    x  = result["x"]
    y  = result["y"]
    vx = result["vx"]
    vy = result["vy"]
    vox = result["vox"]
    voy = result["voy"]
    t_total = result["t_total"]

    # Peak point
    peak_idx = int(np.argmax(y))
    x_peak = x[peak_idx]
    y_peak = y[peak_idx]
    t_peak = t[peak_idx]

    fig = plt.figure(figsize=(15, 10))
    fig.patch.set_facecolor('#0d1117')

    gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.35,
                          left=0.07, right=0.97, top=0.88, bottom=0.07)

    ax1 = fig.add_subplot(gs[0, 0])   # x-y trajectory
    ax2 = fig.add_subplot(gs[0, 1])   # Vx, Vy vs time
    ax3 = fig.add_subplot(gs[1, 0])   # Speed vs time
    ax4 = fig.add_subplot(gs[1, 1])   # Data table

    for a in [ax1, ax2, ax3, ax4]:
        a.set_facecolor('#161b22')
        a.tick_params(colors='#c9d1d9', labelsize=8)
        for spine in a.spines.values():
            spine.set_edgecolor('#30363d')

    CYAN   = '#58a6ff'
    ORANGE = '#f78166'
    GREEN  = '#3fb950'
    YELLOW = '#e3b341'
    GRAY   = '#8b949e'

    # ── 1. x–y Trajectory ────────────────────────────────────────
    ax1.plot(x, y, color=CYAN, linewidth=2.5, label='Trajectory', zorder=3)
    ax1.fill_between(x, 0, y, color=CYAN, alpha=0.07)

    # Velocity arrows (sampled)
    n_arrows = 10
    arrow_t = np.linspace(0, t_total, n_arrows)
    scale = t_total * 0.08
    for at in arrow_t:
        avx = vox + ax_input * at
        avy = voy + ay_input * at
        ax_pos = vox * at + 0.5 * ax_input * at**2
        ay_pos = max(voy * at + 0.5 * ay_input * at**2, 0.0)
        mag = math.hypot(avx, avy)
        if mag > 0:
            ax1.annotate('', xy=(ax_pos + avx/mag*scale,
                                  ay_pos + avy/mag*scale),
                         xytext=(ax_pos, ay_pos),
                         arrowprops=dict(arrowstyle='->', color=ORANGE,
                                         lw=1.2, mutation_scale=12))

    # Key markers
    ax1.scatter([0], [0],       color=GREEN,  s=80, zorder=5, label='Launch')
    ax1.scatter([x_peak], [y_peak], color=YELLOW, s=80, zorder=5,
                label=f'Peak  ({x_peak:.1f} m, {y_peak:.1f} m)')
    ax1.scatter([x[-1]], [0],   color=ORANGE, s=80, zorder=5,
                label=f'Landing  ({x[-1]:.1f} m, 0)')

    ax1.annotate(f'Peak\n({x_peak:.1f}, {y_peak:.1f}) m',
                 xy=(x_peak, y_peak),
                 xytext=(x_peak + t_total * 0.5, y_peak + y_peak * 0.1),
                 color=YELLOW, fontsize=7.5,
                 arrowprops=dict(arrowstyle='->', color=YELLOW, lw=0.8))

    ax1.set_xlabel('x  (m)', color=GRAY)
    ax1.set_ylabel('y  (m)', color=GRAY)
    ax1.set_title('x – y  Trajectory', color='white', fontweight='bold')
    ax1.legend(fontsize=7.5, facecolor='#21262d', labelcolor='white',
               edgecolor='#30363d')
    ax1.grid(True, color='#21262d', linewidth=0.8)
    ax1.set_ylim(bottom=0)

    # ── 2. Vx & Vy vs time ──────────────────────────────────────
    ax2.plot(t, vx, color=CYAN,   linewidth=2, label='Vx (m/s)')
    ax2.plot(t, vy, color=ORANGE, linewidth=2, label='Vy (m/s)')
    ax2.axhline(0, color=GRAY, linewidth=0.7, linestyle='--')
    ax2.axvline(t_peak, color=YELLOW, linewidth=1, linestyle=':',
                label=f'Peak  t={t_peak:.2f} s')

    # Annotate Vy=0 at peak
    ax2.scatter([t_peak], [0], color=YELLOW, s=50, zorder=5)
    ax2.annotate('Vy = 0\nat peak', xy=(t_peak, 0),
                 xytext=(t_peak + t_total*0.05, max(vy)*0.3),
                 color=YELLOW, fontsize=7.5,
                 arrowprops=dict(arrowstyle='->', color=YELLOW, lw=0.8))

    ax2.set_xlabel('t  (s)', color=GRAY)
    ax2.set_ylabel('Velocity  (m/s)', color=GRAY)
    ax2.set_title('Vx  &  Vy  vs  Time', color='white', fontweight='bold')
    ax2.legend(fontsize=7.5, facecolor='#21262d', labelcolor='white',
               edgecolor='#30363d')
    ax2.grid(True, color='#21262d', linewidth=0.8)

    # ── 3. Speed vs time ─────────────────────────────────────────
    speed = np.hypot(vx, vy)
    ax3.plot(t, speed, color=GREEN, linewidth=2, label='|V| (m/s)')
    ax3.axvline(t_peak, color=YELLOW, linewidth=1, linestyle=':',
                label=f'Peak  t={t_peak:.2f} s')
    ax3.set_xlabel('t  (s)', color=GRAY)
    ax3.set_ylabel('Speed  |V|  (m/s)', color=GRAY)
    ax3.set_title('Speed  |V|  vs  Time', color='white', fontweight='bold')
    ax3.legend(fontsize=7.5, facecolor='#21262d', labelcolor='white',
               edgecolor='#30363d')
    ax3.grid(True, color='#21262d', linewidth=0.8)

    # ── 4. Data table ────────────────────────────────────────────
    ax4.axis('off')
    col_labels = ['t (s)', 'x (m)', 'y (m)', 'Vx (m/s)', 'Vy (m/s)', '|V| (m/s)']
    table_data = [
        [f"{r['t']:.3f}", f"{r['x']:.3f}", f"{r['y']:.3f}",
         f"{r['vx']:.3f}", f"{r['vy']:.3f}", f"{r['speed']:.3f}"]
        for r in rows
    ]
    tbl = ax4.table(
        cellText=table_data,
        colLabels=col_labels,
        loc='center', cellLoc='center'
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.5)
    tbl.scale(1, 1.3)

    for (row_i, col_i), cell in tbl.get_celld().items():
        cell.set_facecolor('#0d1117' if row_i % 2 == 0 else '#161b22')
        cell.set_edgecolor('#30363d')
        cell.set_text_props(color='white' if row_i > 0 else CYAN, fontsize=7.5)
        if row_i == 0:
            cell.set_facecolor('#1f6feb')

    ax4.set_title('Vx & Vy at Each Point', color='white', fontweight='bold', pad=12)

    # ── Main title ────────────────────────────────────────────────
    fig.text(0.5, 0.95,
             'SN307: GNSS INS Integration  —  Assignment 1  |  Projectile Motion',
             ha='center', color='white', fontsize=13, fontweight='bold')
    fig.text(0.5, 0.925,
             f'V₀ = {v0} m/s   |   θ = {theta_deg}°   |   '
             f'ax = {ax_input} m/s²   |   ay = {ay_input} m/s²   |   '
             f'Vox = {vox:.3f} m/s   |   Voy = {voy:.3f} m/s',
             ha='center', color=GRAY, fontsize=9)

    # ── Team credits ───────────────────────────────────────────────
    team = ("Omar Fawzy  •  Abdel Hamid Ahmed  •  Ali Hamed  •  "
            "Afnan Fawzy  •  Yara Sayed  •  Basmala Owes  •  Sama Ahmed")
    fig.text(0.5, 0.01, f"Team: {team}  |  Dr. Mohamed Elfarran",
             ha='center', color='#484f58', fontsize=7)

    return fig


# ════════════════════════════════════════════════════════════════
#  CONSOLE OUTPUT
# ════════════════════════════════════════════════════════════════

def print_summary(v0, theta_deg, ax_input, ay_input, result, rows):
    vox = result["vox"]
    voy = result["voy"]

    print()
    print("=" * 70)
    print(f"{'SN307 — Projectile Motion':^70}")
    print("=" * 70)
    print(f"  V0  = {v0} m/s          theta = {theta_deg}°")
    print(f"  Vox = V0·cos(θ) = {vox:.4f} m/s")
    print(f"  Voy = V0·sin(θ) = {voy:.4f} m/s")
    print(f"  ax  = {ax_input} m/s²          ay = {ay_input} m/s²")
    print("-" * 70)

    # Peak
    idx_peak = max(range(len(result["y"])), key=lambda i: result["y"][i])
    print(f"  Max Height   : {result['y'][idx_peak]:.4f} m  "
          f"at t = {result['t'][idx_peak]:.4f} s")
    print(f"  Range        : {result['x'][-1]:.4f} m")
    print(f"  Flight Time  : {result['t_total']:.4f} s")
    print("=" * 70)
    print()
    print(f"  {'t (s)':<10} {'x (m)':<12} {'y (m)':<12} "
          f"{'Vx (m/s)':<12} {'Vy (m/s)':<12} {'|V| (m/s)':<12}")
    print("  " + "-" * 68)
    for r in rows:
        print(f"  {r['t']:<10.3f} {r['x']:<12.4f} {r['y']:<12.4f} "
              f"  {r['vx']:<12.4f} {r['vy']:<12.4f} {r['speed']:<12.4f}")
    print("=" * 70)


# ════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════

def get_float(prompt, default):
    raw = input(f"  {prompt} [default = {default}]: ").strip()
    return float(raw) if raw else default


def main():
    print()
    print("=" * 70)
    print("  SN307: GNSS INS Integration  —  Projectile Motion Calculator")
    print("  Dr. Mohamed Elfarran  |  Aerospace Engineering")
    print("=" * 70)
    print()
    print("  Enter inputs (press Enter to use default values):")
    print()

    v0        = get_float("Initial speed V0  (m/s)", 22.0)
    theta_deg = get_float("Launch angle  θ   (deg)", 40.0)
    ax_input  = get_float("Acceleration  ax  (m/s²) [0 = no horizontal accel]", 0.0)
    ay_input  = get_float("Acceleration  ay  (m/s²) [-9.80 = gravity down]",   -9.80)

    # Validation
    if not (0 < theta_deg < 90):
        print("  [!] Angle must be between 0° and 90°. Setting to 40°.")
        theta_deg = 40.0
    if v0 <= 0:
        print("  [!] Speed must be positive. Setting to 22 m/s.")
        v0 = 22.0

    result = compute_trajectory(v0, theta_deg, ax_input, ay_input)
    rows   = key_points_table(result, ax_input, ay_input, n_steps=10)

    print_summary(v0, theta_deg, ax_input, ay_input, result, rows)

    import os
    out_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "trajectory.png")

    matplotlib.use('TkAgg')   # interactive window
    fig = plot_results(v0, theta_deg, ax_input, ay_input, result, rows)
    fig.savefig(out_path, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    print(f"\n  [✓] Plot saved → {out_path}")
    print("  [✓] Showing interactive window (close it to exit)...\n")
    plt.show()


if __name__ == "__main__":
    main()
