# Projectile Motion Trajectory — SN307

**Course:** SN307: GNSS INS Integration  
**Assignment:** 1 — Draw x,y Projectile Path  
**Instructor:** Dr. Mohamed Elfarran, Assistant Professor of Aerospace Engineering

---

## 👥 Team Members

| # | Name |
|---|------|
| 1 | عمر محمد فوزي أبوالروس |
| 2 | عبدالحميد أحمد عبدالحميد |
| 3 | علي محمد حامد أمين |
| 4 | افنان محمد فوزي أحمد |
| 5 | يارا محمد سيد كامل |
| 6 | بسملة محمد عويس محمود |
| 7 | سما احمد محمد علي |

---

## 📌 Problem Statement

Draw the **x, y projectile path** for the Kickoff problem from Lecture 3.

The program accepts **any** initial velocity, angle, and accelerations — then computes `Vx` and `Vy` at **every point** along the trajectory using the kinematic equations from the lecture.

---

## 📐 Equations Used (Lecture 3)

Decompose initial velocity:

$$V_{ox} = V_0 \cdot \cos\theta$$
$$V_{oy} = V_0 \cdot \sin\theta$$

At **every time step** `t`:

| Quantity | Equation | Description |
|---------|----------|-------------|
| $V_x$ | $V_x = V_{ox} + a_x \cdot t$ | Horizontal velocity |
| $V_y$ | $V_y = V_{oy} + a_y \cdot t$ | Vertical velocity |
| $x$   | $x = V_{ox} \cdot t + \frac{1}{2} a_x t^2$ | Horizontal position |
| $y$   | $y = V_{oy} \cdot t + \frac{1}{2} a_y t^2$ | Vertical position |

---

## 🚀 How to Run

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Run the program

```bash
python projectile.py
```

### 3. Enter inputs when prompted

```
Enter initial speed V0  (m/s) [default = 22]:    → e.g. 22
Enter launch angle  θ   (deg) [default = 40]:    → e.g. 40
Enter acceleration  ax  (m/s²) [default = 0]:    → e.g. 0
Enter acceleration  ay  (m/s²) [default = -9.80]:→ e.g. -9.80
```

> Press **Enter** on any field to use the default (lecture values).

---

## 📊 Outputs

The program produces:

1. **Console table** — `t`, `x`, `y`, `Vx`, `Vy`, `|V|` at 10 evenly-spaced points
2. **Interactive plot window** with 4 panels:
   - `x–y Trajectory` with velocity direction arrows
   - `Vx & Vy vs Time`
   - `Speed |V| vs Time`
   - `Data table` of all computed values
3. **PNG file** saved to `output/trajectory.png`

---

## 🧪 Example — Lecture Kickoff Problem

```
V0 = 22 m/s  |  θ = 40°  |  ax = 0  |  ay = -9.80 m/s²
```

| Point | t (s) | x (m) | y (m) | Vx (m/s) | Vy (m/s) |
|-------|--------|--------|--------|-----------|-----------|
| Launch  | 0.000 | 0.000  | 0.000  | 16.853 | +14.141 |
| t=0.5s  | 0.500 | 8.427  | 5.846  | 16.853 | +9.241  |
| t=1.0s  | 1.000 | 16.853 | 9.241  | 16.853 | +4.341  |
| **Peak**| 1.443 | 24.319 | **10.203** | 16.853 | **0.000** |
| t=2.0s  | 2.000 | 33.706 | 8.683  | 16.853 | −5.459  |
| Landing | 2.886 | 48.637 | 0.000  | 16.853 | −14.141 |

**Max Height:** 10.20 m · **Range:** 48.64 m · **Flight Time:** 2.89 s

---

## 📁 File Structure

```
projectile_motion/
├── projectile.py       ← Main program
├── requirements.txt    ← Python dependencies
├── README.md           ← This file
└── output/
    └── trajectory.png  ← Generated plot
```

---

## 📄 License

Academic project — SN307, Aerospace Engineering Department.
