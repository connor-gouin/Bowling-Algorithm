
Bowling Bot GUI (Tkinter)
=========================

What you get
------------
- Clickable "ghost pins": click any pin to toggle standing on/off.
- "Generate Shot" chooses a plan and previews it:
  * If **2 pins** stand → simple *ghost‑ball* carom: aim so ball sends one pin into the other.
  * If **>2 pins** stand → complex scan that picks a straight path that **maximizes direct hits** (or choose "min_angle" strategy).
  * If **1 pin** stands → aim straight at it.
- Blue line = ball path. Dashed blue circle = ghost‑ball location (two‑pin plan). Green rings = predicted hit pins.
- Assumptions: no spin, negligible friction, straight trajectories.

Run it
------
1. Ensure you have Python 3.9+.
2. Tkinter is included with most Python installs. If missing on Linux, install your distro's Tk (e.g., `sudo apt install python3-tk`).
3. In a terminal:
   ```bash
   cd bowling_bot_gui
   python main.py
   ```

Files
-----
- `main.py` – GUI, drawing, and user interactions.
- `pins.py` – World configuration and standard 10‑pin layout with IDs (1..10).
- `solver.py` – Geometry and shot planners (simple ghost‑ball + complex scan).

Notes
-----
- Coordinates use a simplified unit where the **pin spacing = 1**; ball/pin radii are approximate for clean geometry.
- Complex planner uses a straight‑line "corridor" test with width = ball_r + pin_r to count direct ball hits. It’s a clean heuristic for pick‑the‑best straight shot.
- You can tweak `WORLD` constants in `pins.py` to change sizes/layout.
