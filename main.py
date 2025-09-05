
"""
Bowling Bot GUI (Tkinter)
-------------------------
- Click pins to toggle (ghost/off vs standing/on).
- "Generate Shot" chooses a plan:
    * If 2 pins stand -> simple (ghost-ball-style) shot to carom one pin into the other.
    * If >2 pins -> complex scan to maximize direct ball hits along a straight path.
    * If 1 pin -> aim straight at it.
- Straight bowling only (no spin). Friction neglected.
- Launch location is static at the foul line center.
"""

import tkinter as tk
from tkinter import ttk, messagebox
from math import sqrt, cos, sin, atan2, pi
from pins import PinLayout, WORLD
from solver import choose_simple_shot, choose_complex_shot, choose_single_pin_shot
from solver import compute_hits_for_line, line_from_two_points

# ---------------------- Drawing / Coordinate mapping ----------------------

class Drawer:
    def __init__(self, canvas: tk.Canvas, world_to_px, px_to_world):
        self.c = canvas
        self.world_to_px = world_to_px
        self.px_to_world = px_to_world

    def draw_lane(self):
        self.c.delete("lane")
        # Lane rectangle
        x0, y0 = self.world_to_px(-WORLD['lane_width']/2, 0)
        x1, y1 = self.world_to_px(WORLD['lane_width']/2, WORLD['lane_length'])
        self.c.create_rectangle(x0, y0, x1, y1, fill="#e4e2d1", outline="#b6b29b", tags="lane")
        # Foul line
        fx0, fy = self.world_to_px(-WORLD['lane_width']/2, 0)
        fx1, _ = self.world_to_px(WORLD['lane_width']/2, 0)
        self.c.create_line(fx0, fy, fx1, fy, fill="#9b5d2e", width=3, tags="lane")
        # Head pin line (at y = Y0 of the first pin)
        hx0, hy = self.world_to_px(-WORLD['lane_width']/2, WORLD['head_y'])
        hx1, _  = self.world_to_px(WORLD['lane_width']/2, WORLD['head_y'])
        self.c.create_line(hx0, hy, hx1, hy, fill="#d4cfc4", width=1, dash=(2,3), tags="lane")

    def draw_pins(self, layout):
        self.c.delete("pin")
        # Draw all pin positions; standing pins are filled, ghosts are outline only
        for p in layout.pins:
            x,y = p.pos
            px,py = self.world_to_px(x,y)
            r = WORLD['pin_r'] * WORLD['px_per_u']
            if p.standing:
                self.c.create_oval(px-r, py-r, px+r, py+r, fill="white", outline="black", width=2, tags=("pin", f"pin_{p.pid}"))
            else:
                self.c.create_oval(px-r, py-r, px+r, py+r, outline="#888", width=1, dash=(3,2), tags=("pin", f"pin_{p.pid}"))

    def draw_ball_origin(self):
        self.c.delete("ball")
        bx, by = self.world_to_px(*WORLD['ball_origin'])
        r = WORLD['ball_r'] * WORLD['px_per_u']
        self.c.create_oval(bx-r, by-r, bx+r, by+r, fill="#333", outline="black", width=1, tags="ball")

    def draw_shot_preview(self, aim_line=None, ghost_ball=None, hits=None):
        # Clear old preview
        self.c.delete("preview")
        if aim_line is not None:
            p0, p1 = aim_line  # world points (x,y)
            x0,y0 = self.world_to_px(*p0)
            x1,y1 = self.world_to_px(*p1)
            self.c.create_line(x0,y0,x1,y1, width=3, fill="#1e88e5", tags="preview")

        if ghost_ball is not None:
            gx,gy = ghost_ball
            px,py = self.world_to_px(gx,gy)
            r = WORLD['ball_r'] * WORLD['px_per_u']
            self.c.create_oval(px-r, py-r, px+r, py+r, outline="#1e88e5", width=2, dash=(3,3), tags="preview")

        # highlight predicted hits
        if hits:
            for p in hits:
                x,y = p.pos
                px,py = self.world_to_px(x,y)
                rr = WORLD['pin_r'] * WORLD['px_per_u']
                self.c.create_oval(px-rr, py-rr, px+rr, py+rr, outline="#2e7d32", width=3, tags="preview")


# ---------------------- GUI App ----------------------

class BowlingBotApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Bowling Bot GUI")
        self.geometry("1100x700")
        self.resizable(False, False)

        # World <-> pixel mapping
        self.canvas_margin = 40
        self.canvas_w = 820
        self.canvas_h = 620

        self.canvas = tk.Canvas(self, width=self.canvas_w, height=self.canvas_h, bg="#f7f7f7", highlightthickness=0)
        self.canvas.grid(row=0, column=0, padx=20, pady=10, sticky="nsew")

        # Compute scale so that lane fits vertically with margins
        usable_h = self.canvas_h - 2*self.canvas_margin
        px_per_u = usable_h / WORLD['lane_length']
        WORLD['px_per_u'] = px_per_u

        # center x
        def world_to_px(x, y):
            cx = self.canvas_w/2
            px = cx + x * px_per_u
            py = self.canvas_h - self.canvas_margin - y * px_per_u
            return px, py

        def px_to_world(px, py):
            cx = self.canvas_w/2
            x = (px - cx) / px_per_u
            y = (self.canvas_h - self.canvas_margin - py) / px_per_u
            return x, y

        self.world_to_px = world_to_px
        self.px_to_world = px_to_world

        self.drawer = Drawer(self.canvas, world_to_px, px_to_world)
        self.layout = PinLayout()  # default is a full rack

        # Controls
        self.ctrl_frame = ttk.Frame(self)
        self.ctrl_frame.grid(row=0, column=1, padx=(0,20), pady=10, sticky="ns")

        ttk.Label(self.ctrl_frame, text="Bowling Bot", font=("Segoe UI", 16, "bold")).grid(row=0, column=0, pady=(0,10))

        self.strategy_var = tk.StringVar(value="max_hits")
        ttk.Label(self.ctrl_frame, text="Strategy (>2 pins):").grid(row=1, column=0, sticky="w", pady=(5,0))
        self.strategy_cb = ttk.Combobox(self.ctrl_frame, state="readonly", textvariable=self.strategy_var,
                                        values=["max_hits", "min_angle"])
        self.strategy_cb.grid(row=2, column=0, sticky="ew", pady=(0,5))

        self.sample_var = tk.IntVar(value=181)
        ttk.Label(self.ctrl_frame, text="Angle Samples:").grid(row=3, column=0, sticky="w")
        ttk.Spinbox(self.ctrl_frame, from_=61, to=721, increment=20, textvariable=self.sample_var, width=8).grid(row=4, column=0, sticky="w")

        ttk.Button(self.ctrl_frame, text="Generate Shot", command=self.on_generate_shot).grid(row=5, column=0, pady=10, sticky="ew")
        ttk.Button(self.ctrl_frame, text="Reset Rack", command=self.on_reset_rack).grid(row=6, column=0, sticky="ew")
        ttk.Button(self.ctrl_frame, text="Clear All", command=self.on_clear_all).grid(row=7, column=0, sticky="ew")

        ttk.Separator(self.ctrl_frame, orient="horizontal").grid(row=8, column=0, sticky="ew", pady=10)

        # Presets
        ttk.Label(self.ctrl_frame, text="Spare Presets:").grid(row=9, column=0, sticky="w")
        self.preset_var = tk.StringVar(value="(none)")
        self.preset_cb = ttk.Combobox(self.ctrl_frame, state="readonly", textvariable=self.preset_var,
                                        values=["(none)", "7-10", "2-4-5", "3-6-10", "2-7", "3-10"])
        self.preset_cb.grid(row=10, column=0, sticky="ew")
        ttk.Button(self.ctrl_frame, text="Load Preset", command=self.on_load_preset).grid(row=11, column=0, sticky="ew", pady=(5,0))

        ttk.Label(self.ctrl_frame, text="Tips:\n- Click a pin to toggle.\n- Blue line = ball path\n- Dashed blue circle = ghost-ball\n- Green outline = predicted hits",
                  justify="left", wraplength=200).grid(row=12, column=0, sticky="w", pady=10)

        # Bindings
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        # Initial draw
        self.redraw()

    # ------------------ Event Handlers ------------------
    def on_canvas_click(self, ev):
        wx, wy = self.px_to_world(ev.x, ev.y)
        # Find a pin within selection radius
        sel_r = WORLD['pin_r'] * 1.8
        closest = None
        best_d2 = (sel_r*sel_r)+1
        for p in self.layout.pins:
            dx = wx - p.pos[0]
            dy = wy - p.pos[1]
            d2 = dx*dx + dy*dy
            if d2 <= sel_r*sel_r and d2 < best_d2:
                best_d2 = d2
                closest = p
        if closest:
            closest.standing = not closest.standing
            self.redraw(preview=False)

    def on_generate_shot(self):
        standing = [p for p in self.layout.pins if p.standing]
        n = len(standing)
        if n == 0:
            messagebox.showinfo("No Pins", "No pins are standing.")
            self.clears_preview()
            return
        if n == 1:
            plan = choose_single_pin_shot(standing[0])
            aim_line = plan['aim_line']
            self.redraw(preview=True, aim_line=aim_line, ghost_ball=None, hits=[standing[0]])
            return
        if n == 2:
            plan = choose_simple_shot(standing, WORLD)
            aim_line = plan['aim_line']
            ghost = plan['ghost_ball']
            hits = plan['pred_hits']
            self.redraw(preview=True, aim_line=aim_line, ghost_ball=ghost, hits=hits)
            return
        # Complex
        plan = choose_complex_shot(standing, WORLD, strategy=self.strategy_var.get(), samples=int(self.sample_var.get()))
        aim_line = plan['aim_line']
        hits = plan['pred_hits']
        self.redraw(preview=True, aim_line=aim_line, ghost_ball=None, hits=hits)

    def on_reset_rack(self):
        self.layout.reset_full_rack()
        self.redraw(preview=False)

    def on_clear_all(self):
        self.layout.clear_all()
        self.redraw(preview=False)

    def on_load_preset(self):
        name = self.preset_var.get()
        self.layout.clear_all()
        if name == "(none)":
            pass
        elif name == "7-10":
            self.layout.set_by_ids([7,10])
        elif name == "2-4-5":
            self.layout.set_by_ids([2,4,5])
        elif name == "3-6-10":
            self.layout.set_by_ids([3,6,10])
        elif name == "2-7":
            self.layout.set_by_ids([2,7])
        elif name == "3-10":
            self.layout.set_by_ids([3,10])
        self.redraw(preview=False)

    # ------------------ Drawing helpers ------------------
    def clears_preview(self):
        self.canvas.delete("preview")

    def redraw(self, preview=False, aim_line=None, ghost_ball=None, hits=None):
        self.drawer.draw_lane()
        self.drawer.draw_pins(self.layout)
        self.drawer.draw_ball_origin()
        if preview:
            self.drawer.draw_shot_preview(aim_line=aim_line, ghost_ball=ghost_ball, hits=hits)


if __name__ == "__main__":
    app = BowlingBotApp()
    app.mainloop()
