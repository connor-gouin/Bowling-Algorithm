
from dataclasses import dataclass
from math import sqrt

# --- World configuration ---
# Units are "pin spacing" (s = 1). Pin triangle row gap = sqrt(3)/2.
WORLD = {
    "s": 1.0,
    "row_gap": 3**0.5 / 2.0,  # ~0.866
    "head_y": 10.0,           # y of head pin (row 1)
    "lane_width": 4.0,        # ~ 4 spacings wide
    "lane_length": 16.0,      # enough to draw nicely
    "ball_origin": (0.0, 0.0),
    "ball_r": 0.45,           # approximate bowling ball radius in "s" units
    "pin_r": 0.20,            # approximate pin contact radius
    "px_per_u": 50.0,         # set at runtime by GUI
    "ball_m": 5.0,
    "pin_m": 1.0,
    "speed": 1.0
}

@dataclass
class Pin:
    pid: int    # 1..10 using standard bowling numbering
    pos: tuple  # (x, y)
    standing: bool = True

class PinLayout:
    """
    A standard 10-pin triangle. IDs follow bowling: head pin is 1, then 2-3, 4-5-6, 7-8-9-10.
    Coordinates are centered on lane x=0 with head pin at y=WORLD['head_y'].
    """
    def __init__(self):
        self.pins = []
        self._build_default_positions()

    def _build_default_positions(self):
        s = WORLD["s"]
        g = WORLD["row_gap"]
        y0 = WORLD["head_y"]
        # Row 1: 1
        P1  = (0.0, y0)
        # Row 2: 2-3
        P2  = (-s/2, y0 + g)
        P3  = (+s/2, y0 + g)
        # Row 3: 4-5-6
        P4  = (-s,   y0 + 2*g)
        P5  = (0.0,  y0 + 2*g)
        P6  = (+s,   y0 + 2*g)
        # Row 4: 7-8-9-10
        P7  = (-1.5*s, y0 + 3*g)
        P8  = (-0.5*s, y0 + 3*g)
        P9  = (+0.5*s, y0 + 3*g)
        P10 = (+1.5*s, y0 + 3*g)

        positions = [None, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10]
        self.pins = [Pin(pid=i, pos=positions[i], standing=True) for i in range(1,11)]

    def reset_full_rack(self):
        for p in self.pins:
            p.standing = True

    def clear_all(self):
        for p in self.pins:
            p.standing = False

    def set_by_ids(self, ids):
        for p in self.pins:
            p.standing = (p.pid in ids)
