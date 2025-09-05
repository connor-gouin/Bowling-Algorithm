
from math import sqrt, atan2, cos, sin, radians, inf
from typing import List, Dict, Tuple
from pins import WORLD, Pin
import copy
import heapq
from dataclasses import dataclass
from collections import deque

# ---------------- Geometry helpers ----------------
def norm(v):
    x,y = v
    d = sqrt(x*x + y*y)
    return (x/d, y/d) if d else (0.0,0.0)

def sub(a,b): return (a[0]-b[0], a[1]-b[1])
def add(a,b): return (a[0]+b[0], a[1]+b[1])
def mul(a, k): return (a[0]*k, a[1]*k)

def line_from_two_points(p0, p1):
    return (p0, p1)

def extend_line_from(p0, theta, length):
    return (p0[0] + length*cos(theta), p0[1] + length*sin(theta))

def point_line_distance(p0, p1, p):
    x0,y0 = p0; x1,y1 = p1; xp,yp = p
    vx,vy = x1-x0, y1-y0
    wx,wy = xp-x0, yp-y0
    vv = vx*vx + vy*vy
    if vv == 0:
        return sqrt(wx*wx+wy*wy), 0.0
    t = (wx*vx + wy*vy)/vv
    proj = (x0 + t*vx, y0 + t*vy)
    dx,dy = xp - proj[0], yp - proj[1]
    return sqrt(dx*dx + dy*dy), t

def pins_hit_along_line(pins: List[Pin], p0, p1, corridor_radius: float) -> List[Pin]:
    hits = []
    for pin in pins:
        d, t = point_line_distance(p0, p1, pin.pos)
        if 0.0 <= t <= 1.0 and d <= corridor_radius:
            hits.append((t, pin))
    hits.sort(key=lambda x: x[0])
    return [pin for _,pin in hits]

# ---------------- Shot planners ----------------

def choose_single_pin_shot(pin: Pin) -> Dict:
    origin = WORLD['ball_origin']
    dx, dy = pin.pos[0]-origin[0], pin.pos[1]-origin[1]
    if dy <= 0: dy = 1e-6
    theta = atan2(dy, dx)
    far = extend_line_from(origin, theta, WORLD['lane_length']-origin[1])
    return {"aim_line": (origin, far)}

def choose_simple_shot(standing: List[Pin], world: Dict) -> Dict:
    assert len(standing) == 2
    A, B = standing[0], standing[1]
    origin = world['ball_origin']
    rb = world['ball_r']; rp = world['pin_r']

    def plan(A, B):
        v = sub(A.pos, B.pos)
        u = norm(v)
        ghost = add(A.pos, mul(u, rb+rp))
        dx, dy = ghost[0]-origin[0], ghost[1]-origin[1]
        if dy <= 1e-6: dy = 1e-6
        theta = atan2(dy, dx)
        far = extend_line_from(origin, theta, world['lane_length']-origin[1])
        hit_ids = simulate_shot([A,B], theta)
        pred_hits = [p for p in [A,B] if p.pid in hit_ids]
        return {"aim_line": (origin, far), "ghost_ball": ghost, "theta": theta, "pred_hits": pred_hits}

    plan1 = plan(A,B)
    plan2 = plan(B,A)
    best = plan1 if abs(plan1["theta"]) <= abs(plan2["theta"]) else plan2
    return best

def choose_complex_shot(standing: List[Pin], world: Dict, strategy="breadth_first", samples=181) -> Dict:
    origin = world['ball_origin']
    rb = world['ball_r']
    end_y = world['lane_length']

    # typically you'd cap to lane edges minus ball radius:
    half_w = world['lane_width'] / 2 + 3 * rb
    xs = [(-half_w + i * (2 * half_w) / (samples - 1)) for i in range(samples)]
    shots = []
    for x_end in xs:
        dx = x_end - origin[0]
        dy = end_y - origin[1]
        theta = atan2(dy, dx)
        far = (x_end, end_y)
        if strategy == "breadth_first":
            knocked, hit_ids, moves = simulate_shot_bf(standing, theta)
        if strategy == "recursive":
            knocked, hit_ids, moves = simulate_shot_recursive(standing, theta)
        hits = [p for p in standing if p.pid in hit_ids]
        shots.append({"score": knocked, "theta": theta, "aim_line": (origin, far), "pred_hits": hits, "moves": moves})
    # max score among all sampled shots
    max_score = max(s["score"] for s in shots)
    # find widest contiguous run(s) where score == max_score
    best_range = None
    best_len = 0
    cur_start = None

    for i, s in enumerate(shots):
        if s["score"] == max_score:
            if cur_start is None:
                cur_start = i
        else:
            if cur_start is not None:
                cur_len = i - cur_start  # length of the finished run
                if cur_len > best_len:
                    best_len = cur_len
                    best_range = (cur_start, i - 1)
                cur_start = None
    # handle a run that reaches the end
    if cur_start is not None:
        cur_len = len(shots) - cur_start
        if cur_len > best_len:
            best_len = cur_len
            best_range = (cur_start, len(shots) - 1)
    print(best_range)
    # pick the middle index of the widest run
    start, stop = best_range
    mid = (start + stop) // 2

    # optional tie-breaker: within the widest run, pick the shot with smallest |theta|
    # mid = min(range(start, stop + 1), key=lambda i: abs(shots[i]["theta"]))

    return shots[mid]


    # origin = world['ball_origin']
    # rb = world['ball_r']
    # end_y = world['lane_length']
    # half_w = world['lane_width']/2 - rb
    # xs = [(-half_w + i*(2*half_w)/(samples-1)) for i in range(samples)]

    # best = None
    # for x_end in xs:
    #     dx = x_end - origin[0]
    #     dy = end_y - origin[1]
    #     theta = atan2(dy, dx)
    #     far = (x_end, end_y)
    #     knocked, hit_ids, moves = simulate_shot(standing, theta)
    #     hits = [p for p in standing if p.pid in hit_ids]
    #     score = knocked
    #     if best is None:
    #         best = {"score":score, "theta":theta, "aim_line":(origin, far), "pred_hits":hits, "moves": moves}
    #     else:
    #         if strategy == "max_hits":
    #             if score > best["score"] or (score == best["score"] and abs(theta) < abs(best["theta"])):
    #                 best = {"score":score, "theta":theta, "aim_line":(origin, far), "pred_hits":hits, "moves": moves}
    #         elif strategy == "min_angle":
    #             if abs(theta) < abs(best["theta"]) or (abs(theta)==abs(best["theta"]) and score>best["score"]):
    #                 best = {"score":score, "theta":theta, "aim_line":(origin, far), "pred_hits":hits}
    # return best

# ---------------- Physics simulation ----------------

def vadd(a,b): return (a[0]+b[0], a[1]+b[1])
def vsub(a,b): return (a[0]-b[0], a[1]-b[1])
def vmul(a,k): return (a[0]*k, a[1]*k)
def dot(a,b): return a[0]*b[0]+a[1]*b[1]
def normv(a):
    d = sqrt(dot(a,a))
    return (a[0]/d, a[1]/d) if d else (0.0,0.0)
def length(a): return sqrt(dot(a,a))

def resolve_elastic(p1, v1, m1, p2, v2, m2):
    n = vsub(p2, p1)
    dist2 = dot(n,n)
    if dist2 == 0:
        n = (1.0, 0.0); dist2 = 1.0
    un = (n[0]/sqrt(dist2), n[1]/sqrt(dist2))
    rel = vsub(v1, v2)
    rel_n = dot(rel, un)
    if rel_n <= 0:
        return v1, v2
    J = (2*rel_n) / (1/m1 + 1/m2)
    v1p = vsub(v1, (un, J/m1))
    v2p = vadd(v2, vmul(un, J/m2))
    return v1p, v2p

def elastic_collision(p1, s1, th1, m1,
                      p2, s2, th2, m2,
                      e):
    """
    2D frictionless collision between two disks (no spin).
    Inputs:
      p1, p2 : (x, y) centers at the instant of collision
      s1, s2 : speeds (>= 0)
      th1, th2 : headings in radians (0 = +x, pi/2 = +y)
      m1, m2 : masses (> 0)
      e : coefficient of restitution (1.0 = perfectly elastic, 0 = perfectly inelastic)

    Returns:
      (s1_after, th1_after), (s2_after, th2_after)
    """

    # Convert to Cartesian velocities
    v1 = (s1 * cos(th1), s1 * sin(th1))
    v2 = (s2 * cos(th2), s2 * sin(th2))

    # Line-of-centers unit normal (from object 1 to object 2)
    n = (p2[0] - p1[0], p2[1] - p1[1])
    d2 = n[0]*n[0] + n[1]*n[1]
    if d2 == 0.0:
        # Degenerate overlap: pick an arbitrary normal
        un = (1.0, 0.0)
    else:
        invd = 1.0 / sqrt(d2)
        un = (n[0]*invd, n[1]*invd)

    # Relative velocity along the normal (positive => closing)
    relx = (v1[0] - v2[0])
    rely = (v1[1] - v2[1])
    rel_n = relx * un[0] + rely * un[1]

    # If not closing (<=0), no normal impulse; return unchanged
    if rel_n <= 0.0:
        return (s1, th1), (s2, th2)

    # Normal impulse magnitude with restitution
    J = (1.0 + e) * rel_n / (1.0/m1 + 1.0/m2)

    # Apply impulse along normal
    v1p = (v1[0] - (J/m1) * un[0], v1[1] - (J/m1) * un[1])
    v2p = (v2[0] + (J/m2) * un[0], v2[1] + (J/m2) * un[1])

    # Convert back to (speed, theta)
    s1_after = sqrt(v1p[0]*v1p[0] + v1p[1]*v1p[1])
    s2_after = sqrt(v2p[0]*v2p[0] + v2p[1]*v2p[1])
    th1_after = atan2(v1p[1], v1p[0])
    th2_after = atan2(v2p[1], v2p[0])

    return (s1_after, th1_after), (s2_after, th2_after)

def first_hit_centers(radius, pins, theta, origin):
    """
    Analytic first-contact test using ray–circle intersection with Minkowski sum.
    Treat the moving object as a point and each pin as a circle of radius R = radius + rp.

    Parameters
    ----------
    radius : float
        Radius of the moving object (e.g., ball or pin).
    pins : iterable
        Each item must have .pos -> (x, y) and .standing (bool).
    theta : float
        Aim angle in radians (direction of motion).
    origin : tuple(float, float)
        Starting center (x0, y0) of the moving object.

    Returns
    -------
    (mx, my), (px, py)  or  (None, None)
        (moving_center_at_contact, hit_pin_center) for the **first** hit; or (None, None) if no hit.
    """
    ox, oy = origin
    dx, dy = cos(theta), sin(theta)

    best_t = float("inf")
    best_pin = None

    for p in pins:
        if not getattr(p, "standing", True):
            continue  # skip knocked-down pins

        cx, cy = p.pos
        rp = getattr(p, "radius", WORLD["pin_r"])
        R = radius + rp

        # Solve |(o + d*t) - c|^2 = R^2  ->  t^2 + 2*b*t + c0 = 0
        mx, my = ox - cx, oy - cy
        b = dx*mx + dy*my
        c0 = mx*mx + my*my - R*R
        disc = b*b - c0
        if disc < 0:
            continue  # no intersection

        root = sqrt(disc)
        t_enter = -b - root
        t_exit  = -b + root

        # First non-negative intersection along the ray
        if t_enter >= 0 and t_enter < best_t:
            best_t, best_pin = t_enter, p
        elif t_enter < 0 <= t_exit and t_exit < best_t:
            # started "inside" enlarged circle; take the exit point
            best_t, best_pin = t_exit, p

    if best_pin is None:
        return None, None, None, None

    # Moving object's center at contact
    mx = ox + dx * best_t
    my = oy + dy * best_t

    # Pin center (as-is)
    px, py = best_pin.pos
    pid = best_pin.pid
    return best_t, (mx, my), (px, py), pid

def recursive_topple(radius, pins, theta, origin, speed, mass):
    rp = WORLD['pin_r']
    rb = WORLD['ball_r']
    mp = WORLD['pin_m']
    knocked = 0
    hit_ids = set()
    moves = []

    best_t, moving_center, pin_center, pid = first_hit_centers(radius, pins, theta, origin)

    if pid is None or speed < 0.25*WORLD['speed']:
        return knocked, hit_ids, moves
    else: 
        knocked+=1
        for pin in pins:
            if pin.pid == pid:
                pin.standing = False
            hit_ids.add(pid)
    e = WORLD['elasticity']
    (speed1,theta1),(speed2,theta2) = elastic_collision(p1=moving_center,s1=speed,th1=theta,m1=mass,p2=pin_center,s2=0.0,th2=0.0,m2=mp,e=e)
    moves.append({
        "moving_origin": origin,
        "moving_hit": moving_center,
        "moving_radius": radius
    })
    knocked1, hit_ids1, moves1 = recursive_topple(rp, pins, theta2, pin_center, speed2, mp)
    knocked2, hit_ids2, moves2 = recursive_topple(radius, pins, theta1, moving_center, speed1, mass)
    knocked += knocked1 + knocked2
    for id1 in hit_ids1:
        hit_ids.add(id1)
    for id2 in hit_ids2:
        hit_ids.add(id2)
    for move1 in moves1:
        moves.append(move1)
    for move2 in moves2:
        moves.append(move2)
    return knocked, hit_ids, moves
    
@dataclass
class Node:
    radius: float
    mass: float
    origin: tuple   # (x, y)
    speed: float
    theta: float
    kind: str       # "ball" or "pin" (for logging)

def topple_bfs(pins, theta0):
    rb = WORLD['ball_r']; mb = WORLD.get('ball_m', 10.0)
    pin_r = WORLD['pin_r']; pin_m = WORLD.get('pin_m', 1.0)
    e = WORLD.get('elasticity', 1.0)
    speed_floor = 0.25 * WORLD['speed']

    q = deque()
    q.append(Node(radius=rb, mass=mb, origin=WORLD['ball_origin'],
                  speed=WORLD.get('ball_speed', WORLD['speed']),
                  theta=theta0, kind="ball"))

    knocked_ids = set()
    moves = []

    # process by levels
    while q:
        level_size = len(q)

        # (optional) sort nodes within this level by time-to-next-hit
        # to get a more realistic ordering while keeping BFS levels
        level_nodes = []
        for _ in range(level_size):
            level_nodes.append(q.popleft())

        def time_to_next_hit(node):
            t_hit, _, pin_obj, _ = first_hit_centers(node.radius, pins, node.theta, node.origin)
            if t_hit is None or node.speed <= 0: return float('inf')
            return t_hit / node.speed
        level_nodes.sort(key=time_to_next_hit)  # remove if you want pure FIFO

        # advance each node by exactly one collision
        for node in level_nodes:
            if node.speed < speed_floor:
                continue

            t_hit, contact, pin_center, pin_id = first_hit_centers(
                node.radius, pins, node.theta, node.origin
            )

            for pin in pins:
                if pin.pid == pin_id:
                    pin_obj = pin

            if t_hit is None:
                continue
            if not getattr(pin_obj, "standing", True):
                # someone else in this level (earlier in list) already took it down
                continue
            
            # knock the pin
            pin_obj.standing = False
            knocked_ids.add(pin_obj.pid)

            # log path (origin -> contact), in world units
            moves.append({
                "moving_origin": node.origin,
                "moving_hit": contact,
                "moving_radius": node.radius,
                "kind": node.kind,
            })

            # resolve velocities at contact
            (s1, th1), (s2, th2) = elastic_collision(
                p1=contact, s1=node.speed, th1=node.theta, m1=node.mass,
                p2=pin_center, s2=0.0, th2=0.0, m2=pin_m, e=e
            )

            # small nudge along new headings
            eps = 1e-4
            next_ball_origin = (contact[0] + eps * cos(th1), contact[1] + eps * sin(th1))
            next_pin_origin  = (pin_center[0] + eps * cos(th2), pin_center[1] + eps * sin(th2))

            # enqueue children for the NEXT level (one step per level)
            if s1 > speed_floor:
                q.append(Node(radius=node.radius, mass=node.mass,
                              origin=next_ball_origin, speed=s1, theta=th1, kind=node.kind))
            if s2 > speed_floor:
                q.append(Node(radius=pin_r, mass=pin_m,
                              origin=next_pin_origin, speed=s2, theta=th2, kind="pin"))

    return len(knocked_ids), knocked_ids, moves

def simulate_shot_recursive(standing, theta):
    rb = WORLD['ball_r']
    rp = WORLD['pin_r']
    ball_origin = WORLD['ball_origin']
    speed = WORLD['speed']
    ball_mass = WORLD['ball_m']
    standing_copy = copy.deepcopy(standing)
    knocked, hit_ids, moves = recursive_topple(rb,standing_copy,theta,ball_origin,speed,ball_mass)
    return knocked, hit_ids, moves

def simulate_shot_bf(standing, theta):
    rb = WORLD['ball_r']
    rp = WORLD['pin_r']
    ball_origin = WORLD['ball_origin']
    speed = WORLD['speed']
    ball_mass = WORLD['ball_m']
    standing_copy = copy.deepcopy(standing)
    knocked, hit_ids, moves = topple_bfs(standing_copy,theta)
    return knocked, hit_ids, moves



    # rb = WORLD['ball_r']; rp = WORLD['pin_r']
    # m_ball, m_pin = 10.0, 1.0
    # R = rb + rp
    # dt, T_max, speed_floor = 0.01, 5.0, 0.02

    # ball_p = WORLD['ball_origin']
    # ball_v = (cos(theta), sin(theta))

    # pins = list(standing)
    # pin_p = [p.pos for p in pins]
    # pin_v = [(0.0,0.0) for _ in pins]
    # knocked = set()

    # t = 0.0
    # while t < T_max:
    #     ball_p = vadd(ball_p, vmul(ball_v, dt))

    #     for i, p in enumerate(pins):
    #         d = vsub(ball_p, pin_p[i])
    #         if length(d) <= R:
    #             knocked.add(p.pid)
    #             n = normv(d) if length(d) != 0 else (1.0,0.0)
    #             overlap = R - length(d)
    #             ball_p = vadd(ball_p, vmul(n, overlap+1e-4))
    #             nv_ball, nv_pin = resolve_elastic(ball_p, ball_v, m_ball, pin_p[i], pin_v[i], m_pin)
    #             ball_v, pin_v[i] = nv_ball, nv_pin

    #     for i in range(len(pins)):
    #         for j in range(i+1, len(pins)):
    #             d = vsub(pin_p[i], pin_p[j])
    #             if length(d) <= 2*rp:
    #                 knocked.add(pins[i].pid); knocked.add(pins[j].pid)
    #                 n = normv(d) if length(d) != 0 else (1.0,0.0)
    #                 overlap = 2*rp - length(d)
    #                 pin_p[i] = vadd(pin_p[i], vmul(n, overlap/2+1e-4))
    #                 pin_p[j] = vsub(pin_p[j], vmul(n, overlap/2+1e-4))
    #                 pin_v[i], pin_v[j] = resolve_elastic(pin_p[i], pin_v[i], m_pin, pin_p[j], pin_v[j], m_pin)

    #     for i in range(len(pins)):
    #         pin_p[i] = vadd(pin_p[i], vmul(pin_v[i], dt))

    #     if length(ball_v) < speed_floor and all(length(v) < speed_floor for v in pin_v):
    #         break
    #     if ball_p[1] > WORLD['lane_length'] + 1.0:
    #         break
    #     t += dt

    # return knocked

def compute_hits_for_line(pins, p0, p1, corridor):
    return pins_hit_along_line(pins, p0, p1, corridor)

