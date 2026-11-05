# =============================================================================
#  ik_helper.py  -  Blender plumbing for the IK assignment.
#
#  YOU DO NOT NEED TO READ OR EDIT THIS FILE.
#  Everything that is *about IK* is in  ik_main.py.  This file only:
#     - reads the rig and writes joint angles into the Blender bones
#     - checks your three functions against the real Blender rig
#     - runs the LIVE (drag the target) and DEMO (touch the box and the marker) modes
#  Keep it in the same folder as ik_robotarm_start.blend.
# =============================================================================

import math
import random
import bpy
from mathutils import Vector

ARMATURE_NAME = "Armature"
CHAIN_BONES = ["Bone.003", "Bone.004", "Bone.005"]     # the 3 IK links, root -> tip
TARGET_NAME = "IK_Target"
BOX_NAME = "Box"
PLACE_NAME = "Place_Marker"
HAND_ABOVE_BOX = 1.9      # DEMO: chain tip height above the box centre when grabbing


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# =============================================================================
#  The rig:  Blender bones  <->  (u, v) plane and joint angles q
# =============================================================================
class PlanarArm:
    """u = how far forward (-Y in Blender), v = how high (Z).  q = relative angles (radians)."""

    def __init__(self):
        self.obj = bpy.data.objects[ARMATURE_NAME]
        bones = self.obj.data.bones
        uv = lambda p: (-p.y, p.z)
        self.root = uv(bones[CHAIN_BONES[0]].head_local)
        self.lengths, absolute = [], []
        for name in CHAIN_BONES:
            h, t = uv(bones[name].head_local), uv(bones[name].tail_local)
            self.lengths.append(math.hypot(t[0] - h[0], t[1] - h[1]))
            absolute.append(math.atan2(t[1] - h[1], t[0] - h[0]))
        # joint angles of the pose the arm was modelled in (all pose-bone rotations = 0)
        self.rest_q = [absolute[0]] + [wrap_angle(absolute[i] - absolute[i - 1]) for i in range(1, len(absolute))]

    # --- writing / reading Blender ----------------------------------------
    def apply(self, q, update=True):
        for name, qi, q0 in zip(CHAIN_BONES, q, self.rest_q):
            b = self.obj.pose.bones[name]
            b.rotation_mode = 'XYZ'
            b.rotation_euler = (-(qi - q0), 0.0, 0.0)     # bone X axis points opposite to our "counter-clockwise"
        if update:
            bpy.context.view_layer.update()

    def reset(self):
        for b in self.obj.pose.bones:                     # every bone: plain Euler rotation, all zero
            b.rotation_mode = 'XYZ'
            b.rotation_euler = (0.0, 0.0, 0.0)
        self.apply(self.rest_q)

    def keyframe(self, frame):
        for name in CHAIN_BONES:
            self.obj.pose.bones[name].keyframe_insert("rotation_euler", index=0, frame=frame)

    def joints_uv(self):
        """Where Blender REALLY put the joints: [root, end of link 1, end of link 2, tip] as (u, v)."""
        pb = self.obj.pose.bones
        pts = [pb[CHAIN_BONES[0]].head] + [pb[n].tail for n in CHAIN_BONES]
        return [(-p.y, p.z) for p in pts]

    def target_uv(self):
        p = self.obj.matrix_world.inverted() @ bpy.data.objects[TARGET_NAME].matrix_world.translation
        return (-p.y, p.z)

    def place_target(self, uv):
        """Put the IK_Target empty at (u, v) (world = armature space: the armature sits at the origin)."""
        t = bpy.data.objects[TARGET_NAME]
        t.location = self.obj.matrix_world @ Vector((0.0, -uv[0], uv[1]))


# =============================================================================
#  Which of YOUR solvers moves the arm in LIVE / DEMO mode
# =============================================================================
class Driver:
    def __init__(self, arm, solver, two_link, ccd, limits, max_iter, tol):
        self.arm, self.name, self.two_link, self.ccd = arm, solver, two_link, ccd
        self.limits, self.max_iter, self.tol = limits, max_iter, tol
        self.q = list(arm.rest_q)
        self.warned = False

    def _ok(self, q):
        return all(lo - 1e-6 <= a <= hi + 1e-6 for a, (lo, hi) in zip(q, self.limits))

    def solve(self, target):
        """New joint angles for `target` (u, v), or None = 'leave the arm where it is'."""
        a = self.arm
        if self.name == "CCD":
            q, _ = self.ccd(a.root, a.lengths, self.q, target, self.limits, self.max_iter, self.tol)
            return q
        # TWO_LINK: freeze joint 3, so links 2+3 act like ONE longer, slightly bent link.
        l1, l2, l3 = a.lengths
        phi = a.rest_q[2]
        hx, hy = l2 + l3 * math.cos(phi), l3 * math.sin(phi)
        sol = self.two_link(a.root, l1, math.hypot(hx, hy), target, True)
        if sol is None:
            return None
        q = [sol[0], sol[1] - math.atan2(hy, hx), phi]
        return q if self._ok(q) else None

    def move_to(self, target, update=True):
        q = self.solve(target)
        if q is None:
            if not self.warned:
                print("%s: cannot reach (%.2f, %.2f) - arm stays where it is." % (self.name, target[0], target[1]))
                self.warned = True
            return False
        self.warned = False
        self.q = q
        self.arm.apply(q, update=update)
        return True


# =============================================================================
#  LIVE mode: drag IK_Target in the 3D viewport, the arm follows
# =============================================================================
_live = {"driver": None, "last": None, "err": False}


def _live_update(scene, depsgraph=None):
    d = _live["driver"]
    if d is None or TARGET_NAME not in bpy.data.objects:
        return
    uv = d.arm.target_uv()
    key = (round(uv[0], 5), round(uv[1], 5))
    if key == _live["last"]:
        return                                   # target did not move: don't touch the pose (avoids update loops)
    _live["last"] = key
    try:
        d.move_to(uv, update=False)
    except Exception as e:                       # show a bug once, don't spam the console
        if not _live["err"]:
            print("LIVE: your solver crashed: %s: %s" % (type(e).__name__, e))
            _live["err"] = True


def stop_live():
    for h in list(bpy.app.handlers.depsgraph_update_post):
        if getattr(h, "__name__", "") == "_live_update":
            bpy.app.handlers.depsgraph_update_post.remove(h)
    _live["driver"] = None


def start_live(driver):
    stop_live()
    arm = driver.arm
    for name in (ARMATURE_NAME, TARGET_NAME, BOX_NAME):
        bpy.data.objects[name].animation_data_clear()    # a keyed IK_Target cannot be dragged
    arm.reset()
    arm.place_target(arm.joints_uv()[-1])                # start with the target on the arm tip
    _live.update(driver=driver, last=None, err=False)
    bpy.app.handlers.depsgraph_update_post.append(_live_update)
    bpy.context.view_layer.update()
    print("LIVE mode, solver = %s." % driver.name)
    print("  Object Mode -> click IK_Target -> press G (X is locked) and move it. Set MODE = 'CHECK' to stop.")


# =============================================================================
#  DEMO mode: the tip visits the box and the green marker, the arm follows
# =============================================================================
def run_demo(driver):
    arm, scene = driver.arm, bpy.context.scene
    obj, target = arm.obj, bpy.data.objects[TARGET_NAME]
    stop_live()
    for name in (ARMATURE_NAME, TARGET_NAME):
        bpy.data.objects[name].animation_data_clear()
    inv = obj.matrix_world.inverted()

    def above(name, dv):
        p = inv @ bpy.data.objects[name].matrix_world.translation
        return (-p.y, p.z + dv)

    arm.reset()
    rest_tip = arm.joints_uv()[-1]
    box = above(BOX_NAME, HAND_ABOVE_BOX)              # chain tip just over the box
    marker = above(PLACE_NAME, HAND_ABOVE_BOX)         # ... and just over the green marker
    lift = 1.5
    hover = lambda p: (p[0], p[1] + lift)
    stations = [(1, rest_tip), (25, hover(box)), (40, box), (55, box), (70, hover(box)),
                (100, hover(marker)), (115, marker), (130, marker), (145, hover(marker))]

    prefs = bpy.context.preferences.edit
    old = prefs.keyframe_new_interpolation_type
    prefs.keyframe_new_interpolation_type = 'LINEAR'
    try:
        driver.q = list(arm.rest_q)
        for (f0, p0), (f1, p1) in zip(stations, stations[1:]):
            for f in range(f0, f1 + 1):
                t = (f - f0) / float(f1 - f0)
                uv = (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
                arm.place_target(uv)
                target.keyframe_insert("location", frame=f)
                driver.move_to(uv)
                arm.keyframe(f)
    finally:
        prefs.keyframe_new_interpolation_type = old
    scene.frame_start, scene.frame_end = 1, stations[-1][0]
    scene.frame_set(1)
    print("DEMO baked (solver = %s): frames 1-%d. Press Space in the 3D viewport to play." % (driver.name, scene.frame_end))


# =============================================================================
#  SELF-CHECK: your three functions against the real Blender rig (7.0 points)
# =============================================================================
class _Report:
    def __init__(self):
        self.rows = []

    def show(self):
        print("\n" + "=" * 72)
        print(" IK SELF-CHECK")
        print("=" * 72)
        total = earned = 0.0
        for name, pts, got, msg in self.rows:
            total += pts
            earned += got
            mark = "PASS" if got >= pts - 1e-9 else ("TODO" if msg.startswith("not implemented") else "FAIL")
            print(" [%s] %-40s %.2f / %.2f   %s" % (mark, name, got, pts, "" if mark == "PASS" else msg))
        print("-" * 72)
        print(" Auto-checked points: %.2f / %.2f   (written answers + code quality are marked by hand)" % (earned, total))
        print("=" * 72 + "\n")
        return earned, total


def _guard(rep, name, pts, fn):
    try:
        got, msg = fn()
        rep.rows.append((name, pts, got, msg))
    except NotImplementedError as e:
        rep.rows.append((name, pts, 0.0, "not implemented yet (%s)" % e))
    except Exception as e:                       # a crash must not stop the other checks
        rep.rows.append((name, pts, 0.0, "crashed: %s: %s" % (type(e).__name__, e)))


def self_check(arm, fk, two_link, ccd, limits, max_iter, tol):
    rep, rng = _Report(), random.Random(475)
    root, L = arm.root, arm.lengths

    def rand_q(margin=0.08):
        return [rng.uniform(lo + margin, hi - margin) for lo, hi in limits]

    # ---- Task 1 ----------------------------------------------------------
    def t1a():
        worst = 0.0
        for q in [list(arm.rest_q)] + [rand_q() for _ in range(8)]:
            arm.apply(q)
            real, mine = arm.joints_uv(), fk(root, L, q)
            if len(mine) != len(real):
                return 0.0, "expected %d points, got %d" % (len(real), len(mine))
            worst = max(worst, max(_dist(a, b) for a, b in zip(real, mine)))
        return (1.0, "") if worst < 1e-3 else (0.0, "FK differs from Blender by up to %.3f units" % worst)
    _guard(rep, "Task 1a  forward_kinematics vs Blender", 1.0, t1a)

    def t1b():
        for lens, q in [([1.0, 1.0], [math.pi / 2, -math.pi / 2]), ([1.0, 2.0, 0.5, 1.5], [0.3, -0.2, 0.5, 0.1])]:
            pts = fk((0.0, 0.0), lens, q)
            a, u, v = 0.0, 0.0, 0.0
            for l, qi in zip(lens, q):
                a += qi
                u, v = u + l * math.cos(a), v + l * math.sin(a)
            if len(pts) != len(lens) + 1 or _dist(pts[-1], (u, v)) > 1e-9 or _dist(pts[0], (0.0, 0.0)) > 1e-9:
                return 0.0, "must work for any number of links (tried %d links)" % len(lens)
        return 0.5, ""
    _guard(rep, "Task 1b  works for any number of links", 0.5, t1b)

    # ---- Task 2 ----------------------------------------------------------
    l1, l2 = L[0], L[1]

    def t2a():
        for _ in range(8):
            q = rand_q()
            arm.apply(q)
            wrist = arm.joints_uv()[2]
            for up in (True, False):
                sol = two_link(root, l1, l2, wrist, up)
                if sol is None:
                    return 0.0, "returned None for a reachable target %s" % (tuple(round(c, 2) for c in wrist),)
                arm.apply([sol[0], sol[1], q[2]])
                if _dist(arm.joints_uv()[2], wrist) > 1e-3:
                    return 0.0, "in Blender the wrist is %.3f away from the target (elbow_up=%s)" % (_dist(arm.joints_uv()[2], wrist), up)
        return 1.0, ""
    _guard(rep, "Task 2a  two-link lands on the target", 1.0, t2a)

    def t2b():
        for _ in range(6):
            arm.apply(rand_q())
            wrist = arm.joints_uv()[2]
            for up in (True, False):
                sol = two_link(root, l1, l2, wrist, up)
                if sol is None or (sol[1] < 0) != up:
                    return 0.0, "elbow_up=True must give q2 < 0 and elbow_up=False must give q2 > 0"
        return 0.5, ""
    _guard(rep, "Task 2b  both elbow solutions", 0.5, t2b)

    def t2c():
        far = (root[0] + l1 + l2 + 0.5, root[1])
        near = (root[0] + 0.5 * abs(l1 - l2), root[1])
        if two_link(root, l1, l2, far, True) is not None or two_link(root, l1, l2, near, True) is not None:
            return 0.0, "an unreachable target (too far / too close) must return None"
        return 0.5, ""
    _guard(rep, "Task 2c  unreachable target gives None", 0.5, t2c)

    # ---- Task 3 (CCD) ----------------------------------------------------
    targets = []
    for _ in range(20):
        arm.apply(rand_q())
        targets.append(arm.joints_uv()[-1])
    reach = sum(L)
    stats = {}

    def run_all():
        if stats:
            return stats
        hit, worst = 0, 0.0
        for tg in targets:
            q, _ = ccd(root, L, list(arm.rest_q), tg, limits, max_iter, tol)
            arm.apply(q)
            hit += _dist(arm.joints_uv()[-1], tg) < 0.01
            worst = max(worst, max(max(lo - a, a - hi, 0.0) for a, (lo, hi) in zip(q, limits)))
        stats.update(frac=hit / float(len(targets)), worst=worst)
        return stats

    def t3a():
        f = run_all()["frac"]
        return (1.5, "") if f >= 0.85 else (0.0, "only %d%% of the reachable targets were hit (need 85%%)" % round(100 * f))
    _guard(rep, "Task 3a  CCD reaches the target", 1.5, t3a)

    def t3b():
        w = run_all()["worst"]
        return (1.0, "") if w < 1e-6 else (0.0, "a joint limit is violated by %.1f deg" % math.degrees(w))
    _guard(rep, "Task 3b  CCD respects joint limits", 1.0, t3b)

    def t3c():
        for tg in [(root[0] + reach + 2.0, root[1] + 1.0), (root[0] - 3.0, root[1] - 6.0), (root[0], root[1])]:
            before = _dist(fk(root, L, arm.rest_q)[-1], tg)
            q, ok = ccd(root, L, list(arm.rest_q), tg, limits, max_iter, tol)
            if any(math.isnan(a) for a in q):
                return 0.0, "NaN in the result"
            after = _dist(fk(root, L, q)[-1], tg)
            if after > before + 1e-6:
                return 0.0, "an unreachable target made the tip FARTHER away (%.2f -> %.2f)" % (before, after)
        return 0.5, ""
    _guard(rep, "Task 3c  CCD survives unreachable targets", 0.5, t3c)

    def t3d():
        q0 = list(arm.rest_q)
        keep = list(q0)
        q, ok = ccd(root, L, q0, targets[0], limits, max_iter, tol)
        if q0 != keep or q is q0:
            return 0.0, "ik_ccd must not modify q_start - return a NEW list"
        if not ok:
            return 0.0, "converged should be True for a reachable target"
        far = (root[0] + reach + 2.0, root[1])
        q, ok = ccd(root, L, list(arm.rest_q), far, limits, max_iter, tol)
        if ok:
            return 0.0, "converged must be False when the target cannot be reached"
        return 0.5, ""
    _guard(rep, "Task 3d  honest return value", 0.5, t3d)

    arm.reset()
    return rep


# =============================================================================
#  EXPERIMENTS: numbers to quote in the written answers
# =============================================================================
def experiments(arm, two_link, ccd, limits, tol):
    fmt = lambda q: "[" + ", ".join("%7.1f" % math.degrees(a) for a in q) + "] deg"
    r, L = arm.root, arm.lengths
    reach = sum(L)
    near = (r[0] + 2.5, r[1] - 1.0)
    cases = {"near": near, "far (97% of reach)": (r[0] + 0.97 * reach, r[1]),
             "unreachable": (r[0] + reach + 1.5, r[1] + 0.5)}
    print("=" * 72)
    print(" EXPERIMENTS  (quote these numbers in answers.md)")
    print("=" * 72)
    try:
        print("\n[1] Two links, target 'near' = (%.2f, %.2f) - how many solutions?" % near)
        for up in (True, False):
            s = two_link(r, L[0], L[1], near, up)
            print("    elbow %-4s -> (q1, q2) = %s" % ("up" if up else "down", fmt(s) if s else "None"))
        far = (r[0] + L[0] + L[1] + 0.5, r[1])
        print("    out of reach (%.2f, %.2f) -> %s" % (far[0], far[1], two_link(r, L[0], L[1], far, True)))
    except NotImplementedError as e:
        print("    (needs %s)" % e)
    try:
        print("\n[2] CCD: distance tip-to-target after k sweeps (start = rest pose)")
        ks = (1, 2, 3, 5, 10, 20, 50, 100)
        print("    %-20s" % "target" + "".join("%8s" % ("k=%d" % k) for k in ks))
        for name, tg in cases.items():
            row = []
            for k in ks:
                q, _ = ccd(r, L, list(arm.rest_q), tg, limits, k, 1e-9)
                row.append(_dist(_tip(r, L, q), tg))
            print("    %-20s" % name + "".join("%8.3f" % d for d in row))
        print("\n[3] Three links = more than one solution. CCD, same target 'near', two start poses:")
        for label, q_start in (("start = rest pose", list(arm.rest_q)),
                               ("start = arm folded up", [math.radians(100), math.radians(-120), math.radians(60)])):
            q, ok = ccd(r, L, q_start, near, limits, 200, tol)
            tip = _tip(r, L, q)
            print("    %-24s -> q = %s  tip error %.4f" % (label, fmt(q), _dist(tip, near)))
    except NotImplementedError as e:
        print("    (needs %s)" % e)
    print("=" * 72 + "\n")


def _tip(root, lengths, q):
    a, u, v = 0.0, root[0], root[1]
    for l, qi in zip(lengths, q):
        a += qi
        u, v = u + l * math.cos(a), v + l * math.sin(a)
    return (u, v)


# =============================================================================
#  Challenge check (used by ik_challenge.py)
# =============================================================================
def check_challenge(solvers, limits=None, tol=0.001):
    """solvers = {"name": (function, max_iter)}.  Prints hit-rate for random reachable targets."""
    arm = PlanarArm()
    limits = limits or [(math.radians(-20), math.radians(150)), (math.radians(-150), math.radians(-5)),
                        (math.radians(-130), math.radians(100))]
    rng = random.Random(99)
    targets = []
    for _ in range(15):
        arm.apply([rng.uniform(lo + 0.08, hi - 0.08) for lo, hi in limits])
        targets.append(arm.joints_uv()[-1])
    arm.reset()
    print("\nCHALLENGE CHECK (15 random reachable targets, hit = tip within 0.01)")
    for name, (fn, iters) in solvers.items():
        try:
            hit = 0
            for tg in targets:
                q, ok = fn(arm.root, arm.lengths, list(arm.rest_q), tg, limits, iters, tol)
                arm.apply(q)
                hit += _dist(arm.joints_uv()[-1], tg) < 0.01
            print("  [%s] %-22s %2d / %d targets hit (max_iter=%d)" % ("PASS" if hit >= 12 else "FAIL", name, hit, len(targets), iters))
        except NotImplementedError as e:
            print("  [TODO] %-22s not implemented yet (%s)" % (name, e))
        except Exception as e:
            print("  [FAIL] %-22s crashed: %s: %s" % (name, type(e).__name__, e))
    arm.reset()


# =============================================================================
#  Entry point called by ik_main.py
# =============================================================================
def run(mode, solver, fk, two_link, ccd, limits_deg, max_iter, tol):
    limits = [(math.radians(a), math.radians(b)) for a, b in limits_deg]
    arm = PlanarArm()
    stop_live()                                   # remove a handler left by an earlier run
    print("\nChain lengths :", [round(l, 3) for l in arm.lengths])
    print("Chain root    : (u, v) = (%.3f, %.3f)" % arm.root)
    print("Rest angles   :", [round(math.degrees(a), 1) for a in arm.rest_q], "deg")
    mode, solver = mode.upper(), solver.upper()
    if mode == "CHECK":
        arm.obj.animation_data_clear()
        arm.reset()
        self_check(arm, fk, two_link, ccd, limits, max_iter, tol).show()
        arm.reset()
        experiments(arm, two_link, ccd, limits, tol)
        return
    if solver not in ("CCD", "TWO_LINK"):
        raise ValueError("SOLVER must be 'CCD' or 'TWO_LINK'")
    driver = Driver(arm, solver, two_link, ccd, limits, max_iter, tol)
    if mode == "LIVE":
        start_live(driver)
    elif mode == "DEMO":
        try:
            run_demo(driver)
        except NotImplementedError as e:
            print("DEMO needs your %s - finish that TODO first (MODE = 'CHECK' shows what is missing)." % e)
    else:
        raise ValueError("MODE must be 'CHECK', 'LIVE' or 'DEMO'")
