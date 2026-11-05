# =============================================================================
#  OPTIONAL CHALLENGE (+1 bonus point) - two more IK algorithms.  Pick ONE or both.
#  Same signature as ik_ccd, so ik_helper can compare them.  Needs no other file
#  of yours: forward_kinematics is repeated here so this file stands alone.
#  Run like ik_main.py (Alt+P with ik_robotarm_start.blend open).
# =============================================================================

import math
import os
import sys
import bpy

sys.path.insert(0, os.path.dirname(bpy.data.filepath))
import ik_helper


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def forward_kinematics(root, lengths, q):
    points, u, v, a = [tuple(root)], root[0], root[1], 0.0
    for length, qi in zip(lengths, q):
        a += qi
        u += length * math.cos(a)
        v += length * math.sin(a)
        points.append((u, v))
    return points


# -----------------------------------------------------------------------------
# CHALLENGE A - Jacobian transpose                                   [0.5 pt]
# -----------------------------------------------------------------------------
def ik_jacobian_transpose(root, lengths, q_start, target, limits, max_iter=2000, tol=0.001):
    """Return (q, converged).

    The Jacobian says how the tip moves when each joint turns a little.  For joint i
    at point p_i, with the tip at t:     column_i = (-(t_v - p_v), (t_u - p_u))
    Error e = target - tip.   Update:    q_i += alpha * dot(column_i, e)
    A safe step size:   alpha = dot(e, J J^T e) / dot(J J^T e, J J^T e)
        where  (J J^T e) = sum over joints of column_i * dot(column_i, e)
    Clamp every q_i into its limits.
    """
    # TODO challenge A
    raise NotImplementedError("ik_jacobian_transpose")


# -----------------------------------------------------------------------------
# CHALLENGE B - FABRIK                                               [0.5 pt]
# -----------------------------------------------------------------------------
def ik_fabrik(root, lengths, q_start, target, limits, max_iter=100, tol=0.001):
    """Return (q, converged).  FABRIK moves POINTS, not angles.  (It ignores `limits`.)

    p[0..n] = joint positions, p[0] = root.
    Target farther than sum(lengths)?  Stretch the arm straight at it.  Otherwise repeat:
      backward pass: p[n] = target; for i = n-1 .. 0: put p[i] on the line from p[i+1]
                     through its old p[i], at distance lengths[i] from p[i+1]
      forward pass:  p[0] = root;   for i = 0 .. n-1: put p[i+1] on the line from p[i]
                     through its old p[i+1], at distance lengths[i] from p[i]
    Stop when |p[n] - target| < tol.  Finally turn the points back into angles:
      a_i = atan2 of segment i;  q1 = a_1;  q_i = wrap_angle(a_i - a_(i-1)).
    """
    # TODO challenge B
    raise NotImplementedError("ik_fabrik")


if __name__ == "__main__":
    ik_helper.check_challenge({"A  Jacobian transpose": (ik_jacobian_transpose, 2000),
                               "B  FABRIK": (ik_fabrik, 100)})
