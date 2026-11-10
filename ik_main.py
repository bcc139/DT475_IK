# =============================================================================
#  DT475 Animation & Modeling  |  Assignment: Inverse Kinematics from scratch
#  Blender 5.2 + Python (bpy)  |  Student template
# =============================================================================
#
#  GOAL   Make the robot arm reach a target using IK maths that YOU write.
#         No Blender IK constraint, no bpy.ops.pose.*  -  only `math`.
#
#  RUN    1. Open ik_robotarm_start.blend (this file and ik_helper.py sit next to it).
#         2. Scripting workspace > Text > Open > this file, then press Alt+P.
#         3. Read the report in the System Console (Window > Toggle System Console).
#
#  THE ARM IN 2D   The arm moves in ONE vertical plane, so every point is (u, v):
#         u = forward (Blender -Y)      v = up (Blender Z)
#     3 links, lengths [L1, L2, L3];  the first joint sits at root = (u, v).
#     q = [q1, q2, q3] are RELATIVE joint angles in radians, counter-clockwise
#     positive.  Link i points at the absolute angle  a_i = q1 + ... + q_i.
#
#  YOU EDIT   only the three functions marked TODO.  Everything else is set-up.
# =============================================================================

import math
import os
import sys
import bpy

sys.path.insert(0, os.path.dirname(bpy.data.filepath))   # ik_helper.py lives next to the .blend
import ik_helper

# ----------------------------------------------------------------------------
#  SETTINGS
# ----------------------------------------------------------------------------
MODE = "CHECK"      # "CHECK" = test my code | "LIVE" = drag IK_Target and the arm follows
SOLVER = "CCD"      # which of MY solvers moves the arm in LIVE:  "CCD" or "TWO_LINK"

JOINT_LIMITS_DEG = [(-20, 150), (-150, -5), (-130, 100)]   # (low, high) for q1, q2, q3
CCD_MAX_ITER = 100  # most sweeps CCD may use
CCD_TOL = 0.001     # "close enough" distance, in Blender units


def wrap_angle(a):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# -----------------------------------------------------------------------------
# TASK 1 - Forward kinematics                                        [1.5 pt]
# -----------------------------------------------------------------------------
def forward_kinematics(root, lengths, q):
    """Return the position of every joint: [(u0, v0), (u1, v1), ..., (un, vn)].

    Point 0 is `root`, point i is the END of link i, the last point is the tip.
    It must work for any number of links.

    Idea: walk along the chain.  Keep a running angle  a += q[i]  and move
    by  length * (cos a, sin a).
    """
    # TODO 1: build and return the list of points
    raise NotImplementedError("forward_kinematics")


# -----------------------------------------------------------------------------
# TASK 2 - Analytic IK for TWO links                                 [2.0 pt]
# -----------------------------------------------------------------------------
def ik_two_link(root, l1, l2, target, elbow_up=True):
    """Return (q1, q2) so the end of link 2 lands on `target`, or None if it can't.

    1. d = distance from root to target.  Reachable only if |l1 - l2| <= d <= l1 + l2.
    2. Law of cosines:   cos(q2) = (d*d - l1*l1 - l2*l2) / (2*l1*l2)
       Two answers, +acos and -acos.  elbow_up=True is the NEGATIVE one.
       (clamp the cosine into [-1, 1] before acos)
    3. q1 = atan2(dy, dx) - atan2(l2*sin(q2), l1 + l2*cos(q2))
       where (dx, dy) = target - root.
    """
    # TODO 2: return (q1, q2) or None
    raise NotImplementedError("ik_two_link")


# -----------------------------------------------------------------------------
# TASK 3 - Cyclic Coordinate Descent (CCD)                           [3.5 pt]
# -----------------------------------------------------------------------------
def ik_ccd(root, lengths, q_start, target, limits, max_iter=100, tol=0.001):
    """Return (q, converged) - the new joint angles, and True if the tip is within `tol`.

    One SWEEP visits the joints from the TIP back to the ROOT.  For joint i:
      - joint = point i of the chain,   tip = last point   (use forward_kinematics)
      - angle_to_tip    = atan2 of (tip    - joint)
      - angle_to_target = atan2 of (target - joint)
      - q[i] += wrap_angle(angle_to_target - angle_to_tip)
      - clamp q[i] into limits[i] = (low, high)          <- add this second
    Recompute the points after EVERY joint (the tip just moved!).
    Stop after a sweep that ends within `tol`, or after `max_iter` sweeps.

    Also:  do not change q_start (return a new list), and an unreachable
    target must not crash - the arm just ends up stretched towards it.
    """
    # TODO 3: return (q, converged)
    raise NotImplementedError("ik_ccd")


# ----------------------------------------------------------------------------
if __name__ == "__main__":
    ik_helper.run(MODE, SOLVER, forward_kinematics, ik_two_link, ik_ccd,
                  JOINT_LIMITS_DEG, CCD_MAX_ITER, CCD_TOL)
