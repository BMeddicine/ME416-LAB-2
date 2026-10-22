"""Stamp holder GUI: webcam photo -> 26 stamp positions -> click a stamp, the arm picks it up
with the magnet and puts it back.

    python src/stamp_hover_gui.py [stampholder.json] [--port COM6] [--camera 1]

1. Live view shows whether tags 0, 1 and 3 are seen. Keep the arm out of the shot (Park),
   then Take photo (or SPACE). The frame goes to photos/webcam_<time>.png and MATLAB runs
   locateStampHolder on it (about 20 s). Load result... reuses a logs/stampholder_*.json.
2. Click a stamp on the annotated photo. The tool point (EE) picks it up, then puts it back:
     travel    lift / retreat / rotate / extend to above the stamp at the safe height
               (if the base doesn't need to turn: lift, then straight across)
     grip      straight down until the EE is GRIP_DEPTH_mm below the stamp top
     pull out  straight up to the carry height (safe height + the stamp hanging below the EE)
     lower     straight down to the slide height (grip height + SLIDE_RAISE_mm, so the
               end plate clears the well rim): the stamp hangs in its well
     slide     in along the arm (radially toward the base) by the slide distance, at the
               slide height: the well holds the stamp and the magnet shears off
     clear     straight up to the safe height
   Legs that are already done (e.g. the lift when the arm is already high) are skipped.
   Pick up / Put down run the halves on their own (put down = into the selected well).
   While carrying, a click puts the stamp down in the clicked well.
3. Position check: Hover over takes the EE to the hover height above the selected stamp.
   The base jog buttons turn the base alone, the reach jog buttons move the EE in/out along
   the arm at the same height. Jog until the EE is centred on the stamp; Record offset
   appends both jogs to logs/stamp_offsets.csv (logs/base_offsets.csv has the older,
   base-only records).

4. Stamp job tab: with a stamp selected, Run stamp job picks it up, presses it into the ink
   pad (tag 2 + config/stamp_job.json offset) the set number of times, presses it on the
   paper spot, puts it back in its well and parks. The presses go down to "approach" above
   contact at normal speed, then press / press up at the press speed with a dwell at the
   bottom. Every press (ink and paper) then rocks: the EE goes "rock" mm out along the arm,
   in, and back to the centre, "rocks" times, before lifting. "Click on photo picks: paper spot" moves the spot. Hover ink / Hover paper check
   the targets. While this tab is open, clicking a stamp (with "Move as soon as a stamp is
   clicked" ticked) runs the stamp job instead of Pick & return.
   Stamp word does the same for every stamp in the Word entry, in order (e.g. "L3 U7 L3"),
   without parking in between: the first letter at the paper spot, each next one the letter
   spacing further along +V. The whole route is planned before the arm moves. "Click on photo
   picks: add to word" appends the clicked stamp to the word.

Each leg is one firmware goto, which moves the joints in step, i.e. a straight line in
servo space, not in space. A leg whose tool path would bow more than PATH_TOL_mm away
from the straight line (FINE_PATH_TOL_mm for the legs at the well) is split at its midpoint
until it doesn't.

The safe height starts at the tallest stamp top + SAFE_CLEARANCE_mm. Geometry and the IK are
the same as src/move_to_tag.py (config/arm_bryson.json). Not connected = plan only.
"""

import argparse
import csv
import datetime
import json
import math
import queue
import re
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import NamedTuple

import cv2
import numpy as np
import serial
import serial.tools.list_ports

from check_stamp_holder import matlabCommand
from ik_gui import BASE, ELBOW, SHOULDER, Kinematics, loadCalibration
from move_to_tag import (DEFAULT_ARM_CONFIG, DEFAULT_CAMERA, DEFAULT_MATLAB, DEFAULT_SPEED_degps,
                         DEFAULT_WEBCAM_CALIBRATION, PHOTO_DIR, WEBCAM_SIZE_px,
                         WEBCAM_WARMUP_FRAMES, Arm, ReachError, ReachModel, loadArmConfig)
from servo_angle_gui import LOG_DIR, REPO_ROOT, loadJointConfig

HOLDER_CONFIG = REPO_ROOT / "config" / "stamp_holder.json"
JOB_CONFIG = REPO_ROOT / "config" / "stamp_job.json"
LAYOUT_CONFIG = REPO_ROOT / "config" / "layout.json"
NEEDED_TAGS = ((0, "base tag 0"), (1, "holder tag 1 (left)"), (3, "holder tag 3 (right)"))

GRIP_DEPTH_mm = 15.0            # EE this far below the stamp top to grip it
DEFAULT_SLIDE_mm = 20.0         # slide in along the arm this far to shear the magnet off
DEFAULT_HOVER_mm = 5.0          # Hover over: EE this far above the stamp top
JOG_STEPS_deg = (-1.0, -0.5, 0.5, 1.0)  # base servo jog buttons
REACH_STEPS_mm = (-2.0, -0.5, 0.5, 2.0)  # reach jog buttons (+ = out, away from the base)
SETTLE_s = 0.5                  # pause before the grip / lower legs so the arm stops swinging
SETTLE_BEFORE = {"grip", "lower", "press"}
BASE_OVERSHOOT_deg = 5.0        # base servo deg: every base move ends coming DOWN (clockwise
                                # from above) from at least this far above its target, so the
                                # servo's ~3 deg direction-dependent band lands on the same side
OFFSETS_FILE = LOG_DIR / "stamp_offsets.csv"  # Record offset appends here
SLIDE_RAISE_mm = 8.0            # slide this far above the grip height (rim = stamp top - 10)
SAFE_CLEARANCE_mm = 20.0        # default safe height = tallest stamp top + this
DEFAULT_RETREAT_mm = 90.0       # reach (along the arm) to pull in to before turning the base
PATH_TOL_mm = 2.0               # largest allowed bow of a leg away from its straight line
FINE_PATH_TOL_mm = 1.0          # same, for the legs in and next to the well
FINE_LEGS = {"grip", "pull out", "lower", "slide", "clear", "approach", "press", "rock",
             "press up"}
PATH_SAMPLES = 12
MAX_SPLIT_DEPTH = 4             # up to 16 gotos per leg
SAME_POSE_deg = 0.3             # legs that move no servo more than this are skipped
SAME_BASE_deg = 0.5             # base servo this close to the target: no retreat/rotate needed

DISPLAY_SCALE = 1.5             # 640x480 photo shown at up to 960x720 (less on small screens)
PICK_RADIUS_px = 25             # click within this of a stamp top (display pixels)
LIVE_PERIOD_ms = 60

COLORS = {"lower": "#00dcff", "upper": "#3cff3c", "bad": "#e53935", "selected": "#ffd600",
          "arm": "#ff00ff", "ink": "#ff8c00", "paper": "#4da3ff"}


# ---------------------------------------------------------------- route planning
def armPose(model, theta0, R, W):
    """Pose with the tool point at reach R along the arm (base angle theta0) and height W."""
    t = math.radians(theta0)
    pose = model.solve(R * math.cos(t) - model.ds * math.sin(t),
                       R * math.sin(t) + model.ds * math.cos(t), W)
    pose["R"] = R
    return pose


def poseFromServos(model, s):
    """(theta0, R, W) of the tool point for servo angles s: the forward kinematics."""
    theta0 = model.baseAngleOfServo(s[BASE])
    _, (rI, zI) = model.kin.fkFromServos(s[SHOULDER], s[ELBOW])
    W = model.zA + zI + model.dz
    return theta0, model.reachFromI(theta0, rI, W), W


def reachOf(model, U, V):
    return math.sqrt(U ** 2 + V ** 2 - model.ds ** 2)


def bow(model, a, b):
    """Largest distance (mm, in the arm's plane) of the servo-space path a -> b from the
    straight line between its ends."""
    (_, Ra, Wa), (_, Rb, Wb) = poseFromServos(model, a), poseFromServos(model, b)
    dR, dW = Rb - Ra, Wb - Wa
    length2 = dR ** 2 + dW ** 2
    worst = 0.0
    for k in range(1, PATH_SAMPLES):
        f = k / PATH_SAMPLES
        _, R, W = poseFromServos(model, [sa + f * (sb - sa) for sa, sb in zip(a, b)])
        u = 0.0 if length2 == 0 else min(1.0, max(0.0, ((R - Ra) * dR + (W - Wa) * dW) / length2))
        worst = max(worst, math.hypot(R - Ra - u * dR, W - Wa - u * dW))
    return worst


def splitLeg(model, startS, end, tol, depth=0):
    """Waypoints (poses) from servo angles startS to the pose end, split until each bows
    less than tol mm. Only end keeps its "event"."""
    if depth >= MAX_SPLIT_DEPTH or bow(model, startS, end["s"]) <= tol:
        return [end]
    t0, R0, W0 = poseFromServos(model, startS)
    try:
        mid = armPose(model, (t0 + end["theta0"]) / 2, (R0 + end["R"]) / 2, (W0 + end["W"]) / 2)
    except ReachError:
        return [end]
    mid["label"] = end["label"]
    mid["slow"] = end.get("slow", False)
    return (splitLeg(model, startS, mid, tol, depth + 1)
            + splitLeg(model, mid["s"], end, tol, depth + 1))


def planRoute(model, startS, legs):
    """legs = [(label, pose)]; drops legs that don't move, splits the rest. Returns poses."""
    route, s, pending = [], list(startS), None
    for label, pose in legs:
        if max(abs(a - b) for a, b in zip(s, pose["s"])) <= SAME_POSE_deg:
            pending = pose.get("event", pending)        # already there: fire it on the next move
            continue
        pose["label"] = label
        route += splitLeg(model, s, pose, FINE_PATH_TOL_mm if label in FINE_LEGS else PATH_TOL_mm)
        if pending and route[-1].get("event") is None:
            route[-1]["event"] = pending
        pending = None
        s = pose["s"]
    return route


def endServos(startS, legs):
    """Servo angles at the end of legs (startS if there are none)."""
    return list(legs[-1][1]["s"]) if legs else list(startS)


class Settings(NamedTuple):
    grip: float         # EE below the stamp top when gripping, mm
    slide: float        # slide in along the arm to release, mm
    slideRaise: float   # slide this far above the grip height, mm
    safeW: float        # travel height without a stamp
    retreatR: float     # reach to pull in to before turning the base
    stampHeight: float

    @property
    def carryW(self):
        """Travel height with a stamp: it hangs (stamp height - grip) below the EE."""
        return self.safeW + max(0.0, self.stampHeight - self.grip)


def stampPoint(model, stamp):
    """(U, V, top W, reach R, base angle theta0) of a stamp."""
    U, V, Wtop = stamp["top_UVW_mm"]
    return U, V, Wtop, reachOf(model, U, V), model.solve(U, V, Wtop)["theta0"]


def atStamp(model, U, V, W):
    pose = model.solve(U, V, W)
    pose["R"] = reachOf(model, U, V)
    return pose


def retreatLegs(model, startS, safeW, retreatR):
    """Lift straight up to safeW, then pull in to retreatR, at the current base angle."""
    theta0, R, W = poseFromServos(model, startS)
    legs = []
    if W < safeW:
        try:
            legs.append(("lift", armPose(model, theta0, R, safeW)))
        except ReachError as err:
            raise ReachError(f"can't lift straight up from here ({err}). Move the arm up first.")
    legs.append(("retreat", armPose(model, theta0, retreatR, safeW)))
    return legs


def travelLegs(model, startS, U, V, travelW, retreatR):
    """To above (U, V) at travelW. If the base has to turn: lift, retreat, rotate, extend.
    If it doesn't (e.g. already above that stamp): lift, then straight across."""
    above = atStamp(model, U, V, travelW)
    if abs(above["s"][BASE] - startS[BASE]) > SAME_BASE_deg:
        legs = retreatLegs(model, startS, travelW, retreatR)
        if above["s"][BASE] > startS[BASE] - BASE_OVERSHOOT_deg:
            # Turning counterclockwise (or only a little clockwise): go past, then come back down.
            legs.append(("overshoot", armPose(model, above["theta0"] + BASE_OVERSHOOT_deg / model.baseA,
                                              retreatR, travelW)))
        return legs + [
            ("rotate", armPose(model, above["theta0"], retreatR, travelW)),
            ("extend", above),
        ]
    theta0, R, W = poseFromServos(model, startS)
    legs = []
    if W < travelW:
        try:
            legs.append(("lift", armPose(model, theta0, R, travelW)))
        except ReachError as err:
            raise ReachError(f"can't lift straight up from here ({err}). Move the arm up first.")
    return legs + [("extend", above)]


def pickLegs(model, startS, stamp, st):
    """Travel to above the stamp, down until the EE is st.grip below its top, then straight
    up to the carry height. The "gripped" event fires at the bottom."""
    U, V, Wtop, _, _ = stampPoint(model, stamp)
    grip = atStamp(model, U, V, Wtop - st.grip)
    grip["event"] = "gripped"
    return travelLegs(model, startS, U, V, st.safeW, st.retreatR) + [
        ("grip", grip),
        ("pull out", atStamp(model, U, V, st.carryW)),
    ]


def putLegs(model, startS, stamp, st):
    """Travel to above the well at the carry height, straight down to the slide height
    (st.slideRaise above the grip height), slide in along the arm by st.slide (the well keeps
    the stamp, the magnet shears off), then straight up to the safe height. The "released"
    event fires at the end of the slide."""
    U, V, Wtop, R, theta0 = stampPoint(model, stamp)
    slideW = Wtop - st.grip + st.slideRaise
    slide = armPose(model, theta0, R - st.slide, slideW)
    slide["event"] = "released"
    return travelLegs(model, startS, U, V, st.carryW, st.retreatR) + [
        ("lower", atStamp(model, U, V, slideW)),
        ("slide", slide),
        ("clear", armPose(model, theta0, R - st.slide, st.safeW)),
    ]


def hoverLegs(model, startS, stamp, hover, st):
    """Travel to above the stamp at the safe height, then straight down to hover mm above
    its top."""
    U, V, Wtop, _, _ = stampPoint(model, stamp)
    return travelLegs(model, startS, U, V, st.safeW, st.retreatR) + [
        ("descend", atStamp(model, U, V, Wtop + hover)),
    ]


def pickAndReturnLegs(model, startS, stamp, st):
    pick = pickLegs(model, startS, stamp, st)
    return pick + putLegs(model, endServos(startS, pick), stamp, st)


def parkLegs(model, startS, st, homePose, travelW=None):
    """Retreat at travelW (default the safe height), turn the base to its home angle, then
    go to the firmware's home pose."""
    travelW = st.safeW if travelW is None else travelW
    home = {"s": list(homePose)}
    home["theta0"], home["R"], home["W"] = poseFromServos(model, homePose)
    return retreatLegs(model, startS, travelW, st.retreatR) + [
        ("rotate", armPose(model, home["theta0"], st.retreatR, travelW)),
        ("home", home),
    ]


class JobSettings(NamedTuple):
    inkDepth: float     # stamp face this far below the ink pad surface, mm
    dabs: int           # presses into the ink
    paperDepth: float   # stamp face this far below the paper surface, mm
    approach: float     # face this far above the surface before pressing, mm
    dwell: float        # wait at the bottom of each press, s
    pressSpeed: float   # servo speed for press / press up, deg/s
    rock: float = 0.0   # each press: rock the EE this far out and in along the arm, mm
    rocks: int = 0      # each press: this many out-and-in rocks at the bottom


def pressHeights(target, depth, st, job):
    """Tool point W (approach, bottom of the press) for pressing the carried stamp's face
    depth mm into the surface at target; the face hangs (stamp height - grip) below the EE."""
    hang = st.stampHeight - st.grip
    return target[2] + hang + job.approach, target[2] + hang - depth


def pressProblem(model, target, depth, st, job):
    """None if the press at target is reachable, else why not (with the lowest W possible)."""
    U, V, _ = target
    approachW, pressW = pressHeights(target, depth, st, job)
    try:
        for W in (approachW, pressW):
            model.solve(U, V, W)
        return None
    except ReachError as err:
        lowest = next((W for W in range(0, 200) if _reachable(model, U, V, W)), None)
        hint = "" if lowest is None else f"; lowest tool W here is ~{lowest}"
        return (f"needs tool W {pressW:.1f} at reach {reachOf(model, U, V):.0f} ({err}){hint}. "
                "Move it further from the base (more -U).")


def _reachable(model, U, V, W):
    try:
        model.solve(U, V, W)
        return True
    except ReachError:
        return False


def pressLegs(model, startS, target, depth, st, job, event, times=1, rock=0.0, rocks=0):
    """Carrying a stamp: travel to above target = (U, V, surface W) at the carry height,
    down until the stamp face is job.approach above the surface, then `times` slow presses
    to depth below it (dwell at the bottom) and back up. event fires after the last one.
    With rock and rocks, each press rocks at the bottom: the EE moves rock mm out along the
    arm, rock mm in, and back to the centre, rocks times. The end plate stays level, so the
    rocking comes from the stamp tipping on the magnet while the paper holds its face."""
    U, V, surfaceW = target
    approachW, pressW = pressHeights(target, depth, st, job)
    try:
        legs = travelLegs(model, startS, U, V, st.carryW, st.retreatR)
        legs.append(("approach", atStamp(model, U, V, approachW)))
        bottom = atStamp(model, U, V, pressW)
        rocking = []
        for _ in range(rocks if rock > 0 else 0):
            for dR in (rock, -rock):
                rocking.append(armPose(model, bottom["theta0"], bottom["R"] + dR, pressW))
            rocking.append(atStamp(model, U, V, pressW))
    except ReachError as err:
        name = {"inked": "ink pad", "stamped": "paper"}.get(event, event)
        raise ReachError(f"can't reach the {name} press at [{U:.1f}, {V:.1f}] (tool W "
                         f"{pressW:.1f}, reach {reachOf(model, U, V):.1f}): {err}") from None
    for k in range(times):
        press = atStamp(model, U, V, pressW)
        press["slow"], press["dwell"] = True, job.dwell
        legs.append(("press", press))
        for pose in rocking:
            pose = dict(pose, s=list(pose["s"]), slow=True)
            legs.append(("rock", pose))
        up = atStamp(model, U, V, approachW)
        up["slow"] = True
        if k == times - 1:
            up["event"] = event
        legs.append(("press up", up))
    return legs


def letterLegs(model, startS, stamp, ink, spot, st, job):
    """One letter: pick the stamp up, ink it (job.dabs presses), press it on the paper at
    spot, put it back in its well. ink / spot = (U, V, surface W)."""
    legs = pickLegs(model, startS, stamp, st)
    legs += pressLegs(model, endServos(startS, legs), ink, job.inkDepth, st, job, "inked",
                      job.dabs, rock=job.rock, rocks=job.rocks)
    legs += pressLegs(model, endServos(startS, legs), spot, job.paperDepth, st, job, "stamped",
                      rock=job.rock, rocks=job.rocks)
    return legs + putLegs(model, endServos(startS, legs), stamp, st)


def stampJobLegs(model, startS, stamp, ink, spot, st, job, homePose):
    """The whole job: one letter, then park."""
    legs = letterLegs(model, startS, stamp, ink, spot, st, job)
    return legs + parkLegs(model, endServos(startS, legs), st, homePose)


def stampWordLegs(model, startS, stamps, ink, spots, st, job, homePose):
    """Several letters in one go: letterLegs for each (stamp, spot) in order, straight from
    one to the next, then park once. Each letter's events get its index appended
    ("gripped 0", "stamped 2", ...)."""
    legs = []
    for k, (stamp, spot) in enumerate(zip(stamps, spots)):
        try:
            letter = letterLegs(model, endServos(startS, legs), stamp, ink, spot, st, job)
        except ReachError as err:
            raise ReachError(f"letter {k + 1} ({stamp['name']}): {err}") from None
        for _, pose in letter:
            if pose.get("event"):
                pose["event"] = f"{pose['event']} {k}"
        legs += letter
    return legs + parkLegs(model, endServos(startS, legs), st, homePose)


def parseWord(text, stamps):
    """Stamps named in text (space or comma separated, any case), in order. Raises
    ValueError naming the first unknown name."""
    byName = {s["name"].upper(): s for s in stamps}
    names = [n for n in re.split(r"[\s,]+", text.strip().upper()) if n]
    unknown = next((n for n in names if n not in byName), None)
    if unknown:
        raise ValueError(f"no stamp named {unknown} (names are L1..L13, U1..U13)")
    return [byName[n] for n in names]


def wordSpots(spot, advance, n):
    """Paper spots of n letters: the first at spot, each next one advance mm along +V."""
    U, V, W = spot
    return [(U, V + k * advance, W) for k in range(n)]


def loadHolder():
    return json.loads(HOLDER_CONFIG.read_text())


def loadJob():
    return json.loads(JOB_CONFIG.read_text())


def baseTagCenter():
    """Tag 0's centre [U, V] from config/layout.json (it defines the world frame)."""
    return json.loads(LAYOUT_CONFIG.read_text())["baseTag"]["centerUV_mm"]


def paperCorner(job):
    """[U, V] of the paper's corner: its edge runs from here toward +V, the paper is at -U."""
    U0, V0 = baseTagCenter()
    return U0 + job["paperEdgeU_mm"], V0 + job["paperCornerV_mm"]


def inkPadPoint(result, job):
    """(U, V, surface W) of the ink pad centre from the ink tag in the result; None if the
    tag wasn't seen (or the result predates other tags being saved)."""
    tags = result.get("tags") or []
    if isinstance(tags, dict):
        tags = [tags]
    tag = next((t for t in tags if t["id"] == job["inkTagId"]), None)
    if tag is None:
        return None
    dU, dV, dW = job["inkOffsetUVW_mm"]
    return tag["centerUV_mm"][0] + dU, tag["centerUV_mm"][1] + dV, tag["height_mm"] + dW


def defaultSafeHeight(holder):
    """Tallest stamp top in the holder CAD (holder on the plate) + SAFE_CLEARANCE_mm."""
    tallest = max(s["seatXYZ_mm"][2] for s in holder["stamps"]) + holder["stampHeight_mm"]
    return math.ceil(tallest + SAFE_CLEARANCE_mm)


def describe(pose):
    s1, s2, s3 = pose["s"]
    event = f"  [{pose['event']}]" if pose.get("event") else ""
    if pose.get("slow"):
        event = "  (slow)" + event
    return (f"{pose['label']:<8} theta0 {pose['theta0']:7.2f}  reach {pose['R']:6.1f}  "
            f"W {pose['W']:6.1f} mm   s = {s1:6.1f} {s2:6.1f} {s3:6.1f}{event}")


# ---------------------------------------------------------------- stamp results
def stampPixels(result):
    """Raw-photo pixel of each stamp top. Older JSON without topPx: project with the pose
    (no lens distortion, so a few px off near the edges)."""
    return [tuple(s["topPx"]) if "topPx" in s else worldToPixel(result, s["top_UVW_mm"])
            for s in result["stamps"]]


def distortionScale(result, xn):
    """MATLAB's radial model: distorted = undistorted * (1 + k1 r^2 + k2 r^4 ...)."""
    k = np.ravel(result.get("radialDistortion") or [])
    r2 = float(xn @ xn)
    return 1.0 + sum(kk * r2 ** (i + 1) for i, kk in enumerate(k))


def worldToPixel(result, X):
    """Raw-photo pixel of world point X = [U, V, W] (with the lens distortion when the result
    has it; older results project without it, a few px off near the edges)."""
    pose = result["pose"]
    xc = np.array(pose["R"]) @ np.asarray(X, float) + np.ravel(pose["t"])
    xn = xc[:2] / xc[2]
    xn = xn * distortionScale(result, xn)
    K = np.array(pose["K"])
    return K[0, 0] * xn[0] + K[0, 2], K[1, 1] * xn[1] + K[1, 2]


def pixelToWorld(result, px, py, W):
    """[U, V] on the plane at height W seen at raw-photo pixel (px, py)."""
    pose = result["pose"]
    K, R, t = np.array(pose["K"]), np.array(pose["R"]), np.ravel(pose["t"])
    distorted = np.array([(px - K[0, 2]) / K[0, 0], (py - K[1, 2]) / K[1, 1]])
    xn = distorted
    for _ in range(10):                               # undo the radial model by iteration
        xn = distorted / distortionScale(result, xn)
    centre = -R.T @ t
    ray = R.T @ np.array([xn[0], xn[1], 1.0])
    point = centre + (W - centre[2]) / ray[2] * ray
    return float(point[0]), float(point[1])


def locateWithMatlab(matlab, calibration, photoPath):
    """Run locateStampHolder on the photo. Returns (json path, MATLAB output) or (None, output)."""
    run = subprocess.run([matlab, "-batch", matlabCommand(photoPath, calibration)], cwd=REPO_ROOT,
                         capture_output=True, text=True)
    output = (run.stdout + run.stderr).strip()
    saved = re.search(r"Saved (.+?\.json)", output)
    if run.returncode != 0 or not saved:
        return None, output
    return Path(saved[1].strip()), output


# ---------------------------------------------------------------- GUI
class StampHoverGui:
    def __init__(self, root, args, model):
        self.root = root
        self.args = args
        self.model = model
        self.homePose = [j["home"] for j in model.kin.joints]
        self.arm = None
        self.cap = None
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
        self.events = queue.Queue()          # callables posted by worker threads
        self.abort = threading.Event()
        self.armBusy = False
        self.matlabBusy = False
        self.live = False
        self.lastFrame = None
        self.result = None
        self.pixels = []
        self.selected = None                 # index into result["stamps"]
        self.armAt = None                    # stamp the arm last picked up or put down
        self.photo = None
        self.holder = loadHolder()
        self.job = loadJob()
        self.inkPad = None                   # (U, V, surface W) from the loaded result
        self.neededTags = NEEDED_TAGS + ((self.job["inkTagId"],
                                          f"ink pad tag {self.job['inkTagId']}"),)

        root.title("Stamp pick and return")
        self._buildConnectionBar()
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True)
        self.scale = max(1.0, min(DISPLAY_SCALE, (root.winfo_screenheight() - 280) / WEBCAM_SIZE_px[1],
                                  (root.winfo_screenwidth() - 520) / WEBCAM_SIZE_px[0]))
        w, h = (round(v * self.scale) for v in WEBCAM_SIZE_px)
        self.canvas = tk.Canvas(body, width=w, height=h, background="#202020",
                                highlightthickness=0, cursor="crosshair")
        self.canvas.pack(side="left", padx=6, pady=6)
        self.imageItem = self.canvas.create_image(0, 0, anchor="nw")
        self.canvas.bind("<Button-1>", self.onClick)
        side = ttk.Frame(body)
        side.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        self._buildPhotoPanel(side)
        self._buildMotionPanel(side)
        self._buildLog(side)

        root.bind("<Escape>", lambda _e: self.stop())
        root.bind("<space>", self.onSpace)
        root.protocol("WM_DELETE_WINDOW", self.onClose)
        self.refreshPorts()
        self.pollEvents()
        if args.result:
            self.loadResult(Path(args.result))
        else:
            self.startLive()

    # ---------------------------------------------------------------- layout
    def _buildConnectionBar(self):
        bar = ttk.Frame(self.root, padding=6)
        bar.pack(fill="x")
        ttk.Label(bar, text="Port:").pack(side="left")
        self.portVar = tk.StringVar(value=self.args.port or "")
        self.portBox = ttk.Combobox(bar, textvariable=self.portVar, width=32)
        self.portBox.pack(side="left", padx=4)
        ttk.Button(bar, text="Refresh", command=self.refreshPorts).pack(side="left")
        self.connectButton = ttk.Button(bar, text="Connect", command=self.toggleConnection)
        self.connectButton.pack(side="left", padx=4)
        self.connStatus = ttk.Label(bar, text="Disconnected (plan only)", foreground="#b00020")
        self.connStatus.pack(side="left", padx=8)
        tk.Button(bar, text="STOP (Esc)", command=self.stop, bg="#c62828", fg="white",
                  font=("Segoe UI", 10, "bold"), padx=14).pack(side="right")
        ttk.Button(bar, text="Set", command=self.setSpeed).pack(side="right", padx=4)
        self.speedVar = tk.StringVar(value=f"{self.args.speed:g}")
        ttk.Entry(bar, textvariable=self.speedVar, width=6).pack(side="right")
        ttk.Label(bar, text="Speed (deg/s):").pack(side="right")

    def _buildPhotoPanel(self, side):
        box = ttk.LabelFrame(side, text="Photo", padding=6)
        box.pack(fill="x")
        row = ttk.Frame(box)
        row.pack(fill="x")
        self.liveButton = ttk.Button(row, text="Live view", command=self.startLive)
        self.liveButton.pack(side="left")
        self.photoButton = ttk.Button(row, text="Take photo (Space)", command=self.takePhoto)
        self.photoButton.pack(side="left", padx=4)
        ttk.Button(row, text="Load result...", command=self.chooseResult).pack(side="left")
        self.resultVar = tk.StringVar(value="No stamp positions yet.")
        ttk.Label(box, textvariable=self.resultVar, wraplength=330, justify="left").pack(
            fill="x", pady=(6, 0))

    def _buildMotionPanel(self, side):
        notebook = self.notebook = ttk.Notebook(side)
        notebook.pack(fill="x", pady=6)
        box = ttk.Frame(notebook, padding=6)
        notebook.add(box, text="Pick & place (tool point, mm)")
        job = self.jobTab = ttk.Frame(notebook, padding=6)
        notebook.add(job, text="Stamp job")
        self._buildJobPanel(job)
        self.gripVar = tk.StringVar(value=f"{self.args.grip:g}")
        self.slideVar = tk.StringVar(value=f"{self.args.slide:g}")
        self.slideRaiseVar = tk.StringVar(value=f"{self.args.slide_raise:g}")
        self.safeVar = tk.StringVar(value=f"{defaultSafeHeight(self.holder):g}")
        self.retreatVar = tk.StringVar(value=f"{self.args.retreat:g}")
        for row, (label, var) in enumerate((("Grip: EE below stamp top", self.gripVar),
                                            ("Slide back to release", self.slideVar),
                                            ("Slide above grip height", self.slideRaiseVar),
                                            ("Safe travel height W", self.safeVar),
                                            ("Retreat reach", self.retreatVar))):
            ttk.Label(box, text=label).grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(box, textvariable=var, width=8)
            entry.grid(row=row, column=1, sticky="w", padx=4, pady=1)
            entry.bind("<Return>", lambda _e: self.redraw())
            entry.bind("<FocusOut>", lambda _e: self.redraw())
        self.carryVar = tk.StringVar()
        ttk.Label(box, textvariable=self.carryVar).grid(row=5, column=0, columnspan=2, sticky="w")
        self.holdingVar = tk.BooleanVar(value=False)
        ttk.Checkbutton(box, text="Carrying a stamp (untick if it fell off)",
                        variable=self.holdingVar).grid(row=6, column=0, columnspan=2,
                                                       sticky="w", pady=(4, 0))
        self.moveOnClickVar = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="Move as soon as a stamp is clicked",
                        variable=self.moveOnClickVar).grid(row=7, column=0, columnspan=2,
                                                           sticky="w")
        buttons = ttk.Frame(box)
        buttons.grid(row=8, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Button(buttons, text="Pick & return", command=self.pickAndReturn).pack(side="left")
        ttk.Button(buttons, text="Pick up", command=self.pickUp).pack(side="left", padx=4)
        ttk.Button(buttons, text="Put down", command=self.putDown).pack(side="left")
        buttons = ttk.Frame(box)
        buttons.grid(row=9, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Button(buttons, text="Retreat", command=self.retreat).pack(side="left")
        ttk.Button(buttons, text="Park (home)", command=self.park).pack(side="left", padx=4)
        self._buildBaseCheck(box)
        self.selectedVar = tk.StringVar(value="Click a stamp on the photo.")
        ttk.Label(box, textvariable=self.selectedVar, wraplength=330, justify="left",
                  font=("Consolas", 9)).grid(row=10, column=0, columnspan=2, sticky="w",
                                             pady=(6, 0))
        self.updateCarryText()

    def _buildBaseCheck(self, box):
        frame = ttk.LabelFrame(box, text="Position check", padding=4)
        frame.grid(row=11, column=0, columnspan=2, sticky="we", pady=(8, 0))
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Hover above top").pack(side="left")
        self.hoverVar = tk.StringVar(value=f"{self.args.hover:g}")
        ttk.Entry(row, textvariable=self.hoverVar, width=6).pack(side="left", padx=4)
        ttk.Button(row, text="Hover over", command=self.hoverOver).pack(side="left")
        row = ttk.Frame(frame)
        row.pack(fill="x", pady=(4, 0))
        ttk.Label(row, text="Base servo").pack(side="left")
        for step in JOG_STEPS_deg:
            ttk.Button(row, text=f"{step:+g}", width=5,
                       command=lambda d=step: self.jogBase(d)).pack(side="left", padx=1)
        row = ttk.Frame(frame)
        row.pack(fill="x", pady=(4, 0))
        ttk.Label(row, text="Reach (mm)").pack(side="left")
        for step in REACH_STEPS_mm:
            ttk.Button(row, text=f"{step:+g}", width=5,
                       command=lambda d=step: self.jogReach(d)).pack(side="left", padx=1)
        ttk.Button(frame, text="Record offset", command=self.recordOffset).pack(
            anchor="w", pady=(4, 0))
        self.jogVar = tk.StringVar()
        ttk.Label(frame, textvariable=self.jogVar, wraplength=330, justify="left",
                  font=("Consolas", 9)).pack(fill="x", pady=(4, 0))
        self.hoverAt = None
        self.updateJogText()

    def _buildJobPanel(self, box):
        job = self.job
        corner = paperCorner(job)
        inset = job["paperSpotInset_mm"]
        self.jobVars = {}
        fields = (("inkDepth", "Ink: press face below pad", job["inkPressDepth_mm"]),
                  ("dabs", "Ink: presses", job["inkDabs"]),
                  ("paperDepth", "Paper: press face below paper", job["paperPressDepth_mm"]),
                  ("rock", "Rock out/in along arm", job.get("rock_mm", 0)),
                  ("rocks", "Rocks per press", job.get("rocks", 0)),
                  ("approach", "Slow from this above contact", job["approach_mm"]),
                  ("dwell", "Dwell at the bottom (s)", job["dwell_s"]),
                  ("pressSpeed", "Press speed (deg/s)", job["pressSpeed_degps"]),
                  ("spotU", "Paper spot U", round(corner[0] - inset[0], 1)),
                  ("spotV", "Paper spot V", round(corner[1] + inset[1], 1)),
                  ("advance", "Letter spacing (spot +V by)", job["advance_mm"]))
        for row, (key, label, value) in enumerate(fields):
            var = self.jobVars[key] = tk.StringVar(value=f"{value:g}")
            ttk.Label(box, text=label).grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(box, textvariable=var, width=8)
            entry.grid(row=row, column=1, sticky="w", padx=4, pady=1)
            entry.bind("<Return>", lambda _e: self.redraw())
            entry.bind("<FocusOut>", lambda _e: self.redraw())
        row = len(fields)
        word = ttk.Frame(box)
        word.grid(row=row, column=0, columnspan=2, sticky="we", pady=(4, 0))
        ttk.Label(word, text="Word (stamps in order)").pack(side="left")
        self.wordVar = tk.StringVar()
        entry = ttk.Entry(word, textvariable=self.wordVar, width=24)
        entry.pack(side="left", padx=4, fill="x", expand=True)
        entry.bind("<Return>", lambda _e: self.redraw())
        entry.bind("<FocusOut>", lambda _e: self.redraw())
        ttk.Button(word, text="Clear", width=6,
                   command=lambda: (self.wordVar.set(""), self.redraw())).pack(side="left")
        pick = ttk.Frame(box)
        pick.grid(row=row + 1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(pick, text="Click on photo picks:").pack(side="left")
        self.clickModeVar = tk.StringVar(value="stamp")
        for value, text in (("stamp", "stamp"), ("paper", "paper spot"), ("word", "add to word")):
            ttk.Radiobutton(pick, text=text, value=value,
                            variable=self.clickModeVar).pack(side="left", padx=2)
        buttons = ttk.Frame(box)
        buttons.grid(row=row + 2, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Button(buttons, text="Run stamp job", command=self.runStampJob).pack(side="left")
        ttk.Button(buttons, text="Stamp word", command=self.runStampWord).pack(side="left",
                                                                                padx=4)
        ttk.Button(buttons, text="Hover ink", command=self.hoverInk).pack(side="left")
        ttk.Button(buttons, text="Hover paper", command=self.hoverPaper).pack(side="left",
                                                                               padx=4)
        self.jobInfoVar = tk.StringVar()
        ttk.Label(box, textvariable=self.jobInfoVar, wraplength=330, justify="left",
                  font=("Consolas", 9)).grid(row=row + 3, column=0, columnspan=2, sticky="w",
                                             pady=(6, 0))

    def _buildLog(self, side):
        box = ttk.LabelFrame(side, text="Log", padding=4)
        box.pack(fill="both", expand=True)
        self.logText = tk.Text(box, width=58, height=16, font=("Consolas", 8), state="disabled")
        scroll = ttk.Scrollbar(box, command=self.logText.yview)
        self.logText.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.logText.pack(fill="both", expand=True)

    # ---------------------------------------------------------------- photo / live view
    def openCamera(self):
        cap = cv2.VideoCapture(self.args.camera, cv2.CAP_DSHOW)
        if not cap.isOpened():
            self.log(f"Could not open camera {self.args.camera}. Try another --camera index.")
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_SIZE_px[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_SIZE_px[1])
        for _ in range(WEBCAM_WARMUP_FRAMES):           # let auto exposure settle
            cap.read()
        return cap

    def startLive(self):
        if self.cap is None:
            self.cap = self.openCamera()
            if self.cap is None:
                return
        if not self.live:
            self.live = True
            self.canvas.delete("overlay")
            self.updateLive()

    def updateLive(self):
        if not self.live:
            return
        ok, frame = self.cap.read()
        if not ok:
            self.log("Camera read failed.")
            self.live = False
            return
        if frame.shape[1::-1] != WEBCAM_SIZE_px:
            self.log(f"Camera gives {frame.shape[1]}x{frame.shape[0]}, the calibration needs "
                     f"{WEBCAM_SIZE_px[0]}x{WEBCAM_SIZE_px[1]}.")
            self.live = False
            return
        self.lastFrame = frame
        view = frame.copy()
        corners, ids, _ = self.detector.detectMarkers(frame)
        seen = set() if ids is None else set(ids.ravel().tolist())
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(view, corners, ids)
        for row, (tagId, name) in enumerate(self.neededTags):
            found = tagId in seen
            cv2.putText(view, f"{name}: {'seen' if found else 'NOT SEEN'}", (10, 25 + 25 * row),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0) if found else (0, 0, 255), 2)
        self.showImage(view)
        self.root.after(LIVE_PERIOD_ms, self.updateLive)

    def onSpace(self, event):
        # Space also presses a focused button and types into entries; only take over otherwise.
        if self.live and event.widget.winfo_class() not in ("TEntry", "TButton", "Button",
                                                            "TCheckbutton", "TCombobox"):
            self.takePhoto()

    def takePhoto(self):
        if self.matlabBusy:
            self.log("MATLAB is still locating the last photo.")
            return
        if not self.live or self.lastFrame is None:
            self.log("Start the live view first.")
            return
        self.live = False
        PHOTO_DIR.mkdir(exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        photoPath = PHOTO_DIR / f"webcam_{stamp}.png"
        cv2.imwrite(str(photoPath), self.lastFrame)
        self.showImage(self.lastFrame)
        self.log(f"Saved {photoPath.name}. Locating the stamp holder in MATLAB (about 20 s)...")
        self.matlabBusy = True
        self.photoButton.state(["disabled"])

        def work():
            jsonPath, output = locateWithMatlab(self.args.matlab, self.args.calibration, photoPath)
            self.post(self.matlabDone, jsonPath, output)
        threading.Thread(target=work, daemon=True).start()

    def matlabDone(self, jsonPath, output):
        self.matlabBusy = False
        self.photoButton.state(["!disabled"])
        if jsonPath is None:
            self.log(output)
            self.log("MATLAB did not produce stamp positions. Check the messages above, then "
                     "Live view and try again.")
            return
        for line in output.splitlines():
            if line.startswith(("WARNING", "Holder tags", "Base tag")):
                self.log(line)
        self.loadResult(jsonPath)

    def chooseResult(self):
        path = filedialog.askopenfilename(initialdir=LOG_DIR, title="Stamp holder result",
                                          filetypes=[("Stamp holder", "stampholder_*.json")])
        if path:
            self.loadResult(Path(path))

    def loadResult(self, path):
        try:
            result = json.loads(path.read_text())
            stamps = result["stamps"]
        except (OSError, ValueError, KeyError) as err:
            messagebox.showerror("Load result", f"Could not read {path}:\n{err}")
            return
        if isinstance(stamps, dict):                    # jsonencode of a single struct
            result["stamps"] = [stamps]
        image = None
        for candidate in (path.with_suffix(".jpg"), Path(result.get("photoPath", ""))):
            if candidate.is_file():
                image = cv2.imread(str(candidate))
                if image is not None:
                    break
        if image is None:
            messagebox.showerror("Load result", f"No image found for {path.name}.")
            return
        self.live = False
        self.result = result
        self.resultName = path.name
        self.pixels = stampPixels(result)
        self.background = image
        self.selected = self.armAt = self.hoverAt = None
        holder = result.get("holder", {})
        text = f"{path.name}"
        if "baselineError_mm" in holder:
            text += f"\nHolder tags {holder['baseline_mm']:.1f} mm apart " \
                    f"(CAD {holder['baselineError_mm']:+.1f} mm)"
        if any("topPx" not in s for s in result["stamps"]):
            text += "\n(older result without topPx: click targets ignore lens distortion)"
        self.inkPad = inkPadPoint(result, self.job)
        if self.inkPad is None:
            text += f"\nInk pad tag {self.job['inkTagId']} not in this result: no stamp job"
        self.resultVar.set(text)
        if isinstance(result.get("warnings"), str):
            result["warnings"] = [result["warnings"]]
        for warning in result.get("warnings") or []:
            self.log(f"WARNING: {warning}")
        self.log(f"Loaded {len(self.pixels)} stamps from {path.name}.")
        self.redraw()

    def showImage(self, bgr):
        view = cv2.resize(bgr, None, fx=self.scale, fy=self.scale,
                          interpolation=cv2.INTER_LINEAR)
        ok, buf = cv2.imencode(".ppm", view)
        if ok:
            self.photo = tk.PhotoImage(data=buf.tobytes())
            self.canvas.itemconfigure(self.imageItem, image=self.photo)

    # ---------------------------------------------------------------- stamp overlay
    def motionSettings(self):
        """Settings from the entries; None (and a log line) if one is bad."""
        try:
            grip, slide, slideRaise, safeW, retreatR = (float(v.get()) for v in (
                self.gripVar, self.slideVar, self.slideRaiseVar, self.safeVar, self.retreatVar))
        except ValueError:
            self.log("Grip, slide, slide height, safe height and retreat reach must be numbers.")
            return None
        if grip < 0 or slide < 0:
            self.log("Grip and slide must be >= 0 mm.")
            return None
        return Settings(grip, slide, slideRaise, safeW, retreatR, self.holder["stampHeight_mm"])

    def updateCarryText(self):
        st = self.motionSettings()
        self.carryVar.set("" if st is None else
                          f"Carry height W {st.carryW:g} (stamp hangs "
                          f"{st.carryW - st.safeW:g} mm below the EE)")

    def reachProblem(self, stamp, st):
        """None if every pose of picking up / putting down this stamp is reachable, else why not."""
        U, V, Wtop = stamp["top_UVW_mm"]
        if st.safeW < Wtop:
            return f"safe height {st.safeW:g} is below the stamp top ({Wtop:.1f})"
        try:
            _, _, _, R, theta0 = stampPoint(self.model, stamp)
            for W in (Wtop - st.grip, Wtop - st.grip + st.slideRaise, st.safeW, st.carryW):
                self.model.solve(U, V, W)
            for W in (st.safeW, st.carryW):
                armPose(self.model, theta0, st.retreatR, W)
            for W in (Wtop - st.grip + st.slideRaise, st.safeW):
                armPose(self.model, theta0, R - st.slide, W)
        except ReachError as err:
            return str(err)
        return None

    def redraw(self):
        self.updateCarryText()
        if self.result is None or self.live:
            return
        self.showImage(self.background)
        self.canvas.delete("overlay")
        settings = self.motionSettings()
        for i, (stamp, (x, y)) in enumerate(zip(self.result["stamps"], self.pixels)):
            x, y = x * self.scale, y * self.scale
            bad = settings is None or self.reachProblem(stamp, settings) is not None
            color = COLORS["bad"] if bad else COLORS.get(stamp.get("row"), COLORS["lower"])
            r = 9
            self.canvas.create_oval(x - r, y - r, x + r, y + r, outline=color, width=2,
                                    tags="overlay")
            if bad:
                self.canvas.create_line(x - r, y - r, x + r, y + r, fill=color, width=2,
                                        tags="overlay")
            if i == self.armAt:
                self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=COLORS["arm"],
                                        outline="", tags="overlay")
            if i == self.selected:
                r = 15
                self.canvas.create_oval(x - r, y - r, x + r, y + r, outline=COLORS["selected"],
                                        width=3, tags="overlay")
        self.drawJobOverlay()
        self.updateSelectedText()

    def drawJobOverlay(self):
        """Paper edge (dashed, from its corner along +V), paper spot (cross) and ink pad."""
        def toCanvas(X):
            return [v * self.scale for v in worldToPixel(self.result, X)]
        Wp = self.job["paperHeight_mm"]
        cU, cV = paperCorner(self.job)
        edge = toCanvas([cU - 30, cV, Wp]) + toCanvas([cU, cV, Wp]) + toCanvas([cU, cV + 200, Wp])
        self.canvas.create_line(*edge, fill=COLORS["paper"], width=2, dash=(6, 3), tags="overlay")
        spot = self.paperSpot()
        if spot is not None:
            x, y = toCanvas(spot)
            self.canvas.create_line(x - 10, y, x + 10, y, fill=COLORS["paper"], width=2,
                                    tags="overlay")
            self.canvas.create_line(x, y - 10, x, y + 10, fill=COLORS["paper"], width=2,
                                    tags="overlay")
            self.canvas.create_text(x + 12, y - 12, text="paper", fill=COLORS["paper"],
                                    anchor="sw", tags="overlay")
        plan = self.wordPlan(log=lambda _text: None)
        if plan is not None:
            for stamp, X in zip(*plan):
                x, y = toCanvas(X)
                self.canvas.create_rectangle(x - 4, y - 4, x + 4, y + 4, outline=COLORS["paper"],
                                             width=2, tags="overlay")
                self.canvas.create_text(x - 6, y + 6, text=stamp["name"], fill=COLORS["paper"],
                                        anchor="ne", tags="overlay")
        if self.inkPad is not None:
            x, y = toCanvas(self.inkPad)
            self.canvas.create_oval(x - 12, y - 12, x + 12, y + 12, outline=COLORS["ink"],
                                    width=2, tags="overlay")
            self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=COLORS["ink"], outline="",
                                    tags="overlay")
            self.canvas.create_text(x + 14, y - 14, text="ink", fill=COLORS["ink"], anchor="sw",
                                    tags="overlay")
        self.updateJobText()

    def paperSpot(self):
        """(U, V, paper W) of the stamp spot from the entries; None if they aren't numbers."""
        try:
            return (float(self.jobVars["spotU"].get()), float(self.jobVars["spotV"].get()),
                    self.job["paperHeight_mm"])
        except ValueError:
            return None

    def wordPlan(self, log=None):
        """(stamps, paper spots) of the word in the entry; None if it's empty or something is
        wrong, after passing why to log (default: the GUI log)."""
        log = log or self.log
        if self.result is None:
            log("Load a stamp holder result first.")
            return None
        try:
            stamps = parseWord(self.wordVar.get(), self.result["stamps"])
        except ValueError as err:
            log(f"Word: {err}.")
            return None
        if not stamps:
            log("Word: type the stamps in order (e.g. L3 U7 L3), or pick 'add to word' and "
                "click them on the photo.")
            return None
        spot = self.paperSpot()
        try:
            advance = float(self.jobVars["advance"].get())
        except ValueError:
            spot = None
        if spot is None:
            log("Word: paper spot U, V and the letter spacing must be numbers.")
            return None
        if len(stamps) > 1 and advance == 0:
            log("Word: set the letter spacing, or every letter lands on the same spot.")
            return None
        return stamps, wordSpots(spot, advance, len(stamps))

    def jobSettings(self, quiet=False):
        """JobSettings from the entries; None (and a log line unless quiet) if one is bad."""
        log = (lambda _text: None) if quiet else self.log
        v = {key: var.get() for key, var in self.jobVars.items()}
        try:
            job = JobSettings(float(v["inkDepth"]), int(v["dabs"]), float(v["paperDepth"]),
                              float(v["approach"]), float(v["dwell"]), float(v["pressSpeed"]),
                              float(v["rock"]), int(v["rocks"]))
        except ValueError:
            log("Stamp job: depths, approach, dwell, speed and rock must be numbers, presses "
                "and rocks whole numbers.")
            return None
        if (job.dabs < 1 or job.approach <= 0 or job.dwell < 0 or job.pressSpeed <= 0
                or job.rock < 0 or job.rocks < 0):
            log("Stamp job: need presses >= 1, approach > 0, dwell >= 0, press speed > 0, "
                "rock and rocks >= 0.")
            return None
        return job

    def updateJobText(self):
        lines = []
        table = self.model.baseTable
        st = self.motionSettings()
        job = self.jobSettings(quiet=True)
        depths = {"Ink pad": job and job.inkDepth, "Paper spot": job and job.paperDepth}
        for name, target in (("Ink pad", self.inkPad), ("Paper spot", self.paperSpot())):
            if target is None:
                lines.append(f"{name}: -")
                continue
            U, V, W = target
            theta0 = self.model.baseAngleOf(U, V)
            if theta0 is None:
                lines.append(f"{name}: [{U:.1f}, {V:.1f}, {W:.1f}] too close to the base")
                continue
            lines.append(f"{name}: [{U:.1f}, {V:.1f}, {W:.1f}]\n"
                         f"  reach {reachOf(self.model, U, V):.1f}, theta0 {theta0 % 360:.1f} deg")
            problem = st and job and pressProblem(self.model, target, depths[name], st, job)
            if problem:
                lines.append(f"  CAN'T REACH the press: {problem}")
            if table and not table[0][0] <= theta0 % 360 <= table[-1][0]:
                lines.append(f"  outside the base calibration (theta0 {table[0][0]:g}-"
                             f"{table[-1][0]:g}): may land off by a few mm")
        spot = self.paperSpot()
        cU, cV = paperCorner(self.job)
        if spot is not None and (spot[0] > cU or spot[1] < cV):
            lines.append("Paper spot is OFF the paper (paper is U < "
                         f"{cU:g}, V > {cV:g})")
        why = []
        plan = self.wordPlan(log=why.append) if self.wordVar.get().strip() else None
        if plan is not None:
            stamps, spots = plan
            U, V, _ = spots[-1]
            lines.append(f"Word: {' '.join(s['name'] for s in stamps)} ({len(stamps)} letters), "
                         f"last at [{U:.1f}, {V:.1f}]")
            for stamp, target in zip(stamps, spots):
                problem = ((st and self.reachProblem(stamp, st))
                           or (st and job and pressProblem(self.model, target, job.paperDepth,
                                                           st, job)))
                if problem:
                    lines.append(f"  CAN'T DO {stamp['name']} at V {target[1]:.1f}: {problem}")
                    break
                if target[0] > cU or target[1] < cV:
                    lines.append(f"  {stamp['name']} at V {target[1]:.1f} is OFF the paper")
                    break
        elif why:
            lines.append(why[0])
        self.jobInfoVar.set("\n".join(lines))

    def updateSelectedText(self):
        if self.selected is None:
            self.selectedVar.set("Click a stamp on the photo.")
            return
        stamp = self.result["stamps"][self.selected]
        U, V, W = stamp["top_UVW_mm"]
        text = (f"{stamp['name']}: top [{U:.1f}, {V:.1f}, {W:.1f}] mm\n"
                f"reach {reachOf(self.model, U, V):.1f} mm, "
                f"base angle {self.model.baseAngleOf(U, V):.1f} deg")
        settings = self.motionSettings()
        problem = settings and self.reachProblem(stamp, settings)
        if problem:
            text += f"\nCAN'T REACH: {problem}"
        self.selectedVar.set(text)

    def onClick(self, event):
        if self.result is None or self.live or self.matlabBusy:
            return
        x, y = event.x / self.scale, event.y / self.scale
        if self.clickModeVar.get() == "paper":
            U, V = pixelToWorld(self.result, x, y, self.job["paperHeight_mm"])
            self.jobVars["spotU"].set(f"{U:.1f}")
            self.jobVars["spotV"].set(f"{V:.1f}")
            self.log(f"Paper spot set to [{U:.1f}, {V:.1f}] mm.")
            self.redraw()
            return
        dist =[math.hypot(px - x, py - y) for px, py in self.pixels]
        nearest = min(range(len(dist)), key=dist.__getitem__)
        if dist[nearest] * self.scale > PICK_RADIUS_px:
            return
        if self.clickModeVar.get() == "word":
            self.wordVar.set(f"{self.wordVar.get().strip()} "
                             f"{self.result['stamps'][nearest]['name']}".strip())
            self.redraw()
            return
        self.selected = nearest
        self.redraw()
        if not self.moveOnClickVar.get():
            return
        if self.holdingVar.get():
            self.putDown()
        elif self.notebook.select() == str(self.jobTab):
            self.runStampJob()                  # Stamp job tab open: a click stamps that letter
        else:
            self.pickAndReturn()

    # ---------------------------------------------------------------- motion
    def selectedStamp(self):
        if self.selected is None:
            self.log("Select a stamp first.")
            return None
        return self.result["stamps"][self.selected]

    def stampEvents(self, stamp):
        """Callbacks for the route's events: keep "Carrying a stamp" in step with the arm."""
        def gripped():
            self.holdingVar.set(True)
            self.log(f"   picked up {stamp['name']}")

        def released():
            self.holdingVar.set(False)
            self.log(f"   put down {stamp['name']}")
        return {"gripped": gripped, "released": released}

    def pickAndReturn(self):
        stamp, index = self.selectedStamp(), self.selected
        if stamp is None:
            return
        if self.holdingVar.get():
            self.log("Already carrying a stamp: Put down first (or untick Carrying).")
            return
        self.runMotion(f"pick up and return {stamp['name']}",
                       lambda s, st: pickAndReturnLegs(self.model, s, stamp, st),
                       lambda: setattr(self, "armAt", index), self.stampEvents(stamp))

    def pickUp(self):
        stamp, index = self.selectedStamp(), self.selected
        if stamp is None:
            return
        if self.holdingVar.get():
            self.log("Already carrying a stamp: Put down first (or untick Carrying).")
            return
        self.runMotion(f"pick up {stamp['name']}",
                       lambda s, st: pickLegs(self.model, s, stamp, st),
                       lambda: setattr(self, "armAt", index), self.stampEvents(stamp))

    def putDown(self):
        stamp, index = self.selectedStamp(), self.selected
        if stamp is None:
            return
        if not self.holdingVar.get():
            self.log("Not carrying a stamp: Pick up first (or tick Carrying).")
            return
        self.runMotion(f"put down in {stamp['name']}",
                       lambda s, st: putLegs(self.model, s, stamp, st),
                       lambda: setattr(self, "armAt", index), self.stampEvents(stamp))

    def retreat(self):
        holding = self.holdingVar.get()       # read here: the legs are made on the worker thread
        self.runMotion("retreat", lambda s, st: retreatLegs(
            self.model, s, st.carryW if holding else st.safeW, st.retreatR))

    def hoverOver(self):
        stamp, index = self.selectedStamp(), self.selected
        if stamp is None:
            return
        if self.holdingVar.get():
            self.log("Carrying a stamp: Put down before hovering (or untick Carrying).")
            return
        try:
            hover = float(self.hoverVar.get())
        except ValueError:
            self.log("Hover must be a number.")
            return
        U, V, Wtop = stamp["top_UVW_mm"]
        try:
            target = atStamp(self.model, U, V, Wtop + hover)
        except ReachError as err:
            self.log(f"Can't hover over {stamp['name']}: {err}")
            return

        def arrived():
            self.armAt = index
            self.hoverAt = {"stamp": stamp, "pose": target, "s1": target["s"][BASE],
                            "dR": 0.0, "s23": tuple(target["s"][1:])}
            self.updateJogText()
        self.runMotion(f"hover over {stamp['name']}",
                       lambda s, st: hoverLegs(self.model, s, stamp, hover, st), arrived)

    def jogBase(self, step):
        if self.hoverAt is None:
            self.log("Hover over a stamp first: the jog starts from there.")
            return
        if self.armBusy:
            self.log("The arm is still moving.")
            return
        self.hoverAt["s1"] += step
        self.updateJogText()
        if self.arm is None:
            return
        # Like the travel moves: always come down onto the new angle from BASE_OVERSHOOT_deg above.
        s1, (s2, s3) = self.hoverAt["s1"], self.hoverAt["s23"]
        self.jogMoves([[s1 + BASE_OVERSHOOT_deg, s2, s3], [s1, s2, s3]])

    def jogReach(self, step):
        """Move the EE step mm out (+) or in along the arm at the hover height, base unchanged."""
        h = self.hoverAt
        if h is None:
            self.log("Hover over a stamp first: the jog starts from there.")
            return
        if self.armBusy:
            self.log("The arm is still moving.")
            return
        pose = h["pose"]
        try:
            moved = armPose(self.model, pose["theta0"], pose["R"] + h["dR"] + step, pose["W"])
        except ReachError as err:
            self.log(f"Can't jog the reach: {err}")
            return
        h["dR"] += step
        h["s23"] = tuple(moved["s"][1:])
        self.updateJogText()
        if self.arm is not None:
            self.jogMoves([[h["s1"], *h["s23"]]])

    def jogMoves(self, targets):
        """Run the jog gotos in a worker thread."""
        arm = self.arm
        self.armBusy = True
        self.abort.clear()

        def work():
            try:
                for target in targets:
                    arm.moveTo(target, abort=self.abort)
            except ReachError as err:
                if str(err) != "stopped":
                    arm.send("stop")
                self.post(self.log, f"-- jog: {err}")
            finally:
                self.post(setattr, self, "armBusy", False)
        threading.Thread(target=work, daemon=True).start()

    def updateJogText(self):
        h = self.hoverAt
        if h is None:
            self.jogVar.set("Hover over a stamp, then jog base and reach until the EE is centred.")
            return
        delta = h["s1"] - h["pose"]["s"][BASE]
        self.jogVar.set(f"{h['stamp']['name']} at W {h['pose']['W']:.1f}: base servo {h['s1']:.2f} "
                        f"(commanded {h['pose']['s'][BASE]:.2f}, jog {delta:+.2f} = "
                        f"{delta / self.model.baseA:+.2f} deg of theta0)\n"
                        f"reach jog {h['dR']:+.1f} mm (+ = out)")

    def recordOffset(self):
        h = self.hoverAt
        if h is None:
            self.log("Hover over a stamp and jog it centred first.")
            return
        pose, cmd = h["pose"], h["pose"]["s"][BASE]
        new = not OFFSETS_FILE.exists()
        with open(OFFSETS_FILE, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "result", "stamp", "U_mm", "V_mm", "W_mm", "R_mm", "theta0_deg",
                            "s1_cmd_deg", "s1_centred_deg", "jog_deg", "reachJog_mm",
                            "reachCorrection_mm", "baseA", "baseB"])
            w.writerow([datetime.datetime.now().isoformat(timespec="seconds"), self.resultName,
                        h["stamp"]["name"], f"{pose['U']:.2f}", f"{pose['V']:.2f}",
                        f"{pose['W']:.2f}", f"{pose['R']:.2f}", f"{pose['theta0']:.3f}",
                        f"{cmd:.2f}", f"{h['s1']:.2f}", f"{h['s1'] - cmd:+.2f}", f"{h['dR']:+.1f}",
                        f"{self.model.reachCorrectionAt(pose['W']) + self.model.reachTableAt(pose['theta0'], pose['R']):.2f}",
                        self.model.baseA, self.model.baseB])
        self.log(f"Recorded {h['stamp']['name']} at W {pose['W']:.1f}: base servo "
                 f"{h['s1'] - cmd:+.2f}, reach {h['dR']:+.1f} mm in {OFFSETS_FILE.name}")

    def park(self):
        """Retreat, turn the base to its home angle, then go to the firmware's home pose."""
        if self.holdingVar.get():
            self.log("Carrying a stamp: Put down before parking (or untick Carrying).")
            return
        self.runMotion("park", lambda s, st: parkLegs(self.model, s, st, self.homePose))

    # ---------------------------------------------------------------- stamp job
    def runStampJob(self):
        """Selected stamp: pick up, ink, press on the paper spot, put back, park."""
        stamp = self.selectedStamp()
        if stamp is None:
            return
        if self.holdingVar.get():
            self.log("Already carrying a stamp: Put down first (or untick Carrying).")
            return
        if self.inkPad is None:
            self.log(f"No ink pad: tag {self.job['inkTagId']} isn't in this result. Take a "
                     "photo with it in view.")
            return
        job, spot, ink = self.jobSettings(), self.paperSpot(), self.inkPad
        if job is None or spot is None:
            if spot is None:
                self.log("Paper spot U and V must be numbers.")
            return
        events = self.stampEvents(stamp)
        events["inked"] = lambda: self.log(f"   inked {stamp['name']}")

        def stamped():
            self.log(f"   stamped {stamp['name']} at [{spot[0]:.1f}, {spot[1]:.1f}]")
            try:
                advance = float(self.jobVars["advance"].get())
            except ValueError:
                return
            if advance:
                self.jobVars["spotV"].set(f"{spot[1] + advance:.1f}")
                self.redraw()
        events["stamped"] = stamped
        self.runMotion(f"stamp job {stamp['name']}",
                       lambda s, st: stampJobLegs(self.model, s, stamp, ink, spot, st, job,
                                                  self.homePose),
                       events=events, pressSpeed=job.pressSpeed)

    def runStampWord(self):
        """The stamps in the Word entry, in order: each one picked up, inked, pressed on the
        paper (the first at the paper spot, each next one the letter spacing along +V) and
        put back, without parking in between; then park. The whole route is planned (and
        every pose checked) before the arm moves."""
        if self.holdingVar.get():
            self.log("Already carrying a stamp: Put down first (or untick Carrying).")
            return
        if self.inkPad is None:
            self.log(f"No ink pad: tag {self.job['inkTagId']} isn't in this result. Take a "
                     "photo with it in view.")
            return
        job, plan, ink = self.jobSettings(), self.wordPlan(), self.inkPad
        if job is None or plan is None:
            return
        stamps, spots = plan
        names = [s["name"] for s in stamps]
        advance = float(self.jobVars["advance"].get())   # a number: wordPlan checked it
        events = {}
        for k, (stamp, spot) in enumerate(zip(stamps, spots)):
            for name, fn in self.stampEvents(stamp).items():
                events[f"{name} {k}"] = fn
            events[f"inked {k}"] = lambda s=stamp: self.log(f"   inked {s['name']}")

            def stamped(k=k, spot=spot):
                self.log(f"   stamped {names[k]} ({k + 1}/{len(names)}) at "
                         f"[{spot[0]:.1f}, {spot[1]:.1f}]")
                # Spot and word = what's left, so after a STOP, Stamp word carries on.
                self.jobVars["spotV"].set(f"{spot[1] + advance:.1f}")
                self.wordVar.set(" ".join(names[k + 1:]))
                self.redraw()
            events[f"stamped {k}"] = stamped
        word = " ".join(names)
        self.runMotion(f"stamp word {word}",
                       lambda s, st: stampWordLegs(self.model, s, stamps, ink, spots, st, job,
                                                   self.homePose),
                       lambda: (self.wordVar.set(word), self.redraw()),
                       events=events, pressSpeed=job.pressSpeed)

    def hoverInk(self):
        if self.inkPad is None:
            self.log(f"No ink pad: tag {self.job['inkTagId']} isn't in this result.")
            return
        self.hoverTarget("ink pad", self.inkPad)

    def hoverPaper(self):
        spot = self.paperSpot()
        if spot is None:
            self.log("Paper spot U and V must be numbers.")
            return
        self.hoverTarget("paper spot", spot)

    def hoverTarget(self, name, target):
        """EE (or, when carrying, the stamp face) Hover-above mm over target = (U, V, W)."""
        try:
            hover = float(self.hoverVar.get())
        except ValueError:
            self.log("Hover must be a number.")
            return
        holding = self.holdingVar.get()
        U, V, W = target

        def legs(s, st):
            hang = st.stampHeight - st.grip if holding else 0.0
            return travelLegs(self.model, s, U, V, st.carryW if holding else st.safeW,
                              st.retreatR) + [("descend", atStamp(self.model, U, V, W + hang + hover))]
        self.runMotion(f"hover {hover:g} mm over the {name}"
                       + (" (stamp face)" if holding else ""), legs)

    def runMotion(self, what, makeLegs, onDone=None, events=None, pressSpeed=None):
        """Plan from the arm's current servo angles (home if not connected) and run it in a
        worker thread. makeLegs(startServos, Settings) -> [(label, pose)]. events maps a
        pose's "event" to a callback run (on the Tk thread) once the arm reaches that pose.
        Poses marked "slow" run at pressSpeed deg/s (if given); a pose's "dwell" (s) waits
        there after arriving."""
        if self.armBusy:
            self.log("The arm is still moving. STOP first to change target.")
            return
        settings = self.motionSettings()
        if settings is None:
            return
        try:
            normalSpeed = float(self.speedVar.get())
        except ValueError:
            normalSpeed = self.args.speed
        self.armBusy = True
        self.armAt = self.hoverAt = None
        self.abort.clear()
        arm = self.arm
        events = events or {}
        slow = [False]                       # press speed currently set on the firmware
        at = [""]                            # which move is running, for error messages

        def work():
            try:
                start = [pos for pos, _ in arm.status()] if arm else list(self.homePose)
                route = planRoute(self.model, start, makeLegs(start, settings))
                self.post(self.log, f"-- {what}: {len(route)} moves"
                          + ("" if arm else " (not connected, plan from home only)"))
                for pose in route:
                    self.post(self.log, "   " + describe(pose))
                if arm is None:
                    return
                previous = None
                for step, pose in enumerate(route, 1):
                    at[0] = f" (at move {step}/{len(route)}, {pose['label']})"
                    if self.abort.is_set():
                        raise ReachError("stopped")
                    if pose["label"] in SETTLE_BEFORE and pose["label"] != previous:
                        time.sleep(SETTLE_s)        # let the arm stop swinging before going down
                    previous = pose["label"]
                    wantSlow = bool(pressSpeed and pose.get("slow"))
                    if wantSlow != slow[0]:
                        arm.send(f"speed {pressSpeed if wantSlow else normalSpeed:g}")
                        slow[0] = wantSlow
                    arm.moveTo(pose["s"], abort=self.abort)
                    if pose.get("dwell"):
                        time.sleep(pose["dwell"])
                    if pose.get("event") in events:
                        self.post(events[pose["event"]])
                self.post(self.log, f"-- done: {what}")
                if onDone:
                    self.post(onDone)
            except ReachError as err:
                if arm and str(err) != "stopped":
                    try:
                        arm.send("stop")
                    except ReachError:
                        pass
                self.post(self.log, f"-- {what}{at[0]}: {err}")
            finally:
                if arm and slow[0]:
                    try:
                        arm.send(f"speed {normalSpeed:g}")
                    except ReachError:
                        pass
                self.post(self.motionDone)
        threading.Thread(target=work, daemon=True).start()

    def motionDone(self):
        self.armBusy = False
        self.redraw()

    def stop(self):
        self.abort.set()
        if self.arm:
            self.arm.send("stop")
            self.log(">> stop")

    def setSpeed(self):
        try:
            speed = float(self.speedVar.get())
        except ValueError:
            return
        if speed > 0 and self.arm:
            self.arm.send(f"speed {speed:g}")
            self.log(f">> speed {speed:g}")

    # ---------------------------------------------------------------- serial
    def refreshPorts(self):
        ports = list(serial.tools.list_ports.comports())
        labels = [f"{p.device} - {p.description}" for p in ports]
        self.portBox["values"] = labels
        if labels and not self.portVar.get():
            preferred = [l for l in labels if "USB" in l or "JTAG" in l]
            self.portVar.set((preferred or labels)[0])

    def toggleConnection(self):
        if self.arm:
            if self.armBusy:
                self.log("STOP the arm before disconnecting.")
                return
            self.arm.close()
            self.arm = None
            self.connectButton.configure(text="Connect")
            self.connStatus.configure(text="Disconnected (plan only)", foreground="#b00020")
            return
        device = self.portVar.get().split(" - ")[0]
        if not device:
            messagebox.showwarning("No port", "Pick a serial port first.")
            return
        try:
            self.arm = Arm(device)
        except serial.SerialException as err:
            messagebox.showerror("Connect failed",
                                 f"{err}\n\nIs the PlatformIO monitor or another GUI open?")
            return
        self.connectButton.configure(text="Disconnect")
        self.connStatus.configure(text=f"Connected ({device})", foreground="#1b7f3b")
        self.setSpeed()

    # ---------------------------------------------------------------- misc
    def post(self, fn, *args):
        """Run fn(*args) on the Tk thread (worker threads must not touch widgets)."""
        self.events.put((fn, args))

    def pollEvents(self):
        try:
            while True:
                fn, args = self.events.get_nowait()
                fn(*args)
        except queue.Empty:
            pass
        self.root.after(50, self.pollEvents)

    def log(self, text):
        self.logText.configure(state="normal")
        self.logText.insert("end", text + "\n")
        self.logText.see("end")
        self.logText.configure(state="disabled")

    def onClose(self):
        self.stop()
        self.live = False
        if self.cap is not None:
            self.cap.release()
        if self.arm:
            self.arm.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("result", nargs="?", help="start from this logs/stampholder_*.json")
    parser.add_argument("--arm", default=DEFAULT_ARM_CONFIG, help="per-arm config JSON")
    parser.add_argument("--port", help="serial port, e.g. COM6")
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA,
                        help=f"webcam index (default {DEFAULT_CAMERA}; 0 is the laptop's own camera)")
    parser.add_argument("--calibration", default=DEFAULT_WEBCAM_CALIBRATION,
                        help=f"camera calibration (default {DEFAULT_WEBCAM_CALIBRATION})")
    parser.add_argument("--matlab", default=DEFAULT_MATLAB, help="matlab.exe to run the vision")
    parser.add_argument("--hover", type=float, default=DEFAULT_HOVER_mm,
                        help=f"Hover over: tool point above the stamp top, mm (default {DEFAULT_HOVER_mm:g})")
    parser.add_argument("--grip", type=float, default=GRIP_DEPTH_mm,
                        help=f"tool point below the stamp top to grip it, mm (default {GRIP_DEPTH_mm:g})")
    parser.add_argument("--slide", type=float, default=DEFAULT_SLIDE_mm,
                        help=f"slide in along the arm to release, mm (default {DEFAULT_SLIDE_mm:g})")
    parser.add_argument("--slide-raise", type=float, default=SLIDE_RAISE_mm,
                        help=f"slide this far above the grip height, mm (default {SLIDE_RAISE_mm:g})")
    parser.add_argument("--retreat", type=float, default=DEFAULT_RETREAT_mm,
                        help=f"reach to pull in to before turning, mm (default {DEFAULT_RETREAT_mm:g})")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_degps,
                        help=f"move speed, deg/s (default {DEFAULT_SPEED_degps:g})")
    args = parser.parse_args()

    cfg = loadArmConfig(args.arm)
    joints, _ = loadJointConfig()
    model = ReachModel(cfg, Kinematics(joints, loadCalibration(cfg["servoCalibration"])))
    root = tk.Tk()
    StampHoverGui(root, args, model)
    root.mainloop()


if __name__ == "__main__":
    main()
