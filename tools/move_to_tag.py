"""Reach test: drive the tool point to an AprilTag, then log where it actually landed.

1. Photo with the base tag (ID 0) and one other tag, then in MATLAB:
       locateTagTarget(<id>)        -> logs/tagtarget_<time>.json
2.     python tools/move_to_tag.py [tagtarget.json] [--port COM5] [--hover 5]
   Moves the tool point to hover above the tag centre, asks for the measured miss and
   appends it to logs/tag_reach_trials.csv.
   Or, with the webcam, steps 1 and 2 in a loop (photo -> MATLAB -> move -> measure -> home):
       python tools/move_to_tag.py --webcam <id> [--camera 1] --port COM6 --hover 20
3.     python tools/move_to_tag.py --fit
   Refits the base servo from every logged miss and reports the radial (in-plane) error.

Geometry (all from config/arm_<name>.json, see its _notes):
    base frame [U, V, W]: origin on the base axis at plate level (config/layout.json)
    arm frame [r, s, z]: r outward in the arm's plane at base angle theta0, s 90 deg CCW
    A = shoulder at shoulderOffset_mm = [rA, sA, zA] (t1 of the FK); I = forearm end,
    (rI, zI) from A; tool point P = I + toolOffset_mm [dr, sP, dz] in the arm frame
    ds = sA + sP = how far the arm's plane is beside the base axis (to the left if > 0)

Inverse kinematics for P = [U, V, W]:
    rho = |[U, V]|,  R = sqrt(rho^2 - ds^2)            (R = rA + rI + dr - c(W), along r)
    theta0 = atan2(V, U) - atan2(ds, R)                 (the side offset ds turns the base)
    rI = R - rA - dr + c(W),  zI = W - zA - dz  -> the two-link IK from ik_gui.py
    c(W) = reachCorrection_mm + reachCorrectionPerHeight * W: measured under-reach along r
        (> 0 sends I further out); it grows with the tool height W, not with the reach
    s1 = a*theta0 + b (baseFit), s2, s3 from the joint 2 / 3 fits
"""

import argparse
import collections
import csv
import datetime
import json
import math
import queue
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import serial
import serial.tools.list_ports

from ik_gui import BASE, Kinematics, loadCalibration
from servo_angle_gui import LOG_DIR, REPO_ROOT, STATUS_RE, SerialLink, fitLine, loadJointConfig

DEFAULT_ARM_CONFIG = REPO_ROOT / "config" / "arm_bryson.json"
TRIALS_FILE = LOG_DIR / "tag_reach_trials.csv"
PHOTO_DIR = REPO_ROOT / "photos"
TRIAL_FIELDS = [
    "time", "photo", "tagId", "tagU_mm", "tagV_mm", "tagW_mm",
    "U_mm", "V_mm", "W_mm", "rho_mm", "phi_deg",
    "theta0_deg", "rI_mm", "zI_mm", "theta1_deg", "theta2_deg",
    "s1_deg", "s2_deg", "s3_deg", "baseA", "baseB", "reachCorrection_mm",
    "missU_mm", "missV_mm", "missW_mm",
]

DEFAULT_HOVER_mm = 5.0          # tool point above the tag, so it never presses on the plate
APPROACH_LIFT_mm = 40.0         # come down onto the target from this far above it
DEFAULT_SPEED_degps = 20.0
ARRIVE_TOL_deg = 0.15           # firmware prints positions to 0.1 deg
ARRIVE_TIMEOUT_s = 30.0
STATUS_POLL_s = 0.4
STATUS_ATTEMPTS = 4             # ask for status this many times before giving up
RECENT_LINES = 20               # firmware lines kept for error messages
RESET_MARKERS = ("RASTICrane servo controller", "Assuming last saved pose")  # boot banner
MIN_FIT_SPREAD_deg = 5.0        # base angles must spread this much to fit the slope a
MIN_HEIGHT_SPREAD_mm = 15.0     # tool heights must spread this much to fit the per-height term

# Webcam mode: the image size must match the calibration file.
DEFAULT_CAMERA = 1
DEFAULT_WEBCAM_CALIBRATION = "calibration/webcamParams.mat"
WEBCAM_SIZE_px = (640, 480)
WEBCAM_WARMUP_FRAMES = 30
DEFAULT_MATLAB = r"C:\Program Files\MATLAB\R2024b\bin\matlab.exe"   # has the CV Toolbox


class ReachError(Exception):
    pass


def loadArmConfig(path):
    """Read the per-arm JSON and stop with the field name if a required value is null."""
    cfg = json.loads(Path(path).read_text())

    def require(value, name):
        values = value if isinstance(value, list) else [value]
        if value is None or any(v is None for v in values):
            sys.exit(f"{Path(path).name}: '{name}' is not filled in (null). Measure it first.")
        return value

    return {
        "name": cfg.get("name", Path(path).stem),
        "servoCalibration": REPO_ROOT / cfg["servoCalibration"],
        "shoulder": require(cfg["shoulderOffset_mm"], "shoulderOffset_mm"),
        "tool": require(cfg["toolOffset_mm"], "toolOffset_mm"),
        "baseA": require(cfg["baseFit"]["a"], "baseFit.a"),
        "baseB": require(cfg["baseFit"]["b"], "baseFit.b"),
        "baseCorrection": cfg.get("baseCorrection"),
        "reachTable": cfg.get("reachTable"),
        "minToolHeight_mm": cfg.get("minToolHeight_mm", 0.0),
        "reachCorrection_mm": cfg.get("reachCorrection_mm", 0.0),
        "reachCorrectionPerHeight": cfg.get("reachCorrectionPerHeight", 0.0),
    }


class ReachModel:
    """Tool point [U, V, W] <-> base angle + planar (r, z) of I, with the servo fits."""

    def __init__(self, cfg, kin):
        self.kin = kin
        self.rA, sA, self.zA = cfg["shoulder"]
        self.dr, sP, self.dz = cfg["tool"]
        self.ds = sA + sP              # both side offsets act the same way on the base angle
        self.baseA, self.baseB = cfg["baseA"], cfg["baseB"]
        table = cfg.get("baseCorrection") or {}
        self.baseTable = sorted(zip(table.get("theta0_deg", []), table.get("servo_deg", [])))
        self.minW = cfg["minToolHeight_mm"]
        self.reachCorrection = cfg["reachCorrection_mm"]
        self.reachPerHeight = cfg["reachCorrectionPerHeight"]
        self.reachTable = cfg.get("reachTable")

    def reachCorrectionAt(self, W):
        """Under-reach along r (mm) at tool height W; I is sent this much further out."""
        return self.reachCorrection + self.reachPerHeight * W

    def baseAngleOf(self, U, V):
        """theta0 (deg) that puts a tool point at [U, V]; None if inside the side offset."""
        rho = math.hypot(U, V)
        if rho <= abs(self.ds):
            return None
        R = math.sqrt(rho ** 2 - self.ds ** 2)
        return math.degrees(math.atan2(V, U) - math.atan2(self.ds, R))

    def reachTableAt(self, theta0_deg, R):
        """mm added to the commanded reach at (theta0, R): bilinear in the reachTable grid,
        held at the edge values outside it, 0 without a table."""
        table = self.reachTable
        if not table:
            return 0.0

        def bracket(nodes, x):
            """(i, f): x sits fraction f of the way from nodes[i] to nodes[i + 1]."""
            if x <= nodes[0]:
                return 0, 0.0
            for i in range(len(nodes) - 1):
                if x <= nodes[i + 1]:
                    return i, (x - nodes[i]) / (nodes[i + 1] - nodes[i])
            return len(nodes) - 2, 1.0

        thetas, reaches, dR = table["theta0_deg"], table["R_mm"], table["dR_mm"]
        i, fi = bracket(thetas, theta0_deg)
        j, fj = bracket(reaches, R)
        row = [dR[jj][i] * (1 - fi) + dR[jj][i + 1] * fi for jj in (j, j + 1)]
        return row[0] * (1 - fj) + row[1] * fj

    def reachFromI(self, theta0_deg, rI, W):
        """Tool reach R for forearm end I at rI: the inverse of solve()'s reach terms
        (the table changes slowly with R, so a few iterations converge)."""
        raw = self.rA + rI + self.dr - self.reachCorrectionAt(W)
        R = raw
        for _ in range(5):
            R = raw - self.reachTableAt(theta0_deg, R)
        return R

    def baseCorrectionAt(self, theta0_deg):
        """Servo deg added to the baseFit line at theta0: linear between the baseCorrection
        points, held at the end values outside them, 0 without a table."""
        table = self.baseTable
        if not table:
            return 0.0
        if theta0_deg <= table[0][0]:
            return table[0][1]
        for (t0, c0), (t1, c1) in zip(table, table[1:]):
            if theta0_deg <= t1:
                return c0 + (c1 - c0) * (theta0_deg - t0) / (t1 - t0)
        return table[-1][1]

    def servoOf(self, theta0_deg):
        return self.baseA * theta0_deg + self.baseB + self.baseCorrectionAt(theta0_deg)

    def baseAngleOfServo(self, s1):
        """Inverse of servoOf (the table's slope is small, so a few iterations converge)."""
        t = (s1 - self.baseB) / self.baseA
        for _ in range(5):
            t = (s1 - self.baseB - self.baseCorrectionAt(t)) / self.baseA
        return t

    def baseServo(self, theta0_deg):
        """s1 for theta0, trying theta0 +- 360 so the wrap never hides a reachable angle."""
        for t in (theta0_deg, theta0_deg - 360, theta0_deg + 360):
            s1 = self.servoOf(t)
            if self.kin.inLimits(BASE, s1):
                return t, s1
        return None, self.servoOf(theta0_deg)

    def solve(self, U, V, W):
        if W < self.minW:
            raise ReachError(f"tool height {W:.1f} mm is below minToolHeight_mm ({self.minW:g})")
        rho = math.hypot(U, V)
        theta0 = self.baseAngleOf(U, V)
        if theta0 is None:
            raise ReachError(f"rho = {rho:.1f} mm is inside the side offset ({self.ds:g} mm)")
        R = math.sqrt(rho ** 2 - self.ds ** 2)
        rI = R + self.reachTableAt(theta0, R) - self.rA - self.dr + self.reachCorrectionAt(W)
        zI = W - self.zA - self.dz
        sol, reason = self.kin.ik(rI, zI)
        if sol is None:
            raise ReachError(f"I at (r, z) = ({rI:.1f}, {zI:.1f}) mm: {reason}")
        theta0, s1 = self.baseServo(theta0)
        if theta0 is None:
            raise ReachError(f"joint 1 limit (servo {s1:.1f})")
        return {"U": U, "V": V, "W": W, "rho": rho, "phi": math.degrees(math.atan2(V, U)),
                "theta0": theta0, "rI": rI, "zI": zI, "theta1": sol["theta1"],
                "theta2": sol["theta2"], "s": (s1, sol["s2"], sol["s3"])}

    def toolPoint(self, theta0_deg, rI, zI):
        """Forward kinematics, the inverse of solve() (used by the self-check)."""
        t = math.radians(theta0_deg)
        W = self.zA + zI + self.dz
        R = self.reachFromI(theta0_deg, rI, W)
        return (R * math.cos(t) - self.ds * math.sin(t),
                R * math.sin(t) + self.ds * math.cos(t),
                W)


class Arm:
    """Blocking wrapper around SerialLink: send a goto and wait until the arm is there."""

    def __init__(self, device):
        self.lines = queue.Queue()
        self.link = SerialLink(self.lines)
        self.link.open(device)
        self.recent = collections.deque(maxlen=RECENT_LINES)   # for error messages
        time.sleep(0.3)
        self.drain()

    def close(self):
        self.link.close()

    def send(self, line):
        if not self.link.send(line):
            raise ReachError("serial port is closed")

    def _take(self, line):
        """Note a firmware line; raise if it shows the controller restarted or the port died."""
        self.recent.append(line)
        if line.startswith(RESET_MARKERS):
            raise ReachError("the arm controller restarted mid-move (power dip / brownout when "
                             "the servos stall?). It will glide home on its own; wait for it, "
                             "then check the stamp and replan.")
        if line.startswith("!! port lost") or line.startswith("!! write failed"):
            raise ReachError(f"serial link lost ({line[3:]})")

    def drain(self):
        """Print and return the lines the firmware sent since the last call."""
        out = []
        while True:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                return out
            out.append(line)
            if line.startswith(("Rejected", "!!", "STOP")):
                print(f"   arm: {line}")
            self._take(line)

    def status(self, timeout_s=2.0, attempts=STATUS_ATTEMPTS):
        """[(position, target)] per joint from the firmware's status reply. A missed or garbled
        reply is asked for again, up to attempts times."""
        for _ in range(attempts):
            self.drain()
            self.send("status")
            found = {}
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                try:
                    line = self.lines.get(timeout=0.05)
                except queue.Empty:
                    continue
                self._take(line)
                m = STATUS_RE.search(line)
                if m:
                    found[int(m[1]) - 1] = (float(m[2]), float(m[3]))
                    if len(found) == 3:
                        return [found[i] for i in range(3)]
        last = " | ".join(list(self.recent)[-5:]) or "nothing"
        raise ReachError(f"no status reply from the arm after {attempts} tries "
                         f"(last lines from it: {last})")

    def moveTo(self, s, abort=None):
        """abort: optional threading.Event; once set, stop waiting (the caller sends 'stop')."""
        self.send(f"goto {s[0]:.1f} {s[1]:.1f} {s[2]:.1f}")
        time.sleep(0.2)
        if any(line.startswith("Rejected") for line in self.drain()):
            raise ReachError("the firmware rejected the move")
        deadline = time.monotonic() + ARRIVE_TIMEOUT_s
        while time.monotonic() < deadline:
            time.sleep(STATUS_POLL_s)
            if abort is not None and abort.is_set():
                raise ReachError("stopped")
            joints = self.status()
            if all(abs(pos - want) <= ARRIVE_TOL_deg and abs(tgt - want) <= ARRIVE_TOL_deg
                   for (pos, tgt), want in zip(joints, s)):
                return joints
        raise ReachError("timed out waiting for the arm to arrive")


# ---------------------------------------------------------------- helpers
def newestTargetFile():
    files = sorted(LOG_DIR.glob("tagtarget_*.json"), key=lambda p: p.stat().st_mtime)
    if not files:
        sys.exit("No logs/tagtarget_*.json yet. Run locateTagTarget(<id>) in MATLAB first.")
    return files[-1]


def pickPort(requested):
    if requested:
        return requested
    ports = list(serial.tools.list_ports.comports())
    likely = [p for p in ports if "USB" in p.description or "JTAG" in p.description] or ports
    if len(likely) == 1:
        return likely[0].device
    listing = "\n".join(f"  {p.device} - {p.description}" for p in ports) or "  (none)"
    sys.exit(f"Pick a port with --port. Available:\n{listing}")


def printPlan(name, p):
    s1, s2, s3 = p["s"]
    print(f"{name:<9} P = [{p['U']:7.1f}, {p['V']:7.1f}, {p['W']:6.1f}] mm   "
          f"theta0 {p['theta0']:7.2f}  I (r, z) = ({p['rI']:6.1f}, {p['zI']:6.1f})  "
          f"theta1 {p['theta1']:7.2f}  theta2 {p['theta2']:7.2f}   "
          f"s = {s1:6.1f} {s2:6.1f} {s3:6.1f}")


def askMiss():
    print("\nMeasure the tool point's miss from the tag centre: actual - desired, in mm,")
    print("along U and V (parallel to tag 0's x and y edges), optionally W.")
    while True:
        text = input("dU dV [dW] (blank = skip): ").strip()
        if not text:
            return None
        try:
            values = [float(v) for v in text.replace(",", " ").split()]
        except ValueError:
            values = []
        if len(values) in (2, 3):
            return values + [None] * (3 - len(values))
        print("  Enter two or three numbers, e.g. 3.5 -2")


def appendTrial(row):
    LOG_DIR.mkdir(exist_ok=True)
    if TRIALS_FILE.exists():
        with open(TRIALS_FILE, newline="") as f:
            header = next(csv.reader(f), [])
        if header != TRIAL_FIELDS:              # older column layout: keep it, start a new file
            old = TRIALS_FILE.with_name(TRIALS_FILE.stem + "_old.csv")
            TRIALS_FILE.replace(old)
            print(f"(Old trials file has different columns; moved to {old.name}.)")
    isNew = not TRIALS_FILE.exists()
    with open(TRIALS_FILE, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=TRIAL_FIELDS)
        if isNew:
            writer.writeheader()
        writer.writerow(row)


def r3(x):
    return "" if x is None else round(x, 3)


# ---------------------------------------------------------------- one trial
def planTarget(targetPath, model, hover):
    """Read a tagtarget JSON and solve the approach and target poses; None if unreachable."""
    located = json.loads(Path(targetPath).read_text())
    target = located["target"]
    tagU, tagV, tagW = target["UVW_mm"]
    print(f"Target: tag {target['tagId']} from {Path(targetPath).name}")
    print(f"  tag centre [{tagU:.1f}, {tagV:.1f}, {tagW:.1f}] mm, rho {target['rho_mm']:.1f}, "
          f"phi {target['phi_deg']:.1f}; tool point {hover:g} mm above it\n")
    try:
        final = model.solve(tagU, tagV, tagW + hover)
    except ReachError as err:
        print(f"Can't reach the target: {err}")
        return None
    try:
        approach = model.solve(tagU, tagV, tagW + hover + APPROACH_LIFT_mm)
    except ReachError as err:
        print(f"(No approach point {APPROACH_LIFT_mm:g} mm above: {err}. Going straight there.)")
        approach = None
    if approach:
        printPlan("approach", approach)
    printPlan("target", final)
    return {"located": located, "approach": approach, "final": final}


def runTrial(arm, model, plan):
    """Move to the target, ask for the miss, log it, lift back up."""
    target = plan["located"]["target"]
    approach, final = plan["approach"], plan["final"]
    tagU, tagV, tagW = target["UVW_mm"]
    if approach:
        arm.moveTo(approach["s"])
    reached = arm.moveTo(final["s"])
    print("Arrived. Firmware reports s = " + " ".join(f"{pos:.1f}" for pos, _ in reached))

    miss = askMiss()
    if miss:
        appendTrial({
            "time": datetime.datetime.now().isoformat(timespec="seconds"),
            "photo": plan["located"].get("photoPath", ""),
            "tagId": target["tagId"], "tagU_mm": r3(tagU), "tagV_mm": r3(tagV), "tagW_mm": r3(tagW),
            "U_mm": r3(final["U"]), "V_mm": r3(final["V"]), "W_mm": r3(final["W"]),
            "rho_mm": r3(final["rho"]), "phi_deg": r3(final["phi"]),
            "theta0_deg": r3(final["theta0"]), "rI_mm": r3(final["rI"]), "zI_mm": r3(final["zI"]),
            "theta1_deg": r3(final["theta1"]), "theta2_deg": r3(final["theta2"]),
            "s1_deg": r3(final["s"][0]), "s2_deg": r3(final["s"][1]), "s3_deg": r3(final["s"][2]),
            "baseA": model.baseA, "baseB": model.baseB,
            "reachCorrection_mm": r3(model.reachCorrectionAt(final["W"])),    # as applied
            "missU_mm": r3(miss[0]), "missV_mm": r3(miss[1]), "missW_mm": r3(miss[2]),
        })
        print(f"Logged to {TRIALS_FILE}: miss {math.hypot(miss[0], miss[1]):.1f} mm in the plane")
    else:
        print("Not logged (no measurement).")
    if approach:
        print("Lifting back to the approach point.")
        arm.moveTo(approach["s"])


def openArm(args):
    arm = Arm(pickPort(args.port))
    arm.send(f"speed {args.speed:g}")
    return arm


# ---------------------------------------------------------------- webcam
def openCamera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {index}. Try another --camera index.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_SIZE_px[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_SIZE_px[1])
    for _ in range(WEBCAM_WARMUP_FRAMES):           # let auto exposure settle
        cap.read()
    return cap


def previewAndCapture(cap, targetId):
    """Live view with the tags OpenCV sees. SPACE returns the raw frame, Q / Esc returns None.
    The detection here only helps framing; the measurement is done by MATLAB."""
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    window = "move_to_tag: SPACE = take photo, Q = quit"
    cv2.namedWindow(window)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed.")
                return None
            if frame.shape[1::-1] != WEBCAM_SIZE_px:
                print(f"Camera gives {frame.shape[1]}x{frame.shape[0]}, the calibration needs "
                      f"{WEBCAM_SIZE_px[0]}x{WEBCAM_SIZE_px[1]}.")
                return None
            view = frame.copy()
            corners, ids, _ = detector.detectMarkers(frame)
            seen = set() if ids is None else set(ids.ravel().tolist())
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(view, corners, ids)
            for row, (tagId, name) in enumerate(((0, "base tag 0"), (targetId, f"target {targetId}"))):
                found = tagId in seen
                cv2.putText(view, f"{name}: {'seen' if found else 'NOT SEEN'}", (10, 25 + 25 * row),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0) if found else (0, 0, 255), 2)
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                return frame
            if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                return None
    finally:
        cv2.destroyWindow(window)
        cv2.waitKey(1)


def locateWithMatlab(args, photoPath):
    """Run locateTagTarget on the photo; returns the tagtarget JSON path or None."""
    photo = photoPath.relative_to(REPO_ROOT).as_posix()
    command = (f"addpath(genpath('src')); locateTagTarget({args.webcam}, '{photo}', "
               f"Calibration='{args.calibration}');")
    print(f"Locating tag {args.webcam} in MATLAB (about 20 s)...")
    run = subprocess.run([args.matlab, "-batch", command], cwd=REPO_ROOT, capture_output=True,
                         text=True)
    output = (run.stdout + run.stderr).strip()
    print(output)
    saved = re.search(r"Saved (.+?\.json)", output)
    if run.returncode != 0 or not saved:
        print("MATLAB did not produce a target. Fix the view and take another photo.")
        return None
    return Path(saved[1].strip())


# ---------------------------------------------------------------- modes
def runMove(args, model):
    """One trial from an existing tagtarget JSON."""
    plan = planTarget(Path(args.target) if args.target else newestTargetFile(), model, args.hover)
    if plan is None:
        sys.exit(1)
    if args.dry_run or input("\nMove the arm? [y/N] ").strip().lower() != "y":
        return
    arm = openArm(args)
    try:
        runTrial(arm, model, plan)
    except KeyboardInterrupt:
        arm.send("stop")
        print("\nStopped.")
    except ReachError as err:
        arm.send("stop")
        print(f"Stopped: {err}")
    finally:
        arm.close()


def runWebcam(args, model):
    """Repeat: webcam photo -> MATLAB locates the tag -> move -> measure -> park at home."""
    cap = openCamera(args.camera)
    arm = None
    homePose = [j["home"] for j in model.kin.joints]
    try:
        while True:
            print(f"\nFrame the shot (tag 0 and tag {args.webcam}, arm clear of both), "
                  "then press SPACE in the camera window.")
            frame = previewAndCapture(cap, args.webcam)
            if frame is None:
                break
            PHOTO_DIR.mkdir(exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            photoPath = PHOTO_DIR / f"webcam_{stamp}.png"
            cv2.imwrite(str(photoPath), frame)
            targetPath = locateWithMatlab(args, photoPath)
            if targetPath is None:
                continue
            plan = planTarget(targetPath, model, args.hover)
            if plan is None or args.dry_run:
                continue
            if input("\nMove the arm? [y/N] ").strip().lower() != "y":
                continue
            if arm is None:
                arm = openArm(args)
            try:
                runTrial(arm, model, plan)
                print("Parking at home so the arm is out of the next photo.")
                arm.moveTo(homePose)
            except ReachError as err:
                arm.send("stop")
                print(f"Stopped: {err}")
    except KeyboardInterrupt:
        if arm:
            arm.send("stop")
        print("\nStopped.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if arm:
            arm.close()


def runFit(model, since=None):
    """Base servo fit from the measured misses, plus the in-plane reach error.
    since: ISO time; only trials logged at or after it are used."""
    try:
        with open(TRIALS_FILE, newline="") as f:
            rows = [r for r in csv.DictReader(f) if r["missU_mm"] and r["missV_mm"]
                    and (since is None or r["time"] >= since)]
    except FileNotFoundError:
        rows = []
    if not rows:
        sys.exit(f"No measured trials in {TRIALS_FILE}" + (f" since {since}." if since else " yet."))

    print(f"{'tag':>4} {'theta0 cmd':>10} {'actual':>8} {'error':>7} {'radial':>7} "
          f"{'tangent':>8} {'miss':>6}   (deg, mm)")
    points, radial, tangent, corrections = [], [], [], []    # corrections: (W, needed c)
    for r in rows:
        U, V = float(r["U_mm"]), float(r["V_mm"])
        dU, dV = float(r["missU_mm"]), float(r["missV_mm"])
        theta0 = float(r["theta0_deg"])
        actual = model.baseAngleOf(U + dU, V + dV)
        if actual is None:
            continue
        actual += 360 * round((theta0 - actual) / 360)       # same turn as the command
        # Miss split into reach error along the arm (R = sqrt(rho^2 - ds^2), independent of
        # the base angle) and base angle error times the reach (across the arm).
        reachDesired = math.sqrt(U ** 2 + V ** 2 - model.ds ** 2)
        radial.append(math.sqrt((U + dU) ** 2 + (V + dV) ** 2 - model.ds ** 2) - reachDesired)
        tangent.append(math.radians(actual - theta0) * reachDesired)
        points.append((actual, float(r["s1_deg"]) - model.baseCorrectionAt(actual)))  # line part
        corrections.append((float(r["W_mm"]), float(r.get("reachCorrection_mm") or 0) - radial[-1]))
        print(f"{r['tagId']:>4} {theta0:10.2f} {actual:8.2f} {actual - theta0:7.2f} "
              f"{radial[-1]:7.1f} {tangent[-1]:8.1f} {math.hypot(dU, dV):6.1f}")

    def stats(values):
        mean = sum(values) / len(values)
        return mean, math.sqrt(sum(v * v for v in values) / len(values))

    n = len(points)
    meanR, rmsR = stats(radial)
    meanT, rmsT = stats(tangent)
    print(f"\n{n} trials. Radial miss mean {meanR:+.1f}, rms {rmsR:.1f} mm "
          f"(along the arm: shoulder offset, tool offset, joint 2/3 fits).")
    print(f"Tangential miss mean {meanT:+.1f}, rms {rmsT:.1f} mm (base angle: baseFit).")

    # Reach correction needed per trial = correction applied - radial miss; fit it against W.
    heights = [w for w, _ in corrections]
    if max(heights) - min(heights) >= MIN_HEIGHT_SPREAD_mm:
        k, c0, rms = fitLine(corrections)
        how = f"fit against tool height W, residual {rms:.1f} mm"
    else:
        k = model.reachPerHeight
        c0 = sum(c - k * w for w, c in corrections) / n
        how = (f"constant only, per-height term kept (need tool heights spread over "
               f"{MIN_HEIGHT_SPREAD_mm:g} mm or more to fit it)")
    print(f"\nReach correction c(W) = c0 + k*W, {how}:")
    print(f"  current  c0 = {model.reachCorrection:g}  k = {model.reachPerHeight:g}")
    print(f'  new      "reachCorrection_mm": {c0:.1f}, "reachCorrectionPerHeight": {k:.3f}')

    thetas = [p[0] for p in points]
    if n >= 2 and max(thetas) - min(thetas) >= MIN_FIT_SPREAD_deg:
        a, b, rms = fitLine(points)
        how = f"fit of a and b, residual {rms:.2f} servo deg"
    else:
        a = model.baseA
        b = sum(s - a * t for t, s in points) / n
        how = (f"b only, a kept at {a:g} (need base angles spread over "
               f"{MIN_FIT_SPREAD_deg:g} deg or more to fit a)")
    print(f"\nBase servo, {how}:")
    print(f"  current  a = {model.baseA:+.4f}  b = {model.baseB:.2f}")
    print(f'  new      "baseFit": {{ "a": {a:.4f}, "b": {b:.2f} }}')


def selfCheck(model):
    """solve() followed by toolPoint() must land back on the same point."""
    for U, V, W in ((150, 40, 5), (60, 160, 20), (-80, 120, 5)):
        try:
            p = model.solve(U, V, W)
        except ReachError:
            continue
        back = model.toolPoint(p["theta0"], p["rI"], p["zI"])
        if max(abs(a - b) for a, b in zip(back, (U, V, W))) > 1e-6:
            sys.exit(f"IK self-check failed at {U, V, W}: FK gives {back}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("target", nargs="?", help="tagtarget_*.json (default: newest in logs/)")
    parser.add_argument("--arm", default=DEFAULT_ARM_CONFIG, help="per-arm config JSON")
    parser.add_argument("--port", help="serial port, e.g. COM5 (default: the only USB port)")
    parser.add_argument("--hover", type=float, default=DEFAULT_HOVER_mm,
                        help=f"tool point height above the tag, mm (default {DEFAULT_HOVER_mm:g})")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_degps,
                        help=f"move speed, deg/s (default {DEFAULT_SPEED_degps:g})")
    parser.add_argument("--dry-run", action="store_true", help="print the plan, don't move")
    parser.add_argument("--fit", action="store_true",
                        help="refit the base servo and reach correction from logs/tag_reach_trials.csv")
    parser.add_argument("--since", metavar="TIME",
                        help="--fit: only use trials logged at or after this ISO time, "
                             "e.g. 2026-10-15T10:25")
    parser.add_argument("--webcam", type=int, metavar="TAG_ID",
                        help="loop: webcam photo -> locate this tag in MATLAB -> move -> measure")
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA,
                        help=f"webcam index for --webcam (default {DEFAULT_CAMERA})")
    parser.add_argument("--calibration", default=DEFAULT_WEBCAM_CALIBRATION,
                        help=f"camera calibration for --webcam (default {DEFAULT_WEBCAM_CALIBRATION})")
    parser.add_argument("--matlab", default=DEFAULT_MATLAB, help="matlab.exe to run the vision")
    args = parser.parse_args()

    cfg = loadArmConfig(args.arm)
    joints, _ = loadJointConfig()
    model = ReachModel(cfg, Kinematics(joints, loadCalibration(cfg["servoCalibration"])))
    selfCheck(model)
    if args.fit:
        runFit(model, args.since)
    elif args.webcam is not None:
        runWebcam(args, model)
    else:
        runMove(args, model)


if __name__ == "__main__":
    main()
