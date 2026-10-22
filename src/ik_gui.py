"""Two-link inverse kinematics GUI for the RASTICrane, in the arm's (r, z) plane.

Drag the end point I of the two-bar model and the arm follows. Geometry (see the side
view in the lab handout):
    A = shoulder (origin), D = elbow, I = end of the forearm
    a1 = |AD| = 80 mm, a2 = |DI| = 80 mm
    theta1 = angle of AD from +r              (joint 2, "180 #1")
    theta2 = angle of DI from +r (absolute)   (joint 3, "180 #2")
    theta2L = theta2 - theta1 = relative elbow angle of the textbook two-link arm

Inverse kinematics, elbow up (theta2L <= 0):
    cos(theta2L) = (r^2 + z^2 - a1^2 - a2^2) / (2 a1 a2)
    theta1 = atan2(z, r) - atan2(a2 sin(theta2L), a1 + a2 cos(theta2L))
    theta2 = theta1 + theta2L

Servo angles come from the per-joint fit s = a*theta + b, computed from the points
recorded with servo_angle_gui.py. A pose is only sent if both servo angles are inside the
firmware's limits (read from firmware/src/main.cpp). The base (joint 1) only sets the
direction of r, so it gets its own slider in servo degrees.

Run:  python src/ik_gui.py [calibration.csv]
"""

import csv
import datetime
import math
import queue
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports

from servo_angle_gui import (LOG_DIR, MOVE_RE, REACHED_RE, STATUS_RE, SerialLink,
                             fitLine, loadJointConfig)

A1_mm = 80.0
A2_mm = 80.0
DEFAULT_CALIBRATION = LOG_DIR / "servo_angle_points_corrected.csv"
# Points where the real mechanism hit something; kept between sessions.
LIMIT_MARKS_FILE = LOG_DIR / "ik_limit_marks.csv"
MARK_FIELDS = ["r_mm", "z_mm", "s1_deg", "s2_deg", "s3_deg", "time"]
MARK_PICK_RADIUS_px = 12

BASE = 0        # firmware joint indices
SHOULDER = 1
ELBOW = 2

DRAG_SEND_PERIOD_ms = 150
WORKSPACE_STEP_mm = 3.0
GRID_STEP_mm = 20.0
EDGE_SEARCH_STEPS = 20

COLORS = {
    "bg": "#ffffff", "grid": "#eeeeee", "axis": "#999999", "workspace": "#e3f0e6",
    "link1": "#e8a85c", "link2": "#5c8ee8", "joint": "#333333", "target": "#c0392b",
    "actual": "#b5b5b5", "bad": "#c0392b", "mark": "#7b1fa2",
}


def loadCalibration(path):
    """Fit s = a*theta + b for joints 2 and 3 from a servo_angle_gui CSV."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    fits = {}
    for jointNum in (SHOULDER + 1, ELBOW + 1):
        pts = [(float(r["ref_deg"]), float(r[f"s{jointNum}_deg"]))
               for r in rows if int(r["joint"]) == jointNum]
        if len({t for t, _ in pts}) < 2:
            raise ValueError(f"joint {jointNum} needs points at two or more angles")
        a, b, rms = fitLine(pts)
        fits[jointNum - 1] = {"a": a, "b": b, "rms": rms, "n": len(pts)}
    return fits


class Kinematics:
    def __init__(self, joints, fits):
        self.joints = joints
        self.fits = fits

    def thetaToServo(self, jointIdx, theta_deg):
        fit = self.fits[jointIdx]
        return fit["a"] * theta_deg + fit["b"]

    def servoToTheta(self, jointIdx, servo_deg):
        fit = self.fits[jointIdx]
        return (servo_deg - fit["b"]) / fit["a"]

    def inLimits(self, jointIdx, servo_deg):
        joint = self.joints[jointIdx]
        return joint["min"] <= servo_deg <= joint["max"]

    def ik(self, r_mm, z_mm):
        """Returns (solution dict, None) or (None, reason)."""
        c2 = (r_mm ** 2 + z_mm ** 2 - A1_mm ** 2 - A2_mm ** 2) / (2 * A1_mm * A2_mm)
        if c2 > 1.0:
            return None, "out of reach"
        if c2 < -1.0:
            return None, "too close to the shoulder"
        theta2L = -math.acos(c2)                        # elbow up
        theta1 = math.atan2(z_mm, r_mm) - math.atan2(A2_mm * math.sin(theta2L),
                                                     A1_mm + A2_mm * math.cos(theta2L))
        theta1_deg = math.degrees(theta1)
        theta2L_deg = math.degrees(theta2L)
        theta2_deg = theta1_deg + theta2L_deg
        s2 = self.thetaToServo(SHOULDER, theta1_deg)
        s3 = self.thetaToServo(ELBOW, theta2_deg)
        if not self.inLimits(SHOULDER, s2):
            return None, f"joint 2 limit (servo {s2:.1f})"
        if not self.inLimits(ELBOW, s3):
            return None, f"joint 3 limit (servo {s3:.1f})"
        return {"r": r_mm, "z": z_mm, "theta1": theta1_deg, "theta2": theta2_deg,
                "theta2L": theta2L_deg, "s2": s2, "s3": s3}, None

    def fkFromServos(self, s2, s3):
        """Elbow D and end point I in mm for the given servo angles."""
        t1 = math.radians(self.servoToTheta(SHOULDER, s2))
        t2 = math.radians(self.servoToTheta(ELBOW, s3))
        d = (A1_mm * math.cos(t1), A1_mm * math.sin(t1))
        return d, (d[0] + A2_mm * math.cos(t2), d[1] + A2_mm * math.sin(t2))


class IkGui:
    def __init__(self, root, calibrationPath):
        self.root = root
        self.joints, limitsSource = loadJointConfig()
        self.lines = queue.Queue()
        self.link = SerialLink(self.lines)
        self.kin = None
        self.reachable = []            # (r, z) grid cells inside the workspace
        self.view = None               # r_min, r_max, z_min, z_max (mm)
        self.origin = None             # canvas pixel of A; set by redrawAll
        self.scale = 1.0               # px per mm

        n = len(self.joints)
        self.position_deg = [None] * n
        self.reportedTarget_deg = [None] * n
        self.statusSeen = set()
        self.target = None             # current IK solution dict
        self.lastDragSend = 0.0
        self.dragging = False
        self.pendingMark = False       # waiting for a status reply to place a mark
        self.marks = self.loadMarks()

        root.title("RASTICrane (r, z) inverse kinematics")
        root.minsize(1000, 640)
        self._buildConnectionBar()
        body = ttk.Frame(root, padding=(10, 6))
        body.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(body, background=COLORS["bg"], highlightthickness=1,
                                highlightbackground="#cccccc", cursor="crosshair")
        self.canvas.pack(side="left", fill="both", expand=True)
        side = ttk.Frame(body, padding=(10, 0, 0, 0), width=320)
        side.pack(side="left", fill="y")
        self._buildSidePanel(side)
        self._buildLog()

        self.canvas.bind("<Configure>", lambda _e: self.redrawAll())
        self.canvas.bind("<Button-1>", self.onPress)
        self.canvas.bind("<B1-Motion>", self.onDrag)
        self.canvas.bind("<ButtonRelease-1>", self.onRelease)
        self.canvas.bind("<Button-3>", self.onRightClick)
        root.bind("<Escape>", lambda _e: self.stop())
        root.bind("<KeyPress-m>", self.onMarkKey)
        root.protocol("WM_DELETE_WINDOW", self.onClose)

        self.log(limitsSource)
        self.log("Click or drag the end point. Esc = STOP.")
        self.loadCalibrationFile(calibrationPath)
        self.refreshPorts()
        root.after(30, self.pollSerial)

    # ---------------------------------------------------------------- layout
    def _buildConnectionBar(self):
        bar = ttk.Frame(self.root, padding=(10, 8, 10, 0))
        bar.pack(fill="x")
        ttk.Label(bar, text="Port:").pack(side="left")
        self.portVar = tk.StringVar()
        self.portBox = ttk.Combobox(bar, textvariable=self.portVar, width=40, state="readonly")
        self.portBox.pack(side="left", padx=4)
        ttk.Button(bar, text="Refresh", command=self.refreshPorts).pack(side="left")
        self.connectButton = ttk.Button(bar, text="Connect", command=self.toggleConnection)
        self.connectButton.pack(side="left", padx=4)
        self.connStatus = ttk.Label(bar, text="Disconnected", foreground="#b00020")
        self.connStatus.pack(side="left", padx=8)

    def _buildSidePanel(self, side):
        style = ttk.Style()
        style.configure("Stop.TButton", foreground="#b00020", font=("Segoe UI", 12, "bold"),
                        padding=(18, 8))
        style.configure("Big.TButton", font=("Segoe UI", 11), padding=(14, 8))

        buttons = ttk.Frame(side)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="STOP  (Esc)", style="Stop.TButton", command=self.stop).pack(
            side="left")
        ttk.Button(buttons, text="Go Home", style="Big.TButton", command=self.home).pack(
            side="left", padx=8)

        row = ttk.Frame(side)
        row.pack(fill="x", pady=(8, 0))
        ttk.Button(row, text="Sync from arm", command=self.requestStatus).pack(side="left")
        ttk.Label(row, text="  Speed:").pack(side="left")
        self.speedVar = tk.StringVar(value="30")
        speed = ttk.Entry(row, textvariable=self.speedVar, width=5)
        speed.pack(side="left")
        speed.bind("<Return>", lambda _e: self.setSpeed())
        ttk.Button(row, text="Set", width=4, command=self.setSpeed).pack(side="left", padx=2)

        self.liveDrag = tk.BooleanVar(value=False)
        ttk.Checkbutton(side, text="Send while dragging (otherwise on release)",
                        variable=self.liveDrag).pack(anchor="w", pady=(8, 0))

        # Mechanism limit marks
        marks = ttk.LabelFrame(side, text="Mechanism limits", padding=6)
        marks.pack(fill="x", pady=(10, 0))
        ttk.Button(marks, text="✕  Mark limit here  (M)", command=self.markLimit).pack(
            fill="x")
        markRow = ttk.Frame(marks)
        markRow.pack(fill="x", pady=(4, 0))
        ttk.Button(markRow, text="Undo last", command=self.undoMark).pack(side="left")
        ttk.Button(markRow, text="Clear all", command=self.clearMarks).pack(side="left", padx=4)
        self.markCountVar = tk.StringVar()
        ttk.Label(markRow, textvariable=self.markCountVar).pack(side="left", padx=4)
        ttk.Label(marks, text="Right-click an ✕ to delete it.", foreground="#666").pack(
            anchor="w")
        self.updateMarkCount()

        # Typed target
        goto = ttk.LabelFrame(side, text="Go to (mm)", padding=6)
        goto.pack(fill="x", pady=(10, 0))
        self.rVar = tk.StringVar()
        self.zVar = tk.StringVar()
        ttk.Label(goto, text="r").pack(side="left")
        rEntry = ttk.Entry(goto, textvariable=self.rVar, width=7)
        rEntry.pack(side="left", padx=(2, 8))
        ttk.Label(goto, text="z").pack(side="left")
        zEntry = ttk.Entry(goto, textvariable=self.zVar, width=7)
        zEntry.pack(side="left", padx=2)
        ttk.Button(goto, text="Go", width=4, command=self.gotoTyped).pack(side="left", padx=6)
        rEntry.bind("<Return>", lambda _e: self.gotoTyped())
        zEntry.bind("<Return>", lambda _e: self.gotoTyped())

        # Base
        base = self.joints[BASE]
        baseFrame = ttk.LabelFrame(side, text=f"Joint 1 ({base['name']}), servo deg", padding=6)
        baseFrame.pack(fill="x", pady=(10, 0))
        self.baseVar = tk.DoubleVar(value=base["home"])
        scale = tk.Scale(baseFrame, from_=base["min"], to=base["max"], orient="horizontal",
                         resolution=0.5, variable=self.baseVar, length=280)
        scale.pack(fill="x")
        scale.bind("<ButtonRelease-1>", lambda _e: self.sendPose())
        scale.bind("<KeyRelease>", lambda _e: self.sendPose())

        # Readout
        readout = ttk.LabelFrame(side, text="Target", padding=6)
        readout.pack(fill="x", pady=(10, 0))
        self.readoutVar = tk.StringVar()
        ttk.Label(readout, textvariable=self.readoutVar, font=("Consolas", 10),
                  justify="left").pack(anchor="w")
        self.reasonVar = tk.StringVar()
        ttk.Label(readout, textvariable=self.reasonVar, foreground=COLORS["bad"]).pack(anchor="w")

        # Calibration
        cal = ttk.LabelFrame(side, text="Servo fit  s = a·θ + b", padding=6)
        cal.pack(fill="x", pady=(10, 0))
        self.calVar = tk.StringVar()
        ttk.Label(cal, textvariable=self.calVar, font=("Consolas", 9), justify="left").pack(
            anchor="w")
        ttk.Button(cal, text="Load calibration CSV…", command=self.chooseCalibration).pack(
            anchor="w", pady=(4, 0))

    def _buildLog(self):
        frame = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        frame.pack(fill="x")
        self.logText = tk.Text(frame, height=6, font=("Consolas", 9), state="disabled")
        self.logText.pack(side="left", fill="x", expand=True)
        scroll = ttk.Scrollbar(frame, command=self.logText.yview)
        scroll.pack(side="left", fill="y")
        self.logText.configure(yscrollcommand=scroll.set)

    # ---------------------------------------------------------------- calibration
    def chooseCalibration(self):
        path = filedialog.askopenfilename(initialdir=LOG_DIR, filetypes=[("CSV", "*.csv")])
        if path:
            self.loadCalibrationFile(Path(path))

    def loadCalibrationFile(self, path):
        try:
            fits = loadCalibration(path)
        except (OSError, ValueError, KeyError) as err:
            messagebox.showerror("Calibration", f"Could not use {path}:\n{err}")
            return
        self.kin = Kinematics(self.joints, fits)
        lines = [Path(path).name]
        for idx in (SHOULDER, ELBOW):
            fit = fits[idx]
            joint = self.joints[idx]
            lo = self.kin.servoToTheta(idx, joint["min"])
            hi = self.kin.servoToTheta(idx, joint["max"])
            lines.append(f"J{idx + 1}: a={fit['a']:+.4f} b={fit['b']:.2f} rms {fit['rms']:.2f}")
            lines.append(f"    θ{idx} in [{min(lo, hi):.1f}, {max(lo, hi):.1f}] deg")
        self.calVar.set("\n".join(lines))
        self.log(f"Calibration loaded from {path}")
        self.computeWorkspace()
        if self.target is None:
            self.setTargetFromServos(self.joints[SHOULDER]["home"], self.joints[ELBOW]["home"])
        else:
            self.trySetTarget(self.target["r"], self.target["z"])
        self.redrawAll()

    def computeWorkspace(self):
        reach = A1_mm + A2_mm
        step = WORKSPACE_STEP_mm
        cells = []
        n = int(reach / step) + 1
        for i in range(-n, n + 1):
            for k in range(-n, n + 1):
                r, z = i * step, k * step
                if self.kin.ik(r, z)[0] is not None:
                    cells.append((r, z))
        self.reachable = cells
        rs = [0.0] + [c[0] for c in cells]
        zs = [0.0] + [c[1] for c in cells]
        margin = 15.0
        self.view = (min(rs) - margin, max(rs) + margin, min(zs) - margin, max(zs) + margin)

    # ---------------------------------------------------------------- drawing
    def toCanvas(self, r, z):
        return self.origin[0] + r * self.scale, self.origin[1] - z * self.scale

    def toWorld(self, x, y):
        return (x - self.origin[0]) / self.scale, (self.origin[1] - y) / self.scale

    def redrawAll(self):
        if self.view is None:
            return
        w = max(self.canvas.winfo_width(), 50)
        h = max(self.canvas.winfo_height(), 50)
        rMin, rMax, zMin, zMax = self.view
        self.scale = min(w / (rMax - rMin), h / (zMax - zMin))
        self.origin = ((w - (rMax - rMin) * self.scale) / 2 - rMin * self.scale,
                       (h - (zMax - zMin) * self.scale) / 2 + zMax * self.scale)
        c = self.canvas
        c.delete("all")

        # Grid and axes
        g = GRID_STEP_mm
        for i in range(int(math.floor(rMin / g)), int(math.ceil(rMax / g)) + 1):
            x, _ = self.toCanvas(i * g, 0)
            c.create_line(x, 0, x, h, fill=COLORS["grid"])
            if i:
                c.create_text(x, self.origin[1] + 9, text=f"{i * g:g}", fill=COLORS["axis"],
                              font=("Segoe UI", 7))
        for k in range(int(math.floor(zMin / g)), int(math.ceil(zMax / g)) + 1):
            _, y = self.toCanvas(0, k * g)
            c.create_line(0, y, w, y, fill=COLORS["grid"])
            if k:
                c.create_text(self.origin[0] - 12, y, text=f"{k * g:g}", fill=COLORS["axis"],
                              font=("Segoe UI", 7))

        # Reachable workspace (elbow up, inside both servos' limits)
        half = WORKSPACE_STEP_mm * self.scale / 2 + 0.5
        for r, z in self.reachable:
            x, y = self.toCanvas(r, z)
            c.create_rectangle(x - half, y - half, x + half, y + half, fill=COLORS["workspace"],
                               outline="")

        c.create_line(0, self.origin[1], w, self.origin[1], fill=COLORS["axis"], arrow="last")
        c.create_line(self.origin[0], h, self.origin[0], 0, fill=COLORS["axis"], arrow="last")
        c.create_text(w - 10, self.origin[1] - 10, text="+r", fill=COLORS["axis"], anchor="e")
        c.create_text(self.origin[0] + 8, 10, text="+z", fill=COLORS["axis"], anchor="w")
        c.create_text(10, h - 10, anchor="sw", fill=COLORS["axis"], font=("Segoe UI", 8),
                      text="shaded = reachable (elbow up, within servo limits)   grid 20 mm")
        self.drawMarks()
        self.drawArm()

    def drawMarks(self):
        c = self.canvas
        c.delete("marks")
        if self.origin is None:
            return
        for mark in self.marks:
            x, y = self.toCanvas(mark["r_mm"], mark["z_mm"])
            c.create_line(x - 6, y - 6, x + 6, y + 6, fill=COLORS["mark"], width=2, tags="marks")
            c.create_line(x - 6, y + 6, x + 6, y - 6, fill=COLORS["mark"], width=2, tags="marks")
        if c.find_withtag("arm"):
            c.tag_lower("marks", "arm")

    def drawArm(self):
        if self.origin is None:
            return
        c = self.canvas
        c.delete("arm")
        a = self.toCanvas(0, 0)

        # Last pose reported by the firmware, in grey.
        s2, s3 = self.position_deg[SHOULDER], self.position_deg[ELBOW]
        if self.kin and s2 is not None and s3 is not None:
            d, i = self.kin.fkFromServos(s2, s3)
            pts = [a, self.toCanvas(*d), self.toCanvas(*i)]
            c.create_line(*pts[0], *pts[1], *pts[2], fill=COLORS["actual"], width=5,
                          capstyle="round", dash=(6, 4), tags="arm")

        if self.target:
            t1 = math.radians(self.target["theta1"])
            d = (A1_mm * math.cos(t1), A1_mm * math.sin(t1))
            dC = self.toCanvas(*d)
            iC = self.toCanvas(self.target["r"], self.target["z"])
            c.create_line(*a, *dC, fill=COLORS["link1"], width=7, capstyle="round", tags="arm")
            c.create_line(*dC, *iC, fill=COLORS["link2"], width=7, capstyle="round", tags="arm")
            for (x, y), label in ((a, "A"), (dC, "D")):
                c.create_oval(x - 5, y - 5, x + 5, y + 5, fill="white", outline=COLORS["joint"],
                              width=2, tags="arm")
                c.create_text(x - 10, y - 10, text=label, font=("Segoe UI", 9, "bold"),
                              fill=COLORS["joint"], tags="arm")
            x, y = iC
            c.create_oval(x - 9, y - 9, x + 9, y + 9, fill=COLORS["target"], outline="white",
                          width=2, tags="arm")
            c.create_text(x + 14, y - 12, text="I", font=("Segoe UI", 9, "bold"),
                          fill=COLORS["target"], tags="arm")

    def drawCursorMiss(self, r, z):
        self.canvas.delete("miss")
        if self.origin is None:
            return
        x, y = self.toCanvas(r, z)
        self.canvas.create_line(x - 6, y - 6, x + 6, y + 6, fill=COLORS["bad"], width=2, tags="miss")
        self.canvas.create_line(x - 6, y + 6, x + 6, y - 6, fill=COLORS["bad"], width=2, tags="miss")

    def updateReadout(self):
        t = self.target
        if not t:
            self.readoutVar.set("")
            return
        self.readoutVar.set(
            f"r = {t['r']:7.1f} mm   z = {t['z']:7.1f} mm\n"
            f"θ1        = {t['theta1']:7.2f} deg\n"
            f"θ2 (robot)= {t['theta2']:7.2f} deg  (from +r)\n"
            f"θ2 (link) = {t['theta2L']:7.2f} deg  (elbow)\n"
            f"servo 1 = {self.baseVar.get():6.1f}\n"
            f"servo 2 = {t['s2']:6.1f}   servo 3 = {t['s3']:6.1f}")

    # ---------------------------------------------------------------- targeting
    def trySetTarget(self, r, z):
        sol, reason = self.kin.ik(r, z)
        if sol is None:
            return reason
        self.target = sol
        self.rVar.set(f"{r:.1f}")
        self.zVar.set(f"{z:.1f}")
        self.updateReadout()
        self.drawArm()
        return None

    def moveTargetToward(self, r, z):
        """Go to (r, z), or as far toward it as the workspace allows."""
        reason = self.trySetTarget(r, z)
        if reason is None:
            self.reasonVar.set("")
            self.canvas.delete("miss")
            return
        self.reasonVar.set(reason)
        self.drawCursorMiss(r, z)
        if self.target is None:
            return
        # Bisect along the segment from the current target to the cursor, so the end
        # point slides along the edge of the workspace instead of freezing.
        lo = (self.target["r"], self.target["z"])
        hi = (r, z)
        for _ in range(EDGE_SEARCH_STEPS):
            mid = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)
            if self.kin.ik(*mid)[0] is not None:
                lo = mid
            else:
                hi = mid
        self.trySetTarget(*lo)

    def setTargetFromServos(self, s2, s3):
        _, (r, z) = self.kin.fkFromServos(s2, s3)
        if self.trySetTarget(r, z) is not None:
            # Outside the elbow-up workspace: show it, but don't adopt it as a target.
            self.reasonVar.set("arm pose is outside the elbow-up workspace")

    def onPress(self, event):
        if not self.kin:
            return
        self.dragging = True
        self.moveTargetToward(*self.toWorld(event.x, event.y))
        self.maybeSendWhileDragging()

    def onDrag(self, event):
        if not self.dragging:
            return
        self.moveTargetToward(*self.toWorld(event.x, event.y))
        self.maybeSendWhileDragging()

    def onRelease(self, _event):
        if not self.dragging:
            return
        self.dragging = False
        self.canvas.delete("miss")
        self.sendPose()

    def maybeSendWhileDragging(self):
        if not self.liveDrag.get():
            return
        now = time.monotonic()
        if (now - self.lastDragSend) * 1000 >= DRAG_SEND_PERIOD_ms:
            self.lastDragSend = now
            self.sendPose()

    def gotoTyped(self):
        try:
            r, z = float(self.rVar.get()), float(self.zVar.get())
        except ValueError:
            return
        reason = self.trySetTarget(r, z)
        if reason:
            self.reasonVar.set(f"({r:g}, {z:g}): {reason}")
            return
        self.reasonVar.set("")
        self.sendPose()

    # ---------------------------------------------------------------- commands
    def sendPose(self):
        if not self.target:
            return
        base = self.baseVar.get()
        s = (base, self.target["s2"], self.target["s3"])
        for idx, value in enumerate(s):
            if not self.kin.inLimits(idx, value):
                self.log(f"Not sent: joint {idx + 1} = {value:.1f} is outside its limits")
                return
        self.updateReadout()
        self.send(f"goto {s[0]:.1f} {s[1]:.1f} {s[2]:.1f}")

    def stop(self):
        self.send("stop")
        self.root.after(100, self.requestStatus)

    def home(self):
        if self.send("home"):
            self.baseVar.set(self.joints[BASE]["home"])
            self.setTargetFromServos(self.joints[SHOULDER]["home"], self.joints[ELBOW]["home"])

    def setSpeed(self):
        try:
            speed = float(self.speedVar.get())
        except ValueError:
            return
        if speed > 0:
            self.send(f"speed {speed:g}")

    # ---------------------------------------------------------------- serial
    def refreshPorts(self):
        ports = list(serial.tools.list_ports.comports())
        labels = [f"{p.device} - {p.description}" for p in ports]
        self.portBox["values"] = labels
        if labels and not self.portVar.get():
            preferred = [l for l in labels if "USB" in l or "JTAG" in l]
            self.portVar.set((preferred or labels)[0])

    def toggleConnection(self):
        if self.link.isOpen:
            self.link.close()
            self.setConnected(False)
            return
        label = self.portVar.get()
        if not label:
            messagebox.showwarning("No port", "Pick a serial port first.")
            return
        try:
            self.link.open(label.split(" - ")[0])
        except serial.SerialException as err:
            messagebox.showerror("Connect failed",
                                 f"{err}\n\nIs the PlatformIO monitor or the other GUI open?")
            return
        self.setConnected(True)
        self.root.after(400, self.requestStatus)

    def setConnected(self, connected):
        self.connectButton.configure(text="Disconnect" if connected else "Connect")
        self.connStatus.configure(text="Connected" if connected else "Disconnected",
                                  foreground="#1b7f3b" if connected else "#b00020")

    def send(self, line):
        if self.link.send(line):
            self.log(f">> {line}")
            return True
        self.log(f"(not connected) {line}")
        return False

    def requestStatus(self):
        self.statusSeen.clear()
        self.send("status")

    def pollSerial(self):
        try:
            while True:
                self.handleLine(self.lines.get_nowait())
        except queue.Empty:
            pass
        if not self.link.isOpen and self.connectButton.cget("text") == "Disconnect":
            self.setConnected(False)
        self.root.after(30, self.pollSerial)

    def handleLine(self, line):
        self.log(line)
        m = STATUS_RE.search(line)
        if m:
            i = int(m[1]) - 1
            if 0 <= i < len(self.joints):
                self.position_deg[i] = float(m[2])
                self.reportedTarget_deg[i] = float(m[3])
                self.statusSeen.add(i)
                if len(self.statusSeen) == len(self.joints):
                    self.statusSeen.clear()
                    if self.pendingMark:
                        self.placePendingMark()
                    self.syncFromArm()
            return
        m = REACHED_RE.search(line)
        if m:
            i = int(m[1]) - 1
            if 0 <= i < len(self.joints):
                self.position_deg[i] = float(m[2])
                self.drawArm()
            return
        m = MOVE_RE.search(line)
        if m:
            i = int(m[1]) - 1
            if 0 <= i < len(self.joints):
                self.reportedTarget_deg[i] = float(m[3])

    # ---------------------------------------------------------------- limit marks
    def loadMarks(self):
        try:
            with open(LIMIT_MARKS_FILE, newline="") as f:
                marks = []
                for row in csv.DictReader(f):
                    mark = {k: float(row[k]) for k in MARK_FIELDS if k != "time" and row.get(k)}
                    mark["time"] = row.get("time", "")
                    marks.append(mark)
                return marks
        except FileNotFoundError:
            return []
        except (OSError, ValueError, KeyError) as err:
            messagebox.showwarning("Limit marks", f"Could not read {LIMIT_MARKS_FILE}:\n{err}")
            return []

    def saveMarks(self):
        try:
            LOG_DIR.mkdir(exist_ok=True)
            with open(LIMIT_MARKS_FILE, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=MARK_FIELDS, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(self.marks)
        except OSError as err:
            self.log(f"!! could not save limit marks: {err}")

    def marksChanged(self):
        self.saveMarks()
        self.updateMarkCount()
        self.drawMarks()

    def updateMarkCount(self):
        self.markCountVar.set(f"{len(self.marks)} marked")

    def markLimit(self):
        """Mark where the arm is now. Asks the firmware first, so a mark right after STOP
        lands where the arm froze rather than where the GUI target was."""
        if self.link.isOpen:
            self.pendingMark = True
            self.requestStatus()
        elif self.target:
            self.log("Not connected: marking the GUI target instead of the arm position.")
            self.addMark(self.target["r"], self.target["z"],
                         (self.baseVar.get(), self.target["s2"], self.target["s3"]))

    def onMarkKey(self, event):
        if not isinstance(event.widget, (tk.Entry, ttk.Entry)):   # typing an r/z value
            self.markLimit()

    def placePendingMark(self):
        self.pendingMark = False
        s = self.position_deg
        if not self.kin or s[SHOULDER] is None or s[ELBOW] is None:
            return
        _, (r, z) = self.kin.fkFromServos(s[SHOULDER], s[ELBOW])
        self.addMark(r, z, s)

    def addMark(self, r, z, servos):
        mark = {"r_mm": round(r, 2), "z_mm": round(z, 2),
                "time": datetime.datetime.now().isoformat(timespec="seconds")}
        for i, value in enumerate(servos):
            mark[f"s{i + 1}_deg"] = round(value, 2)
        self.marks.append(mark)
        self.marksChanged()
        self.log(f"Limit marked at r = {r:.1f}, z = {z:.1f} mm")

    def undoMark(self):
        if self.marks:
            self.marks.pop()
            self.marksChanged()

    def clearMarks(self):
        if self.marks and messagebox.askyesno("Clear marks", "Delete all limit marks?"):
            self.marks.clear()
            self.marksChanged()

    def onRightClick(self, event):
        if self.origin is None or not self.marks:
            return
        def pixelDistance(mark):
            x, y = self.toCanvas(mark["r_mm"], mark["z_mm"])
            return math.hypot(x - event.x, y - event.y)
        nearest = min(self.marks, key=pixelDistance)
        if pixelDistance(nearest) <= MARK_PICK_RADIUS_px:
            self.marks.remove(nearest)
            self.marksChanged()
            self.log(f"Deleted mark at r = {nearest['r_mm']:.1f}, z = {nearest['z_mm']:.1f} mm")

    def syncFromArm(self):
        """Point the GUI at where the arm is headed (after connect, stop or sync)."""
        targets = self.reportedTarget_deg
        if targets[BASE] is not None:
            self.baseVar.set(targets[BASE])
        if self.kin and targets[SHOULDER] is not None and targets[ELBOW] is not None:
            self.setTargetFromServos(targets[SHOULDER], targets[ELBOW])
        self.updateReadout()
        self.drawArm()

    # ---------------------------------------------------------------- misc
    def log(self, text):
        self.logText.configure(state="normal")
        self.logText.insert("end", text + "\n")
        self.logText.see("end")
        self.logText.configure(state="disabled")

    def onClose(self):
        self.link.close()
        self.root.destroy()


def main():
    calibration = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CALIBRATION
    root = tk.Tk()
    IkGui(root, calibration)
    root.mainloop()


if __name__ == "__main__":
    main()
