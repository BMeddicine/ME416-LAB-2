"""Servo angle calibration GUI for the RASTICrane.

Drag each joint's slider until a link lines up with a visually identifiable direction
(+r = 0 deg, straight up = +90 deg, ...), then record a data point. Points are fit per
joint as s = a*theta + b, so b is the servo angle at which that link points along +r.

Talks to the ESP32 firmware (firmware/src/main.cpp) over USB serial using its existing
commands: "<joint> <angle>", "home", "stop", "speed", "status".
Joint limits and home angles are read from the kJoints table in main.cpp, so the
sliders can never ask for an angle the firmware considers unsafe.

Run:  python src/servo_angle_gui.py
Needs: pyserial  (pip install pyserial)
"""

import csv
import datetime
import math
import queue
import re
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports

REPO_ROOT = Path(__file__).resolve().parent.parent
FIRMWARE_MAIN = REPO_ROOT / "firmware" / "src" / "main.cpp"
LOG_DIR = REPO_ROOT / "logs"

BAUD = 115200
DRAG_SEND_PERIOD_ms = 150     # throttle for "send while dragging"
SLIDER_RESOLUTION_deg = 0.5
NUDGE_STEPS_deg = (-5, -1, -0.5, 0.5, 1, 5)
REFERENCE_ANGLES_deg = ("0", "90", "-90", "180", "45", "-45")

# Used only if main.cpp can't be read.
FALLBACK_JOINTS = [
    {"name": "base 300", "min": 0.0, "max": 300.0, "home": 45.0},
    {"name": "180 #1", "min": 0.0, "max": 180.0, "home": 45.0},
    {"name": "180 #2", "min": 0.0, "max": 180.0, "home": 150.0},
]

STATUS_RE = re.compile(r"Joint (\d+) \(.*?\): (-?[\d.]+) -> (-?[\d.]+) deg")
REACHED_RE = re.compile(r"Joint (\d+) reached (-?[\d.]+) deg")
MOVE_RE = re.compile(r"Joint (\d+): (-?[\d.]+) -> (-?[\d.]+) deg")


def loadJointConfig():
    """Parse name / min / max / home for each joint from the firmware's kJoints table."""
    row = re.compile(
        r'\{\s*"([^"]+)"\s*,\s*\d+\s*,\s*\w+\s*,\s*([\d.]+)f?\s*,\s*\d+\s*,\s*\d+\s*,'
        r'\s*(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?\s*\}'
    )
    try:
        text = FIRMWARE_MAIN.read_text()
        table = text[text.index("kJoints[]"):]
        table = table[:table.index("};")]
        joints = [
            {"name": m[1], "min": float(m[3]), "max": float(m[4]), "home": float(m[5])}
            for m in row.finditer(table)
        ]
        if joints:
            return joints, f"Joint limits read from {FIRMWARE_MAIN.relative_to(REPO_ROOT)}"
    except (OSError, ValueError):
        pass
    return FALLBACK_JOINTS, "WARNING: could not read main.cpp, using built-in limits"


def fitLine(points):
    """Least squares s = a*theta + b. points: list of (theta, s). Returns a, b, rms."""
    n = len(points)
    mt = sum(t for t, _ in points) / n
    ms = sum(s for _, s in points) / n
    stt = sum((t - mt) ** 2 for t, _ in points)
    a = sum((t - mt) * (s - ms) for t, s in points) / stt
    b = ms - a * mt
    rms = math.sqrt(sum((s - (a * t + b)) ** 2 for t, s in points) / n)
    return a, b, rms


class SerialLink:
    """Owns the port and a reader thread that pushes received lines onto a queue."""

    def __init__(self, lines):
        self.lines = lines
        self.port = None
        self.reader = None

    @property
    def isOpen(self):
        return self.port is not None and self.port.is_open

    def open(self, device):
        port = serial.Serial()
        port.port = device
        port.baudrate = BAUD
        port.timeout = 0.1
        # Keep DTR/RTS low so opening the port doesn't reset the ESP32-S3.
        port.dtr = False
        port.rts = False
        port.open()
        self.port = port
        self.reader = threading.Thread(target=self._readLoop, daemon=True)
        self.reader.start()

    def close(self):
        port, self.port = self.port, None
        if port:
            try:
                port.close()
            except serial.SerialException:
                pass

    def send(self, line):
        if not self.isOpen:
            return False
        try:
            self.port.write((line + "\n").encode())
            return True
        except serial.SerialException as err:
            self.lines.put(f"!! write failed: {err}")
            self.close()
            return False

    def _readLoop(self):
        buf = b""
        while self.isOpen:
            try:
                chunk = self.port.read(256)
            except (serial.SerialException, TypeError, AttributeError) as err:
                self.lines.put(f"!! port lost: {err}")
                self.close()
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").replace("\r", "").strip()
                # The firmware echoes typed characters after its "> " prompt.
                text = text.lstrip("> ").strip()
                if text:
                    self.lines.put(text)


class ServoAngleGui:
    def __init__(self, root):
        self.root = root
        self.joints, self.limitsSource = loadJointConfig()
        self.lines = queue.Queue()
        self.link = SerialLink(self.lines)

        n = len(self.joints)
        self.targetVars = [tk.DoubleVar(value=j["home"]) for j in self.joints]
        self.positionVars = [tk.StringVar(value="?") for _ in self.joints]
        self.position_deg = [None] * n      # last position reported by the firmware
        self.reportedTarget_deg = [None] * n
        self.lastDragSend = [0.0] * n

        self.points = []                    # dicts: joint, ref_deg, s1..sn, time
        self.pendingRecord = None           # (joint index, reference angle) awaiting status
        self.statusSeen = set()
        self.csvPath = LOG_DIR / f"servo_angle_points_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"

        root.title("RASTICrane servo angle calibration")
        root.minsize(900, 640)
        self._buildConnectionBar()
        self._buildJointPanel()
        self._buildControlBar()
        self._buildRecordPanel()
        self._buildLog()

        root.bind("<Escape>", lambda _e: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.onClose)
        self.log(self.limitsSource)
        self.log("Esc = STOP at any time.")
        self.refreshPorts()
        self.root.after(30, self.pollSerial)

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

    def _buildJointPanel(self):
        panel = ttk.LabelFrame(self.root, text="Joints (servo angle, deg)", padding=10)
        panel.pack(fill="x", padx=10, pady=8)
        panel.columnconfigure(1, weight=1)
        self.liveDrag = tk.BooleanVar(value=False)

        for i, joint in enumerate(self.joints):
            ttk.Label(panel, text=f"{i + 1}: {joint['name']}", width=12).grid(
                row=i, column=0, sticky="w")

            scale = tk.Scale(panel, from_=joint["min"], to=joint["max"], orient="horizontal",
                             resolution=SLIDER_RESOLUTION_deg, variable=self.targetVars[i],
                             showvalue=False, length=360,
                             command=lambda _v, i=i: self.onSliderMove(i))
            scale.grid(row=i, column=1, sticky="ew", padx=6)
            scale.bind("<ButtonRelease-1>", lambda _e, i=i: self.sendJoint(i))
            scale.bind("<KeyRelease>", lambda _e, i=i: self.sendJoint(i))

            spin = ttk.Spinbox(panel, from_=joint["min"], to=joint["max"],
                               increment=SLIDER_RESOLUTION_deg, width=7,
                               textvariable=self.targetVars[i],
                               command=lambda i=i: self.sendJoint(i))
            spin.grid(row=i, column=2, padx=4)
            spin.bind("<Return>", lambda _e, i=i: self.sendJoint(i))
            spin.bind("<FocusOut>", lambda _e, i=i: self.clampVar(i))

            nudges = ttk.Frame(panel)
            nudges.grid(row=i, column=3, padx=4)
            for step in NUDGE_STEPS_deg:
                ttk.Button(nudges, text=f"{step:+g}", width=4,
                           command=lambda i=i, s=step: self.nudge(i, s)).pack(side="left")

            ttk.Label(panel, text=f"[{joint['min']:g}, {joint['max']:g}]  home {joint['home']:g}",
                      foreground="#666").grid(row=i, column=4, padx=6)
            ttk.Label(panel, text="at:").grid(row=i, column=5)
            ttk.Label(panel, textvariable=self.positionVars[i], width=8,
                      font=("Consolas", 10, "bold")).grid(row=i, column=6, sticky="w")

        ttk.Checkbutton(panel, text="Send while dragging (otherwise sends on release)",
                        variable=self.liveDrag).grid(row=len(self.joints), column=1,
                                                     sticky="w", pady=(6, 0))

    def _buildControlBar(self):
        bar = ttk.Frame(self.root, padding=(10, 0))
        bar.pack(fill="x")
        style = ttk.Style()
        style.configure("Stop.TButton", foreground="#b00020", font=("Segoe UI", 12, "bold"),
                        padding=(18, 8))
        style.configure("Big.TButton", font=("Segoe UI", 11), padding=(14, 8))
        ttk.Button(bar, text="STOP  (Esc)", style="Stop.TButton", command=self.stop).pack(
            side="left")
        ttk.Button(bar, text="Go Home", style="Big.TButton", command=self.home).pack(
            side="left", padx=8)
        ttk.Button(bar, text="Sync from arm", command=self.requestStatus).pack(side="left")

        ttk.Label(bar, text="Speed (deg/s):").pack(side="left", padx=(20, 2))
        self.speedVar = tk.StringVar(value="30")
        speed = ttk.Entry(bar, textvariable=self.speedVar, width=6)
        speed.pack(side="left")
        speed.bind("<Return>", lambda _e: self.setSpeed())
        ttk.Button(bar, text="Set", command=self.setSpeed).pack(side="left", padx=2)

    def _buildRecordPanel(self):
        panel = ttk.LabelFrame(self.root, text="Data points", padding=10)
        panel.pack(fill="both", expand=True, padx=10, pady=8)

        top = ttk.Frame(panel)
        top.pack(fill="x")
        ttk.Label(top, text="Joint:").pack(side="left")
        self.recordJoint = tk.IntVar(value=2 if len(self.joints) > 1 else 1)
        for i, joint in enumerate(self.joints):
            ttk.Radiobutton(top, text=f"{i + 1} ({joint['name']})", value=i + 1,
                            variable=self.recordJoint).pack(side="left", padx=3)
        ttk.Label(top, text="   Link is at θ (deg from +r):").pack(side="left")
        self.refVar = tk.StringVar(value="0")
        ttk.Combobox(top, textvariable=self.refVar, values=REFERENCE_ANGLES_deg, width=6).pack(
            side="left", padx=4)
        ttk.Button(top, text="Record point", command=self.recordPoint).pack(side="left", padx=6)

        body = ttk.Frame(panel)
        body.pack(fill="both", expand=True, pady=(8, 0))

        columns = ("joint", "ref") + tuple(f"s{i + 1}" for i in range(len(self.joints))) + ("time",)
        self.table = ttk.Treeview(body, columns=columns, show="headings", height=8)
        headings = {"joint": "Joint", "ref": "θ ref", "time": "Time"}
        for col in columns:
            self.table.heading(col, text=headings.get(col, f"servo {col[1:]}"))
            self.table.column(col, width=70 if col != "time" else 80, anchor="center")
        self.table.pack(side="left", fill="both", expand=True)

        side = ttk.Frame(body, padding=(10, 0, 0, 0))
        side.pack(side="left", fill="both", expand=True)
        buttons = ttk.Frame(side)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="Delete selected", command=self.deleteSelected).pack(side="left")
        ttk.Button(buttons, text="Clear all", command=self.clearPoints).pack(side="left", padx=4)
        ttk.Button(buttons, text="Export CSV…", command=self.exportCsv).pack(side="left")
        self.fitText = tk.Text(side, height=11, width=58, font=("Consolas", 9), state="disabled",
                               relief="flat", background=self.root.cget("background"))
        self.fitText.pack(fill="both", expand=True, pady=(6, 0))
        self.updateFits()

    def _buildLog(self):
        frame = ttk.LabelFrame(self.root, text="Serial", padding=(10, 4))
        frame.pack(fill="both", padx=10, pady=(0, 10))
        self.logText = tk.Text(frame, height=7, font=("Consolas", 9), state="disabled")
        self.logText.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(frame, command=self.logText.yview)
        scroll.pack(side="left", fill="y")
        self.logText.configure(yscrollcommand=scroll.set)
        entry = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        entry.pack(fill="x")
        ttk.Label(entry, text="Raw command:").pack(side="left")
        self.rawVar = tk.StringVar()
        raw = ttk.Entry(entry, textvariable=self.rawVar)
        raw.pack(side="left", fill="x", expand=True, padx=4)
        raw.bind("<Return>", lambda _e: self.sendRaw())

    # ---------------------------------------------------------------- serial
    def refreshPorts(self):
        ports = list(serial.tools.list_ports.comports())
        labels = [f"{p.device} - {p.description}" for p in ports]
        self.portBox["values"] = labels
        if labels and not self.portVar.get():
            # Prefer something that looks like the ESP32's native USB port.
            preferred = [l for l in labels if re.search(r"USB|JTAG|Espressif|303A", l, re.I)]
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
            messagebox.showerror("Connect failed", f"{err}\n\nIs the PlatformIO monitor still open?")
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
                self.setPosition(i, float(m[2]))
                self.reportedTarget_deg[i] = float(m[3])
                self.statusSeen.add(i)
                if len(self.statusSeen) == len(self.joints):
                    self.onStatusComplete()
            return
        m = REACHED_RE.search(line)
        if m:
            i = int(m[1]) - 1
            if 0 <= i < len(self.joints):
                self.setPosition(i, float(m[2]))
                self.reportedTarget_deg[i] = float(m[2])
            return
        m = MOVE_RE.search(line)
        if m:
            i = int(m[1]) - 1
            if 0 <= i < len(self.joints):
                self.reportedTarget_deg[i] = float(m[3])
                self.positionVars[i].set("moving")

    def setPosition(self, i, angle_deg):
        self.position_deg[i] = angle_deg
        self.positionVars[i].set(f"{angle_deg:.1f}")

    def requestStatus(self):
        self.statusSeen.clear()
        self.send("status")

    def onStatusComplete(self):
        self.statusSeen.clear()
        if self.pendingRecord is not None:
            joint, ref = self.pendingRecord
            self.pendingRecord = None
            self.finishRecord(joint, ref)
        else:
            # Plain sync: put the sliders where the arm is headed.
            for i, target in enumerate(self.reportedTarget_deg):
                if target is not None:
                    self.targetVars[i].set(target)

    # ---------------------------------------------------------------- motion
    def clampVar(self, i):
        joint = self.joints[i]
        try:
            value = float(self.targetVars[i].get())
        except (tk.TclError, ValueError):
            value = self.position_deg[i] if self.position_deg[i] is not None else joint["home"]
        value = min(max(value, joint["min"]), joint["max"])
        value = round(value / SLIDER_RESOLUTION_deg) * SLIDER_RESOLUTION_deg
        self.targetVars[i].set(value)
        return value

    def sendJoint(self, i):
        value = self.clampVar(i)
        self.send(f"{i + 1} {value:g}")

    def onSliderMove(self, i):
        if not self.liveDrag.get():
            return
        now = time.monotonic()
        if (now - self.lastDragSend[i]) * 1000 >= DRAG_SEND_PERIOD_ms:
            self.lastDragSend[i] = now
            self.sendJoint(i)

    def nudge(self, i, step):
        base = self.clampVar(i)
        self.targetVars[i].set(base + step)
        self.sendJoint(i)

    def stop(self):
        self.pendingRecord = None
        self.send("stop")
        self.root.after(100, self.requestStatus)   # sliders follow where it froze

    def home(self):
        if self.send("home"):
            for i, joint in enumerate(self.joints):
                self.targetVars[i].set(joint["home"])

    def setSpeed(self):
        try:
            speed = float(self.speedVar.get())
        except ValueError:
            return
        if speed > 0:
            self.send(f"speed {speed:g}")

    def sendRaw(self):
        line = self.rawVar.get().strip()
        if line:
            self.send(line)
            self.rawVar.set("")

    # ---------------------------------------------------------------- data points
    def recordPoint(self):
        try:
            ref = float(self.refVar.get())
        except ValueError:
            messagebox.showwarning("Reference angle", "Enter the link angle θ in degrees.")
            return
        if not self.link.isOpen:
            messagebox.showwarning("Not connected", "Connect to the arm first.")
            return
        # Ask the arm where it actually is; the point is stored when the reply arrives.
        self.pendingRecord = (self.recordJoint.get(), ref)
        self.requestStatus()

    def finishRecord(self, joint, ref):
        moving = any(
            p is None or t is None or abs(p - t) > 1e-3
            for p, t in zip(self.position_deg, self.reportedTarget_deg)
        )
        if moving:
            messagebox.showwarning("Still moving", "Wait for the arm to stop, then record again.")
            return
        point = {"joint": joint, "ref_deg": ref, "time": datetime.datetime.now()}
        for i, p in enumerate(self.position_deg):
            point[f"s{i + 1}_deg"] = p
        self.points.append(point)
        self.addRow(point)
        self.updateFits()
        self.autosave()
        self.log(f"Recorded joint {joint}: servo {self.position_deg[joint - 1]:.1f} at θ = {ref:g}")

    def addRow(self, point):
        values = [point["joint"], f"{point['ref_deg']:g}"]
        values += [f"{point[f's{i + 1}_deg']:.1f}" for i in range(len(self.joints))]
        values.append(point["time"].strftime("%H:%M:%S"))
        self.table.insert("", "end", iid=str(id(point)), values=values)

    def deleteSelected(self):
        selected = set(self.table.selection())
        if not selected:
            return
        self.points = [p for p in self.points if str(id(p)) not in selected]
        for iid in selected:
            self.table.delete(iid)
        self.updateFits()
        self.autosave()

    def clearPoints(self):
        if self.points and messagebox.askyesno("Clear", "Delete all recorded points?"):
            self.points.clear()
            self.table.delete(*self.table.get_children())
            self.updateFits()
            self.autosave()

    def updateFits(self):
        lines = ["Fit per joint: s = a*θ + b   (b = servo angle when link is along +r)", ""]
        for j in range(1, len(self.joints) + 1):
            pts = [(p["ref_deg"], p[f"s{j}_deg"]) for p in self.points if p["joint"] == j]
            if not pts:
                continue
            name = self.joints[j - 1]["name"]
            refs = sorted({t for t, _ in pts})
            lines.append(f"Joint {j} ({name}): {len(pts)} pts at θ = {', '.join(f'{r:g}' for r in refs)}")
            for r in refs:
                s = [sv for t, sv in pts if t == r]
                mean = sum(s) / len(s)
                sd = math.sqrt(sum((v - mean) ** 2 for v in s) / (len(s) - 1)) if len(s) > 1 else 0.0
                lines.append(f"   θ={r:>6g}: mean s = {mean:7.2f}  sd {sd:4.2f}  (n={len(s)})")
            if len(refs) >= 2:
                a, b, rms = fitLine(pts)
                lines.append(f"   a = {a:+.4f}   b (+r) = {b:7.2f} deg   rms {rms:.2f}")
            else:
                mean = sum(s for _, s in pts) / len(pts)
                lines.append(f"   one θ only: b = {mean - refs[0]:.2f} (if a=+1) "
                             f"or {mean + refs[0]:.2f} (if a=-1)")
            lines.append("")
        if len(lines) == 2:
            lines.append("No points yet.")
        self.fitText.configure(state="normal")
        self.fitText.delete("1.0", "end")
        self.fitText.insert("1.0", "\n".join(lines))
        self.fitText.configure(state="disabled")

    def writeCsv(self, path):
        fields = ["joint", "ref_deg"] + [f"s{i + 1}_deg" for i in range(len(self.joints))] + ["time"]
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for p in self.points:
                writer.writerow({**p, "time": p["time"].isoformat(timespec="seconds")})

    def autosave(self):
        try:
            LOG_DIR.mkdir(exist_ok=True)
            self.writeCsv(self.csvPath)
        except OSError as err:
            self.log(f"!! autosave failed: {err}")

    def exportCsv(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialdir=LOG_DIR,
                                            initialfile=self.csvPath.name,
                                            filetypes=[("CSV", "*.csv")])
        if path:
            self.writeCsv(path)
            self.log(f"Exported {len(self.points)} points to {path}")

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
    root = tk.Tk()
    ServoAngleGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
