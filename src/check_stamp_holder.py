"""Visual check of the stamp holder offsets: webcam photo -> MATLAB -> annotated image.

    python src/check_stamp_holder.py                 # webcam loop (camera 1)
    python src/check_stamp_holder.py --camera 0
    python src/check_stamp_holder.py --photo photos/webcam_20261015_101500.png

1. A live view shows whether the base tag 0 and the holder tags 1 and 3 are seen.
   Press SPACE to take the photo, Q / Esc to quit.
2. The frame is saved to photos/webcam_<time>.png and MATLAB runs
   locateStampHolder on it with the webcam calibration (about 20 s).
3. The annotated image opens. For every one of the 26 positions:
     outline = well opening at rim height  -> should sit on the real well
     dot     = stamp top (seat + 30 mm)    -> should sit on the top of a loaded stamp
   cyan = front/lower row (L1..L13), green = back/upper row (U1..U13).
   Any key closes it and goes back to the live view.

The geometry is in config/stamp_holder.json; the math is vision/solveStampHolder.m.
Nothing here moves the arm.
"""

import argparse
import datetime
import json
import re
import subprocess
import sys
from pathlib import Path

import cv2

from move_to_tag import (DEFAULT_CAMERA, DEFAULT_MATLAB, DEFAULT_WEBCAM_CALIBRATION, PHOTO_DIR,
                         REPO_ROOT, WEBCAM_SIZE_px, openCamera)

NEEDED_TAGS = ((0, "base tag 0"), (1, "holder tag 1 (left)"), (3, "holder tag 3 (right)"))
RESULT_SCALE = 2          # the 640x480 result is small; show it larger


def previewAndCapture(cap):
    """Live view with the needed tags marked seen / NOT SEEN. SPACE returns the frame, Q returns None."""
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    window = "check_stamp_holder: SPACE = take photo, Q = quit"
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
            for row, (tagId, name) in enumerate(NEEDED_TAGS):
                found = tagId in seen
                cv2.putText(view, f"{name}: {'seen' if found else 'NOT SEEN'}", (10, 25 + 25 * row),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0) if found else (0, 0, 255), 2)
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                return frame
            if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                return None
    finally:
        cv2.destroyWindow(window)
        cv2.waitKey(1)


def writeTagHints(photoPath):
    """OpenCV's tag detections for the photo -> <photo>.tags.json, for MATLAB's TagHints.
    readAprilTag misses tag 3 on the real rack (its border merges into the black pad);
    detectTags reads it again inside this outline. Returns the path."""
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    corners, ids, _ = detector.detectMarkers(cv2.imread(str(photoPath)))
    hints = [] if ids is None else [{"id": int(i), "cornerPx": c[0].tolist()}
                                    for i, c in zip(ids.ravel(), corners)]
    hintPath = photoPath.with_suffix(".tags.json")
    hintPath.write_text(json.dumps(hints))
    return hintPath


def matlabCommand(photoPath, calibration):
    """locateStampHolder call for the photo, with OpenCV's detections as tag hints."""
    hints = writeTagHints(photoPath).resolve().as_posix()
    return (f"addpath(genpath('vision')); locateStampHolder('{photoPath.resolve().as_posix()}', "
            f"Calibration='{calibration}', TagHints='{hints}');")


def locateWithMatlab(args, photoPath):
    """Run locateStampHolder on the photo; returns the annotated image path or None."""
    command = matlabCommand(photoPath, args.calibration)
    print("Locating the stamp holder in MATLAB (about 20 s)...")
    run = subprocess.run([args.matlab, "-batch", command], cwd=REPO_ROOT, capture_output=True,
                         text=True)
    output = (run.stdout + run.stderr).strip()
    print(output)
    saved = re.search(r"Saved .+?\.json\s+(.+?\.jpg)", output)
    if run.returncode != 0 or not saved:
        print("MATLAB did not produce a result. Check the messages above.")
        return None
    return Path(saved[1].strip())


def showResult(imagePath):
    img = cv2.imread(str(imagePath))
    if img is None:
        print(f"Could not open {imagePath}")
        return
    img = cv2.resize(img, None, fx=RESULT_SCALE, fy=RESULT_SCALE, interpolation=cv2.INTER_NEAREST)
    window = f"stamp holder check: {imagePath.name} (any key to close)"
    cv2.imshow(window, img)
    cv2.waitKey(0)
    cv2.destroyWindow(window)
    cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--photo", help="use this photo instead of the webcam (one run, no loop)")
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA,
                        help=f"webcam index (default {DEFAULT_CAMERA}; 0 is the laptop's own camera)")
    parser.add_argument("--calibration", default=DEFAULT_WEBCAM_CALIBRATION,
                        help=f"camera calibration (default {DEFAULT_WEBCAM_CALIBRATION})")
    parser.add_argument("--matlab", default=DEFAULT_MATLAB, help="matlab.exe to run the vision")
    args = parser.parse_args()

    if args.photo:
        imagePath = locateWithMatlab(args, Path(args.photo))
        if imagePath:
            showResult(imagePath)
        return

    cap = openCamera(args.camera)
    try:
        while True:
            print("\nFrame the shot (tags 0, 1 and 3 all seen), then press SPACE in the camera window.")
            frame = previewAndCapture(cap)
            if frame is None:
                break
            PHOTO_DIR.mkdir(exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            photoPath = PHOTO_DIR / f"webcam_{stamp}.png"
            cv2.imwrite(str(photoPath), frame)
            imagePath = locateWithMatlab(args, photoPath)
            if imagePath:
                showResult(imagePath)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
