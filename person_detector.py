
#!/usr/bin/env python3
"""
Smart Home AI Surveillance Pipeline
------------------------------------
Supports:
    Windows/Linux webcam: --camera pc
    Raspberry Pi camera:  --camera picam

Features:
    - YOLO11 person detection
    - ByteTrack person tracking
    - OpenCV MOG2 motion detection
    - Periodic AI inference even without motion
    - Detection debounce (3 of 5 inference frames)
    - Person disappearance timeout
    - JSON event generation
    - Blynk V20 updates to multiple devices
    - Blynk keep-alive
    - Authenticated Flask MJPEG live stream

Install PC dependencies:
    pip install opencv-python ultralytics flask requests numpy

Install Raspberry Pi camera:
    sudo apt install python3-picamera2

Run:
    python smart_home_ai.py --camera pc

    python smart_home_ai.py --camera picam
"""

import argparse
import hmac
import json
import os
import queue
import signal
import threading
import time

from collections import deque
from datetime import datetime, timezone

import cv2
import requests

from flask import Flask, Response, request
from ultralytics import YOLO


# ============================================================
# CONFIGURATION
# ============================================================

MODEL_NAME = "yolo11n.pt"

# Camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30

# YOLO
IMGSZ = 320
CONFIDENCE = 0.5
TRACKER = "bytetrack.yaml"
MIN_BOX_SIZE = 40

# Motion detection
MOTION_THRESHOLD = 500
IDLE_INFERENCE_INTERVAL = 1.0

# Debounce
HIT_WINDOW = 5
HITS_NEEDED = 3
CLEAR_AFTER_SECONDS = 5.0

# Blynk
BLYNK_SERVER = "blynk.cloud"
BLYNK_PIN = "V20"
KEEPALIVE_SECONDS = 30.0

# Stream
STREAM_PORT = 8000
JPEG_QUALITY = 75
STREAM_USERNAME = "camera"
STREAM_PASSWORD = "change-me"

# Detection event channel
CHANNEL_ID = 1


# ============================================================
# SHARED STATE
# ============================================================

stop_event = threading.Event()


class FrameStore:
    """Stores the latest encoded frame for MJPEG clients."""

    def __init__(self):
        self.jpeg = None
        self.condition = threading.Condition()

    def update(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()

    def get(self, previous=None):
        with self.condition:
            self.condition.wait_for(
                lambda: self.jpeg is not None
                and self.jpeg is not previous,
                timeout=5
            )
            return self.jpeg


frame_store = FrameStore()


# ============================================================
# CAMERA ABSTRACTION
# ============================================================

class PCCamera:
    """Laptop webcam using OpenCV."""

    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index)

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open PC camera index {index}"
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        print("[camera] PC webcam initialized")

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


class PiCamera:
    """Raspberry Pi camera using Picamera2."""

    def __init__(self):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 is unavailable. Install it on Raspberry Pi OS."
            ) from exc

        self.camera = Picamera2()

        config = self.camera.create_video_configuration(
            main={
                "size": (FRAME_WIDTH, FRAME_HEIGHT),
                "format": "RGB888"
            }
        )

        self.camera.configure(config)
        self.camera.start()

        # Enable continuous autofocus when supported.
        try:
            from libcamera import controls

            self.camera.set_controls({
                "AfMode": controls.AfModeEnum.Continuous
            })
        except Exception:
            pass

        print("[camera] Raspberry Pi camera initialized")

    def read(self):
        try:
            frame = self.camera.capture_array()
            return True, frame
        except Exception as exc:
            print(f"[camera] Capture error: {exc}")
            return False, None

    def release(self):
        self.camera.stop()


def create_camera(args):
    if args.camera == "pc":
        return PCCamera(args.camera_index)

    if args.camera == "picam":
        return PiCamera()

    raise ValueError("Unknown camera type")


# ============================================================
# JSON EVENT GENERATION
# ============================================================

def timestamp_utc():
    return datetime.now(timezone.utc).isoformat().replace(
        "+00:00", "Z"
    )


def emit_event(
    event_type,
    track_id=None,
    confidence=None,
    object_class="person"
):
    """Print a structured event for the software backend."""

    event = {
        "event_type": event_type,
        "channel": CHANNEL_ID,
        "timestamp": timestamp_utc(),
        "track_id": (
            str(int(track_id)) if track_id is not None else None
        ),
        "object_class": object_class,
        "confidence": (
            round(float(confidence), 2)
            if confidence is not None else None
        )
    }

    print(
        "[event] " + json.dumps(event),
        flush=True
    )


# ============================================================
# BLYNK INTEGRATION
# ============================================================

class BlynkSender:
    """Asynchronous Blynk sender supporting multiple device tokens."""

    def __init__(self, tokens):
        self.tokens = tokens
        self.tasks = queue.Queue(maxsize=8)
        self.shutdown = threading.Event()
        self.thread = None

        if self.tokens:
            self.thread = threading.Thread(
                target=self.worker,
                daemon=True
            )
            self.thread.start()
            print(
                f"[blynk] Enabled for {len(self.tokens)} device(s)"
            )
        else:
            print("[blynk] Disabled: no device tokens configured")

    def send(self, value):
        if not self.tokens:
            return

        try:
            self.tasks.put_nowait(value)
        except queue.Full:
            try:
                self.tasks.get_nowait()
            except queue.Empty:
                pass

            self.tasks.put_nowait(value)

    def worker(self):
        while not self.shutdown.is_set() or not self.tasks.empty():
            try:
                value = self.tasks.get(timeout=0.5)
            except queue.Empty:
                continue

            for token in self.tokens:
                success = False

                for attempt in range(3):
                    try:
                        response = requests.get(
                            f"https://{BLYNK_SERVER}/external/api/update",
                            params={
                                "token": token,
                                BLYNK_PIN: value
                            },
                            timeout=5
                        )

                        if response.status_code == 200:
                            print(
                                f"[blynk] {BLYNK_PIN}={value} OK",
                                flush=True
                            )
                            success = True
                            break

                        print(
                            f"[blynk] HTTP {response.status_code}: "
                            f"{response.text[:100]}"
                        )

                    except requests.RequestException as exc:
                        print(f"[blynk] {exc}")

                    time.sleep(1)

                if not success:
                    print("[blynk] Failed to update one device")

            self.tasks.task_done()

    def close(self, send_zero=False):
        if not self.tokens:
            return

        if send_zero:
            self.send(0)

        self.shutdown.set()

        if self.thread:
            self.thread.join(timeout=15)


# ============================================================
# FLASK MJPEG STREAM
# ============================================================

def create_flask_app():
    app = Flask(__name__)

    @app.before_request
    def authenticate():
        auth = request.authorization

        valid = (
            auth is not None
            and hmac.compare_digest(
                auth.username or "", STREAM_USERNAME
            )
            and hmac.compare_digest(
                auth.password or "", STREAM_PASSWORD
            )
        )

        if not valid:
            return Response(
                "Authentication required",
                401,
                {
                    "WWW-Authenticate": 'Basic realm="Smart Home Camera"'
                }
            )

    @app.route("/")
    def index():
        return """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Smart Home CCTV</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body {
                    background: #111;
                    color: white;
                    text-align: center;
                    font-family: Arial;
                }
                img {
                    width: 95%;
                    max-width: 960px;
                }
            </style>
        </head>
        <body>
            <h2>Smart Home AI Surveillance</h2>
            <img src="/stream.mjpg">
        </body>
        </html>
        """

    @app.route("/stream.mjpg")
    def stream():
        def generate():
            previous = None

            while not stop_event.is_set():
                jpeg = frame_store.get(previous)

                if jpeg is None or jpeg is previous:
                    continue

                previous = jpeg

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )

        return Response(
            generate(),
            mimetype="multipart/x-mixed-replace; boundary=frame"
        )

    return app


def run_stream_server():
    app = create_flask_app()

    app.run(
        host="0.0.0.0",
        port=STREAM_PORT,
        threaded=True,
        use_reloader=False
    )


# ============================================================
# YOLO MODEL
# ============================================================

def load_model(model_name):
    print(f"[yolo] Loading {model_name}...", flush=True)

    model = YOLO(model_name)

    print("[yolo] Model loaded", flush=True)

    return model


# ============================================================
# DETECTION PIPELINE
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Smart Home AI Surveillance"
    )

    parser.add_argument(
        "--camera",
        choices=["pc", "picam"],
        default="pc",
        help="Camera source: pc or picam"
    )

    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="OpenCV webcam index"
    )

    parser.add_argument(
        "--model",
        default=MODEL_NAME,
        help="YOLO model file"
    )

    parser.add_argument(
        "--no-motion-gate",
        action="store_true",
        help="Run YOLO on every frame"
    )

    parser.add_argument(
        "--enable-blynk",
        action="store_true",
        help="Enable Blynk notifications"
    )

    args = parser.parse_args()

    signal.signal(
        signal.SIGINT,
        lambda *_: stop_event.set()
    )

    if hasattr(signal, "SIGTERM"):
        signal.signal(
            signal.SIGTERM,
            lambda *_: stop_event.set()
        )

    # --------------------------------------------------------
    # Initialize components
    # --------------------------------------------------------

    model = load_model(args.model)

    camera = create_camera(args)

    # BLYNK_TOKENS accepts multiple comma-separated tokens.
    # BLYNK_TOKEN is supported for a single device.
    tokens = []

    if args.enable_blynk:
        tokens = [
            token.strip()
            for token in os.getenv(
                "BLYNK_TOKENS",
                os.getenv("BLYNK_TOKEN", "")
            ).split(",")
            if token.strip()
        ]

        if not tokens:
            print("[blynk] WARNING: No tokens supplied")

    blynk = BlynkSender(tokens)

    threading.Thread(
        target=run_stream_server,
        daemon=True
    ).start()

    print(
        f"[boot] Camera: {args.camera}",
        flush=True
    )

    print(
        f"[boot] Stream: http://localhost:{STREAM_PORT}/",
        flush=True
    )

    # --------------------------------------------------------
    # State initialization
    # --------------------------------------------------------

    motion_detector = cv2.createBackgroundSubtractorMOG2(
        history=500,
        varThreshold=50,
        detectShadows=False
    )

    hits = deque(maxlen=HIT_WINDOW)

    present = False
    last_seen = 0.0
    last_sent = 0.0
    last_inference = 0.0

    active_tracks = []
    seen_track_ids = set()

    fps = 0.0
    previous_frame_time = time.time()

    blynk.send(0)
    last_sent = time.time()

    # --------------------------------------------------------
    # Main loop
    # --------------------------------------------------------

    try:
        while not stop_event.is_set():

            success, frame = camera.read()

            if not success or frame is None:
                print("[camera] Frame unavailable")
                time.sleep(0.1)
                continue

            now = time.time()

            # Motion detection
            motion_mask = motion_detector.apply(frame)

            motion_level = cv2.countNonZero(motion_mask)

            # Run YOLO if motion exists, or periodically
            # even when there is no motion.
            should_infer = (
                args.no_motion_gate
                or motion_level > MOTION_THRESHOLD
                or now - last_inference >= IDLE_INFERENCE_INTERVAL
            )

            detection_count = 0

            if should_infer:

                results = model.track(
                    frame,
                    classes=[0],
                    conf=CONFIDENCE,
                    imgsz=IMGSZ,
                    tracker=TRACKER,
                    persist=True,
                    verbose=False
                )

                result = results[0]

                last_inference = now
                active_tracks = []

                if result.boxes is not None:

                    boxes = result.boxes.xyxy.cpu().numpy()
                    confidences = result.boxes.conf.cpu().numpy()

                    if result.boxes.id is not None:
                        track_ids = result.boxes.id.cpu().numpy()
                    else:
                        track_ids = [None] * len(boxes)

                    for box, track_id, confidence in zip(
                        boxes, track_ids, confidences
                    ):

                        x1, y1, x2, y2 = box

                        width = x2 - x1
                        height = y2 - y1

                        # Ignore very small detections.
                        if width < MIN_BOX_SIZE or height < MIN_BOX_SIZE:
                            continue

                        detection_count += 1

                        active_tracks.append(
                            (box, track_id, confidence)
                        )

                        # Generate an event once per new tracking ID.
                        if track_id is not None:
                            track_id = int(track_id)

                            if track_id not in seen_track_ids:
                                seen_track_ids.add(track_id)

                                emit_event(
                                    "detection",
                                    track_id,
                                    confidence
                                )

                # ------------------------------------------------
                # Debounce person detection
                # ------------------------------------------------

                hits.append(detection_count > 0)

                if detection_count > 0:
                    last_seen = now

                if not present and sum(hits) >= HITS_NEEDED:

                    present = True
                    blynk.send(1)
                    last_sent = now

                    print("[detect] PERSON PRESENT", flush=True)

                    emit_event("person_present")

                elif present and now - last_seen > CLEAR_AFTER_SECONDS:

                    present = False
                    hits.clear()

                    blynk.send(0)
                    last_sent = now

                    print("[detect] PERSON GONE", flush=True)

                    emit_event("person_gone")

            # ------------------------------------------------
            # Blynk keep-alive
            # ------------------------------------------------

            if present and now - last_sent >= KEEPALIVE_SECONDS:

                blynk.send(1)
                last_sent = now

            # ------------------------------------------------
            # Draw detection boxes
            # ------------------------------------------------

            annotated = frame.copy()

            for box, track_id, confidence in active_tracks:

                x1, y1, x2, y2 = map(int, box)

                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                label = (
                    f"ID: {int(track_id)} "
                    if track_id is not None else ""
                )

                label += f"Person {confidence:.2f}"

                cv2.putText(
                    annotated,
                    label,
                    (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

            # ------------------------------------------------
            # FPS and status
            # ------------------------------------------------

            elapsed = max(now - previous_frame_time, 0.001)

            current_fps = 1.0 / elapsed

            fps = (
                0.9 * fps + 0.1 * current_fps
                if fps else current_fps
            )

            previous_frame_time = now

            status = (
                "PERSON PRESENT"
                if present else "NO PERSON"
            )

            color = (
                (0, 0, 255)
                if present else (0, 200, 0)
            )

            cv2.putText(
                annotated,
                f"{status} | {fps:.1f} FPS",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

            cv2.putText(
                annotated,
                f"Motion: {motion_level} | "
                f"YOLO: {'ON' if should_infer else 'IDLE'}",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

            # ------------------------------------------------
            # Encode and publish MJPEG frame
            # ------------------------------------------------

            success, buffer = cv2.imencode(
                ".jpg",
                annotated,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

            if success:
                frame_store.update(buffer.tobytes())

            # Local preview
            cv2.imshow(
                "Smart Home AI Surveillance",
                annotated
            )

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[exit] Interrupted by user")

    finally:
        print("[exit] Shutting down...")

        if present:
            emit_event("person_gone")

        blynk.close(send_zero=present)

        stop_event.set()

        camera.release()

        cv2.destroyAllWindows()

        print("[exit] Shutdown complete")


if __name__ == "__main__":
    main()
