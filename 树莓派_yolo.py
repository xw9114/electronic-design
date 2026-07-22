from __future__ import annotations

import atexit
import logging
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Generator, Optional, Sequence

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template_string
from ultralytics import YOLO


# ============================================================
# 1. 基本参数
# ============================================================

MODEL_PATH = "/home/xw/yolo_project/yolo11n_ncnn_model"

CAMERA_DEVICE = "/dev/video0"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 60
CAMERA_FOURCC = "MJPG"

YOLO_IMAGE_SIZE = 320
CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.45
MAX_DETECTIONS = 20

JPEG_QUALITY = 60
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

# 检测框超过这个时间仍未更新，就不再画旧框。
DETECTION_MAX_AGE_S = 0.20

# 每隔多少秒打印一次性能。
PRINT_INTERVAL_S = 1.0


# ============================================================
# 2. 摄像头曝光设置
# ============================================================

ENABLE_MANUAL_EXPOSURE = True
CAMERA_EXPOSURE_ABSOLUTE = 100
CAMERA_GAIN = 10
CAMERA_POWER_LINE_FREQUENCY = 1
CAMERA_EXPOSURE_AUTO_PRIORITY = 0
CAMERA_AUTO_WHITE_BALANCE = True


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class WindowFpsCounter:
    """按固定时间窗口统计真实 FPS，避免突发交付导致假高帧率。"""

    def __init__(self, sample_seconds: float = 1.0) -> None:
        self.sample_seconds = max(0.2, float(sample_seconds))
        self.window_start = time.monotonic()
        self.frame_count = 0
        self.value = 0.0

    def tick(self) -> float:
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.window_start

        if elapsed >= self.sample_seconds:
            self.value = self.frame_count / elapsed
            self.frame_count = 0
            self.window_start = now

        return self.value


def _read_v4l2_controls() -> dict[str, tuple[int, int]]:
    if shutil.which("v4l2-ctl") is None:
        print("提示：未安装 v4l2-ctl，跳过曝光设置。")
        return {}

    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", CAMERA_DEVICE, "--list-ctrls"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except Exception as exc:
        print(f"读取摄像头控制失败：{exc}")
        return {}

    controls: dict[str, tuple[int, int]] = {}
    pattern = re.compile(
        r"^\s*([A-Za-z0-9_]+)\s+0x[0-9a-fA-F]+.*?"
        r"min=(-?\d+)\s+max=(-?\d+)",
        re.MULTILINE,
    )

    for name, low, high in pattern.findall(result.stdout):
        controls[name] = (int(low), int(high))

    return controls


def _set_v4l2_control(
    controls: dict[str, tuple[int, int]],
    name: str,
    value: int,
) -> bool:
    if name not in controls:
        return False

    low, high = controls[name]
    value = int(clamp(value, low, high))

    result = subprocess.run(
        ["v4l2-ctl", "-d", CAMERA_DEVICE, f"--set-ctrl={name}={value}"],
        check=False,
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        print(f"设置 {name}={value} 失败：{detail}")
        return False

    print(f"摄像头控制：{name}={value}")
    return True


def _set_first_supported_control(
    controls: dict[str, tuple[int, int]],
    names: Sequence[str],
    value: int,
) -> Optional[str]:
    for name in names:
        if name in controls and _set_v4l2_control(controls, name, value):
            return name
    return None


def configure_camera_exposure() -> None:
    controls = _read_v4l2_controls()
    if not controls:
        return

    _set_v4l2_control(
        controls,
        "power_line_frequency",
        CAMERA_POWER_LINE_FREQUENCY,
    )

    _set_first_supported_control(
        controls,
        ("exposure_dynamic_framerate", "exposure_auto_priority"),
        CAMERA_EXPOSURE_AUTO_PRIORITY,
    )

    if CAMERA_AUTO_WHITE_BALANCE:
        _set_v4l2_control(
            controls,
            "white_balance_temperature_auto",
            1,
        )

    if ENABLE_MANUAL_EXPOSURE:
        _set_first_supported_control(
            controls,
            ("auto_exposure", "exposure_auto"),
            1,
        )

        time.sleep(0.08)

        _set_first_supported_control(
            controls,
            ("exposure_time_absolute", "exposure_absolute"),
            CAMERA_EXPOSURE_ABSOLUTE,
        )

        _set_first_supported_control(
            controls,
            ("gain",),
            CAMERA_GAIN,
        )


# ============================================================
# 3. 打开摄像头
# ============================================================

def open_camera() -> cv2.VideoCapture:
    configure_camera_exposure()

    # 关键优化：
    # 1. 明确请求 MJPG 640x480@60
    # 2. 队列最多保留 1 帧
    # 3. 处理不过来时丢弃旧帧，只拿最新帧
    pipeline = (
        f"v4l2src device={CAMERA_DEVICE} io-mode=2 ! "
        f"image/jpeg,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},"
        f"framerate={CAMERA_FPS}/1 ! "
        "queue leaky=downstream max-size-buffers=1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if cap.isOpened():
        print("摄像头后端：GStreamer")
    else:
        print("GStreamer 打开失败，回退到 V4L2。")
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        if not cap.isOpened():
            raise RuntimeError(f"无法打开摄像头：{CAMERA_DEVICE}")

        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*CAMERA_FOURCC),
        )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(5):
        cap.grab()

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(
        f"摄像头已打开：{actual_w}x{actual_h}，"
        f"驱动报告 FPS={actual_fps:.1f}"
    )

    return cap


# ============================================================
# 4. 共享数据
# ============================================================

@dataclass
class DetectionSnapshot:
    boxes: np.ndarray
    confidences: np.ndarray
    class_ids: np.ndarray
    timestamp: float
    inference_ms: float
    detection_fps: float


class SharedState:
    def __init__(self) -> None:
        self.stop_event = threading.Event()

        self.frame_condition = threading.Condition()
        self.latest_frame: Optional[np.ndarray] = None
        self.frame_id = 0
        self.capture_fps = 0.0

        self.detection_lock = threading.Lock()
        self.detection = DetectionSnapshot(
            boxes=np.empty((0, 4), dtype=np.float32),
            confidences=np.empty((0,), dtype=np.float32),
            class_ids=np.empty((0,), dtype=np.int32),
            timestamp=0.0,
            inference_ms=0.0,
            detection_fps=0.0,
        )

        self.jpeg_condition = threading.Condition()
        self.latest_jpeg: Optional[bytes] = None
        self.jpeg_id = 0
        self.preview_fps = 0.0
        self.jpeg_ms = 0.0


# ============================================================
# 5. 三线程流水线
# ============================================================

def capture_loop(cap: cv2.VideoCapture, state: SharedState) -> None:
    """线程 1：持续读取摄像头，只保存最新帧。"""

    fps_counter = WindowFpsCounter(1.0)

    while not state.stop_event.is_set():
        ok, frame = cap.read()

        if not ok or frame is None:
            time.sleep(0.005)
            continue

        capture_fps = fps_counter.tick()

        with state.frame_condition:
            state.latest_frame = frame
            state.frame_id += 1
            state.capture_fps = capture_fps
            state.frame_condition.notify_all()


def detection_loop(
    model: YOLO,
    state: SharedState,
) -> None:
    """线程 2：对最新帧运行 NCNN YOLO，不等待旧帧。"""

    last_processed_frame_id = -1
    fps_counter = WindowFpsCounter(1.0)
    warmed_up = False

    while not state.stop_event.is_set():
        with state.frame_condition:
            state.frame_condition.wait_for(
                lambda: (
                    state.frame_id != last_processed_frame_id
                    or state.stop_event.is_set()
                ),
                timeout=1.0,
            )

            if state.stop_event.is_set():
                break

            if state.latest_frame is None:
                continue

            frame = state.latest_frame.copy()
            current_frame_id = state.frame_id

        # 直接跳到最新帧；中间积压帧不会逐一处理。
        last_processed_frame_id = current_frame_id

        if not warmed_up:
            print("正在预热 NCNN 模型……")
            for _ in range(3):
                model.predict(
                    source=frame,
                    imgsz=YOLO_IMAGE_SIZE,
                    conf=CONFIDENCE_THRESHOLD,
                    iou=IOU_THRESHOLD,
                    max_det=MAX_DETECTIONS,
                    verbose=False,
                )
            warmed_up = True
            print("NCNN 模型预热完成")

        result = model.predict(
            source=frame,
            imgsz=YOLO_IMAGE_SIZE,
            conf=CONFIDENCE_THRESHOLD,
            iou=IOU_THRESHOLD,
            max_det=MAX_DETECTIONS,
            verbose=False,
        )[0]

        detection_fps = fps_counter.tick()
        inference_ms = float(result.speed.get("inference", 0.0))

        if result.boxes is None or len(result.boxes) == 0:
            boxes = np.empty((0, 4), dtype=np.float32)
            confidences = np.empty((0,), dtype=np.float32)
            class_ids = np.empty((0,), dtype=np.int32)
        else:
            boxes = result.boxes.xyxy.cpu().numpy().astype(np.float32)
            confidences = result.boxes.conf.cpu().numpy().astype(np.float32)
            class_ids = result.boxes.cls.cpu().numpy().astype(np.int32)

        snapshot = DetectionSnapshot(
            boxes=boxes,
            confidences=confidences,
            class_ids=class_ids,
            timestamp=time.monotonic(),
            inference_ms=inference_ms,
            detection_fps=detection_fps,
        )

        with state.detection_lock:
            state.detection = snapshot


def class_color(class_id: int) -> tuple[int, int, int]:
    """根据类别编号生成固定 BGR 颜色。"""

    return (
        int((37 * class_id + 80) % 255),
        int((17 * class_id + 160) % 255),
        int((29 * class_id + 220) % 255),
    )


def draw_detections(
    frame: np.ndarray,
    snapshot: DetectionSnapshot,
    names: dict | list,
    state: SharedState,
) -> np.ndarray:
    """手动画框，比 result.plot() 更轻，也可画在最新摄像头帧上。"""

    out = frame
    now = time.monotonic()

    if now - snapshot.timestamp <= DETECTION_MAX_AGE_S:
        for box, confidence, class_id in zip(
            snapshot.boxes,
            snapshot.confidences,
            snapshot.class_ids,
        ):
            x1, y1, x2, y2 = (int(round(v)) for v in box)
            color = class_color(int(class_id))

            if isinstance(names, dict):
                class_name = names.get(int(class_id), str(int(class_id)))
            else:
                class_name = (
                    names[int(class_id)]
                    if 0 <= int(class_id) < len(names)
                    else str(int(class_id))
                )

            label = f"{class_name} {float(confidence):.2f}"

            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

            text_size, baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                1,
            )

            text_y = max(y1, text_size[1] + baseline + 2)

            cv2.rectangle(
                out,
                (x1, text_y - text_size[1] - baseline - 4),
                (x1 + text_size[0] + 4, text_y),
                color,
                -1,
            )

            cv2.putText(
                out,
                label,
                (x1 + 2, text_y - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    cv2.putText(
        out,
        f"WEB: {state.preview_fps:.1f} FPS",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        out,
        f"DET: {snapshot.detection_fps:.1f} FPS  "
        f"Infer: {snapshot.inference_ms:.1f} ms",
        (12, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        out,
        f"CAM: {state.capture_fps:.1f} FPS",
        (12, 84),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 0),
        2,
        cv2.LINE_AA,
    )

    return out


def preview_loop(
    state: SharedState,
    names: dict | list,
) -> None:
    """线程 3：在最新帧上画最近检测框，再编码成 JPEG。"""

    last_preview_frame_id = -1
    fps_counter = WindowFpsCounter(1.0)
    last_print_time = time.monotonic()

    while not state.stop_event.is_set():
        with state.frame_condition:
            state.frame_condition.wait_for(
                lambda: (
                    state.frame_id != last_preview_frame_id
                    or state.stop_event.is_set()
                ),
                timeout=1.0,
            )

            if state.stop_event.is_set():
                break

            if state.latest_frame is None:
                continue

            frame = state.latest_frame.copy()
            current_frame_id = state.frame_id

        last_preview_frame_id = current_frame_id

        with state.detection_lock:
            snapshot = state.detection

        annotated = draw_detections(
            frame,
            snapshot,
            names,
            state,
        )

        encode_start = time.perf_counter()

        ok, encoded = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)],
        )

        encode_end = time.perf_counter()

        if not ok:
            continue

        preview_fps = fps_counter.tick()
        jpeg_bytes = encoded.tobytes()

        with state.jpeg_condition:
            state.latest_jpeg = jpeg_bytes
            state.jpeg_id += 1
            state.preview_fps = preview_fps
            state.jpeg_ms = (encode_end - encode_start) * 1000
            state.jpeg_condition.notify_all()

        now = time.monotonic()

        if now - last_print_time >= PRINT_INTERVAL_S:
            print(
                f"CAM:{state.capture_fps:5.1f} FPS | "
                f"DET:{snapshot.detection_fps:5.1f} FPS | "
                f"Infer:{snapshot.inference_ms:5.1f} ms | "
                f"WEB:{state.preview_fps:5.1f} FPS | "
                f"JPEG:{state.jpeg_ms:4.1f} ms"
            )
            last_print_time = now


# ============================================================
# 6. Flask 网页
# ============================================================

app = Flask(__name__)
state = SharedState()

HTML_PAGE = """
<!doctype html>
<html lang="zh-CN">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>树莓派 YOLO 实时检测</title>
    <style>
        * { box-sizing: border-box; }
        body {
            margin: 0;
            min-height: 100vh;
            background: #111827;
            color: #f9fafb;
            font-family: Arial, "Microsoft YaHei", sans-serif;
            text-align: center;
        }
        main { padding: 20px 12px; }
        h1 { margin: 0 0 8px; font-size: 26px; }
        p { margin: 0 0 16px; color: #9ca3af; }
        .box {
            display: inline-block;
            max-width: 100%;
            padding: 10px;
            border-radius: 12px;
            background: #1f2937;
        }
        img {
            display: block;
            width: min(96vw, 960px);
            height: auto;
            border-radius: 8px;
            background: #000;
        }
    </style>
</head>
<body>
<main>
    <h1>树莓派 YOLO 实时检测</h1>
    <p>GStreamer 低延迟采集 · NCNN 推理 · 三线程流水线</p>
    <div class="box">
        <img src="/video_feed" alt="YOLO 视频流">
    </div>
</main>
</body>
</html>
"""


@app.route("/")
def index() -> str:
    return render_template_string(HTML_PAGE)


def generate_mjpeg() -> Generator[bytes, None, None]:
    last_jpeg_id = -1

    while not state.stop_event.is_set():
        with state.jpeg_condition:
            state.jpeg_condition.wait_for(
                lambda: (
                    state.jpeg_id != last_jpeg_id
                    or state.stop_event.is_set()
                ),
                timeout=2.0,
            )

            if state.stop_event.is_set():
                break

            if state.latest_jpeg is None:
                continue

            if state.jpeg_id == last_jpeg_id:
                continue

            jpeg = state.latest_jpeg
            last_jpeg_id = state.jpeg_id

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Cache-Control: no-cache\r\n\r\n"
            + jpeg
            + b"\r\n"
        )


@app.route("/video_feed")
def video_feed() -> Response:
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )


@app.route("/status")
def status() -> Response:
    with state.detection_lock:
        snapshot = state.detection

    return jsonify(
        camera_fps=round(state.capture_fps, 2),
        detection_fps=round(snapshot.detection_fps, 2),
        inference_ms=round(snapshot.inference_ms, 2),
        web_fps=round(state.preview_fps, 2),
        jpeg_ms=round(state.jpeg_ms, 2),
    )


@app.route("/favicon.ico")
def favicon() -> tuple[str, int]:
    return "", 204


# ============================================================
# 7. 主程序
# ============================================================

cap: Optional[cv2.VideoCapture] = None
worker_threads: list[threading.Thread] = []


def stop_all() -> None:
    if state.stop_event.is_set():
        return

    print("\n正在停止程序……")
    state.stop_event.set()

    with state.frame_condition:
        state.frame_condition.notify_all()

    with state.jpeg_condition:
        state.jpeg_condition.notify_all()

    if cap is not None and cap.isOpened():
        cap.release()

    for thread in worker_threads:
        if thread.is_alive():
            thread.join(timeout=1.5)

    print("摄像头与线程已释放")


def main() -> None:
    global cap, worker_threads

    if not os.path.isdir(MODEL_PATH):
        raise FileNotFoundError(
            f"找不到 NCNN 模型目录：{MODEL_PATH}"
        )

    # 避免 OpenCV 自己开启大量线程，与 NCNN 抢占 CPU。
    cv2.setNumThreads(2)

    print(f"正在加载 NCNN 模型：{MODEL_PATH}")
    model = YOLO(MODEL_PATH, task="detect")
    print("模型加载完成")

    cap = open_camera()

    worker_threads = [
        threading.Thread(
            target=capture_loop,
            args=(cap, state),
            name="camera-capture",
            daemon=True,
        ),
        threading.Thread(
            target=detection_loop,
            args=(model, state),
            name="yolo-detection",
            daemon=True,
        ),
        threading.Thread(
            target=preview_loop,
            args=(state, model.names),
            name="web-preview",
            daemon=True,
        ),
    ]

    for thread in worker_threads:
        thread.start()

    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    print(f"网页地址：http://树莓派IP:{WEB_PORT}")
    print(f"性能状态：http://树莓派IP:{WEB_PORT}/status")
    print("按 Ctrl+C 结束程序\n")

    app.run(
        host=WEB_HOST,
        port=WEB_PORT,
        threaded=True,
        debug=False,
        use_reloader=False,
    )


def _signal_handler(_signum=None, _frame=None) -> None:
    stop_all()


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
atexit.register(stop_all)


if __name__ == "__main__":
    try:
        main()
    finally:
        stop_all()