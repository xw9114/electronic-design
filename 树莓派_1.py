"""
树莓派 + USB 摄像头 + 纯 OpenCV A4 黑框/圆心追踪 + Emm42_V5.0 双电机控制

用途：
- 替换原 MaixCAM 摄像头和 .mud/.cvimodel 模型；
- 不需要 YOLO、ONNX，也不需要重新训练；
- 使用 /dev/video0 的 USB 摄像头；
- 传统视觉寻找 A4 黑色矩形框，透视展开后得到纸张中心；
- 可选：在透视图中心区域继续用霍夫圆检测实际圆心；
- 保留原程序的水平分段 KP、PID、速度预测、丢失目标停止和 Emm42 串口协议；
- 树莓派无显示器时，可在电脑浏览器打开 http://xw.local:5000 查看画面。

算法思路参考：
Sipeed MaixPy 2025 电赛 E 题开源项目 demo_diansai_2025_E_circle_track。
原项目用 YOLO 粗定位黑框，再用 OpenCV 找四角和透视变换；本文件将粗定位也改成了纯 OpenCV。

安装依赖：
    sudo apt update
    sudo apt install -y python3-opencv python3-numpy python3-serial python3-flask

树莓派 GPIO 串口接线（3.3V TTL）：
    GPIO14/TXD，物理引脚 8
           ├──> 水平驱动器 RX，ID=0x01
           └──> 俯仰驱动器 RX，ID=0x02
    树莓派 GND ───> 两个驱动器 GND

如果使用 USB 转 TTL，把 SERIAL_PORT 改成 /dev/ttyUSB0。
驱动器如果是 RS-232/RS-485 电平，不能直接接树莓派 GPIO，必须使用对应转换模块。
"""

from __future__ import annotations

import math
import signal
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None

try:
    from flask import Flask, Response
except ImportError:
    Flask = None
    Response = None


# ============================================================
# 1. USB 摄像头参数
# ============================================================
CAMERA_DEVICE = "/dev/video0"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_FOURCC = "MJPG"
CAMERA_BUFFER_SIZE = 1

# 每隔几帧压缩一次网页预览；识别与电机控制仍逐帧运行。
PREVIEW_EVERY_N_FRAMES = 2
JPEG_QUALITY = 80


# ============================================================
# 2. A4 黑框识别参数
# ============================================================
# 黑框候选至少占整幅图多少面积。
MIN_RECT_AREA = 3500
# 候选最大面积，避免把整幅画面边缘当作目标。
MAX_RECT_AREA_RATIO = 0.92
# 四边形最短边至少多少像素。
MIN_RECT_SIDE = 35
# 轮廓面积 / 最小外接矩形面积的最低值。
MIN_RECTANGULARITY = 0.55
# A4 长短边比例约为 1.414；透视情况下放宽范围。
MIN_ASPECT_RATIO = 0.72
MAX_ASPECT_RATIO = 2.20
# 多边形近似精度。数值越大越容易得到四边形，但太大会丢角。
APPROX_EPSILON_RATIO = 0.025
# 黑色边框与内部区域平均灰度至少相差多少。
MIN_BORDER_CONTRAST = 8.0

# 自适应二值化参数，必须为奇数。
ADAPTIVE_BLOCK_SIZE = 31
ADAPTIVE_C = 9
MORPH_KERNEL_SIZE = 5

# 透视展开尺寸。横放时为 424x300，竖放时自动交换。
STD_LONG_SIDE = 424
STD_SHORT_SIDE = 300

# False：直接使用纸张几何中心，速度快、稳定，推荐先用这个。
# True：透视展开后继续找实际圆心，纸上圆心不准时再打开。
FIND_ACTUAL_CIRCLE = False
# 开启找圆后，没找到圆是否仍用纸张中心。
FALLBACK_TO_PAPER_CENTER = True
# 霍夫圆参数。
CIRCLE_PARAM1 = 100
CIRCLE_PARAM2 = 18
CIRCLE_MIN_RADIUS_RATIO = 0.020
CIRCLE_MAX_RADIUS_RATIO = 0.160
CIRCLE_SEARCH_SIZE_RATIO = 0.55

# 摄像头中心与激光实际落点有固定偏差时，在这里补偿，单位为原画面像素。
# 正数 X 向右，正数 Y 向下。
AIM_OFFSET_X_PX = 0
AIM_OFFSET_Y_PX = 0

# 是否在网页右下角显示透视展开的小图，调试识别时很有用。
SHOW_WARP_IN_PREVIEW = True


# ============================================================
# 3. 网页实时预览
# ============================================================
ENABLE_WEB_PREVIEW = True
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000


# ============================================================
# 4. Emm42 串口和电机参数
# ============================================================
# 初次测试视觉时可改成 False，程序仍会识别和显示，但不会控制电机。
ENABLE_MOTORS = True
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
CHECKSUM = 0x6B

PAN_ADDR = 0x01
TILT_ADDR = 0x02

PAN_REVERSE = True
TILT_REVERSE = True

MOTOR_ACCEL = 0
MIN_RPM = 5
MAX_RPM = 120
DEBUG_UART = False


# ============================================================
# 5. 追踪控制参数（保留原 MaixCAM 程序参数）
# ============================================================
DEAD_X = 4
DEAD_Y = 7

PAN_KP_NEAR = 0.65
PAN_KP_MID = 0.80
PAN_KP_FAR = 1.00
PAN_MID_ERROR = 15
PAN_FAR_ERROR = 40
PAN_KI = 0.33
PAN_KD = 0.02

PAN_PREDICT_FRAMES = 1.3
PAN_VELOCITY_FILTER_ALPHA = 0.55

TILT_KP = 0.30
TILT_KI = 0.06
TILT_KD = 0.08

INTEGRAL_LIMIT = 120.0
TARGET_FILTER_ALPHA = 0.90

LOCK_CONFIRM_FRAMES = 1
MISS_TOLERANCE_FRAMES = 2
LOST_STOP_MS = 220
CMD_INTERVAL_MS = 20


# ============================================================
# 数据结构
# ============================================================
@dataclass
class TargetResult:
    center: Tuple[int, int]
    quad: np.ndarray
    warped: np.ndarray
    used_circle: bool
    score: float
    binary: Optional[np.ndarray] = None


# ============================================================
# 通用函数
# ============================================================
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def now_ms() -> int:
    return int(time.monotonic() * 1000)


def distance_sq(p1: Sequence[float], p2: Sequence[float]) -> float:
    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])
    return dx * dx + dy * dy


def order_quad_points(points: np.ndarray) -> np.ndarray:
    """把四角排序为：左上、右上、右下、左下。"""
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    rect = np.zeros((4, 2), dtype=np.float32)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    rect[0] = pts[np.argmin(sums)]
    rect[2] = pts[np.argmax(sums)]
    rect[1] = pts[np.argmin(diffs)]
    rect[3] = pts[np.argmax(diffs)]
    return rect


def quad_side_lengths(quad: np.ndarray) -> Tuple[float, float, float, float]:
    tl, tr, br, bl = quad
    top = float(np.linalg.norm(tr - tl))
    right = float(np.linalg.norm(br - tr))
    bottom = float(np.linalg.norm(br - bl))
    left = float(np.linalg.norm(bl - tl))
    return top, right, bottom, left


def max_corner_cosine(quad: np.ndarray) -> float:
    """返回四个角中最大的 |cos(angle)|；越接近 0 越像直角。"""
    values = []
    for i in range(4):
        p0 = quad[(i - 1) % 4] - quad[i]
        p1 = quad[(i + 1) % 4] - quad[i]
        denom = float(np.linalg.norm(p0) * np.linalg.norm(p1)) + 1e-6
        values.append(abs(float(np.dot(p0, p1)) / denom))
    return max(values)


def draw_cross(
    frame: np.ndarray,
    x: int,
    y: int,
    size: int,
    color: Tuple[int, int, int],
    thickness: int = 2,
) -> None:
    cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
    cv2.line(frame, (x, y - size), (x, y + size), color, thickness)


# ============================================================
# 纯 OpenCV A4 黑框/圆心检测器
# ============================================================
class A4CircleDetector:
    def __init__(self) -> None:
        block = int(ADAPTIVE_BLOCK_SIZE)
        if block < 3:
            block = 3
        if block % 2 == 0:
            block += 1
        self.block_size = block
        k = max(1, int(MORPH_KERNEL_SIZE))
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))

    def _build_dark_mask(self, gray: np.ndarray) -> np.ndarray:
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        adaptive = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            self.block_size,
            ADAPTIVE_C,
        )
        # 连接黑框断点，并去除小噪点。
        mask = cv2.morphologyEx(
            adaptive,
            cv2.MORPH_CLOSE,
            self.morph_kernel,
            iterations=2,
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), np.uint8),
            iterations=1,
        )
        return mask

    @staticmethod
    def _border_contrast(gray: np.ndarray, quad: np.ndarray) -> Tuple[float, float, float]:
        """计算候选四边形边带与内部核心区的灰度对比。"""
        mask = np.zeros(gray.shape, dtype=np.uint8)
        poly = np.round(quad).astype(np.int32)
        cv2.fillConvexPoly(mask, poly, 255)

        top, right, bottom, left = quad_side_lengths(quad)
        erode_px = max(3, int(min(top, right, bottom, left) * 0.07))
        kernel_size = erode_px * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        inner = cv2.erode(mask, kernel, iterations=1)
        core = cv2.erode(inner, kernel, iterations=1)
        border = cv2.subtract(mask, inner)

        if cv2.countNonZero(border) < 20 or cv2.countNonZero(core) < 20:
            return -999.0, 255.0, 0.0

        border_mean = float(cv2.mean(gray, mask=border)[0])
        core_mean = float(cv2.mean(gray, mask=core)[0])
        return core_mean - border_mean, border_mean, core_mean

    def _find_quad(
        self,
        gray: np.ndarray,
        mask: np.ndarray,
        previous_center: Optional[Tuple[int, int]],
    ) -> Optional[Tuple[np.ndarray, float]]:
        frame_h, frame_w = gray.shape[:2]
        frame_area = float(frame_h * frame_w)
        max_area = frame_area * MAX_RECT_AREA_RATIO
        frame_diag_sq = float(frame_w * frame_w + frame_h * frame_h)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        best_quad: Optional[np.ndarray] = None
        best_score = -1.0

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < MIN_RECT_AREA or area > max_area:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0:
                continue

            approx = cv2.approxPolyDP(
                contour,
                APPROX_EPSILON_RATIO * perimeter,
                True,
            )
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue

            quad = order_quad_points(approx.reshape(4, 2))
            top, right, bottom, left = quad_side_lengths(quad)
            min_side = min(top, right, bottom, left)
            if min_side < MIN_RECT_SIDE:
                continue

            avg_w = (top + bottom) * 0.5
            avg_h = (left + right) * 0.5
            if min(avg_w, avg_h) < 1:
                continue
            aspect = max(avg_w, avg_h) / min(avg_w, avg_h)
            if not (MIN_ASPECT_RATIO <= aspect <= MAX_ASPECT_RATIO):
                continue

            min_rect = cv2.minAreaRect(contour)
            rect_area = float(min_rect[1][0] * min_rect[1][1])
            if rect_area <= 1:
                continue
            rectangularity = area / rect_area
            if rectangularity < MIN_RECTANGULARITY:
                continue

            angle_cos = max_corner_cosine(quad)
            if angle_cos > 0.80:
                continue

            contrast, border_mean, core_mean = self._border_contrast(gray, quad)
            if contrast < MIN_BORDER_CONTRAST:
                continue

            # A4 比例、直角程度、黑边对比和面积共同评分。
            aspect_error = abs(math.log(max(aspect, 1e-6) / math.sqrt(2.0)))
            aspect_quality = math.exp(-1.8 * aspect_error)
            angle_quality = clamp(1.0 - angle_cos, 0.15, 1.0)
            contrast_quality = clamp(contrast / 80.0, 0.10, 2.0)
            gray_quality = clamp((core_mean - border_mean) / 60.0, 0.10, 2.0)

            score = (
                area
                * clamp(rectangularity, 0.2, 1.1)
                * (0.45 + 0.55 * aspect_quality)
                * (0.45 + 0.55 * angle_quality)
                * (0.55 + 0.45 * contrast_quality)
                * (0.65 + 0.35 * gray_quality)
            )

            if previous_center is not None:
                center = tuple(np.mean(quad, axis=0))
                d2 = distance_sq(center, previous_center)
                # 已锁定后稍微偏向上一帧附近的框，防止跳到干扰矩形。
                score /= 1.0 + 2.0 * d2 / max(frame_diag_sq, 1.0)

            if score > best_score:
                best_score = score
                best_quad = quad

        if best_quad is None:
            return None
        return best_quad, best_score

    @staticmethod
    def _warp(frame: np.ndarray, quad: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        top, right, bottom, left = quad_side_lengths(quad)
        avg_w = (top + bottom) * 0.5
        avg_h = (left + right) * 0.5

        if avg_w >= avg_h:
            dst_w, dst_h = STD_LONG_SIDE, STD_SHORT_SIDE
        else:
            dst_w, dst_h = STD_SHORT_SIDE, STD_LONG_SIDE

        dst = np.array(
            [
                [0, 0],
                [dst_w - 1, 0],
                [dst_w - 1, dst_h - 1],
                [0, dst_h - 1],
            ],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(quad.astype(np.float32), dst)
        inverse = cv2.getPerspectiveTransform(dst, quad.astype(np.float32))
        warped = cv2.warpPerspective(frame, matrix, (dst_w, dst_h))
        return warped, matrix, inverse

    @staticmethod
    def _find_circle_center(warped: np.ndarray) -> Optional[Tuple[float, float, float]]:
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 1.5)
        h, w = gray.shape[:2]
        short_side = min(w, h)
        roi_size = max(40, int(short_side * CIRCLE_SEARCH_SIZE_RATIO))
        roi_size = min(roi_size, w, h)
        x0 = max(0, w // 2 - roi_size // 2)
        y0 = max(0, h // 2 - roi_size // 2)
        roi = gray[y0 : y0 + roi_size, x0 : x0 + roi_size]

        min_radius = max(3, int(short_side * CIRCLE_MIN_RADIUS_RATIO))
        max_radius = max(min_radius + 2, int(short_side * CIRCLE_MAX_RADIUS_RATIO))
        circles = cv2.HoughCircles(
            roi,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(12, roi_size // 4),
            param1=CIRCLE_PARAM1,
            param2=CIRCLE_PARAM2,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is None:
            return None

        expected = (w * 0.5, h * 0.5)
        candidates = np.asarray(circles[0], dtype=np.float32)
        best = min(
            candidates,
            key=lambda c: distance_sq((c[0] + x0, c[1] + y0), expected),
        )
        return float(best[0] + x0), float(best[1] + y0), float(best[2])

    def detect(
        self,
        frame: np.ndarray,
        previous_center: Optional[Tuple[int, int]] = None,
    ) -> Optional[TargetResult]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = self._build_dark_mask(gray)
        quad_result = self._find_quad(gray, mask, previous_center)
        if quad_result is None:
            return None

        quad, score = quad_result
        warped, _, inverse = self._warp(frame, quad)
        h, w = warped.shape[:2]

        used_circle = False
        target_std = (w * 0.5, h * 0.5)

        if FIND_ACTUAL_CIRCLE:
            circle = self._find_circle_center(warped)
            if circle is not None:
                target_std = (circle[0], circle[1])
                used_circle = True
                cv2.circle(
                    warped,
                    (int(round(circle[0])), int(round(circle[1]))),
                    int(round(circle[2])),
                    (0, 255, 0),
                    2,
                )
            elif not FALLBACK_TO_PAPER_CENTER:
                return None

        target_point = np.array([[[target_std[0], target_std[1]]]], dtype=np.float32)
        original = cv2.perspectiveTransform(target_point, inverse)[0][0]
        center_x = int(round(float(original[0]))) + AIM_OFFSET_X_PX
        center_y = int(round(float(original[1]))) + AIM_OFFSET_Y_PX

        cv2.circle(
            warped,
            (int(round(target_std[0])), int(round(target_std[1]))),
            4,
            (0, 0, 255),
            -1,
        )

        return TargetResult(
            center=(center_x, center_y),
            quad=quad,
            warped=warped,
            used_circle=used_circle,
            score=score,
            binary=mask,
        )


# ============================================================
# Emm42_V5.0 串口控制
# ============================================================
class Emm42Controller:
    def __init__(self) -> None:
        self.port = None
        if not ENABLE_MOTORS:
            print("电机控制已关闭：ENABLE_MOTORS=False")
            return
        if serial is None:
            print("警告：未安装 pyserial，进入纯视觉模式。")
            return
        try:
            self.port = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=0,
                write_timeout=0.2,
            )
            time.sleep(0.2)
            print(f"电机串口已打开：{SERIAL_PORT} @ {BAUD_RATE}")
        except Exception as exc:
            print(f"警告：无法打开电机串口 {SERIAL_PORT}：{exc}")
            print("程序继续以纯视觉模式运行。")
            self.port = None

    @property
    def available(self) -> bool:
        return self.port is not None and self.port.is_open

    def send_frame(self, data: Sequence[int], axis_name: str) -> None:
        if not self.available:
            return
        frame = bytes(data)
        try:
            self.port.write(frame)
            if DEBUG_UART:
                print(axis_name, "TX:", " ".join(f"{v:02X}" for v in frame))
        except Exception as exc:
            print(f"串口发送失败：{exc}")

    def enable(self, addr: int, axis_name: str, enabled: bool = True) -> None:
        self.send_frame(
            [addr, 0xF3, 0xAB, 0x01 if enabled else 0x00, 0x00, CHECKSUM],
            axis_name,
        )

    def speed(self, addr: int, axis_name: str, direction: int, speed_rpm: int) -> None:
        rpm = max(0, min(5000, int(speed_rpm)))
        accel = max(0, min(255, int(MOTOR_ACCEL)))
        self.send_frame(
            [
                addr,
                0xF6,
                int(direction) & 0x01,
                (rpm >> 8) & 0xFF,
                rpm & 0xFF,
                accel,
                0x00,
                CHECKSUM,
            ],
            axis_name,
        )

    def stop(self, addr: int, axis_name: str) -> None:
        self.send_frame([addr, 0xFE, 0x98, 0x00, CHECKSUM], axis_name)

    def stop_all(self) -> None:
        self.stop(PAN_ADDR, "PAN/ID01")
        time.sleep(0.005)
        self.stop(TILT_ADDR, "TILT/ID02")

    def close(self) -> None:
        try:
            self.stop_all()
        except Exception:
            pass
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass


# ============================================================
# PID 控制函数
# ============================================================
def get_pan_kp(error: float) -> float:
    abs_error = abs(error)
    if abs_error < PAN_MID_ERROR:
        return PAN_KP_NEAR
    if abs_error < PAN_FAR_ERROR:
        return PAN_KP_MID
    return PAN_KP_FAR


def calc_axis_command(
    error: float,
    previous_error: float,
    integral: float,
    kp: float,
    ki: float,
    kd: float,
    dead_zone: float,
    reverse: bool,
    dt_s: float,
) -> Tuple[Optional[Tuple[int, int]], float]:
    if abs(error) <= dead_zone:
        return None, 0.0

    if previous_error != 0 and error * previous_error < 0:
        integral = 0.0

    old_integral = integral
    candidate_integral = clamp(
        integral + error * dt_s,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT,
    )
    candidate_control = (
        kp * error
        + ki * candidate_integral
        + kd * (error - previous_error)
    )

    if abs(candidate_control) < MAX_RPM:
        integral = candidate_integral
    else:
        integral = old_integral

    control = kp * error + ki * integral + kd * (error - previous_error)
    if reverse:
        control = -control

    direction = 1 if control > 0 else 0
    rpm = int(clamp(abs(control), MIN_RPM, MAX_RPM))
    return (direction, rpm), integral


# ============================================================
# 网页预览
# ============================================================
class WebPreview:
    def __init__(self) -> None:
        self._jpeg: Optional[bytes] = None
        self._lock = threading.Lock()
        self.enabled = bool(ENABLE_WEB_PREVIEW and Flask is not None)
        if ENABLE_WEB_PREVIEW and Flask is None:
            print("警告：未安装 Flask，网页预览已关闭。")

    def update(self, frame: np.ndarray) -> None:
        if not self.enabled:
            return
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)],
        )
        if ok:
            with self._lock:
                self._jpeg = encoded.tobytes()

    def start(self) -> None:
        if not self.enabled:
            return

        app = Flask(__name__)
        preview = self

        @app.route("/")
        def index():
            return (
                "<html><head><meta charset='utf-8'><title>A4 Tracker</title></head>"
                "<body style='background:#111;color:#eee;text-align:center'>"
                "<h2>树莓派 USB 摄像头 A4/圆心追踪</h2>"
                "<img src='/video' style='max-width:96vw;max-height:88vh'>"
                "</body></html>"
            )

        @app.route("/video")
        def video():
            def generate():
                while True:
                    with preview._lock:
                        data = preview._jpeg
                    if data is None:
                        time.sleep(0.03)
                        continue
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + data
                        + b"\r\n"
                    )
                    time.sleep(0.01)

            return Response(
                generate(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        def run_server() -> None:
            import logging

            logging.getLogger("werkzeug").setLevel(logging.ERROR)
            app.run(
                host=WEB_HOST,
                port=WEB_PORT,
                threaded=True,
                use_reloader=False,
            )

        threading.Thread(target=run_server, daemon=True).start()
        print(f"网页预览：http://xw.local:{WEB_PORT}")


# ============================================================
# 摄像头初始化
# ============================================================
def open_camera() -> cv2.VideoCapture:
    pipeline = (
        f"v4l2src device={CAMERA_DEVICE} io-mode=2 ! "
        f"image/jpeg,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},"
        f"framerate={CAMERA_FPS}/1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError(
            f"无法通过 GStreamer 打开 USB 摄像头：{CAMERA_DEVICE}"
        )

    # 丢掉启动时可能残留的旧帧。
    for _ in range(5):
        cap.grab()

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"摄像头已打开：{CAMERA_DEVICE}，{actual_w}x{actual_h}，请求 FPS={actual_fps:.1f}")
    return cap


# ============================================================
# 绘制预览
# ============================================================
def draw_result(
    frame: np.ndarray,
    result: Optional[TargetResult],
    filtered_center: Optional[Tuple[int, int]],
    error_x: int,
    error_y: int,
    predicted_error_x: int,
    fps: float,
    miss_count: int,
    motors_available: bool,
) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]
    center = (w // 2, h // 2)

    draw_cross(out, center[0], center[1], 12, (0, 0, 255), 2)
    cv2.rectangle(
        out,
        (center[0] - DEAD_X, center[1] - DEAD_Y),
        (center[0] + DEAD_X, center[1] + DEAD_Y),
        (0, 0, 255),
        1,
    )

    if result is not None:
        quad_int = np.round(result.quad).astype(np.int32)
        cv2.polylines(out, [quad_int], True, (0, 255, 0), 2)
        raw_x, raw_y = result.center
        draw_cross(out, raw_x, raw_y, 8, (255, 255, 0), 1)

        if filtered_center is not None:
            tx, ty = filtered_center
            draw_cross(out, tx, ty, 10, (0, 255, 0), 2)
            cv2.line(out, center, (tx, ty), (0, 255, 255), 2)

        mode = "CIRCLE" if result.used_circle else "PAPER CENTER"
        cv2.putText(
            out,
            f"TARGET:{mode}",
            (8, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            f"EX:{error_x} PX:{predicted_error_x} EY:{error_y}",
            (8, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if SHOW_WARP_IN_PREVIEW and result.warped is not None:
            inset_w = min(210, w // 3)
            inset_h = max(1, int(result.warped.shape[0] * inset_w / result.warped.shape[1]))
            if inset_h > h // 3:
                inset_h = h // 3
                inset_w = max(1, int(result.warped.shape[1] * inset_h / result.warped.shape[0]))
            inset = cv2.resize(result.warped, (inset_w, inset_h))
            x0 = w - inset_w - 6
            y0 = h - inset_h - 6
            out[y0 : y0 + inset_h, x0 : x0 + inset_w] = inset
            cv2.rectangle(out, (x0, y0), (x0 + inset_w, y0 + inset_h), (255, 255, 255), 1)
    else:
        cv2.putText(
            out,
            f"NO A4 RECT  MISS:{miss_count}",
            (8, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    motor_text = "MOTOR:ON" if motors_available else "MOTOR:OFF/VISION ONLY"
    cv2.putText(
        out,
        f"FPS:{fps:.1f}  {motor_text}",
        (8, h - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return out


# ============================================================
# 主程序
# ============================================================
def main() -> None:
    running = True

    def request_stop(_signum=None, _frame=None) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    cap = open_camera()
    detector = A4CircleDetector()
    motors = Emm42Controller()
    preview = WebPreview()
    preview.start()

    if motors.available:
        motors.enable(PAN_ADDR, "PAN/ID01", True)
        time.sleep(0.02)
        motors.enable(TILT_ADDR, "TILT/ID02", True)
        time.sleep(0.25)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    image_center_x = frame_w // 2
    image_center_y = frame_h // 2

    last_target_center: Optional[Tuple[int, int]] = None
    filtered_cx: Optional[float] = None
    filtered_cy: Optional[float] = None

    last_seen_ms = 0
    last_cmd_ms = 0
    last_control_ms = 0

    previous_error_x = 0.0
    previous_error_y = 0.0
    pan_integral = 0.0
    tilt_integral = 0.0

    previous_target_cx: Optional[int] = None
    filtered_velocity_x = 0.0

    last_pan_cmd = None
    last_tilt_cmd = None
    lock_confirm_count = 0
    miss_count = 0

    fps_frames = 0
    fps_value = 0.0
    fps_start = time.monotonic()
    preview_count = 0

    latest_result: Optional[TargetResult] = None
    shown_error_x = 0
    shown_error_y = 0
    shown_predict_x = 0

    print("纯 OpenCV 模式：无需 .mud/.cvimodel/ONNX")
    print(f"FIND_ACTUAL_CIRCLE={FIND_ACTUAL_CIRCLE}")

    try:
        while running:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("读取摄像头画面失败")
                time.sleep(0.01)
                continue

            now = now_ms()
            result = detector.detect(frame, last_target_center)
            latest_result = result

            if result is not None:
                raw_cx, raw_cy = result.center
                last_target_center = (raw_cx, raw_cy)
                last_seen_ms = now
                miss_count = 0
                lock_confirm_count = min(lock_confirm_count + 1, LOCK_CONFIRM_FRAMES)

                if filtered_cx is None or filtered_cy is None:
                    filtered_cx = float(raw_cx)
                    filtered_cy = float(raw_cy)
                else:
                    a = TARGET_FILTER_ALPHA
                    filtered_cx = (1.0 - a) * filtered_cx + a * raw_cx
                    filtered_cy = (1.0 - a) * filtered_cy + a * raw_cy

                target_cx = int(round(filtered_cx))
                target_cy = int(round(filtered_cy))
                error_x = target_cx - image_center_x
                error_y = target_cy - image_center_y

                if previous_target_cx is None:
                    raw_velocity_x = 0.0
                else:
                    raw_velocity_x = float(target_cx - previous_target_cx)
                previous_target_cx = target_cx

                va = PAN_VELOCITY_FILTER_ALPHA
                filtered_velocity_x = (
                    (1.0 - va) * filtered_velocity_x
                    + va * raw_velocity_x
                )
                control_error_x = int(round(
                    error_x + filtered_velocity_x * PAN_PREDICT_FRAMES
                ))

                shown_error_x = int(error_x)
                shown_error_y = int(error_y)
                shown_predict_x = int(control_error_x)

                if (
                    lock_confirm_count >= LOCK_CONFIRM_FRAMES
                    and now - last_cmd_ms >= CMD_INTERVAL_MS
                ):
                    last_cmd_ms = now
                    if last_control_ms == 0:
                        dt_s = CMD_INTERVAL_MS / 1000.0
                    else:
                        dt_s = clamp(
                            (now - last_control_ms) / 1000.0,
                            0.001,
                            0.100,
                        )
                    last_control_ms = now

                    pan_cmd, pan_integral = calc_axis_command(
                        control_error_x,
                        previous_error_x,
                        pan_integral,
                        get_pan_kp(control_error_x),
                        PAN_KI,
                        PAN_KD,
                        DEAD_X,
                        PAN_REVERSE,
                        dt_s,
                    )
                    tilt_cmd, tilt_integral = calc_axis_command(
                        error_y,
                        previous_error_y,
                        tilt_integral,
                        TILT_KP,
                        TILT_KI,
                        TILT_KD,
                        DEAD_Y,
                        TILT_REVERSE,
                        dt_s,
                    )

                    if pan_cmd is None:
                        if last_pan_cmd != ("stop",):
                            motors.stop(PAN_ADDR, "PAN/ID01")
                            last_pan_cmd = ("stop",)
                    else:
                        pan_dir, pan_rpm = pan_cmd
                        command = ("speed", pan_dir, pan_rpm)
                        if command != last_pan_cmd:
                            motors.speed(PAN_ADDR, "PAN/ID01", pan_dir, pan_rpm)
                            last_pan_cmd = command

                    if tilt_cmd is None:
                        if last_tilt_cmd != ("stop",):
                            motors.stop(TILT_ADDR, "TILT/ID02")
                            last_tilt_cmd = ("stop",)
                    else:
                        tilt_dir, tilt_rpm = tilt_cmd
                        command = ("speed", tilt_dir, tilt_rpm)
                        if command != last_tilt_cmd:
                            motors.speed(TILT_ADDR, "TILT/ID02", tilt_dir, tilt_rpm)
                            last_tilt_cmd = command

                    previous_error_x = float(control_error_x)
                    previous_error_y = float(error_y)

            else:
                miss_count += 1
                if miss_count > MISS_TOLERANCE_FRAMES:
                    lock_confirm_count = 0
                    pan_integral = 0.0
                    tilt_integral = 0.0
                    last_control_ms = 0
                    previous_target_cx = None
                    filtered_velocity_x = 0.0

                if last_seen_ms != 0 and now - last_seen_ms >= LOST_STOP_MS:
                    if last_pan_cmd != ("stop",):
                        motors.stop(PAN_ADDR, "PAN/ID01")
                        last_pan_cmd = ("stop",)
                    if last_tilt_cmd != ("stop",):
                        motors.stop(TILT_ADDR, "TILT/ID02")
                        last_tilt_cmd = ("stop",)

                    filtered_cx = None
                    filtered_cy = None
                    last_target_center = None
                    lock_confirm_count = 0
                    previous_error_x = 0.0
                    previous_error_y = 0.0
                    pan_integral = 0.0
                    tilt_integral = 0.0
                    last_control_ms = 0
                    previous_target_cx = None
                    filtered_velocity_x = 0.0

            fps_frames += 1
            elapsed = time.monotonic() - fps_start
            if elapsed >= 1.0:
                fps_value = fps_frames / elapsed
                fps_frames = 0
                fps_start = time.monotonic()
                print(
                    f"FPS:{fps_value:.1f}  "
                    f"target={'YES' if result is not None else 'NO'}  "
                    f"EX:{shown_error_x} EY:{shown_error_y}"
                )

            preview_count += 1
            if preview_count >= PREVIEW_EVERY_N_FRAMES:
                preview_count = 0
                filtered_center = None
                if filtered_cx is not None and filtered_cy is not None:
                    filtered_center = (int(round(filtered_cx)), int(round(filtered_cy)))
                annotated = draw_result(
                    frame,
                    latest_result,
                    filtered_center,
                    shown_error_x,
                    shown_error_y,
                    shown_predict_x,
                    fps_value,
                    miss_count,
                    motors.available,
                )
                preview.update(annotated)

    finally:
        motors.close()
        cap.release()
        print("程序已停止，电机已发送停止命令。")


if __name__ == "__main__":
    main()
