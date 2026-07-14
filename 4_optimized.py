import math

from maix import camera, display, image, app, uart, pinmap, err, time


# ============================================================
# MaixCAM + 两个张大头 Emm42_V5.0
# 目标：识别画面中的矩形，使矩形中心锁定在画面中心
#
# 硬件接线：
#   俯仰轴：
#       MaixCAM A19(UART1_TX) -> 俯仰驱动器 RX
#       MaixCAM A18(UART1_RX) <- 俯仰驱动器 TX（可不接）
#       俯仰电机地址 = 0x02
#
#   水平轴：
#       MaixCAM A16(UART0_TX) -> 水平驱动器 RX
#       MaixCAM A17(UART0_RX) <- 水平驱动器 TX（可不接）
#       水平电机地址 = 0x01
#
#   MaixCAM与两个驱动器必须共地
#
# 驱动器菜单：
#   P_Serial = UART_FUN
#   UartBaud = 115200
#   Checksum = 0x6B
#   Response 建议先设 None，避免两台电机同时回包冲突
# ============================================================


# -------------------------
# 串口和电机参数
# -------------------------
BAUD_RATE = 115200
CHECKSUM = 0x6B

PAN_ADDR = 0x01       # 水平轴电机地址，UART0发送
TILT_ADDR = 0x02      # 俯仰轴电机地址，UART1发送

# 电机方向不对，只改这里
PAN_REVERSE = True
TILT_REVERSE = True

MOTOR_ACCEL = 8       # 0=无曲线加减速；建议先用 5~15
MIN_RPM = 3
MAX_RPM = 45


# -------------------------
# 图像和矩形识别参数
# -------------------------
IMG_W = 320
IMG_H = 240

# 只在此区域内检测。截图中的误识别位于画面最上方，
# 所以先排除顶部约30像素；若真实目标会进入顶部，可把第二个值改小。
ROI = [8, 30, 304, 202]

# 先转灰度、提取黑色边框，再执行 find_rects
GRAY_THRESHOLD = [0, 105]
USE_CLOSE = True
CLOSE_SIZE = 1

RECT_THRESHOLD = 9000

MIN_RECT_W = 45
MIN_RECT_H = 35
MIN_RECT_AREA = 2500
MAX_RECT_AREA = 36000

# 目标矩形长边/短边比例。
# A4纸约为1.414；此范围可容忍透视变形。
MIN_ASPECT_RATIO = 1.15
MAX_ASPECT_RATIO = 1.80
IDEAL_ASPECT_RATIO = 1.414

# 矩形不能紧贴ROI边缘，否则很可能是画面、桌面或建筑边界。
ROI_EDGE_MARGIN = 5

# 四边形几何过滤。数值越严格，误检越少，但倾斜/透视下的漏检会增加。
MAX_CORNER_COS = 0.45
MIN_OPPOSITE_SIDE_RATIO = 0.55
MIN_QUAD_FILL_RATIO = 0.42

# 单帧目标中心跳变超过该值时，认为不是同一个目标。
MAX_CENTER_JUMP = 70

# 连续识别到有效矩形后才允许电机追踪，过滤偶发误检。
LOCK_CONFIRM_FRAMES = 3

# 中心死区：矩形中心进入此范围后停止
DEAD_X = 7
DEAD_Y = 7

# 丢失目标多久后停止电机
LOST_STOP_MS = 250

# 限制串口命令频率，避免每帧都刷命令
CMD_INTERVAL_MS = 50


# -------------------------
# 外环 PD 参数
# 单位大致为：像素误差 -> RPM
# -------------------------
PAN_KP = 0.22
PAN_KD = 0.09

TILT_KP = 0.22
TILT_KD = 0.09

# 目标中心低通滤波，越小越平滑，越大响应越快
TARGET_FILTER_ALPHA = 0.35


# ============================================================
# UART 初始化
# ============================================================

# UART1：控制俯仰电机，ID = 0x02
err.check_raise(
    pinmap.set_pin_function("A19", "UART1_TX"),
    "A19 设置为 UART1_TX 失败"
)

err.check_raise(
    pinmap.set_pin_function("A18", "UART1_RX"),
    "A18 设置为 UART1_RX 失败"
)

tilt_uart = uart.UART(
    "/dev/ttyS1",
    BAUD_RATE
)


# UART0：控制水平电机，ID = 0x01
err.check_raise(
    pinmap.set_pin_function("A16", "UART0_TX"),
    "A16 设置为 UART0_TX 失败"
)

err.check_raise(
    pinmap.set_pin_function("A17", "UART0_RX"),
    "A17 设置为 UART0_RX 失败"
)

pan_uart = uart.UART(
    "/dev/ttyS0",
    BAUD_RATE
)

time.sleep_ms(200)


# ============================================================
# Emm42_V5.0 自定义串口协议
# ============================================================
def send_frame(serial_port, data, axis_name):
    """从指定串口发送一帧二进制电机命令。"""
    frame = bytes(data)
    serial_port.write(frame)

    print(
        "{} TX:".format(axis_name),
        " ".join("{:02X}".format(v) for v in frame)
    )


def motor_enable(serial_port, addr, axis_name, enable=True):
    # 地址 + F3 + AB + 使能状态 + 同步标志 + 校验
    send_frame(
        serial_port,
        [
            addr,
            0xF3,
            0xAB,
            0x01 if enable else 0x00,
            0x00,
            CHECKSUM
        ],
        axis_name
    )


def motor_speed(
    serial_port,
    addr,
    axis_name,
    direction,
    speed_rpm,
    accel=MOTOR_ACCEL
):
    """
    速度模式：
    地址 + F6 + 方向 + 速度高字节 + 速度低字节
         + 加速度 + 同步标志 + 校验
    """
    speed_rpm = max(0, min(5000, int(speed_rpm)))
    accel = max(0, min(255, int(accel)))

    send_frame(
        serial_port,
        [
            addr,
            0xF6,
            int(direction) & 0x01,
            (speed_rpm >> 8) & 0xFF,
            speed_rpm & 0xFF,
            accel,
            0x00,
            CHECKSUM
        ],
        axis_name
    )


def motor_stop(serial_port, addr, axis_name):
    # 地址 + FE + 98 + 同步标志 + 校验
    send_frame(
        serial_port,
        [
            addr,
            0xFE,
            0x98,
            0x00,
            CHECKSUM
        ],
        axis_name
    )


def stop_all():
    motor_stop(
        pan_uart,
        PAN_ADDR,
        "PAN/UART0"
    )

    motor_stop(
        tilt_uart,
        TILT_ADDR,
        "TILT/UART1"
    )


# ============================================================
# 控制与目标选择
# ============================================================
def clamp(value, low, high):
    return low if value < low else high if value > high else value


def _distance_sq(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def _corner_aspect_ratio(r):
    """
    根据四个角点计算长边/短边比例，比直接使用外接框 w/h
    更适合有少量旋转或透视的矩形。
    """
    try:
        pts = r.corners()
        if len(pts) != 4:
            return None

        side_sq = []
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % 4]
            d2 = (x2 - x1) ** 2 + (y2 - y1) ** 2
            side_sq.append(d2)

        long_side_sq = max(side_sq)
        short_side_sq = min(side_sq)

        if short_side_sq <= 0:
            return None

        return (long_side_sq / short_side_sq) ** 0.5

    except Exception:
        return None


def _quad_metrics(r):
    """计算矩形候选的角度、对边、面积填充率和透视长宽比。"""
    try:
        points = list(r.corners())
        if len(points) != 4:
            return None

        center_x = sum(p[0] for p in points) / 4.0
        center_y = sum(p[1] for p in points) / 4.0

        # 某些固件返回的角点顺序不一定稳定，重新按中心角排序。
        points.sort(
            key=lambda p: math.atan2(
                p[1] - center_y,
                p[0] - center_x
            )
        )

        sides = []
        for i in range(4):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % 4]
            sides.append(
                ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            )

        if min(sides) <= 1.0:
            return None

        # 计算四个角的余弦绝对值；理想直角的余弦为 0。
        corner_cos = []
        for i in range(4):
            p0 = points[(i - 1) % 4]
            p1 = points[i]
            p2 = points[(i + 1) % 4]

            ax = p0[0] - p1[0]
            ay = p0[1] - p1[1]
            bx = p2[0] - p1[0]
            by = p2[1] - p1[1]

            denominator = (ax * ax + ay * ay) ** 0.5
            denominator *= (bx * bx + by * by) ** 0.5
            if denominator <= 0:
                return None

            corner_cos.append(
                abs((ax * bx + ay * by) / denominator)
            )

        # 鞋带公式计算四边形面积。
        polygon_area = 0.0
        for i in range(4):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % 4]
            polygon_area += x1 * y2 - x2 * y1
        polygon_area = abs(polygon_area) * 0.5

        x, y, w, h = r.rect()
        bbox_area = max(1, w * h)

        # 使用两组对边的平均长度计算长宽比，较直接 max/min 更抗透视。
        pair_a = (sides[0] + sides[2]) * 0.5
        pair_b = (sides[1] + sides[3]) * 0.5
        long_side = max(pair_a, pair_b)
        short_side = min(pair_a, pair_b)

        return {
            "cx": int(center_x),
            "cy": int(center_y),
            "ratio": long_side / short_side,
            "opposite_ratio": min(sides[0], sides[2]) / max(sides[0], sides[2]),
            "opposite_ratio_2": min(sides[1], sides[3]) / max(sides[1], sides[3]),
            "max_corner_cos": max(corner_cos),
            "fill_ratio": polygon_area / bbox_area,
            "polygon_area": polygon_area,
        }

    except Exception:
        return None


def choose_target(rects, previous_center):
    """
    抗误识别目标选择：

    1. 限制宽、高、面积；
    2. 限制长宽比，排除截图中顶部的超宽矩形；
    3. 排除紧贴ROI边界的结构线；
    4. 已锁定后优先选择靠近上一帧中心的矩形；
    5. 首次锁定综合考虑比例、面积、边缘强度和画面中心距离。
    """
    valid = []

    roi_x, roi_y, roi_w, roi_h = ROI
    roi_right = roi_x + roi_w
    roi_bottom = roi_y + roi_h

    for r in rects:
        x, y, w, h = r.rect()
        area = w * h

        if w < MIN_RECT_W or h < MIN_RECT_H:
            continue

        if area < MIN_RECT_AREA or area > MAX_RECT_AREA:
            continue

        # 排除紧贴ROI四边的“环境大边框”
        if x <= roi_x + ROI_EDGE_MARGIN:
            continue
        if y <= roi_y + ROI_EDGE_MARGIN:
            continue
        if x + w >= roi_right - ROI_EDGE_MARGIN:
            continue
        if y + h >= roi_bottom - ROI_EDGE_MARGIN:
            continue

        metrics = _quad_metrics(r)
        if metrics is None:
            continue

        ratio = metrics["ratio"]

        if ratio < MIN_ASPECT_RATIO or ratio > MAX_ASPECT_RATIO:
            continue

        if metrics["max_corner_cos"] > MAX_CORNER_COS:
            continue

        if metrics["opposite_ratio"] < MIN_OPPOSITE_SIDE_RATIO:
            continue

        if metrics["opposite_ratio_2"] < MIN_OPPOSITE_SIDE_RATIO:
            continue

        if metrics["fill_ratio"] < MIN_QUAD_FILL_RATIO:
            continue

        # 角点平均位置比外接框中心更适合旋转/透视矩形。
        cx = metrics["cx"]
        cy = metrics["cy"]

        try:
            magnitude = float(r.magnitude())
        except Exception:
            magnitude = 0.0

        valid.append({
            "rect": r,
            "cx": cx,
            "cy": cy,
            "area": area,
            "ratio": ratio,
            "magnitude": magnitude,
            "polygon_area": metrics["polygon_area"],
            "fill_ratio": metrics["fill_ratio"],
        })

    if not valid:
        return None

    if previous_center is not None:
        # 已经锁定后，历史位置优先，避免跳到其他矩形。
        best = min(
            valid,
            key=lambda item:
                _distance_sq(
                    (item["cx"], item["cy"]),
                    previous_center
                )
                + 1800.0 * abs(
                    item["ratio"] - IDEAL_ASPECT_RATIO
                )
                - 0.015 * item["area"]
        )
    else:
        # 首次识别不再单纯选择“面积最大”。
        # 比例越接近目标、边缘越明显、面积适中越优先。
        image_center = (IMAGE_CENTER_X, IMAGE_CENTER_Y)

        best = min(
            valid,
            key=lambda item:
                2800.0 * abs(
                    item["ratio"] - IDEAL_ASPECT_RATIO
                )
                + 0.08 * _distance_sq(
                    (item["cx"], item["cy"]),
                    image_center
                )
                - 0.018 * item["area"]
                - 0.002 * item["magnitude"]
        )

    return (
        best["rect"],
        best["cx"],
        best["cy"],
        best["area"]
    )


def calc_axis_command(error, previous_error, kp, kd, reverse):
    """
    根据像素误差计算方向和速度。
    返回：
      None                  -> 已进入死区，需要停止
      (direction, rpm)      -> 速度模式命令
    """
    if abs(error) <= (DEAD_X if kp == PAN_KP else DEAD_Y):
        return None

    control = kp * error + kd * (error - previous_error)

    if reverse:
        control = -control

    direction = 1 if control > 0 else 0
    rpm = int(abs(control))

    rpm = clamp(rpm, MIN_RPM, MAX_RPM)
    return direction, rpm


# ============================================================
# 摄像头和显示
# ============================================================
cam = camera.Camera(IMG_W, IMG_H)
disp = display.Display()

IMAGE_CENTER_X = IMG_W // 2
IMAGE_CENTER_Y = IMG_H // 2


# ============================================================
# 主程序
# ============================================================
last_target_center = None
candidate_center = None
filtered_cx = None
filtered_cy = None

last_seen_ms = 0
last_cmd_ms = 0

previous_error_x = 0
previous_error_y = 0

last_pan_cmd = None
last_tilt_cmd = None

# 连续有效识别帧数，防止单帧误检直接驱动云台
lock_confirm_count = 0

try:
    print("使能水平轴和俯仰轴")

    # 水平轴：UART0，ID=0x01
    motor_enable(
        pan_uart,
        PAN_ADDR,
        "PAN/UART0",
        True
    )

    time.sleep_ms(30)

    # 俯仰轴：UART1，ID=0x02
    motor_enable(
        tilt_uart,
        TILT_ADDR,
        "TILT/UART1",
        True
    )
    time.sleep_ms(300)

    while not app.need_exit():
        img = cam.read()
        now = time.ticks_ms()

        # 画出检测ROI，ROI外不会参与矩形检测。
        img.draw_rect(
            ROI[0],
            ROI[1],
            ROI[2],
            ROI[3],
            image.COLOR_BLUE,
            thickness=1
        )

        # 画面中心和死区框
        img.draw_cross(
            IMAGE_CENTER_X,
            IMAGE_CENTER_Y,
            image.COLOR_RED,
            size=12,
            thickness=2
        )

        img.draw_rect(
            IMAGE_CENTER_X - DEAD_X,
            IMAGE_CENTER_Y - DEAD_Y,
            DEAD_X * 2,
            DEAD_Y * 2,
            image.COLOR_RED,
            thickness=1
        )

        # 检测图单独转灰度、二值化，显示图仍保留彩色。
        detect_img = img.to_format(
            image.Format.FMT_GRAYSCALE
        )

        detect_img.binary(
            [GRAY_THRESHOLD],
            invert=False,
            copy=False
        )

        if USE_CLOSE:
            detect_img.close(CLOSE_SIZE)

        rects = detect_img.find_rects(
            roi=ROI,
            threshold=RECT_THRESHOLD
        )

        target = choose_target(
            rects,
            last_target_center
        )

        if target is not None:
            r, raw_cx, raw_cy, area = target
            x, y, w, h = r.rect()

            # 候选目标必须连续且位置稳定，确认后才更新已锁定中心。
            if candidate_center is None:
                candidate_center = (raw_cx, raw_cy)
                lock_confirm_count = 1
            elif _distance_sq(
                (raw_cx, raw_cy),
                candidate_center
            ) > MAX_CENTER_JUMP * MAX_CENTER_JUMP:
                candidate_center = (raw_cx, raw_cy)
                lock_confirm_count = 1
            else:
                candidate_center = (raw_cx, raw_cy)
                lock_confirm_count += 1

            last_seen_ms = now

            if lock_confirm_count >= LOCK_CONFIRM_FRAMES:
                last_target_center = candidate_center

            # 对目标中心做低通滤波，减小识别抖动
            if filtered_cx is None:
                filtered_cx = float(raw_cx)
                filtered_cy = float(raw_cy)
            else:
                a = TARGET_FILTER_ALPHA
                filtered_cx = (1.0 - a) * filtered_cx + a * raw_cx
                filtered_cy = (1.0 - a) * filtered_cy + a * raw_cy

            target_cx = int(filtered_cx)
            target_cy = int(filtered_cy)

            error_x = target_cx - IMAGE_CENTER_X
            error_y = target_cy - IMAGE_CENTER_Y

            # 显示锁定对象
            img.draw_rect(
                x, y, w, h,
                image.COLOR_GREEN,
                thickness=2
            )

            img.draw_cross(
                target_cx,
                target_cy,
                image.COLOR_GREEN,
                size=10,
                thickness=2
            )

            img.draw_line(
                IMAGE_CENTER_X,
                IMAGE_CENTER_Y,
                target_cx,
                target_cy,
                image.COLOR_YELLOW,
                thickness=2
            )

            img.draw_string(
                4,
                4,
                "EX:{} EY:{}".format(error_x, error_y),
                image.COLOR_GREEN
            )

            img.draw_string(
                4,
                20,
                "LOCK:{}/{}".format(
                    lock_confirm_count,
                    LOCK_CONFIRM_FRAMES
                ),
                image.COLOR_GREEN
            )

            # 必须连续确认足够帧后，才允许驱动电机。
            can_track = (
                lock_confirm_count
                >= LOCK_CONFIRM_FRAMES
            )

            # 固定频率更新电机命令
            if (
                can_track
                and now - last_cmd_ms >= CMD_INTERVAL_MS
            ):
                last_cmd_ms = now

                pan_cmd = calc_axis_command(
                    error_x,
                    previous_error_x,
                    PAN_KP,
                    PAN_KD,
                    PAN_REVERSE
                )

                tilt_cmd = calc_axis_command(
                    error_y,
                    previous_error_y,
                    TILT_KP,
                    TILT_KD,
                    TILT_REVERSE
                )

                # 水平轴
                if pan_cmd is None:
                    if last_pan_cmd != ("stop",):
                        motor_stop(
                            pan_uart,
                            PAN_ADDR,
                            "PAN/UART0"
                        )
                        last_pan_cmd = ("stop",)
                else:
                    pan_dir, pan_rpm = pan_cmd
                    current_cmd = ("speed", pan_dir, pan_rpm)

                    if current_cmd != last_pan_cmd:
                        motor_speed(
                            pan_uart,
                            PAN_ADDR,
                            "PAN/UART0",
                            pan_dir,
                            pan_rpm
                        )
                        last_pan_cmd = current_cmd

                # 俯仰轴
                if tilt_cmd is None:
                    if last_tilt_cmd != ("stop",):
                        motor_stop(
                            tilt_uart,
                            TILT_ADDR,
                            "TILT/UART1"
                        )
                        last_tilt_cmd = ("stop",)
                else:
                    tilt_dir, tilt_rpm = tilt_cmd
                    current_cmd = ("speed", tilt_dir, tilt_rpm)

                    if current_cmd != last_tilt_cmd:
                        motor_speed(
                            tilt_uart,
                            TILT_ADDR,
                            "TILT/UART1",
                            tilt_dir,
                            tilt_rpm
                        )
                        last_tilt_cmd = current_cmd

                previous_error_x = error_x
                previous_error_y = error_y

        else:
            # 短暂漏检时保留确认状态，避免目标边缘模糊导致反复重锁。
            if (
                last_seen_ms == 0
                or now - last_seen_ms >= LOST_STOP_MS
            ):
                lock_confirm_count = 0
                candidate_center = None

            img.draw_string(
                4,
                4,
                "NO VALID RECT",
                image.COLOR_RED
            )

            # 目标短暂消失时先保持，超过时间再停止
            if last_seen_ms != 0 and now - last_seen_ms >= LOST_STOP_MS:
                if last_pan_cmd != ("stop",):
                    motor_stop(
                        pan_uart,
                        PAN_ADDR,
                        "PAN/UART0"
                    )
                    last_pan_cmd = ("stop",)

                if last_tilt_cmd != ("stop",):
                    motor_stop(
                        tilt_uart,
                        TILT_ADDR,
                        "TILT/UART1"
                    )
                    last_tilt_cmd = ("stop",)

                filtered_cx = None
                filtered_cy = None
                last_target_center = None
                candidate_center = None
                lock_confirm_count = 0
                previous_error_x = 0
                previous_error_y = 0

        disp.show(img)

except Exception as e:
    print("矩形锁定程序异常：", e)

finally:
    try:
        stop_all()
    except Exception:
        pass
