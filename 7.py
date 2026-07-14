from maix import camera, display, image, app, nn, uart, pinmap, err, time


# ============================================================
# MaixCAM + YOLOv5 + 单TX控制两个 Emm42_V5.0
#
# 接线：
#   MaixCAM A19 (UART1_TX)
#              ├──> 水平驱动器 RX，电机 ID = 0x01
#              └──> 俯仰驱动器 RX，电机 ID = 0x02
#   MaixCAM GND ───> 两个驱动器 GND
#
# 只进行单向发送：两个驱动器的 TX 都不要接到 MaixCAM。
# 两个模型文件必须放在同一个目录：
#   /root/models/model_3356.mud
#   /root/models/model_3356.cvimodel
# ============================================================


# -------------------------
# YOLO 参数
# -------------------------
MODEL_PATH = "/root/model/model_3356.mud"
CONF_THRESHOLD = 0.50
IOU_THRESHOLD = 0.45
MIN_BOX_AREA = 500

# 控制瞄准点位于检测框中心向下15%的位置。
# 改为0就是正中心，正数向下，负数向上。
AIM_OFFSET_Y_RATIO = -0.25

# True 可提高 YOLO 帧率，但检测结果会有一帧延迟。
DUAL_BUFFER = True

# 性能优先设置：摄像头请求60fps，预览每3帧刷新一次。
# YOLO和电机控制仍然每帧运行。
CAMERA_FPS = 60
CAMERA_BUFF_NUM = 1
DISPLAY_EVERY_N_FRAMES = 3


# -------------------------
# 单串口和电机参数
# -------------------------
BAUD_RATE = 115200
CHECKSUM = 0x6B

PAN_ADDR = 0x01       # 水平轴电机 ID
TILT_ADDR = 0x02      # 俯仰轴电机 ID

# 某一轴运动方向相反时，只修改对应参数。
PAN_REVERSE = True
TILT_REVERSE = True

MOTOR_ACCEL = 0
MIN_RPM = 5
MAX_RPM = 120

# 正常使用关闭，避免大量串口打印降低FPS。
DEBUG_UART = False


# -------------------------
# 追踪控制参数
# -------------------------
DEAD_X = 4
DEAD_Y = 7

# 水平轴使用分段KP：远处快速追，近处降低增益防止左右摆动。
PAN_KP_NEAR = 0.65
PAN_KP_MID = 0.80
PAN_KP_FAR = 1
PAN_MID_ERROR = 15
PAN_FAR_ERROR = 40

PAN_KI = 0.33
PAN_KD = 0.02

# 根据目标中心移动速度向前预测若干帧，补偿YOLO和双缓冲延迟。
PAN_PREDICT_FRAMES = 1.3
PAN_VELOCITY_FILTER_ALPHA = 0.55

TILT_KP = 0.30
TILT_KI = 0.06
TILT_KD = 0.08

# 积分单位约为“像素·秒”，限制积分最大影响，防止积分饱和。
INTEGRAL_LIMIT = 120.0

# 越大响应越快，越小越平滑。
TARGET_FILTER_ALPHA = 0.90

LOCK_CONFIRM_FRAMES = 1
MISS_TOLERANCE_FRAMES = 2
LOST_STOP_MS = 220
CMD_INTERVAL_MS = 20


# ============================================================
# UART1 初始化：只启用 A19 TX
# ============================================================
err.check_raise(
    pinmap.set_pin_function("A19", "UART1_TX"),
    "A19 设置为 UART1_TX 失败"
)

motor_uart = uart.UART(
    "/dev/ttyS1",
    BAUD_RATE
)

time.sleep_ms(200)


# ============================================================
# Emm42_V5.0 串口协议
# ============================================================
def send_frame(data, axis_name):
    frame = bytes(data)
    motor_uart.write(frame)

    if DEBUG_UART:
        print(
            "{} TX:".format(axis_name),
            " ".join("{:02X}".format(v) for v in frame)
        )


def motor_enable(addr, axis_name, enable=True):
    # 地址 + F3 + AB + 使能状态 + 同步标志 + 校验
    send_frame(
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


def motor_speed(addr, axis_name, direction, speed_rpm):
    # 地址 + F6 + 方向 + 速度高字节 + 速度低字节
    #      + 加速度 + 同步标志 + 校验
    speed_rpm = max(0, min(5000, int(speed_rpm)))
    accel = max(0, min(255, int(MOTOR_ACCEL)))

    send_frame(
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


def motor_stop(addr, axis_name):
    # 地址 + FE + 98 + 同步标志 + 校验
    send_frame(
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
    motor_stop(PAN_ADDR, "PAN/ID01")
    time.sleep_ms(5)
    motor_stop(TILT_ADDR, "TILT/ID02")


# ============================================================
# 目标选择与控制
# ============================================================
def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def distance_sq(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    return dx * dx + dy * dy


def get_pan_kp(error):
    """水平轴按误差大小自动选择KP。"""
    abs_error = abs(error)

    if abs_error < PAN_MID_ERROR:
        return PAN_KP_NEAR

    if abs_error < PAN_FAR_ERROR:
        return PAN_KP_MID

    return PAN_KP_FAR


def choose_yolo_target(objs, previous_center):
    """
    模型只有 rect 类：
    - 初次识别优先选择置信度高且面积大的框；
    - 已锁定后优先选择靠近上一帧位置的框，防止跳目标。
    """
    valid = []

    for obj in objs:
        area = obj.w * obj.h
        if area < MIN_BOX_AREA:
            continue

        cx = obj.x + obj.w // 2
        cy = (
            obj.y
            + obj.h // 2
            + int(obj.h * AIM_OFFSET_Y_RATIO)
        )

        valid.append((obj, cx, cy, area))

    if not valid:
        return None

    if previous_center is None:
        return max(
            valid,
            key=lambda item: item[0].score * 100000 + item[3]
        )

    px, py = previous_center
    return min(
        valid,
        key=lambda item:
            distance_sq(item[1], item[2], px, py)
            - item[0].score * 2500
    )


def calc_axis_command(
    error,
    previous_error,
    integral,
    kp,
    ki,
    kd,
    dead_zone,
    reverse,
    dt_s
):
    if abs(error) <= dead_zone:
        # 进入中心死区后立即清除历史积分。
        return None, 0.0

    # 误差跨过零点，说明已经越过目标，清除旧方向的积分。
    if previous_error != 0 and error * previous_error < 0:
        integral = 0.0

    old_integral = integral
    candidate_integral = clamp(
        integral + error * dt_s,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
    )

    # 先计算候选输出。如果已经达到最高转速，就暂停继续积分。
    candidate_control = (
        kp * error
        + ki * candidate_integral
        + kd * (error - previous_error)
    )

    if abs(candidate_control) < MAX_RPM:
        integral = candidate_integral
    else:
        integral = old_integral

    control = (
        kp * error
        + ki * integral
        + kd * (error - previous_error)
    )

    if reverse:
        control = -control

    direction = 1 if control > 0 else 0
    rpm = clamp(int(abs(control)), MIN_RPM, MAX_RPM)
    return (direction, rpm), integral


# ============================================================
# YOLO、摄像头和显示初始化
# ============================================================
detector = nn.YOLOv5(
    model=MODEL_PATH,
    dual_buff=DUAL_BUFFER
)

IMG_W = detector.input_width()
IMG_H = detector.input_height()
IMAGE_CENTER_X = IMG_W // 2
IMAGE_CENTER_Y = IMG_H // 2

cam = camera.Camera(
    IMG_W,
    IMG_H,
    detector.input_format(),
    fps=CAMERA_FPS,
    buff_num=CAMERA_BUFF_NUM
)

disp = display.Display()


# ============================================================
# 主程序
# ============================================================
last_target_center = None
filtered_cx = None
filtered_cy = None

last_seen_ms = 0
last_cmd_ms = 0

previous_error_x = 0
previous_error_y = 0
pan_integral = 0.0
tilt_integral = 0.0
last_control_ms = 0

previous_target_cx = None
filtered_velocity_x = 0.0

last_pan_cmd = None
last_tilt_cmd = None

lock_confirm_count = 0
miss_count = 0

fps_frames = 0
fps_value = 0
last_fps_ms = time.ticks_ms()
display_frame_count = 0


try:
    print("YOLOv5模型：", MODEL_PATH)
    print("输入尺寸：{}x{}".format(IMG_W, IMG_H))
    print("单串口：A19/UART1_TX，水平ID01，俯仰ID02")

    motor_enable(PAN_ADDR, "PAN/ID01", True)
    time.sleep_ms(20)
    motor_enable(TILT_ADDR, "TILT/ID02", True)
    time.sleep_ms(250)

    while not app.need_exit():
        img = cam.read()
        now = time.ticks_ms()

        display_frame_count += 1
        show_frame = (
            display_frame_count >= DISPLAY_EVERY_N_FRAMES
        )
        if show_frame:
            display_frame_count = 0

        objs = detector.detect(
            img,
            conf_th=CONF_THRESHOLD,
            iou_th=IOU_THRESHOLD
        )

        target = choose_yolo_target(
            objs,
            last_target_center
        )

        # 只在需要刷新预览的帧上绘图，减少CPU开销。
        if show_frame:
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

        if target is not None:
            obj, raw_cx, raw_cy, area = target

            last_seen_ms = now
            miss_count = 0
            lock_confirm_count = min(
                lock_confirm_count + 1,
                LOCK_CONFIRM_FRAMES
            )

            last_target_center = (raw_cx, raw_cy)

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

            # 估计目标在画面中的水平移动速度，并预测下一帧位置。
            if previous_target_cx is None:
                raw_velocity_x = 0.0
            else:
                raw_velocity_x = target_cx - previous_target_cx

            previous_target_cx = target_cx

            va = PAN_VELOCITY_FILTER_ALPHA
            filtered_velocity_x = (
                (1.0 - va) * filtered_velocity_x
                + va * raw_velocity_x
            )

            control_error_x = int(
                error_x
                + filtered_velocity_x * PAN_PREDICT_FRAMES
            )

            if show_frame:
                img.draw_rect(
                    obj.x,
                    obj.y,
                    obj.w,
                    obj.h,
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

                label = detector.labels[obj.class_id]
                img.draw_string(
                    obj.x,
                    max(0, obj.y - 16),
                    "{} {:.2f}".format(label, obj.score),
                    image.COLOR_GREEN
                )

                img.draw_string(
                    4,
                    4,
                    "EX:{} PX:{} EY:{}".format(
                        error_x,
                        control_error_x,
                        error_y
                    ),
                    image.COLOR_GREEN
                )

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
                        0.100
                    )
                last_control_ms = now

                current_pan_kp = get_pan_kp(control_error_x)

                pan_cmd, pan_integral = calc_axis_command(
                    control_error_x,
                    previous_error_x,
                    pan_integral,
                    current_pan_kp,
                    PAN_KI,
                    PAN_KD,
                    DEAD_X,
                    PAN_REVERSE,
                    dt_s
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
                    dt_s
                )

                # 水平轴 ID 01
                if pan_cmd is None:
                    if last_pan_cmd != ("stop",):
                        motor_stop(PAN_ADDR, "PAN/ID01")
                        last_pan_cmd = ("stop",)
                else:
                    pan_dir, pan_rpm = pan_cmd
                    current_cmd = ("speed", pan_dir, pan_rpm)

                    if current_cmd != last_pan_cmd:
                        motor_speed(
                            PAN_ADDR,
                            "PAN/ID01",
                            pan_dir,
                            pan_rpm
                        )
                        last_pan_cmd = current_cmd

                # 俯仰轴 ID 02
                if tilt_cmd is None:
                    if last_tilt_cmd != ("stop",):
                        motor_stop(TILT_ADDR, "TILT/ID02")
                        last_tilt_cmd = ("stop",)
                else:
                    tilt_dir, tilt_rpm = tilt_cmd
                    current_cmd = ("speed", tilt_dir, tilt_rpm)

                    if current_cmd != last_tilt_cmd:
                        motor_speed(
                            TILT_ADDR,
                            "TILT/ID02",
                            tilt_dir,
                            tilt_rpm
                        )
                        last_tilt_cmd = current_cmd

                previous_error_x = control_error_x
                previous_error_y = error_y

        else:
            miss_count += 1

            if miss_count > MISS_TOLERANCE_FRAMES:
                lock_confirm_count = 0
                pan_integral = 0.0
                tilt_integral = 0.0
                last_control_ms = 0
                previous_target_cx = None
                filtered_velocity_x = 0.0

            if show_frame:
                img.draw_string(
                    4,
                    4,
                    "NO RECT  MISS:{}".format(miss_count),
                    image.COLOR_RED
                )

            if (
                last_seen_ms != 0
                and now - last_seen_ms >= LOST_STOP_MS
            ):
                if last_pan_cmd != ("stop",):
                    motor_stop(PAN_ADDR, "PAN/ID01")
                    last_pan_cmd = ("stop",)

                if last_tilt_cmd != ("stop",):
                    motor_stop(TILT_ADDR, "TILT/ID02")
                    last_tilt_cmd = ("stop",)

                filtered_cx = None
                filtered_cy = None
                last_target_center = None
                lock_confirm_count = 0
                miss_count = 0
                previous_error_x = 0
                previous_error_y = 0
                pan_integral = 0.0
                tilt_integral = 0.0
                last_control_ms = 0
                previous_target_cx = None
                filtered_velocity_x = 0.0

        fps_frames += 1
        if now - last_fps_ms >= 1000:
            elapsed_ms = now - last_fps_ms
            fps_value = int(fps_frames * 1000 / elapsed_ms)
            fps_frames = 0
            last_fps_ms = now
            print("FPS:", fps_value)

        if show_frame:
            img.draw_string(
                IMG_W - 62,
                4,
                "FPS:{}".format(fps_value),
                image.COLOR_YELLOW
            )

            disp.show(img)

except Exception as e:
    print("YOLO双电机追踪异常：", e)

finally:
    try:
        stop_all()
    except Exception:
        pass