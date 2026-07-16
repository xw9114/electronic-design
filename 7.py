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
TRACK_CONF_THRESHOLD = 0.56
SUPER_STABLE_CONF_THRESHOLD = 0.64
IOU_THRESHOLD = 0.45
TRACK_IOU_THRESHOLD = 0.38
SUPER_STABLE_IOU_THRESHOLD = 0.32
MIN_BOX_AREA = 500
MAX_BOX_AREA_RATIO = 0.72
MIN_BOX_ASPECT = 0.25
MAX_BOX_ASPECT = 4.00

# Locked target size should change gradually. Sudden area jumps are usually
# another object or a false detection; high confidence can still re-acquire.
TRACK_AREA_CHANGE_MIN = 0.35
TRACK_AREA_CHANGE_MAX = 2.80
TRACK_AREA_REACQUIRE_SCORE = 0.82
TRACK_ASPECT_CHANGE_MIN = 0.45
TRACK_ASPECT_CHANGE_MAX = 2.20
TRACK_ASPECT_REACQUIRE_SCORE = 0.84
TRACK_ASPECT_SIM_WEIGHT = 1700.0

# 控制瞄准点位于检测框中心向下15%的位置。
# 改为0就是正中心，正数向下，负数向上。
AIM_OFFSET_Y_RATIO = -0.25

# True 可提高 YOLO 帧率，但检测结果会有一帧延迟。
DUAL_BUFFER = True

# Performance mode: request 60 FPS from camera.
# Display refresh and YOLO inference cadence are controlled separately.
CAMERA_FPS = 60
CAMERA_BUFF_NUM = 1

# Display is expensive; reduce refresh when the target is stable.
DISPLAY_EVERY_N_FRAMES = 4
DISPLAY_STABLE_EVERY_N_FRAMES = 5
DISPLAY_SUPER_STABLE_EVERY_N_FRAMES = 6

# Run YOLO every frame before lock; after stable lock, use adaptive 2/3-frame cadence.
# Skipped frames use velocity prediction for control and do not count as detection misses.
DETECT_EVERY_N_FRAMES = 1
LOCKED_DETECT_EVERY_N_FRAMES = 2
SUPER_STABLE_DETECT_EVERY_N_FRAMES = 3
STABLE_SKIP_CONFIRM_FRAMES = 3
SUPER_STABLE_CONFIRM_FRAMES = 6

# Reject/penalize sudden target jumps after lock to avoid switching targets.
MAX_TRACK_JUMP = 90

# Only skip YOLO when the target is already close enough to center.
# Far targets still run YOLO every frame for accuracy.
SKIP_MAX_ERROR_X = 55
SKIP_MAX_ERROR_Y = 45

# Skip YOLO only when detection confidence is high and target movement is slow.
# This keeps FPS high in stable lock, but preserves accuracy during fast motion.
SKIP_MIN_SCORE = 0.72
SKIP_MAX_VELOCITY_X = 10.0
SKIP_MAX_VELOCITY_Y = 8.0
SUPER_SKIP_MIN_SCORE = 0.86
SUPER_SKIP_MAX_VELOCITY_X = 4.0
SUPER_SKIP_MAX_VELOCITY_Y = 3.0

# If a locked target suddenly jumps outside MAX_TRACK_JUMP, only accept it
# when the model is very confident; otherwise count it as a miss.
REACQUIRE_MIN_SCORE = 0.78
MEASURE_JUMP_REJECT_X = 72
MEASURE_JUMP_REJECT_Y = 54
MEASURE_JUMP_REACQUIRE_SCORE = 0.84

# Target scoring weights: distance is bad; confidence and area are good.
TRACK_SCORE_WEIGHT = 3500
TRACK_AREA_WEIGHT = 0.025
TRACK_AREA_SIM_WEIGHT = 2600.0
BOOT_SCORE_WEIGHT = 120000
BOOT_AREA_WEIGHT = 1.0

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

# Quantize motor speed commands to reduce UART writes and near-center jitter.
RPM_QUANT_STEP = 3
CMD_RPM_CHANGE_MIN = 3

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
TILT_PREDICT_FRAMES = 0.8
MAX_PREDICT_OFFSET_X = 35
MAX_PREDICT_OFFSET_Y = 26
MAX_RAW_VELOCITY_X = 26.0
MAX_RAW_VELOCITY_Y = 20.0
MAX_FILTERED_VELOCITY_X = 18.0
MAX_FILTERED_VELOCITY_Y = 14.0
PAN_VELOCITY_FILTER_ALPHA = 0.55
TILT_VELOCITY_FILTER_ALPHA = 0.45

TILT_KP = 0.30
TILT_KI = 0.06
TILT_KD = 0.08

# 积分单位约为“像素·秒”，限制积分最大影响，防止积分饱和。
INTEGRAL_LIMIT = 120.0

# Target center adaptive low-pass filter.
# Near center: lower alpha for stability. Far/fast target: higher alpha for response.
TARGET_FILTER_ALPHA_NEAR = 0.45
TARGET_FILTER_ALPHA_MID = 0.68
TARGET_FILTER_ALPHA_FAST = 0.90
TARGET_FILTER_NEAR_ERROR = 12
TARGET_FILTER_MID_ERROR = 38
TARGET_FILTER_FAST_JUMP = 18

LOCK_CONFIRM_FRAMES = 2
MISS_TOLERANCE_FRAMES = 2
MISS_HOLD_MIN_SCORE = 0.70
MISS_HOLD_MAX_VELOCITY_X = 14.0
MISS_HOLD_MAX_VELOCITY_Y = 10.0
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


def quantize_rpm(rpm):
    step = max(1, int(RPM_QUANT_STEP))
    rpm = int(rpm)
    rpm = int((rpm + step // 2) // step) * step
    return clamp(rpm, MIN_RPM, MAX_RPM)


def should_send_speed_command(last_cmd, direction, rpm):
    if last_cmd is None or last_cmd[0] != "speed":
        return True

    _, last_direction, last_rpm = last_cmd
    if direction != last_direction:
        return True

    return abs(rpm - last_rpm) >= CMD_RPM_CHANGE_MIN


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


def get_target_filter_alpha(raw_cx, raw_cy, filtered_cx, filtered_cy):
    """Adaptive center smoothing: stable near center, fast when far or jumping."""
    if filtered_cx is None or filtered_cy is None:
        return TARGET_FILTER_ALPHA_FAST

    raw_error = max(
        abs(raw_cx - IMAGE_CENTER_X),
        abs(raw_cy - IMAGE_CENTER_Y)
    )
    raw_jump = max(
        abs(raw_cx - filtered_cx),
        abs(raw_cy - filtered_cy)
    )

    if raw_jump >= TARGET_FILTER_FAST_JUMP:
        return TARGET_FILTER_ALPHA_FAST

    if raw_error <= TARGET_FILTER_NEAR_ERROR:
        return TARGET_FILTER_ALPHA_NEAR

    if raw_error <= TARGET_FILTER_MID_ERROR:
        return TARGET_FILTER_ALPHA_MID

    return TARGET_FILTER_ALPHA_FAST


def choose_yolo_target(objs, previous_center, previous_area, previous_aspect):
    """
    Fast and stable target picker.

    Algorithm changes for FPS + accuracy:
    - Single pass: no list/dict allocation for every detection.
    - Boot: prefer confidence first, area second.
    - Locked: prefer targets near the previous center and with similar size;
      only high-confidence far/size jumps are allowed as re-acquisition.
    """
    best_obj = None
    best_cx = 0
    best_cy = 0
    best_area = 0
    best_aspect = 0.0
    best_score_value = 0.0

    px = 0
    py = 0
    has_previous = previous_center is not None
    has_previous_area = previous_area is not None and previous_area > 0
    has_previous_aspect = previous_aspect is not None and previous_aspect > 0
    jump_limit_sq = MAX_TRACK_JUMP * MAX_TRACK_JUMP
    max_box_area = int(IMG_W * IMG_H * MAX_BOX_AREA_RATIO)

    if has_previous:
        px, py = previous_center
        best_rank = 1000000000.0
        far_obj = None
        far_cx = 0
        far_cy = 0
        far_area = 0
        far_aspect = 0.0
        far_score_value = 0.0
        far_rank = 1000000000.0
    else:
        best_rank = -1000000000.0

    for obj in objs:
        area = obj.w * obj.h
        if area < MIN_BOX_AREA or area > max_box_area:
            continue

        if obj.h <= 0:
            continue
        aspect = float(obj.w) / float(obj.h)
        if aspect < MIN_BOX_ASPECT or aspect > MAX_BOX_ASPECT:
            continue

        score_value = float(obj.score)
        cx = obj.x + obj.w // 2
        cy = (
            obj.y
            + obj.h // 2
            + int(obj.h * AIM_OFFSET_Y_RATIO)
        )

        if has_previous:
            area_similarity_penalty = 0.0
            if has_previous_area:
                area_change = float(area) / float(previous_area)
                area_jump = (
                    area_change < TRACK_AREA_CHANGE_MIN
                    or area_change > TRACK_AREA_CHANGE_MAX
                )
                if area_jump and score_value < TRACK_AREA_REACQUIRE_SCORE:
                    continue
                area_similarity_penalty = (
                    abs(area_change - 1.0) * TRACK_AREA_SIM_WEIGHT
                )

            aspect_similarity_penalty = 0.0
            if has_previous_aspect:
                aspect_change = aspect / previous_aspect
                aspect_jump = (
                    aspect_change < TRACK_ASPECT_CHANGE_MIN
                    or aspect_change > TRACK_ASPECT_CHANGE_MAX
                )
                if aspect_jump and score_value < TRACK_ASPECT_REACQUIRE_SCORE:
                    continue
                aspect_similarity_penalty = (
                    abs(aspect_change - 1.0) * TRACK_ASPECT_SIM_WEIGHT
                )

            d2 = distance_sq(cx, cy, px, py)
            rank = (
                d2
                + area_similarity_penalty
                + aspect_similarity_penalty
                - score_value * TRACK_SCORE_WEIGHT
                - area * TRACK_AREA_WEIGHT
            )

            if d2 <= jump_limit_sq:
                if rank < best_rank:
                    best_rank = rank
                    best_obj = obj
                    best_cx = cx
                    best_cy = cy
                    best_area = area
                    best_aspect = aspect
                    best_score_value = score_value
            elif score_value >= REACQUIRE_MIN_SCORE:
                # Far re-acquire candidate: allowed only if confidence is high.
                if rank < far_rank:
                    far_rank = rank
                    far_obj = obj
                    far_cx = cx
                    far_cy = cy
                    far_area = area
                    far_aspect = aspect
                    far_score_value = score_value
        else:
            rank = score_value * BOOT_SCORE_WEIGHT + area * BOOT_AREA_WEIGHT
            if rank > best_rank:
                best_rank = rank
                best_obj = obj
                best_cx = cx
                best_cy = cy
                best_area = area
                best_aspect = aspect
                best_score_value = score_value

    if best_obj is None and has_previous:
        best_obj = far_obj
        best_cx = far_cx
        best_cy = far_cy
        best_area = far_area
        best_aspect = far_aspect
        best_score_value = far_score_value

    if best_obj is None:
        return None

    return (
        best_obj,
        best_cx,
        best_cy,
        best_area,
        best_aspect,
        best_score_value,
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
    rpm = quantize_rpm(abs(control))
    return (direction, rpm), integral





def predict_target_state(base_cx, base_cy, velocity_x, velocity_y, frames):
    """Predict target center and error for frames without usable YOLO output."""
    frames = max(1, int(frames))
    offset_x = clamp(
        velocity_x * PAN_PREDICT_FRAMES * frames,
        -MAX_PREDICT_OFFSET_X,
        MAX_PREDICT_OFFSET_X
    )
    offset_y = clamp(
        velocity_y * TILT_PREDICT_FRAMES * frames,
        -MAX_PREDICT_OFFSET_Y,
        MAX_PREDICT_OFFSET_Y
    )

    predict_cx = int(
        clamp(
            base_cx + offset_x,
            0,
            IMG_W - 1
        )
    )
    predict_cy = int(
        clamp(
            base_cy + offset_y,
            0,
            IMG_H - 1
        )
    )
    return (
        predict_cx,
        predict_cy,
        predict_cx - IMAGE_CENTER_X,
        predict_cy - IMAGE_CENTER_Y,
    )


def handle_predict_hold(
    now,
    img,
    show_frame,
    label,
    filtered_cx,
    filtered_cy,
    filtered_velocity_x,
    filtered_velocity_y,
    predict_only_count,
    detect_period,
    previous_error_x,
    previous_error_y,
    pan_integral,
    tilt_integral,
    last_control_ms,
    last_cmd_ms,
    last_pan_cmd,
    last_tilt_cmd
):
    predict_only_count = min(
        predict_only_count + 1,
        max(1, detect_period)
    )
    (
        predict_cx,
        predict_cy,
        predict_error_x,
        predict_error_y,
    ) = predict_target_state(
        filtered_cx,
        filtered_cy,
        filtered_velocity_x,
        filtered_velocity_y,
        predict_only_count
    )

    (
        previous_error_x,
        previous_error_y,
        pan_integral,
        tilt_integral,
        last_control_ms,
        last_cmd_ms,
        last_pan_cmd,
        last_tilt_cmd,
    ) = update_motor_control(
        now,
        predict_error_x,
        predict_error_y,
        previous_error_x,
        previous_error_y,
        pan_integral,
        tilt_integral,
        last_control_ms,
        last_cmd_ms,
        last_pan_cmd,
        last_tilt_cmd
    )

    if show_frame:
        img.draw_cross(
            predict_cx,
            predict_cy,
            image.COLOR_YELLOW,
            size=8,
            thickness=1
        )
        img.draw_string(
            4,
            18,
            "{}:{} {}".format(
                label,
                predict_only_count,
                predict_error_x
            ),
            image.COLOR_YELLOW
        )

    return (
        predict_only_count,
        previous_error_x,
        previous_error_y,
        pan_integral,
        tilt_integral,
        last_control_ms,
        last_cmd_ms,
        last_pan_cmd,
        last_tilt_cmd,
    )

def update_motor_control(
    now,
    error_x,
    error_y,
    previous_error_x,
    previous_error_y,
    pan_integral,
    tilt_integral,
    last_control_ms,
    last_cmd_ms,
    last_pan_cmd,
    last_tilt_cmd
):
    """
    Rate-limited two-axis PID update.
    Shared by real YOLO frames and prediction-only frames so skipped
    inference frames still improve control instead of only holding old speed.
    """
    if now - last_cmd_ms < CMD_INTERVAL_MS:
        return (
            previous_error_x,
            previous_error_y,
            pan_integral,
            tilt_integral,
            last_control_ms,
            last_cmd_ms,
            last_pan_cmd,
            last_tilt_cmd,
        )

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

    current_pan_kp = get_pan_kp(error_x)

    pan_cmd, pan_integral = calc_axis_command(
        error_x,
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

    if pan_cmd is None:
        if last_pan_cmd != ("stop",):
            motor_stop(PAN_ADDR, "PAN/ID01")
            last_pan_cmd = ("stop",)
    else:
        pan_dir, pan_rpm = pan_cmd
        current_cmd = ("speed", pan_dir, pan_rpm)

        if should_send_speed_command(last_pan_cmd, pan_dir, pan_rpm):
            motor_speed(
                PAN_ADDR,
                "PAN/ID01",
                pan_dir,
                pan_rpm
            )
            last_pan_cmd = current_cmd

    if tilt_cmd is None:
        if last_tilt_cmd != ("stop",):
            motor_stop(TILT_ADDR, "TILT/ID02")
            last_tilt_cmd = ("stop",)
    else:
        tilt_dir, tilt_rpm = tilt_cmd
        current_cmd = ("speed", tilt_dir, tilt_rpm)

        if should_send_speed_command(last_tilt_cmd, tilt_dir, tilt_rpm):
            motor_speed(
                TILT_ADDR,
                "TILT/ID02",
                tilt_dir,
                tilt_rpm
            )
            last_tilt_cmd = current_cmd

    previous_error_x = error_x
    previous_error_y = error_y

    return (
        previous_error_x,
        previous_error_y,
        pan_integral,
        tilt_integral,
        last_control_ms,
        last_cmd_ms,
        last_pan_cmd,
        last_tilt_cmd,
    )
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
last_target_score = 0.0
last_target_area = 0
last_target_aspect = 0.0
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
previous_target_cy = None
filtered_velocity_x = 0.0
filtered_velocity_y = 0.0

last_pan_cmd = None
last_tilt_cmd = None

lock_confirm_count = 0
stable_detect_count = 0
miss_count = 0

fps_frames = 0
fps_value = 0
last_fps_ms = time.ticks_ms()
display_frame_count = 0
detect_frame_count = 0
predict_only_count = 0


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
        detect_frame_count += 1

        locked_stable = (
            lock_confirm_count >= LOCK_CONFIRM_FRAMES
            and stable_detect_count >= STABLE_SKIP_CONFIRM_FRAMES
            and filtered_cx is not None
            and miss_count == 0
            and last_target_score >= SKIP_MIN_SCORE
            and abs(previous_error_x) <= SKIP_MAX_ERROR_X
            and abs(previous_error_y) <= SKIP_MAX_ERROR_Y
            and abs(filtered_velocity_x) <= SKIP_MAX_VELOCITY_X
            and abs(filtered_velocity_y) <= SKIP_MAX_VELOCITY_Y
        )
        super_stable = (
            locked_stable
            and stable_detect_count >= SUPER_STABLE_CONFIRM_FRAMES
            and last_target_score >= SUPER_SKIP_MIN_SCORE
            and abs(filtered_velocity_x) <= SUPER_SKIP_MAX_VELOCITY_X
            and abs(filtered_velocity_y) <= SUPER_SKIP_MAX_VELOCITY_Y
        )

        if super_stable:
            display_period = DISPLAY_SUPER_STABLE_EVERY_N_FRAMES
        elif locked_stable:
            display_period = DISPLAY_STABLE_EVERY_N_FRAMES
        else:
            display_period = DISPLAY_EVERY_N_FRAMES
        display_period = max(1, int(display_period))

        show_frame = (display_frame_count >= display_period)
        if show_frame:
            display_frame_count = 0

        if super_stable:
            detect_period = SUPER_STABLE_DETECT_EVERY_N_FRAMES
        elif locked_stable:
            detect_period = LOCKED_DETECT_EVERY_N_FRAMES
        else:
            detect_period = DETECT_EVERY_N_FRAMES
        detect_period = max(1, int(detect_period))

        run_detector = (detect_frame_count % detect_period == 0)

        if run_detector:
            if super_stable:
                detect_conf = SUPER_STABLE_CONF_THRESHOLD
                detect_iou = SUPER_STABLE_IOU_THRESHOLD
            elif locked_stable:
                detect_conf = TRACK_CONF_THRESHOLD
                detect_iou = TRACK_IOU_THRESHOLD
            else:
                detect_conf = CONF_THRESHOLD
                detect_iou = IOU_THRESHOLD

            objs = detector.detect(
                img,
                conf_th=detect_conf,
                iou_th=detect_iou
            )

            selection_center = last_target_center
            if filtered_cx is not None and filtered_cy is not None:
                # Use predicted current center for target selection after skipped
                # frames. This lowers wrong re-locks when the target is moving.
                selection_frames = max(1, predict_only_count + 1)
                predict_cx, predict_cy, _, _ = predict_target_state(
                    filtered_cx,
                    filtered_cy,
                    filtered_velocity_x,
                    filtered_velocity_y,
                    selection_frames
                )
                selection_center = (predict_cx, predict_cy)

            target = choose_yolo_target(
                objs,
                selection_center,
                last_target_area,
                last_target_aspect
            )
        else:
            # Intentional skip: prediction-only frame.
            # It still updates PID from velocity prediction and is not a miss.
            target = None

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
            obj, raw_cx, raw_cy, area, aspect, score_value = target

            reject_measurement = False
            if filtered_cx is not None and filtered_cy is not None:
                reject_measurement = (
                    score_value < MEASURE_JUMP_REACQUIRE_SCORE
                    and (
                        abs(raw_cx - filtered_cx) > MEASURE_JUMP_REJECT_X
                        or abs(raw_cy - filtered_cy) > MEASURE_JUMP_REJECT_Y
                    )
                )

            if reject_measurement:
                stable_detect_count = 0
                miss_count += 1
                (
                    predict_only_count,
                    previous_error_x,
                    previous_error_y,
                    pan_integral,
                    tilt_integral,
                    last_control_ms,
                    last_cmd_ms,
                    last_pan_cmd,
                    last_tilt_cmd,
                ) = handle_predict_hold(
                    now,
                    img,
                    show_frame,
                    "JUMP HOLD",
                    filtered_cx,
                    filtered_cy,
                    filtered_velocity_x,
                    filtered_velocity_y,
                    predict_only_count,
                    detect_period,
                    previous_error_x,
                    previous_error_y,
                    pan_integral,
                    tilt_integral,
                    last_control_ms,
                    last_cmd_ms,
                    last_pan_cmd,
                    last_tilt_cmd
                )
                if miss_count > MISS_TOLERANCE_FRAMES:
                    if last_pan_cmd != ("stop",):
                        motor_stop(PAN_ADDR, "PAN/ID01")
                        last_pan_cmd = ("stop",)
                    if last_tilt_cmd != ("stop",):
                        motor_stop(TILT_ADDR, "TILT/ID02")
                        last_tilt_cmd = ("stop",)
                    lock_confirm_count = 0
                    stable_detect_count = 0
                    last_target_score = 0.0
                    last_target_area = 0
                    last_target_aspect = 0.0
                    pan_integral = 0.0
                    tilt_integral = 0.0
                    last_control_ms = 0
                    previous_target_cx = None
                    previous_target_cy = None
                    filtered_velocity_x = 0.0
                    filtered_velocity_y = 0.0
                    predict_only_count = 0
                target = None
            else:
                predict_only_count = 0
                last_target_score = score_value
                last_target_area = area
                last_target_aspect = aspect
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
                    a = get_target_filter_alpha(
                        raw_cx,
                        raw_cy,
                        filtered_cx,
                        filtered_cy
                    )
                    filtered_cx = (1.0 - a) * filtered_cx + a * raw_cx
                    filtered_cy = (1.0 - a) * filtered_cy + a * raw_cy

                target_cx = int(filtered_cx)
                target_cy = int(filtered_cy)

                error_x = target_cx - IMAGE_CENTER_X
                error_y = target_cy - IMAGE_CENTER_Y

                # Estimate target velocity and predict short-term position.
                # This keeps control accurate on frames where YOLO inference is skipped.
                if previous_target_cx is None or previous_target_cy is None:
                    raw_velocity_x = 0.0
                    raw_velocity_y = 0.0
                else:
                    raw_velocity_x = clamp(
                        target_cx - previous_target_cx,
                        -MAX_RAW_VELOCITY_X,
                        MAX_RAW_VELOCITY_X
                    )
                    raw_velocity_y = clamp(
                        target_cy - previous_target_cy,
                        -MAX_RAW_VELOCITY_Y,
                        MAX_RAW_VELOCITY_Y
                    )

                previous_target_cx = target_cx
                previous_target_cy = target_cy

                va_x = PAN_VELOCITY_FILTER_ALPHA
                va_y = TILT_VELOCITY_FILTER_ALPHA
                filtered_velocity_x = clamp(
                    (1.0 - va_x) * filtered_velocity_x
                    + va_x * raw_velocity_x,
                    -MAX_FILTERED_VELOCITY_X,
                    MAX_FILTERED_VELOCITY_X
                )
                filtered_velocity_y = clamp(
                    (1.0 - va_y) * filtered_velocity_y
                    + va_y * raw_velocity_y,
                    -MAX_FILTERED_VELOCITY_Y,
                    MAX_FILTERED_VELOCITY_Y
                )

                control_error_x = int(
                    error_x
                    + filtered_velocity_x * PAN_PREDICT_FRAMES
                )
                control_error_y = int(
                    error_y
                    + filtered_velocity_y * TILT_PREDICT_FRAMES
                )

                detection_stable = (
                    score_value >= SKIP_MIN_SCORE
                    and abs(error_x) <= SKIP_MAX_ERROR_X
                    and abs(error_y) <= SKIP_MAX_ERROR_Y
                    and abs(filtered_velocity_x) <= SKIP_MAX_VELOCITY_X
                    and abs(filtered_velocity_y) <= SKIP_MAX_VELOCITY_Y
                )
                if detection_stable:
                    stable_detect_count = min(
                        stable_detect_count + 1,
                        SUPER_STABLE_CONFIRM_FRAMES
                    )
                else:
                    stable_detect_count = 0

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
                        "{} {:.2f}".format(label, score_value),
                        image.COLOR_GREEN
                    )

                    img.draw_string(
                        4,
                        4,
                        "EX:{} PX:{} EY:{} S:{} C:{:.2f} I:{:.2f}".format(
                            error_x,
                            control_error_x,
                            error_y,
                            stable_detect_count,
                            detect_conf if run_detector else 0.0,
                            detect_iou if run_detector else 0.0
                        ),
                        image.COLOR_GREEN
                    )

                if lock_confirm_count >= LOCK_CONFIRM_FRAMES:
                    (
                        previous_error_x,
                        previous_error_y,
                        pan_integral,
                        tilt_integral,
                        last_control_ms,
                        last_cmd_ms,
                        last_pan_cmd,
                        last_tilt_cmd,
                    ) = update_motor_control(
                        now,
                        control_error_x,
                        control_error_y,
                        previous_error_x,
                        previous_error_y,
                        pan_integral,
                        tilt_integral,
                        last_control_ms,
                        last_cmd_ms,
                        last_pan_cmd,
                        last_tilt_cmd
                    )

        else:
            if not run_detector:
                if filtered_cx is not None and filtered_cy is not None:
                    predict_only_count = min(
                        predict_only_count + 1,
                        max(1, detect_period)
                    )
                    (
                        predict_cx,
                        predict_cy,
                        predict_error_x,
                        predict_error_y,
                    ) = predict_target_state(
                        filtered_cx,
                        filtered_cy,
                        filtered_velocity_x,
                        filtered_velocity_y,
                        predict_only_count
                    )

                    (
                        previous_error_x,
                        previous_error_y,
                        pan_integral,
                        tilt_integral,
                        last_control_ms,
                        last_cmd_ms,
                        last_pan_cmd,
                        last_tilt_cmd,
                    ) = update_motor_control(
                        now,
                        predict_error_x,
                        predict_error_y,
                        previous_error_x,
                        previous_error_y,
                        pan_integral,
                        tilt_integral,
                        last_control_ms,
                        last_cmd_ms,
                        last_pan_cmd,
                        last_tilt_cmd
                    )

                    if show_frame:
                        img.draw_cross(
                            predict_cx,
                            predict_cy,
                            image.COLOR_YELLOW,
                            size=8,
                            thickness=1
                        )
                        img.draw_string(
                            4,
                            4,
                            "PRED EX:{} EY:{}".format(
                                predict_error_x,
                                predict_error_y
                            ),
                            image.COLOR_YELLOW
                        )
            else:
                miss_count += 1
                stable_detect_count = 0

                can_predict_miss = (
                    miss_count <= MISS_TOLERANCE_FRAMES
                    and lock_confirm_count >= LOCK_CONFIRM_FRAMES
                    and last_target_score >= MISS_HOLD_MIN_SCORE
                    and filtered_cx is not None
                    and filtered_cy is not None
                    and abs(filtered_velocity_x) <= MISS_HOLD_MAX_VELOCITY_X
                    and abs(filtered_velocity_y) <= MISS_HOLD_MAX_VELOCITY_Y
                )
                if can_predict_miss:
                    predict_only_count = min(
                        predict_only_count + 1,
                        max(1, detect_period)
                    )
                    (
                        predict_cx,
                        predict_cy,
                        predict_error_x,
                        predict_error_y,
                    ) = predict_target_state(
                        filtered_cx,
                        filtered_cy,
                        filtered_velocity_x,
                        filtered_velocity_y,
                        predict_only_count
                    )
                    (
                        previous_error_x,
                        previous_error_y,
                        pan_integral,
                        tilt_integral,
                        last_control_ms,
                        last_cmd_ms,
                        last_pan_cmd,
                        last_tilt_cmd,
                    ) = update_motor_control(
                        now,
                        predict_error_x,
                        predict_error_y,
                        previous_error_x,
                        previous_error_y,
                        pan_integral,
                        tilt_integral,
                        last_control_ms,
                        last_cmd_ms,
                        last_pan_cmd,
                        last_tilt_cmd
                    )

                    if show_frame:
                        img.draw_cross(
                            predict_cx,
                            predict_cy,
                            image.COLOR_YELLOW,
                            size=8,
                            thickness=1
                        )
                        img.draw_string(
                            4,
                            18,
                            "MISS HOLD:{} V:{:.1f}".format(
                                miss_count,
                                abs(filtered_velocity_x)
                            ),
                            image.COLOR_YELLOW
                        )

                if miss_count > MISS_TOLERANCE_FRAMES:
                    lock_confirm_count = 0
                    stable_detect_count = 0
                    last_target_score = 0.0
                    last_target_area = 0
                    last_target_aspect = 0.0
                    pan_integral = 0.0
                    tilt_integral = 0.0
                    last_control_ms = 0
                    previous_target_cx = None
                    previous_target_cy = None
                    filtered_velocity_x = 0.0
                    filtered_velocity_y = 0.0
                    predict_only_count = 0

                if show_frame and not can_predict_miss:
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
                    stable_detect_count = 0
                    last_target_score = 0.0
                    last_target_area = 0
                    last_target_aspect = 0.0
                    lock_confirm_count = 0
                    miss_count = 0
                    previous_error_x = 0
                    previous_error_y = 0
                    pan_integral = 0.0
                    tilt_integral = 0.0
                    last_control_ms = 0
                    previous_target_cx = None
                    previous_target_cy = None
                    filtered_velocity_x = 0.0
                    filtered_velocity_y = 0.0
                    predict_only_count = 0
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
                "FPS:{} D:{}".format(fps_value, display_period),
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
