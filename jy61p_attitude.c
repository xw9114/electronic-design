/*
 * jy61p_attitude.c
 *
 * 维特智能 JY61P 姿态模块单文件驱动/解算模板。
 *
 * 适用场景：
 * - 电赛小车航向保持、转角控制、坡面姿态估计、云台姿态反馈。
 * - MCU 工程中通过 UART 接收 JY61P 的 WIT 标准协议数据。
 *
 * 设计思路来自当前项目的 JY901S / WT9011G4K 代码：
 * - 0x55 帧头，11 字节定长帧；
 * - 累加和校验；
 * - 0x51 加速度/温度；
 * - 0x52 角速度；
 * - 0x53 欧拉角；
 * - 可选 0x59 四元数；
 * - 用模块输出角度作长期基准，用角速度作短时预测；
 * - 提供 Yaw 连续化、软件置零、陀螺零偏标定、超时失效判断。
 *
 * 重要说明：
 * - JY61P 是 6 轴 IMU，Roll/Pitch 可由重力长期约束，Yaw 主要是相对航向，
 *   长时间运行会随陀螺零偏缓慢漂移。电赛小车常用相对 Yaw，足够用于短时控制。
 * - 默认量程按 WIT 常见配置：加速度 ±16g，角速度 ±2000 deg/s，角度 ±180 deg。
 *   如果你用上位机改过量程，请同步修改下方宏。
 *
 * 最小用法：
 *
 *     static Jy61pAttitude imu;
 *
 *     void app_init(void)
 *     {
 *         JY61P_Init(&imu, 100.0f);                    // 角度帧标称频率，例如 100Hz
 *         JY61P_StartCalibration(&imu, millis(), 1000); // 上电静止 1s 标定陀螺零偏
 *     }
 *
 *     void uart_rx_callback(uint8_t byte)
 *     {
 *         JY61P_InputByte(&imu, byte, millis());
 *     }
 *
 *     void control_loop(void)
 *     {
 *         if (JY61P_IsValid(&imu, millis())) {
 *             float yaw = JY61P_GetYawDeg(&imu);           // 连续相对航向，已扣软件零点
 *             float rate = JY61P_GetYawRateDps(&imu);      // Z 轴角速度，已扣零偏
 *             float err = JY61P_Wrap180(target_yaw - yaw);
 *             // turn = kp * err - kd * rate;
 *         }
 *     }
 *
 * 编译要求：C99。若链接器提示 sqrtf/fabsf 未定义，GCC 链接时加 -lm。
 */

#include "jy61p_attitude.h"
#include <math.h>
#include <string.h>

/* ============================= ???? ============================= */

static int16_t jy61p_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static float jy61p_norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static uint8_t jy61p_checksum_ok(const uint8_t frame[JY61P_FRAME_SIZE])
{
    uint16_t sum = 0u;

    for (uint8_t i = 0u; i < JY61P_FRAME_SIZE - 1u; ++i) {
        sum += frame[i];
    }

    return (uint8_t)(sum & 0xFFu) == frame[JY61P_FRAME_SIZE - 1u];
}

float JY61P_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static uint8_t jy61p_supported_type(uint8_t type)
{
    return type == JY61P_FRAME_ACC
        || type == JY61P_FRAME_GYRO
        || type == JY61P_FRAME_ANGLE
        || type == JY61P_FRAME_QUAT;
}

/* ============================= 帧解码 ============================= */

static void jy61p_decode_acc(Jy61pAttitude *imu, const uint8_t frame[JY61P_FRAME_SIZE])
{
    const float scale = JY61P_ACC_RANGE_G / 32768.0f;

    imu->acc_g[0] = (float)jy61p_i16_le(&frame[2]) * scale;
    imu->acc_g[1] = (float)jy61p_i16_le(&frame[4]) * scale;
    imu->acc_g[2] = (float)jy61p_i16_le(&frame[6]) * scale;
    imu->temperature_c = (float)jy61p_i16_le(&frame[8]) / 100.0f;
    imu->has_acc = 1u;
    imu->acc_frame_count++;
}

static void jy61p_decode_gyro(Jy61pAttitude *imu, const uint8_t frame[JY61P_FRAME_SIZE])
{
    const float scale = JY61P_GYRO_RANGE_DPS / 32768.0f;

    imu->gyro_raw_dps[0] = (float)jy61p_i16_le(&frame[2]) * scale;
    imu->gyro_raw_dps[1] = (float)jy61p_i16_le(&frame[4]) * scale;
    imu->gyro_raw_dps[2] = (float)jy61p_i16_le(&frame[6]) * scale;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        imu->gyro_dps[axis] = imu->gyro_raw_dps[axis] - imu->gyro_bias_dps[axis];
    }

    imu->has_gyro = 1u;
    imu->gyro_frame_count++;
}

static void jy61p_decode_angle(Jy61pAttitude *imu, const uint8_t frame[JY61P_FRAME_SIZE])
{
    const float scale = JY61P_ANGLE_RANGE_DEG / 32768.0f;

    imu->angle_raw_deg[0] = (float)jy61p_i16_le(&frame[2]) * scale;
    imu->angle_raw_deg[1] = (float)jy61p_i16_le(&frame[4]) * scale;
    imu->angle_raw_deg[2] = (float)jy61p_i16_le(&frame[6]) * scale;
    imu->has_angle = 1u;
    imu->angle_frame_count++;
}

static void jy61p_decode_quat(Jy61pAttitude *imu, const uint8_t frame[JY61P_FRAME_SIZE])
{
    imu->quat[0] = (float)jy61p_i16_le(&frame[2]) / 32768.0f;
    imu->quat[1] = (float)jy61p_i16_le(&frame[4]) / 32768.0f;
    imu->quat[2] = (float)jy61p_i16_le(&frame[6]) / 32768.0f;
    imu->quat[3] = (float)jy61p_i16_le(&frame[8]) / 32768.0f;
    imu->has_quat = 1u;
    imu->quat_frame_count++;
}

/* ============================= 解算核心 ============================= */

static void jy61p_finish_calibration_if_due(Jy61pAttitude *imu, uint32_t now_ms)
{
    if (!imu->calibrating) {
        return;
    }

    if (jy61p_norm3(imu->gyro_raw_dps) <= JY61P_CAL_GYRO_ACCEPT_DPS) {
        for (uint8_t axis = 0u; axis < 3u; ++axis) {
            imu->cal_sum_dps[axis] += imu->gyro_raw_dps[axis];
        }
        imu->cal_count++;
    }

    if ((uint32_t)(now_ms - imu->cal_start_ms) >= imu->cal_duration_ms) {
        if (imu->cal_count >= JY61P_CAL_MIN_SAMPLES) {
            for (uint8_t axis = 0u; axis < 3u; ++axis) {
                imu->gyro_bias_dps[axis] = imu->cal_sum_dps[axis] / (float)imu->cal_count;
                imu->gyro_dps[axis] = imu->gyro_raw_dps[axis] - imu->gyro_bias_dps[axis];
            }
        }
        imu->calibrating = 0u;
    }
}

static void jy61p_stationary_bias_update(Jy61pAttitude *imu)
{
    const float acc_norm = imu->has_acc ? jy61p_norm3(imu->acc_g) : 1.0f;
    const float gyro_norm = jy61p_norm3(imu->gyro_dps);
    const uint8_t looks_stationary =
        imu->stationary_hint
        && fabsf(acc_norm - 1.0f) <= JY61P_STATIONARY_ACC_TOL_G
        && gyro_norm <= JY61P_STATIONARY_GYRO_MAX_DPS;

    if (!looks_stationary || imu->calibrating) {
        return;
    }

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        imu->gyro_bias_dps[axis] =
            (1.0f - JY61P_STATIONARY_BIAS_ALPHA) * imu->gyro_bias_dps[axis]
            + JY61P_STATIONARY_BIAS_ALPHA * imu->gyro_raw_dps[axis];
        imu->gyro_dps[axis] = imu->gyro_raw_dps[axis] - imu->gyro_bias_dps[axis];
    }
}

static uint8_t jy61p_process_angle_solution(Jy61pAttitude *imu, uint32_t now_ms)
{
    const float raw_yaw = JY61P_Wrap180(imu->angle_raw_deg[2]);
    float yaw_delta;
    if (!imu->has_angle) {
        return 0u;
    }

    if (imu->angle_frame_count == 1u) {
        imu->yaw_unwrapped_deg = raw_yaw;
        imu->yaw_raw_last_deg = raw_yaw;
    } else {
        yaw_delta = JY61P_Wrap180(raw_yaw - imu->yaw_raw_last_deg);
        imu->yaw_unwrapped_deg += yaw_delta;
        imu->yaw_raw_last_deg = raw_yaw;
    }

    if (!imu->initialized) {
        imu->roll_deg = imu->angle_raw_deg[0];
        imu->pitch_deg = imu->angle_raw_deg[1];
        imu->yaw_abs_deg = imu->yaw_unwrapped_deg;
        imu->yaw_zero_deg = imu->yaw_abs_deg;
        imu->initialized = 1u;
        imu->last_update_ms = now_ms;
        imu->last_angle_ms = now_ms;
        return 1u;
    }

    /*
     * 电赛控制策略：
     * - Roll/Pitch 直接使用模块 0x53 欧拉角；
     * - Yaw 仅做跨 ±180° 连续化；
     * - 陀螺 Z 轴角速度保留给控制器微分项，不再积分修正姿态。
     */
    imu->roll_deg = imu->angle_raw_deg[0];
    imu->pitch_deg = imu->angle_raw_deg[1];
    imu->yaw_abs_deg = imu->yaw_unwrapped_deg;

    imu->last_update_ms = now_ms;
    imu->last_angle_ms = now_ms;
    return 1u;
}

static uint8_t jy61p_process_frame(Jy61pAttitude *imu, const uint8_t frame[JY61P_FRAME_SIZE], uint32_t now_ms)
{
    uint8_t attitude_updated = 0u;

    imu->last_frame_ms = now_ms;
    imu->frame_count++;

    switch (frame[1]) {
    case JY61P_FRAME_ACC:
        jy61p_decode_acc(imu, frame);
        break;

    case JY61P_FRAME_GYRO:
        jy61p_decode_gyro(imu, frame);
        jy61p_finish_calibration_if_due(imu, now_ms);
        jy61p_stationary_bias_update(imu);
        break;

    case JY61P_FRAME_ANGLE:
        jy61p_decode_angle(imu, frame);
        attitude_updated = jy61p_process_angle_solution(imu, now_ms);
        break;

    case JY61P_FRAME_QUAT:
        jy61p_decode_quat(imu, frame);
        break;

    default:
        break;
    }

    return attitude_updated;
}

/* ============================= 对外接口 ============================= */

void JY61P_Init(Jy61pAttitude *imu, float nominal_hz)
{
    if (imu == NULL) {
        return;
    }

    memset(imu, 0, sizeof(*imu));

    if (nominal_hz <= 0.0f) {
        nominal_hz = 100.0f;
    }

    imu->nominal_hz = nominal_hz;
    imu->quat[0] = 1.0f;
}

void JY61P_StartCalibration(Jy61pAttitude *imu, uint32_t now_ms, uint32_t duration_ms)
{
    if (imu == NULL) {
        return;
    }

    if (duration_ms == 0u) {
        duration_ms = 1000u;
    }

    imu->calibrating = 1u;
    imu->cal_start_ms = now_ms;
    imu->cal_duration_ms = duration_ms;
    imu->cal_count = 0u;
    imu->cal_sum_dps[0] = 0.0f;
    imu->cal_sum_dps[1] = 0.0f;
    imu->cal_sum_dps[2] = 0.0f;
}

void JY61P_SetStationary(Jy61pAttitude *imu, bool stationary)
{
    if (imu == NULL) {
        return;
    }

    imu->stationary_hint = stationary ? 1u : 0u;
}

void JY61P_SetYawZero(Jy61pAttitude *imu)
{
    if (imu == NULL || !imu->initialized) {
        return;
    }

    imu->yaw_zero_deg = imu->yaw_abs_deg;
}

void JY61P_SetYawZeroValue(Jy61pAttitude *imu, float current_yaw_should_be_deg)
{
    if (imu == NULL || !imu->initialized) {
        return;
    }

    imu->yaw_zero_deg = imu->yaw_abs_deg - current_yaw_should_be_deg;
}

uint8_t JY61P_InputByte(Jy61pAttitude *imu, uint8_t byte, uint32_t now_ms)
{
    uint8_t updated = 0u;

    if (imu == NULL) {
        return 0u;
    }

    if (imu->rx_len == 0u) {
        if (byte == JY61P_FRAME_HEAD) {
            imu->rx_buf[0] = byte;
            imu->rx_len = 1u;
        }
        return 0u;
    }

    imu->rx_buf[imu->rx_len++] = byte;

    if (imu->rx_len == 2u && !jy61p_supported_type(imu->rx_buf[1])) {
        imu->rx_len = (byte == JY61P_FRAME_HEAD) ? 1u : 0u;
        imu->rx_buf[0] = byte;
        return 0u;
    }

    if (imu->rx_len < JY61P_FRAME_SIZE) {
        return 0u;
    }

    if (jy61p_checksum_ok(imu->rx_buf)) {
        updated = jy61p_process_frame(imu, imu->rx_buf, now_ms);
    } else {
        imu->bad_frame_count++;
    }

    /* 简单重同步：当前字节若刚好是 0x55，则作为下一帧开头保留。 */
    imu->rx_len = (byte == JY61P_FRAME_HEAD) ? 1u : 0u;
    imu->rx_buf[0] = byte;

    return updated;
}

uint32_t JY61P_InputBytes(Jy61pAttitude *imu, const uint8_t *data, uint32_t len, uint32_t now_ms)
{
    uint32_t updates = 0u;

    if (imu == NULL || data == NULL) {
        return 0u;
    }

    for (uint32_t i = 0u; i < len; ++i) {
        updates += JY61P_InputByte(imu, data[i], now_ms) ? 1u : 0u;
    }

    return updates;
}

uint8_t JY61P_IsValid(const Jy61pAttitude *imu, uint32_t now_ms)
{
    if (imu == NULL || !imu->initialized) {
        return 0u;
    }

    return ((uint32_t)(now_ms - imu->last_update_ms) <= JY61P_VALID_TIMEOUT_MS) ? 1u : 0u;
}

uint8_t JY61P_IsLost(const Jy61pAttitude *imu, uint32_t now_ms)
{
    if (imu == NULL || !imu->initialized) {
        return 1u;
    }

    return ((uint32_t)(now_ms - imu->last_update_ms) > JY61P_LOST_TIMEOUT_MS) ? 1u : 0u;
}

uint8_t JY61P_IsCalibrating(const Jy61pAttitude *imu)
{
    if (imu == NULL) {
        return 0u;
    }

    return imu->calibrating;
}

float JY61P_GetRollDeg(const Jy61pAttitude *imu)
{
    if (imu == NULL || !imu->initialized) {
        return 0.0f;
    }

    return imu->roll_deg;
}

float JY61P_GetPitchDeg(const Jy61pAttitude *imu)
{
    if (imu == NULL || !imu->initialized) {
        return 0.0f;
    }

    return imu->pitch_deg;
}

float JY61P_GetYawDeg(const Jy61pAttitude *imu)
{
    if (imu == NULL || !imu->initialized) {
        return 0.0f;
    }

    return imu->yaw_abs_deg - imu->yaw_zero_deg;
}

float JY61P_GetYawWrappedDeg(const Jy61pAttitude *imu)
{
    return JY61P_Wrap180(JY61P_GetYawDeg(imu));
}

float JY61P_GetYawAbsoluteDeg(const Jy61pAttitude *imu)
{
    if (imu == NULL || !imu->initialized) {
        return 0.0f;
    }

    return imu->yaw_abs_deg;
}

float JY61P_GetYawRateDps(const Jy61pAttitude *imu)
{
    if (imu == NULL) {
        return 0.0f;
    }

    return imu->gyro_dps[2];
}

float JY61P_GetGyroBiasZDps(const Jy61pAttitude *imu)
{
    if (imu == NULL) {
        return 0.0f;
    }

    return imu->gyro_bias_dps[2];
}

void JY61P_GetAccG(const Jy61pAttitude *imu, float out_acc_g[3])
{
    if (imu == NULL || out_acc_g == NULL) {
        return;
    }

    out_acc_g[0] = imu->acc_g[0];
    out_acc_g[1] = imu->acc_g[1];
    out_acc_g[2] = imu->acc_g[2];
}

void JY61P_GetGyroDps(const Jy61pAttitude *imu, float out_gyro_dps[3])
{
    if (imu == NULL || out_gyro_dps == NULL) {
        return;
    }

    out_gyro_dps[0] = imu->gyro_dps[0];
    out_gyro_dps[1] = imu->gyro_dps[1];
    out_gyro_dps[2] = imu->gyro_dps[2];
}

void JY61P_GetQuaternion(const Jy61pAttitude *imu, float out_quat[4])
{
    if (imu == NULL || out_quat == NULL) {
        return;
    }

    out_quat[0] = imu->quat[0];
    out_quat[1] = imu->quat[1];
    out_quat[2] = imu->quat[2];
    out_quat[3] = imu->quat[3];
}

float JY61P_GetTemperatureC(const Jy61pAttitude *imu)
{
    if (imu == NULL) {
        return 0.0f;
    }

    return imu->temperature_c;
}

uint32_t JY61P_GetFrameCount(const Jy61pAttitude *imu)
{
    if (imu == NULL) {
        return 0u;
    }

    return imu->frame_count;
}

uint32_t JY61P_GetBadFrameCount(const Jy61pAttitude *imu)
{
    if (imu == NULL) {
        return 0u;
    }

    return imu->bad_frame_count;
}

