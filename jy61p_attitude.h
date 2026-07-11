/*
 * jy61p_attitude.h
 *
 * 维特智能 JY61P 姿态模块单文件驱动/解算接口。
 * 与 jy61p_attitude.c 配套使用。
 */

#ifndef JY61P_ATTITUDE_H
#define JY61P_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= 可调参数 ============================= */

#define JY61P_FRAME_HEAD       0x55u
#define JY61P_FRAME_SIZE       11u
#define JY61P_FRAME_ACC        0x51u
#define JY61P_FRAME_GYRO       0x52u
#define JY61P_FRAME_ANGLE      0x53u
#define JY61P_FRAME_QUAT       0x59u

#ifndef JY61P_ACC_RANGE_G
#define JY61P_ACC_RANGE_G      16.0f
#endif

#ifndef JY61P_GYRO_RANGE_DPS
#define JY61P_GYRO_RANGE_DPS   2000.0f
#endif

#ifndef JY61P_ANGLE_RANGE_DEG
#define JY61P_ANGLE_RANGE_DEG  180.0f
#endif

#ifndef JY61P_VALID_TIMEOUT_MS
#define JY61P_VALID_TIMEOUT_MS 50u
#endif

#ifndef JY61P_LOST_TIMEOUT_MS
#define JY61P_LOST_TIMEOUT_MS  200u
#endif

#ifndef JY61P_CAL_GYRO_ACCEPT_DPS
#define JY61P_CAL_GYRO_ACCEPT_DPS 5.0f
#endif

#ifndef JY61P_CAL_MIN_SAMPLES
#define JY61P_CAL_MIN_SAMPLES  10u
#endif

#ifndef JY61P_STATIONARY_GYRO_MAX_DPS
#define JY61P_STATIONARY_GYRO_MAX_DPS 1.0f
#endif

#ifndef JY61P_STATIONARY_ACC_TOL_G
#define JY61P_STATIONARY_ACC_TOL_G 0.10f
#endif

#ifndef JY61P_STATIONARY_BIAS_ALPHA
#define JY61P_STATIONARY_BIAS_ALPHA 0.003f
#endif

/*
 * 姿态输出策略：
 * - Roll / Pitch：直接使用模块 0x53 欧拉角；
 * - Yaw：做跨 ±180° 连续化，并支持软件清零；
 * - Z 轴角速度：通过 JY61P_GetYawRateDps() 输出，供控制器微分项使用。
 */

/* ============================= 数据结构 ============================= */

typedef struct {
    uint8_t rx_buf[JY61P_FRAME_SIZE];
    uint8_t rx_len;

    float nominal_hz;

    float acc_g[3];
    float gyro_raw_dps[3];
    float gyro_bias_dps[3];
    float gyro_dps[3];
    float angle_raw_deg[3];
    float quat[4];
    float temperature_c;

    float roll_deg;
    float pitch_deg;
    float yaw_abs_deg;
    float yaw_zero_deg;
    float yaw_raw_last_deg;
    float yaw_unwrapped_deg;

    uint8_t has_acc;
    uint8_t has_gyro;
    uint8_t has_angle;
    uint8_t has_quat;
    uint8_t initialized;
    uint8_t stationary_hint;

    uint32_t last_frame_ms;
    uint32_t last_update_ms;
    uint32_t last_angle_ms;

    uint32_t frame_count;
    uint32_t bad_frame_count;
    uint32_t acc_frame_count;
    uint32_t gyro_frame_count;
    uint32_t angle_frame_count;
    uint32_t quat_frame_count;

    uint8_t calibrating;
    uint32_t cal_start_ms;
    uint32_t cal_duration_ms;
    uint32_t cal_count;
    float cal_sum_dps[3];
} Jy61pAttitude;

/* ============================= 对外接口 ============================= */

float JY61P_Wrap180(float angle_deg);

void JY61P_Init(Jy61pAttitude *imu, float nominal_hz);
void JY61P_StartCalibration(Jy61pAttitude *imu, uint32_t now_ms, uint32_t duration_ms);
void JY61P_SetStationary(Jy61pAttitude *imu, bool stationary);
void JY61P_SetYawZero(Jy61pAttitude *imu);
void JY61P_SetYawZeroValue(Jy61pAttitude *imu, float current_yaw_should_be_deg);

uint8_t JY61P_InputByte(Jy61pAttitude *imu, uint8_t byte, uint32_t now_ms);
uint32_t JY61P_InputBytes(Jy61pAttitude *imu, const uint8_t *data, uint32_t len, uint32_t now_ms);

uint8_t JY61P_IsValid(const Jy61pAttitude *imu, uint32_t now_ms);
uint8_t JY61P_IsLost(const Jy61pAttitude *imu, uint32_t now_ms);
uint8_t JY61P_IsCalibrating(const Jy61pAttitude *imu);

float JY61P_GetRollDeg(const Jy61pAttitude *imu);
float JY61P_GetPitchDeg(const Jy61pAttitude *imu);
float JY61P_GetYawDeg(const Jy61pAttitude *imu);
float JY61P_GetYawWrappedDeg(const Jy61pAttitude *imu);
float JY61P_GetYawAbsoluteDeg(const Jy61pAttitude *imu);
float JY61P_GetYawRateDps(const Jy61pAttitude *imu);
float JY61P_GetGyroBiasZDps(const Jy61pAttitude *imu);
float JY61P_GetTemperatureC(const Jy61pAttitude *imu);

void JY61P_GetAccG(const Jy61pAttitude *imu, float out_acc_g[3]);
void JY61P_GetGyroDps(const Jy61pAttitude *imu, float out_gyro_dps[3]);
void JY61P_GetQuaternion(const Jy61pAttitude *imu, float out_quat[4]);

uint32_t JY61P_GetFrameCount(const Jy61pAttitude *imu);
uint32_t JY61P_GetBadFrameCount(const Jy61pAttitude *imu);

#ifdef __cplusplus
}
#endif

#endif /* JY61P_ATTITUDE_H */

