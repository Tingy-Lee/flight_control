#include "drivers/sensors/imu.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_i2c.h"
#include <string.h>

#define IMU_I2C_ADDR_7BIT         0x23U              /* IMU I2C 7位从机地址。 */

#define IMU_REG_VERSION_H         0x01U              /* 版本号起始寄存器：0x01~0x03 分别是 VERSION_H/M/L。 */
#define IMU_REG_ACCEL_X_L         0x04U              /* 原始数据起始寄存器：0x04~0x0F 为 ACCEL_X/Y/Z 与 GYRO_X/Y/Z（int16，小端）。 */
#define IMU_REG_EULER_ROLL        0x26U              /* 欧拉角起始寄存器：0x26~0x31 为 ROLL/PITCH/YAW（float，小端，弧度）。 */
#define IMU_REG_ALGO_TYPE         0x61U              /* 算法类型配置寄存器：写 0x06/0x09 分别对应六轴/九轴(十轴模块用九轴融合)。 */

#define IMU_ALGO_TYPE_9_AXIS      0x09U              /* 配置为九轴融合算法（协议定义值）。 */

#define IMU_RAW_BLOCK_LEN         12U                /* 连续读取 12 字节：6 组 int16（3轴加速度 + 3轴陀螺仪）。 */
#define IMU_EULER_BLOCK_LEN       12U                /* 连续读取 12 字节：3 组 float（ROLL/PITCH/YAW）。 */

#define IMU_ACCEL_LSB_TO_G        (16.0f / 32767.0f) /* 加速度原始值换算为 g。 */
#define IMU_GYRO_LSB_TO_DPS       (2000.0f / 32767.0f) /* 陀螺仪原始值换算为 °/s。 */

typedef struct {
    bool ready;
    uint8_t version_h;
    uint8_t version_m;
    uint8_t version_l;
} imu_context_t;

static imu_context_t g_imu_ctx;

static bool imu_read_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
    return bsp_i2c2_mem_read(IMU_I2C_ADDR_7BIT, reg, data, len);
}

static bool imu_write_u8(uint8_t reg, uint8_t value)
{
    return bsp_i2c2_mem_write(IMU_I2C_ADDR_7BIT, reg, &value, 1U);
}

static int16_t imu_decode_i16_le(const uint8_t *raw)
{
    return (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
}

static float imu_decode_f32_le(const uint8_t *raw)
{
    uint32_t bits = 0U;
    float value = 0.0f;

    bits = (uint32_t)raw[0]
         | ((uint32_t)raw[1] << 8)
         | ((uint32_t)raw[2] << 16)
         | ((uint32_t)raw[3] << 24);
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool imu_init(void)
{
    uint8_t version[3] = {0};

    memset(&g_imu_ctx, 0, sizeof(g_imu_ctx));

#if FC_ENABLE_I2C2_SENSORS
    if (!imu_read_regs(IMU_REG_VERSION_H, version, sizeof(version))) {
        return false;
    }

    g_imu_ctx.version_h = version[0];
    g_imu_ctx.version_m = version[1];
    g_imu_ctx.version_l = version[2];

    if (!imu_write_u8(IMU_REG_ALGO_TYPE, IMU_ALGO_TYPE_9_AXIS)) {
        return false;
    }

    g_imu_ctx.ready = true;
    return true;
#else
    return false;
#endif
}

bool imu_read(imu_sample_t *sample)
{
    uint8_t raw_motion[IMU_RAW_BLOCK_LEN] = {0};
    uint8_t raw_euler[IMU_EULER_BLOCK_LEN] = {0};
    const uint32_t now_ms = bsp_board_millis();
    int16_t ax_raw = 0;
    int16_t ay_raw = 0;
    int16_t az_raw = 0;
    int16_t gx_raw = 0;
    int16_t gy_raw = 0;
    int16_t gz_raw = 0;

    if (sample == 0) {
        return false;
    }

    if (!g_imu_ctx.ready) {
        sample->timestamp_ms = now_ms;
        sample->healthy = false;
        return false;
    }

    if (!imu_read_regs(IMU_REG_ACCEL_X_L, raw_motion, sizeof(raw_motion))) {
        sample->timestamp_ms = now_ms;
        sample->healthy = false;
        return false;
    }

    if (!imu_read_regs(IMU_REG_EULER_ROLL, raw_euler, sizeof(raw_euler))) {
        sample->timestamp_ms = now_ms;
        sample->healthy = false;
        return false;
    }

    ax_raw = imu_decode_i16_le(&raw_motion[0]);
    ay_raw = imu_decode_i16_le(&raw_motion[2]);
    az_raw = imu_decode_i16_le(&raw_motion[4]);
    gx_raw = imu_decode_i16_le(&raw_motion[6]);
    gy_raw = imu_decode_i16_le(&raw_motion[8]);
    gz_raw = imu_decode_i16_le(&raw_motion[10]);

    sample->timestamp_ms = now_ms;
    sample->accel_g.x = (float)ax_raw * IMU_ACCEL_LSB_TO_G;
    sample->accel_g.y = (float)ay_raw * IMU_ACCEL_LSB_TO_G;
    sample->accel_g.z = (float)az_raw * IMU_ACCEL_LSB_TO_G;
    sample->gyro_dps.x = (float)gx_raw * IMU_GYRO_LSB_TO_DPS;
    sample->gyro_dps.y = (float)gy_raw * IMU_GYRO_LSB_TO_DPS;
    sample->gyro_dps.z = (float)gz_raw * IMU_GYRO_LSB_TO_DPS;
    sample->euler_rad.roll_rad = imu_decode_f32_le(&raw_euler[0]);
    sample->euler_rad.pitch_rad = imu_decode_f32_le(&raw_euler[4]);
    sample->euler_rad.yaw_rad = imu_decode_f32_le(&raw_euler[8]);
    sample->temperature_c = 0.0f;
    sample->healthy = true;

    return true;
}
