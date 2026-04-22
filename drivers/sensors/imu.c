#include "drivers/sensors/imu.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_i2c.h"
#include "bsp/bsp_uart.h"
#include "debug.h"
#include <string.h>

#define IMU_I2C_ADDR_7BIT         0x23U              /* IMU I2C 7位从机地址。 */

#define IMU_REG_VERSION_H         0x01U              /* 版本号起始寄存器：0x01~0x03 分别是 VERSION_H/M/L。 */
#define IMU_REG_ACCEL_X_L         0x04U              /* 原始数据起始寄存器：0x04~0x0F 为 ACCEL_X/Y/Z 与 GYRO_X/Y/Z（int16，小端）。 */
#define IMU_REG_EULER_ROLL        0x26U              /* 欧拉角起始寄存器：0x26~0x31 为 ROLL/PITCH/YAW（float，小端，弧度）。 */
#define IMU_REG_ALGO_TYPE         0x61U              /* 算法类型配置寄存器：写 0x06/0x09 分别对应六轴/九轴(十轴模块用九轴融合)。 */

#define IMU_ALGO_TYPE_9_AXIS      0x09U              /* 配置为九轴融合算法（协议定义值）。 */

#define IMU_RAW_BLOCK_LEN         12U                /* 连续读取 12 字节：6 组 int16（3轴加速度 + 3轴陀螺仪）。 */
#define IMU_EULER_BLOCK_LEN       12U                /* 连续读取 12 字节：3 组 float（ROLL/PITCH/YAW）。 */
#define IMU_INIT_RETRY_COUNT      5U
#define IMU_INIT_RETRY_DELAY_MS   100U
#define IMU_RUNTIME_RETRY_MS      500U
#define IMU_RETRY_LOG_MS          2000U

#define IMU_UART_FRAME_HEAD1      0x7EU
#define IMU_UART_FRAME_HEAD2      0x23U
#define IMU_UART_MAX_FRAME_LEN    64U
#define IMU_UART_FUNC_VERSION     0x01U
#define IMU_UART_FUNC_RAW_MOTION  0x04U
#define IMU_UART_FUNC_EULER       0x26U
#define IMU_UART_FUNC_BARO        0x32U
#define IMU_UART_FUNC_SET_RATE    0x60U
#define IMU_UART_FUNC_ALGO_TYPE   0x61U
#define IMU_UART_FUNC_REQUEST     0x80U
#define IMU_UART_PROCESS_MAX_BYTES 128U

#define IMU_ACCEL_LSB_TO_G        (16.0f / 32767.0f) /* 加速度原始值换算为 g。 */
#define IMU_GYRO_LSB_TO_DPS       (2000.0f / 32767.0f) /* 陀螺仪原始值换算为 °/s。 */

typedef enum {
    IMU_INIT_STATUS_OK = 0,
    IMU_INIT_STATUS_VERSION_READ_FAILED,
    IMU_INIT_STATUS_ALGO_CONFIG_FAILED,
    IMU_INIT_STATUS_DISABLED
} imu_init_status_t;

typedef struct {
    bool ready;
    uint8_t version_h;
    uint8_t version_m;
    uint8_t version_l;
    uint32_t last_init_attempt_ms;
    uint32_t last_init_log_ms;
    imu_init_status_t last_init_status;
} imu_context_t;

typedef struct {
    bool motion_valid;
    bool euler_valid;
    bool baro_valid;
    uint32_t motion_timestamp_ms;
    uint32_t euler_timestamp_ms;
    uint32_t baro_timestamp_ms;
    vec3f_t accel_g;
    vec3f_t gyro_dps;
    euler_t euler_rad;
    float baro_altitude_m;
    float baro_temperature_c;
    float baro_pressure_pa;
    float baro_pressure_reference_pa;
} imu_uart_context_t;

static imu_context_t g_imu_ctx;
static imu_uart_context_t g_imu_uart_ctx;

#if FC_ENABLE_I2C2_SENSORS
static bool imu_read_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
    return bsp_i2c2_mem_read(IMU_I2C_ADDR_7BIT, reg, data, len);
}

static bool imu_write_u8(uint8_t reg, uint8_t value)
{
    return bsp_i2c2_mem_write(IMU_I2C_ADDR_7BIT, reg, &value, 1U);
}
#endif

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

static const char *imu_init_status_text(imu_init_status_t status)
{
    switch (status) {
    case IMU_INIT_STATUS_OK:
        return "ok";
    case IMU_INIT_STATUS_VERSION_READ_FAILED:
        return "version read";
    case IMU_INIT_STATUS_ALGO_CONFIG_FAILED:
        return "algo config";
    case IMU_INIT_STATUS_DISABLED:
        return "i2c sensors disabled";
    default:
        return "unknown";
    }
}

#if FC_ENABLE_IMU_UART
static bool imu_uart_is_fresh(uint32_t now_ms, uint32_t timestamp_ms)
{
    return (now_ms - timestamp_ms) <= FC_IMU_UART_STALE_TIMEOUT_MS;
}

static bool imu_uart_send_command(uint8_t function, const uint8_t *params, uint8_t param_len)
{
    uint8_t frame[8] = {IMU_UART_FRAME_HEAD1, IMU_UART_FRAME_HEAD2, 0U, function, 0U, 0U, 0U, 0U};
    uint8_t frame_len = 0U;
    uint8_t checksum = 0U;

    if ((param_len > 3U) || ((param_len > 0U) && (params == 0))) {
        return false;
    }

    frame_len = (uint8_t)(4U + param_len + 1U);
    frame[2] = frame_len;

    for (uint8_t i = 0; i < param_len; i++) {
        frame[4U + i] = params[i];
    }

    for (uint8_t i = 0; i < (uint8_t)(frame_len - 1U); i++) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[frame_len - 1U] = checksum;

    bsp_uart_imu_write(frame, frame_len);
    return true;
}

static void imu_uart_parse_frame(uint8_t function, const uint8_t *payload, uint8_t payload_len)
{
    const uint32_t now_ms = bsp_board_millis();

    switch (function) {
    case IMU_UART_FUNC_RAW_MOTION:
        if (payload_len < IMU_RAW_BLOCK_LEN) {
            return;
        }

        g_imu_uart_ctx.accel_g.x = (float)imu_decode_i16_le(&payload[0]) * IMU_ACCEL_LSB_TO_G;
        g_imu_uart_ctx.accel_g.y = (float)imu_decode_i16_le(&payload[2]) * IMU_ACCEL_LSB_TO_G;
        g_imu_uart_ctx.accel_g.z = (float)imu_decode_i16_le(&payload[4]) * IMU_ACCEL_LSB_TO_G;
        g_imu_uart_ctx.gyro_dps.x = (float)imu_decode_i16_le(&payload[6]) * IMU_GYRO_LSB_TO_DPS;
        g_imu_uart_ctx.gyro_dps.y = (float)imu_decode_i16_le(&payload[8]) * IMU_GYRO_LSB_TO_DPS;
        g_imu_uart_ctx.gyro_dps.z = (float)imu_decode_i16_le(&payload[10]) * IMU_GYRO_LSB_TO_DPS;
        g_imu_uart_ctx.motion_timestamp_ms = now_ms;
        g_imu_uart_ctx.motion_valid = true;
        break;

    case IMU_UART_FUNC_EULER:
        if (payload_len < 12U) {
            return;
        }

        g_imu_uart_ctx.euler_rad.roll_rad = imu_decode_f32_le(&payload[0]);
        g_imu_uart_ctx.euler_rad.pitch_rad = imu_decode_f32_le(&payload[4]);
        g_imu_uart_ctx.euler_rad.yaw_rad = imu_decode_f32_le(&payload[8]);
        g_imu_uart_ctx.euler_timestamp_ms = now_ms;
        g_imu_uart_ctx.euler_valid = true;
        break;

    case IMU_UART_FUNC_BARO:
        if (payload_len < 16U) {
            return;
        }

        g_imu_uart_ctx.baro_altitude_m = imu_decode_f32_le(&payload[0]);
        g_imu_uart_ctx.baro_temperature_c = imu_decode_f32_le(&payload[4]);
        g_imu_uart_ctx.baro_pressure_pa = imu_decode_f32_le(&payload[8]);
        g_imu_uart_ctx.baro_pressure_reference_pa = imu_decode_f32_le(&payload[12]);
        g_imu_uart_ctx.baro_timestamp_ms = now_ms;
        g_imu_uart_ctx.baro_valid = true;
        break;

    case IMU_UART_FUNC_VERSION:
        if (payload_len < 3U) {
            return;
        }

        g_imu_ctx.version_h = payload[0];
        g_imu_ctx.version_m = payload[1];
        g_imu_ctx.version_l = payload[2];
        break;

    default:
        break;
    }
}

static void imu_uart_process(void)
{
    enum {
        RX_WAIT_HEAD1 = 0,
        RX_WAIT_HEAD2,
        RX_WAIT_LENGTH,
        RX_WAIT_FUNCTION,
        RX_COLLECT_PAYLOAD
    };

    static uint8_t rx_state = RX_WAIT_HEAD1;
    static uint8_t frame_len = 0U;
    static uint8_t frame_function = 0U;
    static uint8_t frame_data[IMU_UART_MAX_FRAME_LEN - 4U];
    static uint8_t frame_index = 0U;

    uint8_t byte = 0U;

    uint16_t budget = IMU_UART_PROCESS_MAX_BYTES;

    while ((budget-- != 0U) && bsp_uart_imu_read_byte(&byte)) {
        switch (rx_state) {
        case RX_WAIT_HEAD1:
            rx_state = (byte == IMU_UART_FRAME_HEAD1) ? RX_WAIT_HEAD2 : RX_WAIT_HEAD1;
            break;

        case RX_WAIT_HEAD2:
            if (byte == IMU_UART_FRAME_HEAD2) {
                rx_state = RX_WAIT_LENGTH;
            } else {
                rx_state = (byte == IMU_UART_FRAME_HEAD1) ? RX_WAIT_HEAD2 : RX_WAIT_HEAD1;
            }
            break;

        case RX_WAIT_LENGTH:
            frame_len = byte;
            if ((frame_len < 5U) || (frame_len > IMU_UART_MAX_FRAME_LEN)) {
                rx_state = RX_WAIT_HEAD1;
            } else {
                rx_state = RX_WAIT_FUNCTION;
            }
            break;

        case RX_WAIT_FUNCTION:
            frame_function = byte;
            frame_index = 0U;
            rx_state = RX_COLLECT_PAYLOAD;
            break;

        case RX_COLLECT_PAYLOAD: {
            const uint8_t data_len = (uint8_t)(frame_len - 4U);

            frame_data[frame_index++] = byte;
            if (frame_index >= data_len) {
                uint8_t checksum = (uint8_t)(IMU_UART_FRAME_HEAD1 + IMU_UART_FRAME_HEAD2 + frame_len + frame_function);

                for (uint8_t i = 0; i < (uint8_t)(data_len - 1U); i++) {
                    checksum = (uint8_t)(checksum + frame_data[i]);
                }

                if (checksum == frame_data[data_len - 1U]) {
                    imu_uart_parse_frame(frame_function, frame_data, (uint8_t)(data_len - 1U));
                }

                rx_state = RX_WAIT_HEAD1;
            }
            break;
        }

        default:
            rx_state = RX_WAIT_HEAD1;
            break;
        }
    }
}

static void imu_uart_configure_device(void)
{
    uint8_t output_hz = (uint8_t)FC_IMU_UART_OUTPUT_HZ;
    uint8_t params[2] = {0U};

    if (output_hz < 10U) {
        output_hz = 10U;
    } else if (output_hz > 100U) {
        output_hz = 100U;
    }

    params[0] = output_hz;
    params[1] = 0x5FU;
    (void)imu_uart_send_command(IMU_UART_FUNC_SET_RATE, params, sizeof(params));
    Delay_Ms(2);

    params[0] = IMU_ALGO_TYPE_9_AXIS;
    params[1] = 0x5FU;
    (void)imu_uart_send_command(IMU_UART_FUNC_ALGO_TYPE, params, sizeof(params));
    Delay_Ms(2);

    params[0] = IMU_UART_FUNC_VERSION;
    params[1] = 0x00U;
    (void)imu_uart_send_command(IMU_UART_FUNC_REQUEST, params, sizeof(params));
}

#endif

static imu_init_status_t imu_try_init(void)
{
#if FC_ENABLE_IMU_UART
    memset(&g_imu_uart_ctx, 0, sizeof(g_imu_uart_ctx));
    g_imu_uart_ctx.baro_temperature_c = 25.0f;
    g_imu_uart_ctx.baro_pressure_pa = FC_SEA_LEVEL_PRESSURE_PA;
    g_imu_uart_ctx.baro_pressure_reference_pa = FC_SEA_LEVEL_PRESSURE_PA;
    imu_uart_configure_device();
    g_imu_ctx.ready = true;
    return IMU_INIT_STATUS_OK;
#elif FC_ENABLE_I2C2_SENSORS
    uint8_t version[3] = {0};

    if (!imu_read_regs(IMU_REG_VERSION_H, version, sizeof(version))) {
        return IMU_INIT_STATUS_VERSION_READ_FAILED;
    }

    g_imu_ctx.version_h = version[0];
    g_imu_ctx.version_m = version[1];
    g_imu_ctx.version_l = version[2];

    if (!imu_write_u8(IMU_REG_ALGO_TYPE, IMU_ALGO_TYPE_9_AXIS)) {
        return IMU_INIT_STATUS_ALGO_CONFIG_FAILED;
    }

    g_imu_ctx.ready = true;
    return IMU_INIT_STATUS_OK;
#else
    return IMU_INIT_STATUS_DISABLED;
#endif
}

static bool imu_retry_init(uint32_t now_ms, bool force, bool verbose_fail)
{
    imu_init_status_t status = IMU_INIT_STATUS_OK;

    if (g_imu_ctx.ready) {
        return true;
    }

    if ((!force) && (now_ms - g_imu_ctx.last_init_attempt_ms < IMU_RUNTIME_RETRY_MS)) {
        return false;
    }

    g_imu_ctx.last_init_attempt_ms = now_ms;
    status = imu_try_init();
    g_imu_ctx.last_init_status = status;

    if (status == IMU_INIT_STATUS_OK) {
#if FC_ENABLE_IMU_UART
        printf("imu uart init ok: baud %u, output %u Hz\r\n",
               (unsigned)FC_IMU_UART_BAUD,
               (unsigned)FC_IMU_UART_OUTPUT_HZ);
#else
        printf("imu init ok: version %u.%u.%u\r\n",
               (unsigned)g_imu_ctx.version_h,
               (unsigned)g_imu_ctx.version_m,
               (unsigned)g_imu_ctx.version_l);
#endif
        return true;
    }

    if (verbose_fail && ((now_ms - g_imu_ctx.last_init_log_ms) >= IMU_RETRY_LOG_MS)) {
        g_imu_ctx.last_init_log_ms = now_ms;
        printf("imu init failed: %s\r\n", imu_init_status_text(status));
    }

    return false;
}

bool imu_init(void)
{
    memset(&g_imu_ctx, 0, sizeof(g_imu_ctx));

    for (uint8_t attempt = 0; attempt < IMU_INIT_RETRY_COUNT; attempt++) {
        if (imu_retry_init(bsp_board_millis(), true, false)) {
            return true;
        }

        printf("imu init attempt %u/%u failed: %s\r\n",
               (unsigned)(attempt + 1U),
               (unsigned)IMU_INIT_RETRY_COUNT,
               imu_init_status_text(g_imu_ctx.last_init_status));

        if ((uint8_t)(attempt + 1U) < IMU_INIT_RETRY_COUNT) {
            Delay_Ms(IMU_INIT_RETRY_DELAY_MS);
        }
    }

    return false;
}

bool imu_read(imu_sample_t *sample)
{
#if FC_ENABLE_IMU_UART
    const uint32_t now_ms = bsp_board_millis();
    bool healthy = false;

    if (sample == 0) {
        return false;
    }

    if (!g_imu_ctx.ready) {
        (void)imu_retry_init(now_ms, false, true);
    }

    imu_uart_process();

    healthy = g_imu_ctx.ready &&
              g_imu_uart_ctx.motion_valid &&
              g_imu_uart_ctx.euler_valid &&
              imu_uart_is_fresh(now_ms, g_imu_uart_ctx.motion_timestamp_ms) &&
              imu_uart_is_fresh(now_ms, g_imu_uart_ctx.euler_timestamp_ms);

    sample->timestamp_ms = now_ms;
    sample->euler_rad = g_imu_uart_ctx.euler_rad;
    sample->gyro_dps = g_imu_uart_ctx.gyro_dps;
    sample->accel_g = g_imu_uart_ctx.accel_g;
    sample->temperature_c = g_imu_uart_ctx.baro_valid ? g_imu_uart_ctx.baro_temperature_c : 0.0f;
    sample->healthy = healthy;

    return healthy;
#elif FC_ENABLE_I2C2_SENSORS
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
        (void)imu_retry_init(now_ms, false, true);
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
#else
    if (sample != 0) {
        sample->timestamp_ms = bsp_board_millis();
        sample->healthy = false;
    }

    return false;
#endif
}

bool imu_read_barometer(baro_sample_t *sample)
{
#if FC_ENABLE_IMU_UART
    const uint32_t now_ms = bsp_board_millis();
    bool healthy = false;

    if (sample == 0) {
        return false;
    }

    imu_uart_process();

    healthy = g_imu_ctx.ready &&
              g_imu_uart_ctx.baro_valid &&
              imu_uart_is_fresh(now_ms, g_imu_uart_ctx.baro_timestamp_ms);

    sample->timestamp_ms = now_ms;
    sample->pressure_pa = g_imu_uart_ctx.baro_valid ? g_imu_uart_ctx.baro_pressure_pa : FC_SEA_LEVEL_PRESSURE_PA;
    sample->pressure_reference_pa = g_imu_uart_ctx.baro_valid ? g_imu_uart_ctx.baro_pressure_reference_pa : FC_SEA_LEVEL_PRESSURE_PA;
    sample->temperature_c = g_imu_uart_ctx.baro_valid ? g_imu_uart_ctx.baro_temperature_c : 25.0f;
    sample->altitude_m = g_imu_uart_ctx.baro_valid ? g_imu_uart_ctx.baro_altitude_m : 0.0f;
    sample->healthy = healthy;

    return healthy;
#else
    if (sample != 0) {
        sample->timestamp_ms = bsp_board_millis();
        sample->healthy = false;
    }

    return false;
#endif
}
