#include "drivers/sensors/barometer.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_i2c.h"
#include "drivers/sensors/imu.h"
#include <string.h>

#if FC_ENABLE_I2C2_SENSORS
#define IMU_I2C_ADDR_7BIT        0x23U
#define IMU_REG_BARO_HEIGHT      0x32U
#define IMU_BARO_BLOCK_LEN       16U
#define IMU_BARO_REFRESH_DIV     10U

typedef struct {
    bool valid;
    float altitude_m;
    float temperature_c;
    float pressure_pa;
    float pressure_reference_pa;
    uint8_t refresh_count;
} baro_context_t;

static baro_context_t g_baro_ctx;

static float decode_f32_le(const uint8_t *raw)
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
#endif

bool barometer_init(void)
{
#if FC_ENABLE_IMU_UART
    return true;
#elif FC_ENABLE_I2C2_SENSORS
    memset(&g_baro_ctx, 0, sizeof(g_baro_ctx));
    g_baro_ctx.temperature_c = 25.0f;
    g_baro_ctx.pressure_pa = FC_SEA_LEVEL_PRESSURE_PA;
    g_baro_ctx.pressure_reference_pa = FC_SEA_LEVEL_PRESSURE_PA;

    return true;
#else
    return false;
#endif
}

bool barometer_read(baro_sample_t *sample)
{
#if FC_ENABLE_IMU_UART
    return imu_read_barometer(sample);
#elif FC_ENABLE_I2C2_SENSORS
    uint8_t raw[IMU_BARO_BLOCK_LEN] = {0};

    if (sample == 0) {
        return false;
    }

    if ((g_baro_ctx.refresh_count == 0U) || (!g_baro_ctx.valid)) {
        if (bsp_i2c2_mem_read(IMU_I2C_ADDR_7BIT, IMU_REG_BARO_HEIGHT, raw, sizeof(raw))) {
            g_baro_ctx.altitude_m = decode_f32_le(&raw[0]);
            g_baro_ctx.temperature_c = decode_f32_le(&raw[4]);
            g_baro_ctx.pressure_pa = decode_f32_le(&raw[8]);
            g_baro_ctx.pressure_reference_pa = decode_f32_le(&raw[12]);
            g_baro_ctx.valid = true;
        }
    }

    g_baro_ctx.refresh_count++;
    if (g_baro_ctx.refresh_count >= IMU_BARO_REFRESH_DIV) {
        g_baro_ctx.refresh_count = 0U;
    }

    sample->timestamp_ms = bsp_board_millis();
    sample->pressure_pa = g_baro_ctx.pressure_pa;
    sample->pressure_reference_pa = g_baro_ctx.pressure_reference_pa;
    sample->temperature_c = g_baro_ctx.temperature_c;
    sample->altitude_m = g_baro_ctx.altitude_m;
    sample->healthy = g_baro_ctx.valid;

    return g_baro_ctx.valid;
#else
    if (sample != 0) {
        sample->timestamp_ms = bsp_board_millis();
        sample->pressure_pa = FC_SEA_LEVEL_PRESSURE_PA;
        sample->pressure_reference_pa = FC_SEA_LEVEL_PRESSURE_PA;
        sample->temperature_c = 25.0f;
        sample->altitude_m = 0.0f;
        sample->healthy = false;
    }

    return false;
#endif
}
