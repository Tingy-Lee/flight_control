#include "drivers/sensors/rc_input.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_uart.h"
#include "debug_diagnostics.h"
#include <string.h>

#define RC_INPUT_POLL_MAX_BYTES       128U
#define IBUS_FRAME_SIZE               32U
#define IBUS_FRAME_LENGTH             0x20U
#define IBUS_COMMAND_CHANNELS         0x40U
#define IBUS_CHECKSUM_INDEX           30U
#define IBUS_PROTOCOL_CHANNEL_COUNT   14U

typedef struct {
    uint8_t frame[IBUS_FRAME_SIZE];
    uint8_t frame_index;
    uint16_t channels_us[IBUS_PROTOCOL_CHANNEL_COUNT];
    uint32_t last_frame_ms;
    bool frame_ready;
    bool failsafe;
    bool frame_lost;
} rc_input_ctx_t;

static rc_input_ctx_t g_rc_ctx;

static void rc_debug_record_byte(uint8_t byte)
{
    g_dbg_rc_input.byte_window[g_dbg_rc_input.byte_window_index & 0x1FU] = byte;
    g_dbg_rc_input.byte_window_index++;
}

static uint16_t rc_pwm_clamp_us(uint16_t value)
{
    if (value < FC_MOTOR_PWM_MIN_US) {
        return FC_MOTOR_PWM_MIN_US;
    }

    if (value > FC_MOTOR_PWM_MAX_US) {
        return FC_MOTOR_PWM_MAX_US;
    }

    return value;
}

static void rc_input_set_default(rc_input_t *rc, uint32_t timestamp_ms)
{
    uint8_t channel;

    rc->timestamp_ms = timestamp_ms;
    for (channel = 0U; channel < FC_RC_INPUT_CHANNEL_COUNT; channel++) {
        rc->channels_us[channel] = FC_RC_PWM_CENTER_US;
    }

    rc->roll_us = FC_RC_PWM_CENTER_US;
    rc->pitch_us = FC_RC_PWM_CENTER_US;
    rc->throttle_us = FC_MOTOR_PWM_MIN_US;
    rc->yaw_us = FC_RC_PWM_CENTER_US;
    rc->arm_switch = false;
    rc->frame_lost = false;
    rc->failsafe = true;
    rc->healthy = false;
}

static uint16_t ibus_checksum_expected(const uint8_t *frame)
{
    uint8_t i;
    uint16_t checksum = 0xFFFFU;

    for (i = 0U; i < IBUS_CHECKSUM_INDEX; i++) {
        checksum = (uint16_t)(checksum - frame[i]);
    }

    return checksum;
}

static bool ibus_checksum_is_valid(const uint8_t *frame)
{
    const uint16_t expected = ibus_checksum_expected(frame);
    const uint16_t received = (uint16_t)frame[IBUS_CHECKSUM_INDEX] |
                              ((uint16_t)frame[IBUS_CHECKSUM_INDEX + 1U] << 8U);

    return expected == received;
}

static bool ibus_parse_frame(const uint8_t *frame)
{
    uint8_t channel;

    if (frame == 0) {
        g_dbg_rc_input.invalid_frame_count++;
        return false;
    }

    g_dbg_rc_input.last_flags = frame[1];
    g_dbg_rc_input.last_footer = frame[IBUS_CHECKSUM_INDEX + 1U];
    g_dbg_rc_input.last_frame_inverted = 0U;

    if ((frame[0] != IBUS_FRAME_LENGTH) || (frame[1] != IBUS_COMMAND_CHANNELS)) {
        g_dbg_rc_input.bad_header_count++;
        g_dbg_rc_input.invalid_frame_count++;
        return false;
    }

    if (!ibus_checksum_is_valid(frame)) {
        g_dbg_rc_input.bad_footer_count++;
        g_dbg_rc_input.invalid_frame_count++;
        return false;
    }

    for (channel = 0U; channel < IBUS_PROTOCOL_CHANNEL_COUNT; channel++) {
        const uint8_t offset = (uint8_t)(2U + (channel * 2U));
        const uint16_t value = (uint16_t)frame[offset] |
                               ((uint16_t)frame[offset + 1U] << 8U);

        g_rc_ctx.channels_us[channel] = rc_pwm_clamp_us(value);
        g_dbg_rc_input.raw_channels[channel] = value;
    }

    g_rc_ctx.frame_lost = false;
    g_rc_ctx.failsafe = false;
    g_rc_ctx.last_frame_ms = bsp_board_millis();
    g_rc_ctx.frame_ready = true;
    g_dbg_rc_input.valid_frame_count++;
    g_dbg_rc_input.last_frame_ms = g_rc_ctx.last_frame_ms;
    return true;
}

static void ibus_reset_frame(uint8_t next_index)
{
    g_rc_ctx.frame_index = next_index;
    g_dbg_rc_input.frame_index = next_index;
}

static void ibus_feed_byte(uint8_t byte)
{
    g_dbg_rc_input.last_byte = byte;
    rc_debug_record_byte(byte);

    if (g_rc_ctx.frame_index == 0U) {
        if (byte != IBUS_FRAME_LENGTH) {
            g_dbg_rc_input.bad_header_count++;
            return;
        }
    } else if ((g_rc_ctx.frame_index == 1U) && (byte != IBUS_COMMAND_CHANNELS)) {
        g_dbg_rc_input.invalid_frame_count++;
        if (byte == IBUS_FRAME_LENGTH) {
            g_rc_ctx.frame[0] = byte;
            ibus_reset_frame(1U);
        } else {
            ibus_reset_frame(0U);
        }
        return;
    }

    g_rc_ctx.frame[g_rc_ctx.frame_index++] = byte;
    g_dbg_rc_input.frame_index = g_rc_ctx.frame_index;
    if (g_rc_ctx.frame_index < IBUS_FRAME_SIZE) {
        return;
    }

    (void)ibus_parse_frame(g_rc_ctx.frame);
    ibus_reset_frame(0U);
}

bool rc_input_init(void)
{
    memset(&g_rc_ctx, 0, sizeof(g_rc_ctx));
#if FC_ENABLE_RC_USART3
    bsp_uart_rc_start_rx_irq();
#endif
    return true;
}

bool rc_input_poll(rc_input_t *rc)
{
    uint8_t byte;
    uint8_t channel;
    uint32_t bytes_read = 0U;
    uint16_t budget = RC_INPUT_POLL_MAX_BYTES;
    const uint32_t now_ms = bsp_board_millis();
    bool frame_stale;

    if (rc == 0) {
        return false;
    }

    g_dbg_rc_input.poll_count++;

    while ((budget-- != 0U) && bsp_uart_rc_read_byte(&byte)) {
        ibus_feed_byte(byte);
        bytes_read++;
    }

    g_dbg_rc_input.poll_byte_count += bytes_read;
    if (bytes_read == 0U) {
        g_dbg_rc_input.no_data_count++;
    }

    frame_stale = (!g_rc_ctx.frame_ready) ||
                  ((uint32_t)(bsp_board_millis() - g_rc_ctx.last_frame_ms) > FC_RC_IBUS_STALE_TIMEOUT_MS);
    if (frame_stale) {
        g_dbg_rc_input.stale_count++;
        rc_input_set_default(rc, now_ms);
        return false;
    }

    rc->timestamp_ms = g_rc_ctx.last_frame_ms;
    for (channel = 0U; channel < FC_RC_INPUT_CHANNEL_COUNT; channel++) {
        rc->channels_us[channel] = g_rc_ctx.channels_us[channel];
        g_dbg_rc_input.channels_us[channel] = rc->channels_us[channel];
    }

    rc->roll_us = rc->channels_us[FC_RC_CHANNEL_ROLL];
    rc->pitch_us = rc->channels_us[FC_RC_CHANNEL_PITCH];
    rc->throttle_us = rc->channels_us[FC_RC_CHANNEL_THROTTLE];
    rc->yaw_us = rc->channels_us[FC_RC_CHANNEL_YAW];
    rc->arm_switch = rc->channels_us[FC_RC_CHANNEL_ARM] >= FC_RC_ARM_THRESHOLD_US;
    rc->frame_lost = g_rc_ctx.frame_lost;
    rc->failsafe = g_rc_ctx.failsafe;
    rc->healthy = !rc->failsafe && !frame_stale;
    if (rc->healthy) {
        g_dbg_rc_input.healthy_count++;
    }

    return rc->healthy;
}
