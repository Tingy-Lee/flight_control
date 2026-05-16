#include "drivers/sensors/gps.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_uart.h"
#include "debug_diagnostics.h"
#include <string.h>

#define GPS_POLL_MAX_BYTES       256U
#define GPS_NMEA_MAX_LENGTH      128U
#define GPS_KNOT_TO_MPS          0.514444f

static gps_sample_t g_gps_latest;
static uint32_t g_gps_last_sentence_ms;
static bool g_gps_have_sample;

static char g_nmea_line[GPS_NMEA_MAX_LENGTH];
static uint8_t g_nmea_line_len;
static bool g_nmea_line_active;

static void gps_debug_copy_sentence(const char *sentence)
{
    uint8_t i = 0U;

    if (sentence == 0) {
        g_dbg_gps.last_sentence[0] = '\0';
        return;
    }

    while ((sentence[i] != '\0') && (i < (DEBUG_GPS_LAST_SENTENCE_LEN - 1U))) {
        g_dbg_gps.last_sentence[i] = sentence[i];
        i++;
    }
    g_dbg_gps.last_sentence[i] = '\0';
}

static int8_t hex_nibble(char ch)
{
    if ((ch >= '0') && (ch <= '9')) {
        return (int8_t)(ch - '0');
    }

    if ((ch >= 'A') && (ch <= 'F')) {
        return (int8_t)(ch - 'A' + 10);
    }

    if ((ch >= 'a') && (ch <= 'f')) {
        return (int8_t)(ch - 'a' + 10);
    }

    return -1;
}

static bool nmea_checksum_ok(const char *sentence)
{
    const char *star = 0;
    uint8_t checksum = 0U;
    int8_t high;
    int8_t low;

    if ((sentence == 0) || (sentence[0] != '$')) {
        return false;
    }

    for (const char *p = sentence + 1; *p != '\0'; p++) {
        if (*p == '*') {
            star = p;
            break;
        }
        checksum ^= (uint8_t)(*p);
    }

    if (star == 0) {
        return false;
    }

    high = hex_nibble(star[1]);
    low = hex_nibble(star[2]);
    if ((high < 0) || (low < 0)) {
        return false;
    }

    return checksum == (uint8_t)(((uint8_t)high << 4) | (uint8_t)low);
}

static bool nmea_has_id(const char *sentence, const char *id)
{
    return (sentence != 0) &&
           (sentence[0] == '$') &&
           (sentence[3] == id[0]) &&
           (sentence[4] == id[1]) &&
           (sentence[5] == id[2]) &&
           (sentence[6] == ',');
}

static bool nmea_get_field(const char *sentence, uint8_t target_index, const char **field, uint8_t *len)
{
    const char *start;
    uint8_t index = 0U;

    if ((sentence == 0) || (field == 0) || (len == 0) || (sentence[0] != '$')) {
        return false;
    }

    start = sentence + 1;
    for (const char *p = start; ; p++) {
        const char ch = *p;

        if ((ch == ',') || (ch == '*') || (ch == '\0')) {
            if (index == target_index) {
                *field = start;
                *len = (uint8_t)(p - start);
                return true;
            }

            if (ch != ',') {
                break;
            }

            index++;
            start = p + 1;
        }
    }

    return false;
}

static bool parse_decimal_field(const char *field, uint8_t len, double *value)
{
    double result = 0.0;
    double fraction = 0.1;
    bool negative = false;
    bool after_decimal = false;
    bool digit_seen = false;

    if ((field == 0) || (value == 0) || (len == 0U)) {
        return false;
    }

    for (uint8_t i = 0U; i < len; i++) {
        const char ch = field[i];

        if ((i == 0U) && ((ch == '-') || (ch == '+'))) {
            negative = ch == '-';
            continue;
        }

        if (ch == '.') {
            if (after_decimal) {
                return false;
            }
            after_decimal = true;
            continue;
        }

        if ((ch < '0') || (ch > '9')) {
            return false;
        }

        digit_seen = true;
        if (after_decimal) {
            result += (double)(ch - '0') * fraction;
            fraction *= 0.1;
        } else {
            result = (result * 10.0) + (double)(ch - '0');
        }
    }

    if (!digit_seen) {
        return false;
    }

    *value = negative ? -result : result;
    return true;
}

static bool parse_uint8_field(const char *field, uint8_t len, uint8_t *value)
{
    uint16_t result = 0U;

    if ((field == 0) || (value == 0) || (len == 0U)) {
        return false;
    }

    for (uint8_t i = 0U; i < len; i++) {
        const char ch = field[i];

        if ((ch < '0') || (ch > '9')) {
            return false;
        }

        result = (uint16_t)((result * 10U) + (uint16_t)(ch - '0'));
        if (result > 255U) {
            return false;
        }
    }

    *value = (uint8_t)result;
    return true;
}

static bool parse_coord_field(const char *coord, uint8_t coord_len,
                              const char *hemisphere, uint8_t hemisphere_len,
                              double *degrees)
{
    double raw;
    uint16_t whole_degrees;
    double minutes;

    if ((hemisphere == 0) || (degrees == 0) || (hemisphere_len != 1U)) {
        return false;
    }

    if (!parse_decimal_field(coord, coord_len, &raw) || (raw < 0.0)) {
        return false;
    }

    whole_degrees = (uint16_t)(raw / 100.0);
    minutes = raw - ((double)whole_degrees * 100.0);
    if ((minutes < 0.0) || (minutes >= 60.0)) {
        return false;
    }

    *degrees = (double)whole_degrees + (minutes / 60.0);
    if ((hemisphere[0] == 'S') || (hemisphere[0] == 'W')) {
        *degrees = -*degrees;
    } else if ((hemisphere[0] != 'N') && (hemisphere[0] != 'E')) {
        return false;
    }

    return true;
}

static bool parse_rmc_sentence(const char *sentence, uint32_t now_ms)
{
    const char *field;
    uint8_t len;
    const char *lat;
    const char *ns;
    const char *lon;
    const char *ew;
    uint8_t lat_len;
    uint8_t ns_len;
    uint8_t lon_len;
    uint8_t ew_len;
    double latitude;
    double longitude;
    double speed_knots;

    g_dbg_gps.rmc_sentence_count++;
    if (!nmea_get_field(sentence, 2U, &field, &len) || (len == 0U)) {
        g_dbg_gps.parse_fail_count++;
        return false;
    }

    g_gps_latest.timestamp_ms = now_ms;
    g_gps_last_sentence_ms = now_ms;
    g_gps_have_sample = true;
    if (field[0] != 'A') {
        g_gps_latest.fix_valid = false;
        g_dbg_gps.no_fix_count++;
        return true;
    }

    if (!nmea_get_field(sentence, 3U, &lat, &lat_len) ||
        !nmea_get_field(sentence, 4U, &ns, &ns_len) ||
        !nmea_get_field(sentence, 5U, &lon, &lon_len) ||
        !nmea_get_field(sentence, 6U, &ew, &ew_len) ||
        !parse_coord_field(lat, lat_len, ns, ns_len, &latitude) ||
        !parse_coord_field(lon, lon_len, ew, ew_len, &longitude)) {
        g_gps_latest.fix_valid = false;
        g_dbg_gps.parse_fail_count++;
        return false;
    }

    g_gps_latest.latitude_deg = latitude;
    g_gps_latest.longitude_deg = longitude;
    g_gps_latest.fix_valid = true;
    g_dbg_gps.valid_fix_count++;

    if (nmea_get_field(sentence, 7U, &field, &len)) {
        if (len == 0U) {
            g_gps_latest.ground_speed_mps = 0.0f;
        } else if (parse_decimal_field(field, len, &speed_knots)) {
            g_gps_latest.ground_speed_mps = (float)speed_knots * GPS_KNOT_TO_MPS;
        }
    }

    return true;
}

static bool parse_gga_sentence(const char *sentence, uint32_t now_ms)
{
    const char *field;
    uint8_t len;
    const char *lat;
    const char *ns;
    const char *lon;
    const char *ew;
    uint8_t lat_len;
    uint8_t ns_len;
    uint8_t lon_len;
    uint8_t ew_len;
    uint8_t fix_quality;
    double latitude;
    double longitude;
    double altitude;

    g_dbg_gps.gga_sentence_count++;
    if (!nmea_get_field(sentence, 6U, &field, &len) ||
        !parse_uint8_field(field, len, &fix_quality)) {
        g_dbg_gps.parse_fail_count++;
        return false;
    }

    g_gps_latest.timestamp_ms = now_ms;
    g_gps_last_sentence_ms = now_ms;
    g_gps_have_sample = true;

    if (nmea_get_field(sentence, 7U, &field, &len)) {
        uint8_t satellites;

        if (parse_uint8_field(field, len, &satellites)) {
            g_gps_latest.satellites = satellites;
        }
    }

    if (fix_quality == 0U) {
        g_gps_latest.fix_valid = false;
        g_dbg_gps.no_fix_count++;
        return true;
    }

    if (!nmea_get_field(sentence, 2U, &lat, &lat_len) ||
        !nmea_get_field(sentence, 3U, &ns, &ns_len) ||
        !nmea_get_field(sentence, 4U, &lon, &lon_len) ||
        !nmea_get_field(sentence, 5U, &ew, &ew_len) ||
        !parse_coord_field(lat, lat_len, ns, ns_len, &latitude) ||
        !parse_coord_field(lon, lon_len, ew, ew_len, &longitude)) {
        g_gps_latest.fix_valid = false;
        g_dbg_gps.parse_fail_count++;
        return false;
    }

    g_gps_latest.latitude_deg = latitude;
    g_gps_latest.longitude_deg = longitude;
    g_gps_latest.fix_valid = true;
    g_dbg_gps.valid_fix_count++;

    if (nmea_get_field(sentence, 9U, &field, &len) &&
        parse_decimal_field(field, len, &altitude)) {
        g_gps_latest.altitude_m = (float)altitude;
    }

    return true;
}

static bool parse_nmea_sentence(const char *sentence, uint32_t now_ms)
{
    if (!nmea_checksum_ok(sentence)) {
        g_dbg_gps.checksum_fail_count++;
        return false;
    }

    if (nmea_has_id(sentence, "RMC")) {
        return parse_rmc_sentence(sentence, now_ms);
    }

    if (nmea_has_id(sentence, "GGA")) {
        return parse_gga_sentence(sentence, now_ms);
    }

    g_dbg_gps.unsupported_sentence_count++;
    return false;
}

static bool gps_feed_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte == '$') {
        g_nmea_line[0] = '$';
        g_nmea_line_len = 1U;
        g_nmea_line_active = true;
        g_dbg_gps.line_start_count++;
        g_dbg_gps.line_len = g_nmea_line_len;
        return false;
    }

    if (!g_nmea_line_active) {
        return false;
    }

    if (byte == '\r') {
        return false;
    }

    if (byte == '\n') {
        g_nmea_line[g_nmea_line_len] = '\0';
        g_nmea_line_active = false;
        g_dbg_gps.line_complete_count++;
        g_dbg_gps.line_len = g_nmea_line_len;
        g_dbg_gps.last_sentence_ms = now_ms;
        gps_debug_copy_sentence(g_nmea_line);
        return parse_nmea_sentence(g_nmea_line, now_ms);
    }

    if (g_nmea_line_len >= (GPS_NMEA_MAX_LENGTH - 1U)) {
        g_nmea_line_len = 0U;
        g_nmea_line_active = false;
        g_dbg_gps.line_overflow_count++;
        g_dbg_gps.line_len = 0U;
        return false;
    }

    g_nmea_line[g_nmea_line_len++] = (char)byte;
    g_dbg_gps.line_len = g_nmea_line_len;
    return false;
}

static void gps_apply_stale_timeout(uint32_t now_ms)
{
    if (g_gps_have_sample &&
        g_gps_latest.fix_valid &&
        ((uint32_t)(now_ms - g_gps_last_sentence_ms) > FC_GPS_STALE_TIMEOUT_MS)) {
        g_gps_latest.timestamp_ms = now_ms;
        g_gps_latest.fix_valid = false;
        g_dbg_gps.stale_count++;
    }
}

bool gps_init(void)
{
    memset(&g_gps_latest, 0, sizeof(g_gps_latest));
    g_gps_last_sentence_ms = 0U;
    g_gps_have_sample = false;
    g_nmea_line_len = 0U;
    g_nmea_line_active = false;
    return true;
}

bool gps_poll(gps_sample_t *sample)
{
    uint8_t byte;
    uint16_t budget = GPS_POLL_MAX_BYTES;
    const uint32_t now_ms = bsp_board_millis();
    uint32_t bytes_read = 0U;

    g_dbg_gps.poll_count++;
    while ((budget-- != 0U) && bsp_uart_gps_read_byte(&byte)) {
        (void)gps_feed_byte(byte, now_ms);
        bytes_read++;
    }
    g_dbg_gps.poll_byte_count += bytes_read;

    if (sample == 0) {
        return false;
    }

    gps_apply_stale_timeout(now_ms);
    *sample = g_gps_latest;

    return g_gps_latest.fix_valid;
}
