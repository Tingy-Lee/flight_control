#ifndef DEBUG_DIAGNOSTICS_H
#define DEBUG_DIAGNOSTICS_H

#include "FreeRTOS.h"
#include <stdint.h>

#define DEBUG_DIAG_APP_TASK_COUNT  5U
#define DEBUG_GPS_LAST_SENTENCE_LEN  96U

typedef struct {
    uint32_t startup_phase;
    uint32_t task_entry_mask;
    uint32_t tick_hook_count;
    uint32_t heap_before_scheduler;
} debug_boot_diag_t;

typedef struct {
    const char *file;
    uint32_t line;
    uint32_t count;
    uint32_t heap_free;
    uint32_t heap_min_free;
    uint32_t return_address;
    uint32_t scheduler_state;
    uint32_t tick;
    void *current_task;
} debug_assert_diag_t;

typedef struct {
    uint32_t reason;
    void *task;
    const char *task_name;
    debug_assert_diag_t assert_state;
} debug_fault_diag_t;

typedef struct {
    uint32_t count;
    uint32_t mcause;
    uint32_t mepc;
    uint32_t mtval;
    uint32_t mstatus;
    uint32_t sp;
} debug_hardfault_diag_t;

typedef struct {
    uint32_t nmi_count;
    debug_hardfault_diag_t hardfault;
} debug_trap_diag_t;

typedef struct {
    uint32_t heap_before_create;
    uint32_t heap_after_create[DEBUG_DIAG_APP_TASK_COUNT];
    BaseType_t create_result[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t loop_count[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t last_tick[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t max_exec_ticks[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t overrun_count[DEBUG_DIAG_APP_TASK_COUNT];
    UBaseType_t stack_free_words[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t stack_low_count[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t period_ticks[DEBUG_DIAG_APP_TASK_COUNT];
    uint32_t runtime_heap_free;
    uint32_t runtime_heap_min_free;
} debug_task_diag_t;

typedef struct {
    uint32_t statr;
    uint32_t ctlr1;
    uint32_t ctlr2;
    uint32_t ctlr3;
    uint32_t brr;
} debug_usart_register_diag_t;

typedef struct {
    uint16_t gpiof_indr;
    uint8_t pf3_level;
    uint8_t pf3_af;
    uint32_t afio_pcfr1;
} debug_imu_uart_pin_diag_t;

typedef struct {
    uint16_t gpiob_indr;
    uint8_t pb11_level;
    uint8_t pb11_af;
    uint32_t afio_pcfr1;
} debug_rc_uart_pin_diag_t;

typedef struct {
    uint32_t enabled;
    uint32_t pending;
    uint32_t active;
    uint32_t own_core;
} debug_irq_route_diag_t;

typedef struct {
    uint32_t irq_count;
    uint32_t rx_count;
    uint32_t read_count;
    uint32_t empty_read_count;
    uint32_t overflow_count;
    uint32_t ore_count;
    uint32_t fe_count;
    uint32_t ne_count;
    uint32_t pe_count;
    uint32_t spurious_count;
    uint32_t error_flags;
    uint8_t last_byte;
    uint32_t rx_irq_start_count;
    debug_usart_register_diag_t regs;
    debug_imu_uart_pin_diag_t rx_pin;
    debug_irq_route_diag_t nvic;
} debug_imu_uart_diag_t;

typedef struct {
    uint32_t irq_count;
    uint32_t rx_count;
    uint32_t overflow_count;
    uint32_t ore_count;
    uint32_t fe_count;
    uint32_t ne_count;
    uint32_t pe_count;
    uint32_t spurious_count;
    uint32_t error_flags;
    uint8_t last_byte;
    uint32_t rx_irq_start_count;
    debug_usart_register_diag_t regs;
    debug_rc_uart_pin_diag_t rx_pin;
    debug_irq_route_diag_t nvic;
} debug_rc_uart_diag_t;

typedef struct {
    uint32_t irq_count;
    uint32_t rx_count;
    uint32_t read_count;
    uint32_t empty_read_count;
    uint32_t overflow_count;
    uint32_t ore_count;
    uint32_t fe_count;
    uint32_t ne_count;
    uint32_t pe_count;
    uint32_t spurious_count;
    uint32_t error_flags;
    uint32_t rx_irq_start_count;
    uint8_t last_byte;
    uint16_t rx_read;
    uint16_t rx_write;
    debug_usart_register_diag_t regs;
    debug_irq_route_diag_t nvic;
} debug_gps_uart_diag_t;

typedef struct {
    uint32_t poll_count;
    uint32_t poll_byte_count;
    uint32_t valid_frame_count;
    uint32_t inverted_frame_count;
    uint32_t invalid_frame_count;
    uint32_t stale_count;
    uint32_t failsafe_count;
    uint32_t frame_lost_count;
    uint32_t healthy_count;
    uint32_t bad_header_count;
    uint32_t bad_footer_count;
    uint32_t no_data_count;
    uint32_t last_frame_ms;
    uint8_t frame_index;
    uint8_t last_byte;
    uint8_t last_flags;
    uint8_t last_footer;
    uint8_t last_frame_inverted;
    uint8_t byte_window_index;
    uint8_t byte_window[32];
    uint16_t raw_channels[18];
    uint16_t channels_us[10];
} debug_rc_input_diag_t;

typedef struct {
    debug_gps_uart_diag_t gps;
    debug_rc_uart_diag_t rc;
    debug_imu_uart_diag_t imu;
    uint32_t tx_timeout_count;
} debug_uart_diag_t;

typedef struct {
    uint32_t poll_count;
    uint32_t poll_byte_count;
    uint32_t line_start_count;
    uint32_t line_complete_count;
    uint32_t line_overflow_count;
    uint32_t checksum_fail_count;
    uint32_t unsupported_sentence_count;
    uint32_t rmc_sentence_count;
    uint32_t gga_sentence_count;
    uint32_t parse_fail_count;
    uint32_t valid_fix_count;
    uint32_t no_fix_count;
    uint32_t stale_count;
    uint32_t last_sentence_ms;
    uint8_t line_len;
    char last_sentence[DEBUG_GPS_LAST_SENTENCE_LEN];
} debug_gps_diag_t;

typedef struct {
    uint32_t timeout_count;
} debug_adc_diag_t;

typedef struct {
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t recover_count;
} debug_i2c_diag_t;

typedef struct {
    uint32_t timeout_count;
} debug_spi_diag_t;

typedef struct {
    debug_adc_diag_t adc;
    debug_i2c_diag_t i2c2;
    debug_spi_diag_t spi2;
} debug_bus_diag_t;

typedef struct {
    uint32_t delay_timeout_count;
    uint32_t printf_timeout_count;
} debug_console_diag_t;

typedef struct {
    uint32_t setup_count;
    uint32_t isr_count;
    uint32_t yield_count;
    uint32_t last_ctlr;
    uint32_t last_cmp;
    uint32_t last_isr;
    uint32_t last_cnt;
    uint32_t pended_switch_count;
} debug_systick_diag_t;

typedef struct {
    UBaseType_t count;
    UBaseType_t top_ready_priority;
    UBaseType_t current_priority;
    UBaseType_t scheduler_suspended;
    UBaseType_t ready_lengths[configMAX_PRIORITIES];
    void *current_tcb;
} debug_switch_diag_t;

typedef struct {
    debug_systick_diag_t systick;
    debug_switch_diag_t context_switch;
} debug_kernel_diag_t;

typedef struct {
    uint32_t write_count;
    uint16_t requested_us[4];
    uint16_t clamped_us[4];
    uint16_t tim1_ccr[4];
} debug_pwm_diag_t;

typedef struct {
    uint32_t process_call_count;
    uint32_t process_byte_count;
    uint32_t invalid_length_count;
    uint32_t checksum_fail_count;
    uint32_t short_payload_count;
    uint32_t unknown_frame_count;
    uint32_t motion_frame_count;
    uint32_t euler_frame_count;
    uint32_t baro_frame_count;
    uint32_t version_frame_count;
    uint32_t last_read_ms;
    uint32_t last_motion_ms;
    uint32_t last_euler_ms;
    uint32_t last_baro_ms;
    uint32_t motion_age_ms;
    uint32_t euler_age_ms;
    uint32_t baro_age_ms;
    uint32_t motion_gap_ms;
    uint32_t euler_gap_ms;
    uint32_t baro_gap_ms;
    uint32_t max_motion_gap_ms;
    uint32_t max_euler_gap_ms;
    uint32_t max_baro_gap_ms;
    uint32_t unhealthy_count;
    uint32_t stale_motion_edge_count;
    uint32_t stale_euler_edge_count;
    uint32_t stale_baro_edge_count;
    uint8_t ready;
    uint8_t motion_valid;
    uint8_t euler_valid;
    uint8_t baro_valid;
    uint8_t healthy;
    uint8_t motion_stale;
    uint8_t euler_stale;
    uint8_t baro_stale;
} debug_imu_sensor_diag_t;

extern volatile debug_boot_diag_t g_dbg_boot;
extern volatile debug_fault_diag_t g_dbg_fault;
extern volatile debug_trap_diag_t g_dbg_trap;
extern volatile debug_task_diag_t g_dbg_tasks;
extern volatile debug_uart_diag_t g_dbg_uart;
extern volatile debug_gps_diag_t g_dbg_gps;
extern volatile debug_rc_input_diag_t g_dbg_rc_input;
extern volatile debug_bus_diag_t g_dbg_bus;
extern volatile debug_console_diag_t g_dbg_console;
extern volatile debug_kernel_diag_t g_dbg_kernel;
extern volatile debug_pwm_diag_t g_dbg_pwm;
extern volatile debug_imu_sensor_diag_t g_dbg_imu;

#endif
