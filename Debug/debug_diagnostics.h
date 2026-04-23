#ifndef DEBUG_DIAGNOSTICS_H
#define DEBUG_DIAGNOSTICS_H

#include "FreeRTOS.h"
#include <stdint.h>

#define DEBUG_DIAG_APP_TASK_COUNT  4U

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
    UBaseType_t stack_free_words[DEBUG_DIAG_APP_TASK_COUNT];
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
    uint32_t enabled;
    uint32_t pending;
    uint32_t active;
    uint32_t own_core;
} debug_irq_route_diag_t;

typedef struct {
    uint32_t irq_count;
    uint32_t rx_count;
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
    debug_imu_uart_diag_t imu;
    uint32_t tx_timeout_count;
} debug_uart_diag_t;

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

extern volatile debug_boot_diag_t g_dbg_boot;
extern volatile debug_fault_diag_t g_dbg_fault;
extern volatile debug_trap_diag_t g_dbg_trap;
extern volatile debug_task_diag_t g_dbg_tasks;
extern volatile debug_uart_diag_t g_dbg_uart;
extern volatile debug_bus_diag_t g_dbg_bus;
extern volatile debug_console_diag_t g_dbg_console;
extern volatile debug_kernel_diag_t g_dbg_kernel;

#endif
