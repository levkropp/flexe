/* Descriptor-driven ESP32-family timer groups and main watchdogs. */
#ifndef FLEXE_TIMER_GROUP_H
#define FLEXE_TIMER_GROUP_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_timer_group flexe_timer_group_t;

typedef enum {
    FLEXE_TIMER_GROUP_WDT_RESET_CPU = 2,
    FLEXE_TIMER_GROUP_WDT_RESET_SYSTEM = 3,
} flexe_timer_group_wdt_action_t;

typedef void (*flexe_timer_group_state_fn)(void *ctx);
typedef void (*flexe_timer_group_irq_fn)(void *ctx, unsigned group,
                                         unsigned event, bool level);
typedef void (*flexe_timer_group_reset_fn)(
    void *ctx, unsigned group, flexe_timer_group_wdt_action_t action);

/* Create the V1 register blocks described by mem's target. The supplied
 * fallback owns unrecognized offsets, which lets RTC calibration compose
 * into the same 4 KiB pages without either model swallowing the other. */
flexe_timer_group_t *flexe_timer_group_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_timer_group_state_fn state_changed, void *state_ctx,
    flexe_timer_group_irq_fn irq_changed, void *irq_ctx,
    flexe_timer_group_reset_fn reset_requested, void *reset_ctx);
void flexe_timer_group_destroy(flexe_timer_group_t *timer_group);

/* Attach the cores that advance the shared target timeline. */
void flexe_timer_group_attach_cpus(flexe_timer_group_t *timer_group,
                                   xtensa_cpu_t *cpu0,
                                   xtensa_cpu_t *cpu1);

/* Apply one timer group's containing SoC clock/reset signals at the current
 * shared-time boundary. Other groups remain independently clocked. */
void flexe_timer_group_set_system_state(
    flexe_timer_group_t *timer_group, unsigned group,
    bool clock_enabled, bool reset_asserted);

/* Event-scheduler integration. Deadlines are returned in the calling CPU's
 * CCOUNT frame so alarms and watchdog stages wake WAITI at their boundary. */
uint32_t flexe_timer_group_next_event(flexe_timer_group_t *timer_group,
                                      xtensa_cpu_t *cpu);
void flexe_timer_group_eval(flexe_timer_group_t *timer_group);

#endif /* FLEXE_TIMER_GROUP_H */
