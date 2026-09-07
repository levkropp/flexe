/* Deep and light sleep, through esp_sleep.
 *
 * `grep -rn esp_sleep src` returned nothing before this: none of the RTC_CNTL
 * sleep registers were modelled, so esp_deep_sleep_start() set SLEEP_EN into a
 * register nobody read and the firmware sat in the poll loop that follows it
 * forever. RTC_CNTL_WAKEUP_STATE answered a hardcoded 1, so
 * esp_sleep_get_wakeup_cause() reported EXT0 on a board that had never slept.
 *
 * The run is a boot sequence, not a single pass. A deep-sleep wake is a
 * *reset* -- setup() runs again from the top -- and the only thing that
 * carries across it is RTC memory, so the count below has to be RTC_DATA_ATTR.
 * That is the property worth testing: losing RTC memory across the wake, or
 * reporting the wrong cause, both look like a firmware that simply reboots.
 */
#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

/* A plain RTC pad -- not 25/26, which are also the DAC outputs, and not the
 * 32 kHz crystal pair. Held across deep sleep below. */
#define HOLD_PIN GPIO_NUM_27

#define SUCCESS_MARKER 0x51EEBEEFu
#define FAIL_BASE      0xBAD00000u

#define SLEEP_US       50000ull    /* 50 ms */

volatile uint32_t flexe_sleep_stage = 0;
volatile uint32_t flexe_sleep_result[8];

/* Survives a deep-sleep wake; a plain global does not. */
RTC_DATA_ATTR static uint32_t boot_count;
RTC_DATA_ATTR static uint32_t rtc_magic;
/* The result array is an ordinary global, so the deep-sleep reset zeroes it
 * along with the rest of .bss -- the light-sleep numbers have to be carried in
 * RTC memory to still be there when the host reads them at the end. */
RTC_DATA_ATTR static uint32_t saved_cold_cause;
RTC_DATA_ATTR static uint32_t saved_light_us;
RTC_DATA_ATTR static uint32_t saved_light_cause;

static void fail(uint32_t code) { flexe_sleep_stage = FAIL_BASE | code; }

void setup() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (boot_count == 0 && rtc_magic == 0) {
        /* Cold boot. Nothing has slept yet, so the cause must say so -- this
         * is the check that a hardcoded wake cause fails. */
        if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) { fail(1); return; }

        boot_count = 1;
        rtc_magic = 0xC0FFEE01u;
        saved_cold_cause = (uint32_t)cause;

        /* Light sleep first: execution has to *resume* here, not restart. */
        flexe_sleep_stage = 1;
        int64_t before = esp_timer_get_time();
        esp_sleep_enable_timer_wakeup(SLEEP_US);
        if (esp_light_sleep_start() != ESP_OK) { fail(2); return; }
        int64_t after = esp_timer_get_time();
        saved_light_us = (uint32_t)(after - before);
        saved_light_cause = (uint32_t)esp_sleep_get_wakeup_cause();
        if (boot_count != 1) { fail(3); return; }  /* light sleep is not a reset */

        /* Park an output high and hold it. Deep sleep powers the digital
         * domain down, so without the hold bit the pad returns to its reset
         * state; with it, the level has to survive into the next boot. This
         * is the only way firmware can keep a rail or an enable line asserted
         * while it sleeps. */
        rtc_gpio_init(HOLD_PIN);
        rtc_gpio_set_direction(HOLD_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(HOLD_PIN, 1);
        rtc_gpio_hold_en(HOLD_PIN);

        /* Then deep sleep, which does not return: the chip resets and
         * setup() runs again from the top. */
        flexe_sleep_stage = 2;
        boot_count = 2;
        esp_sleep_enable_timer_wakeup(SLEEP_US);
        esp_deep_sleep_start();
        fail(4);                       /* reached only if the chip never slept */
        return;
    }

    /* Woken from deep sleep. Republish what the first boot measured. */
    flexe_sleep_result[0] = saved_cold_cause;
    flexe_sleep_result[1] = saved_light_us;
    flexe_sleep_result[2] = saved_light_cause;
    flexe_sleep_result[3] = boot_count;
    flexe_sleep_result[4] = rtc_magic;
    flexe_sleep_result[5] = (uint32_t)cause;
    flexe_sleep_result[6] = (uint32_t)esp_reset_reason();

    if (rtc_magic != 0xC0FFEE01u) { fail(5); return; }   /* RTC memory lost */
    if (boot_count != 2)          { fail(6); return; }
    if (cause != ESP_SLEEP_WAKEUP_TIMER) { fail(7); return; }

    /* The held pad must still be driving high. Reading it back is weaker than
     * the host-side check -- a register the reset failed to clear would read
     * the same -- so the host also samples the pin itself across the wake. */
    flexe_sleep_result[7] = (uint32_t)rtc_gpio_get_level(HOLD_PIN);
    if (flexe_sleep_result[7] != 1u) { fail(8); return; }

    /* Releasing the hold has to hand the pad back: drive it low and the pin
     * must follow, or "held" would just mean "stuck". */
    rtc_gpio_hold_dis(HOLD_PIN);
    rtc_gpio_set_level(HOLD_PIN, 0);

    flexe_sleep_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
