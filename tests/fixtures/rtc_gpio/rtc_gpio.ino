/* RTC GPIO, through driver/rtc_io.h.
 *
 * `grep -rn rtc_gpio src` returned nothing before this. The RTCIO register file
 * was decoded only far enough for the two DAC channels, so RTC_GPIO_OUT and
 * RTC_GPIO_ENABLE landed in a private array nothing read and RTC_GPIO_IN always
 * answered zero -- rtc_gpio_set_level() drove nothing and rtc_gpio_get_level()
 * saw nothing.
 *
 * Both directions are checked against the pad itself rather than against the
 * digital GPIO block. rtc_gpio_init() switches the pad to the RTC mux and
 * disconnects the digital input path, so GPIO_IN is not a defined view of an
 * RTC-muxed pad -- reading it back through digitalRead() would be asserting
 * something hardware does not promise. The host owns the pad, so the output
 * direction is checked there and the input direction is driven from there.
 */
#include <Arduino.h>
#include <driver/rtc_io.h>

#define SUCCESS_MARKER 0x27C6D10Au
#define FAIL_BASE      0xBAD00000u

#define OUT_PIN   GPIO_NUM_25    /* RTC channel 6  */
#define IN_PIN    GPIO_NUM_26    /* RTC channel 7  */
#define SPIN_LIMIT 200000

volatile uint32_t flexe_rtcio_stage = 0;
volatile uint32_t flexe_rtcio_result[8];

static void fail(uint32_t code) { flexe_rtcio_stage = FAIL_BASE | code; }

void setup() {
    if (!rtc_gpio_is_valid_gpio(OUT_PIN)) { fail(1); return; }
    if (!rtc_gpio_is_valid_gpio(IN_PIN))  { fail(2); return; }

    if (rtc_gpio_init(OUT_PIN) != ESP_OK) { fail(3); return; }
    if (rtc_gpio_set_direction(OUT_PIN, RTC_GPIO_MODE_OUTPUT_ONLY) != ESP_OK) {
        fail(4); return;
    }
    if (rtc_gpio_init(IN_PIN) != ESP_OK) { fail(5); return; }
    if (rtc_gpio_set_direction(IN_PIN, RTC_GPIO_MODE_INPUT_ONLY) != ESP_OK) {
        fail(6); return;
    }

    /* Stage 1: drive the pad high from the RTC domain. The host checks the
     * pad, so this catches a register file nothing is wired to. */
    rtc_gpio_set_level(OUT_PIN, 1);
    flexe_rtcio_stage = 1;
    delay(20);

    rtc_gpio_set_level(OUT_PIN, 0);
    flexe_rtcio_stage = 2;
    delay(20);

    /* Stage 3: the host drives the input pad; the RTC domain has to see it. */
    flexe_rtcio_stage = 3;
    uint32_t seen = 0;
    bool ok = false;
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) {
        seen = rtc_gpio_get_level(IN_PIN);
        if (seen == 1) { ok = true; break; }
        delay(1);
    }
    flexe_rtcio_result[2] = seen;
    if (!ok) { fail(7); return; }

    flexe_rtcio_stage = 4;
    ok = false;
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) {
        seen = rtc_gpio_get_level(IN_PIN);
        if (seen == 0) { ok = true; break; }
        delay(1);
    }
    flexe_rtcio_result[3] = seen;
    if (!ok) { fail(8); return; }

    flexe_rtcio_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
