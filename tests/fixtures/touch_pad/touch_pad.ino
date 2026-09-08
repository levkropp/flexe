/* The ESP32 capacitive touch controller, through the Arduino API.
 *
 * Nothing in the tree drove touch_pad_* before this. src/touch_stubs.c is
 * resistive XPT2046/TFT_eSPI symbol hooks -- a different device on a SPI bus --
 * so the SENS touch registers, the below-threshold status latch and the RTC
 * core interrupt were all unmodelled, and firmware calling touchRead() got
 * whatever a zeroed register file happens to say.
 *
 * The counts run *downwards*: a pad reads lower as capacitance rises, so a
 * touch is a value below the threshold. Getting that backwards would leave an
 * untouched panel looking permanently pressed, which is why this checks a
 * released reading as well as a pressed one rather than just watching for the
 * interrupt.
 *
 * Handshake with the host runs through flexe_touch_stage/flexe_touch_ack: the
 * guest publishes a stage and waits until the host has injected that stage's
 * pad value. This is especially important before the first touchRead(), which
 * benchmarks the channel and configures its initial threshold.
 */
#include <Arduino.h>

#define SUCCESS_MARKER 0x70A0C4EDu
#define FAIL_BASE      0xBAD00000u

#define PAD           T0        /* touch pad 0 = GPIO 4 */
#define THRESHOLD     500
#define RELEASE_FLOOR 990
#define SPIN_LIMIT    200000

volatile uint32_t flexe_touch_stage = 0;
volatile uint32_t flexe_touch_ack = 0;
volatile uint32_t flexe_touch_result[8];

static volatile uint32_t isr_hits = 0;

static void fail(uint32_t code) { flexe_touch_stage = FAIL_BASE | code; }

static void IRAM_ATTR on_touch(void) { isr_hits++; }

static bool await_host(uint32_t stage) {
    flexe_touch_stage = stage;
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) {
        if (flexe_touch_ack == stage) return true;
        delay(1);
    }
    return false;
}

/* Poll the pad until it satisfies `want_below`, or give up. Returns the last
 * value seen so a timeout can report what the hardware was actually saying. */
static uint32_t wait_for(bool want_below, uint32_t threshold, bool *ok) {
    uint32_t v = 0;
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) {
        v = touchRead(PAD);
        if (want_below ? (v < threshold) : (v >= threshold)) { *ok = true; return v; }
        delay(1);
    }
    *ok = false;
    return v;
}

void setup() {
    bool ok = false;

    /* Stage 1: the host injects a released value. A plain read has to see it,
     * which is the part that exercises the SW-start measurement path rather
     * than the interrupt. */
    if (!await_host(1)) { fail(1); return; }
    uint32_t released = wait_for(false, THRESHOLD, &ok);
    flexe_touch_result[0] = released;
    if (!ok) { fail(2); return; }

    /* Stage 2: host presses the pad. */
    if (!await_host(2)) { fail(3); return; }
    uint32_t pressed = wait_for(true, THRESHOLD, &ok);
    flexe_touch_result[1] = pressed;
    if (!ok) { fail(4); return; }
    if (pressed >= released) { fail(5); return; }   /* counts must fall */

    /* Stage 3: release again, then let the IIR-filtered value settle past the
     * channel's benchmark threshold before arming the interrupt. Merely
     * crossing THRESHOLD is not enough: the initial auto-threshold is 98.5%
     * of the released benchmark, and leaving the driver logically active
     * would make the following press lack an active edge. */
    if (!await_host(3)) { fail(6); return; }
    (void)wait_for(false, RELEASE_FLOOR, &ok);
    if (!ok) { fail(7); return; }

    isr_hits = 0;
    touchAttachInterrupt(PAD, on_touch, THRESHOLD);

    /* Stage 4: host presses the pad and the callback must run. */
    if (!await_host(4)) { fail(8); return; }
    for (uint32_t i = 0; i < SPIN_LIMIT && isr_hits == 0; i++)
        delay(1);
    flexe_touch_result[2] = isr_hits;
    if (isr_hits == 0) { fail(9); return; }

    /* Reported for visibility only; see the host side for why its exact value
     * is not asserted. */
    flexe_touch_result[3] = touchRead(PAD);
    flexe_touch_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
