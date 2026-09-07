/* CCOUNT and the microsecond clock have to advance together, across every
 * kind of wait.
 *
 * The emulator keeps three clocks: the CCOUNT register the guest reads with
 * RSR, the count of cycles it actually executed, and the time it skipped past
 * without executing any. Guest-visible time is the sum of the last two, and
 * nothing checked it against the first -- so a stub could model a wait by
 * crediting elapsed time while leaving CCOUNT where it was. That is exactly
 * what ets_delay_us did: firmware saw the microsecond clock move and CCOUNT
 * stand still. It matters because CCOMPARE, and therefore the FreeRTOS tick on
 * a firmware running its own kernel, is scheduled against CCOUNT: on Marauder
 * 28,622 of those calls swallowed 2.7 s of a 36 s run and the guest tick ran at
 * 925 Hz instead of 1000 Hz.
 *
 * Each phase below waits the same amount of guest time by a different route,
 * and reports how far each clock moved. The host side requires the ratio to be
 * the CPU frequency every time. A phase that fast-forwards one clock and not
 * the other fails here and nowhere else -- the delay still *returns* after the
 * right amount of time, so nothing that only watches esp_timer_get_time() can
 * see it.
 */
#include <Arduino.h>
#include <esp_timer.h>

#define SUCCESS_MARKER 0xC10CC0DEu
#define FAIL_BASE      0xBAD00000u

#define PHASE_US   50000u    /* guest time each phase should consume */
#define ROM_STEP   500u      /* delayMicroseconds() granularity in phase 2 */

volatile uint32_t flexe_clock_stage = 0;
volatile uint32_t flexe_clock_result[16];

static void fail(uint32_t code) { flexe_clock_stage = FAIL_BASE | code; }

/* Executed work: the control. Both clocks advance the ordinary way here, so a
 * failure in this phase means something far more basic is wrong than the
 * fast-forward paths the other three exercise. */
static void phase_spin(void) {
    int64_t until = esp_timer_get_time() + PHASE_US;
    while (esp_timer_get_time() < until) { }
}

/* The ROM busy-wait, called directly. Not through delayMicroseconds(): that
 * spins on esp_timer_get_time() in guest code and never reaches the ROM at all,
 * so routing this phase through it would silently make it a second copy of
 * phase_spin. ets_delay_us is what the IDF drivers and the PHY bring-up call,
 * and on real silicon it spins on CCOUNT itself. */
extern "C" void ets_delay_us(uint32_t us);

static void phase_rom_delay(void) {
    for (uint32_t i = 0; i < PHASE_US / ROM_STEP; i++)
        ets_delay_us(ROM_STEP);
}

/* Arduino's delay(), which is vTaskDelay() underneath: the task blocks and the
 * core idles rather than spinning. */
static void phase_arduino_delay(void) { delay(PHASE_US / 1000u); }

/* The same wait asked for directly, bypassing the Arduino shim. */
static void phase_task_delay(void) { vTaskDelay(pdMS_TO_TICKS(PHASE_US / 1000u)); }

/* Run one phase and record how far each clock moved. CCOUNT is 32-bit and
 * wraps every ~18 s at 240 MHz; a 50 ms phase cannot span a wrap, and unsigned
 * subtraction is correct even if one lands on it. */
static void measure(void (*body)(void), unsigned slot) {
    uint32_t c0 = ESP.getCycleCount();
    int64_t  t0 = esp_timer_get_time();
    body();
    uint32_t c1 = ESP.getCycleCount();
    int64_t  t1 = esp_timer_get_time();
    flexe_clock_result[slot]     = c1 - c0;
    flexe_clock_result[slot + 1] = (uint32_t)(t1 - t0);
}

void setup() {
    flexe_clock_stage = 1;

    /* Report the frequency the host should hold the ratios to. Reading it
     * before any measurement also gets the ROM's ticks-per-microsecond word
     * settled, which is where the emulator takes its own figure from. */
    flexe_clock_result[0] = getCpuFrequencyMhz();
    if (flexe_clock_result[0] < 10u || flexe_clock_result[0] > 240u) {
        fail(1);
        return;
    }

    measure(phase_spin,          2);
    measure(phase_rom_delay,     4);
    measure(phase_arduino_delay, 6);
    measure(phase_task_delay,    8);

    /* Neither clock may stand still in any phase: a zero would otherwise make
     * the host's ratio check vacuous. */
    for (unsigned s = 2; s <= 8; s += 2) {
        if (flexe_clock_result[s] == 0u)     { fail(10u + s); return; }
        if (flexe_clock_result[s + 1] == 0u) { fail(20u + s); return; }
    }

    flexe_clock_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
