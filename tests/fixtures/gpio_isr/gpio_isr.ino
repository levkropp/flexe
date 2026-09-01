/* Edge-triggered GPIO interrupts through the real driver.
 *
 * attachInterrupt() goes through gpio_install_isr_service(), which allocates
 * the shared GPIO interrupt and dispatches per-pin handlers from the ISR.
 * Nothing else in the tree exercises that: the peripheral gates that use GPIO
 * drive it as a level input and poll. A CYD's touch controller signals with
 * PENIRQ, so this path matters on the target hardware.
 */
#include <Arduino.h>

#define PIN_RISING   4
#define PIN_FALLING  16
#define PIN_CHANGE   17

#define SUCCESS_MARKER 0x9109150Bu
#define FAIL_BASE      0xBAD00000u

volatile uint32_t flexe_gpio_stage = 0;
volatile uint32_t flexe_gpio_result[8];

static volatile uint32_t rising_count = 0;
static volatile uint32_t falling_count = 0;
static volatile uint32_t change_count = 0;
static volatile uint32_t last_level = 0xFF;

static void IRAM_ATTR on_rising()  { rising_count++; }
static void IRAM_ATTR on_falling() { falling_count++; }
static void IRAM_ATTR on_change()  {
    change_count++;
    last_level = (uint32_t)digitalRead(PIN_CHANGE);
}

void setup() {
    pinMode(PIN_RISING, INPUT);
    pinMode(PIN_FALLING, INPUT);
    pinMode(PIN_CHANGE, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_RISING), on_rising, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_FALLING), on_falling, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_CHANGE), on_change, CHANGE);
    flexe_gpio_stage = 1;   /* host: drive the pins now */
}

void loop() {
    flexe_gpio_result[0] = rising_count;
    flexe_gpio_result[1] = falling_count;
    flexe_gpio_result[2] = change_count;
    flexe_gpio_result[3] = last_level;

    if (flexe_gpio_stage == 2) {
        /* Host has finished driving edges. A detached handler must stop
         * counting, which is the other half of the ISR service working. */
        detachInterrupt(digitalPinToInterrupt(PIN_RISING));
        flexe_gpio_result[4] = rising_count;
        flexe_gpio_stage = 3;
    } else if (flexe_gpio_stage == 4) {
        flexe_gpio_result[5] = rising_count;   /* must equal result[4] */
        flexe_gpio_stage = SUCCESS_MARKER;
    }
    delay(1);
}
