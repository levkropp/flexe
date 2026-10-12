/* Soft-AP bring-up and one virtual station.
 *
 * The host harness provisions a single client MAC before the guest runs;
 * the AP_START handler chain, the auto-join at AP start, and a
 * harness-driven leave are all observed through the event payloads, so
 * what is under test is the emulator's association state and event
 * delivery rather than any Arduino wrapper branching on top.
 *
 * Progress is memory-mapped (flexe_ap_stage / flexe_ap_result) rather
 * than printed: the check is the join payload itself.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>

#define SUCCESS_MARKER 0x9F1C0C0Bu
#define FAIL_BASE      0xBAD00000u

#define EXPECTED_MAC_0 0xAAu
#define EXPECTED_MAC_1 0xBBu
#define EXPECTED_MAC_2 0xCCu
#define EXPECTED_MAC_3 0xDDu
#define EXPECTED_MAC_4 0xEEu
#define EXPECTED_MAC_5 0x01u

volatile uint32_t flexe_ap_stage = 0;
volatile uint32_t flexe_ap_result[8];

static void fail(uint32_t code) { flexe_ap_stage = FAIL_BASE | code; }

static volatile uint32_t saw_start = 0;
static volatile uint32_t saw_join = 0;
static volatile uint32_t saw_leave = 0;
static uint8_t join_mac[6];
static uint8_t join_aid = 0;
static uint8_t leave_aid = 0;

static void on_wifi(void *arg, esp_event_base_t base, int32_t id,
                    void *data) {
  (void)arg;
  if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) saw_start++;
  if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
    memcpy(join_mac, e->mac, 6);
    join_aid = e->aid;
    saw_join++;
  }
  if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *e =
        (wifi_event_ap_stadisconnected_t *)data;
    leave_aid = e->aid;
    saw_leave++;
  }
}

void setup() {
  WiFi.mode(WIFI_AP);

  wifi_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  strcpy((char *)cfg.ap.ssid, "FlexeAP");
  strcpy((char *)cfg.ap.password, "secret12");
  cfg.ap.channel = 6;
  cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
  cfg.ap.max_connection = 4;
  if (esp_wifi_set_config(WIFI_IF_AP, &cfg) != ESP_OK) { fail(1); return; }

  /* The configured AP reads back whole: SSID, password, channel,
   * authmode, and client cap. */
  memset(&cfg, 0, sizeof(cfg));
  if (esp_wifi_get_config(WIFI_IF_AP, &cfg) != ESP_OK) { fail(1); return; }
  if (strcmp((char *)cfg.ap.ssid, "FlexeAP") != 0 ||
      strcmp((char *)cfg.ap.password, "secret12") != 0 ||
      cfg.ap.channel != 6 || cfg.ap.authmode != WIFI_AUTH_WPA2_PSK ||
      cfg.ap.max_connection != 4) { fail(1); return; }
  flexe_ap_stage = 1;

  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi, NULL);
  if (esp_wifi_start() != ESP_OK) { fail(2); return; }

  uint32_t waited = 0;
  while (!saw_start && waited < 20000) {
    delay(10);
    waited += 10;
  }
  if (!saw_start) { fail(3); return; }
  flexe_ap_stage = 2;

  waited = 0;
  while (!saw_join && waited < 30000) {
    delay(10);
    waited += 10;
  }
  flexe_ap_result[0] = saw_start;
  flexe_ap_result[1] = saw_join;
  flexe_ap_result[2] = ((uint32_t)join_mac[0] << 24) |
                       ((uint32_t)join_mac[1] << 16) |
                       ((uint32_t)join_mac[2] << 8) | join_mac[3];
  flexe_ap_result[3] = ((uint32_t)join_mac[4] << 8) | join_mac[5];
  flexe_ap_result[4] = join_aid;
  if (!saw_join) { fail(4); return; }
  if (join_mac[0] != EXPECTED_MAC_0 || join_mac[1] != EXPECTED_MAC_1 ||
      join_mac[2] != EXPECTED_MAC_2 || join_mac[3] != EXPECTED_MAC_3 ||
      join_mac[4] != EXPECTED_MAC_4 || join_mac[5] != EXPECTED_MAC_5 ||
      join_aid != 1) { fail(5); return; }
  flexe_ap_stage = 3;

  /* The harness drops the station once it sees stage 3; the leave event
   * with the same AID completes the association lifecycle. */
  waited = 0;
  while (!saw_leave && waited < 30000) {
    delay(10);
    waited += 10;
  }
  flexe_ap_result[5] = saw_leave;
  flexe_ap_result[6] = leave_aid;
  if (!saw_leave) { fail(6); return; }
  if (leave_aid != 1) { fail(7); return; }
  flexe_ap_stage = SUCCESS_MARKER;
}

void loop() { delay(1000); }
