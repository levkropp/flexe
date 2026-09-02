/* Station provisioning and an outbound TCP client.
 *
 * Two things nothing else reaches. First, saved station credentials:
 * esp_wifi_get_config() used to zero the config unconditionally, so the guest
 * could never see credentials from a previous run -- and firmware that
 * provisions WiFi through a captive portal (WiFiManager, and most things
 * built on it) checks exactly that to decide whether to start the portal.
 * Such firmware could never be driven past provisioning at all.
 *
 * Second, an outbound connection. Both stock-ROM scenarios only ever bind and
 * listen, for their captive portals; nothing connects *out*, which is what
 * every firmware that talks to a server actually does.
 *
 * The host harness writes its listening port into flexe_wifi_port, then
 * echoes whatever it is sent.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>

#define SUCCESS_MARKER 0x9F1C0C0Bu
#define FAIL_BASE      0xBAD00000u

#define PAYLOAD_LEN 1000

volatile uint32_t flexe_wifi_stage = 0;
volatile uint32_t flexe_wifi_port = 0;      /* host writes the port here */
volatile uint32_t flexe_wifi_result[12];

static void fail(uint32_t code) { flexe_wifi_stage = FAIL_BASE | code; }

static volatile uint32_t saw_connected = 0;
static volatile uint32_t saw_got_ip = 0;
static volatile uint32_t event_ssid_hash = 0;
static volatile uint32_t event_ip = 0;

static uint32_t fnv1a_fwd(const uint8_t *p, size_t n);

/* The events the emulator delivers. Registering a handler and checking what
 * arrives is the direct test of event delivery, which is what Arduino's
 * WiFi.status() is built on. */
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
    wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
    event_ssid_hash = fnv1a_fwd(e->ssid, e->ssid_len);
    saw_connected++;
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    event_ip = (uint32_t)e->ip_info.ip.addr;
    saw_got_ip++;
  }
}

static uint32_t fnv1a(const uint8_t *p, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
  return h;
}

static uint32_t fnv1a_fwd(const uint8_t *p, size_t n) { return fnv1a(p, n); }

void setup() {
  WiFi.mode(WIFI_STA);

  /* The credentials a previous run would have saved. This is the check
   * WiFiManager and everything built on it makes to decide whether to start a
   * captive portal, so it is the one that matters: WiFi.SSID() reads the
   * *associated* AP instead, and is empty until we are on the air. */
  wifi_config_t conf;
  memset(&conf, 0, sizeof(conf));
  if (esp_wifi_get_config(WIFI_IF_STA, &conf) != ESP_OK) { fail(1); return; }
  size_t saved_len = strnlen((const char *)conf.sta.ssid,
                             sizeof(conf.sta.ssid));
  flexe_wifi_result[0] = (uint32_t)saved_len;
  flexe_wifi_result[1] = fnv1a(conf.sta.ssid, saved_len);
  if (saved_len == 0) { fail(1); return; }
  flexe_wifi_stage = 1;

  /* Drive the IDF API directly rather than WiFi.begin(): what is under test
   * is the emulator's association and event delivery, not the Arduino
   * wrapper's own branching on top of it. */
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL);
  esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL);
  if (esp_wifi_connect() != ESP_OK) { fail(2); return; }

  uint32_t waited = 0;
  while ((saw_connected == 0 || saw_got_ip == 0) && waited < 5000) {
    delay(10);
    waited += 10;
  }
  flexe_wifi_result[2] = saw_connected;
  flexe_wifi_result[3] = waited;
  flexe_wifi_result[9] = event_ip;
  if (saw_connected == 0 || saw_got_ip == 0) { fail(2); return; }
  if (event_ssid_hash != flexe_wifi_result[1]) { fail(10); return; }

  /* Once associated, the firmware must be able to say what it joined -- this
   * is what a status display reads. */
  wifi_ap_record_t ap;
  memset(&ap, 0, sizeof(ap));
  esp_err_t ap_err = esp_wifi_sta_get_ap_info(&ap);
  size_t joined_len = strnlen((const char *)ap.ssid, sizeof(ap.ssid));
  flexe_wifi_result[11] = (ap_err == ESP_OK)
                          ? fnv1a(ap.ssid, joined_len) : 0xDEAD0000u;
  flexe_wifi_stage = 2;

  /* And now the path real firmware actually takes. WiFi.begin() returns
   * WL_CONNECT_FAILED without ever calling esp_wifi_connect() if
   * esp_netif_dhcpc_start() reports failure, which is what happened when it
   * was unmodelled -- so this covers the Arduino wrapper as well as the IDF
   * calls above. */
  WiFi.disconnect();
  delay(20);
  saw_connected = 0;
  saw_got_ip = 0;
  WiFi.begin("flexe-net", "flexe-secret");
  waited = 0;
  while ((saw_connected == 0 || saw_got_ip == 0) && waited < 5000) {
    delay(10);
    waited += 10;
  }
  flexe_wifi_result[10] = saw_got_ip;
  if (saw_got_ip == 0) { fail(11); return; }

  /* Wait for the harness to publish the port it is listening on. */
  uint32_t spins = 0;
  while (flexe_wifi_port == 0 && spins < 2000) { delay(1); spins++; }
  if (flexe_wifi_port == 0) { fail(3); return; }

  WiFiClient client;
  if (!client.connect(IPAddress(127, 0, 0, 1), (uint16_t)flexe_wifi_port)) {
    fail(4);
    return;
  }
  flexe_wifi_stage = 3;

  /* A short round trip first: proves the connection carries data at all. */
  const char *ping = "PING";
  if (client.write((const uint8_t *)ping, 4) != 4) { fail(5); return; }
  uint8_t reply[8] = {0};
  uint32_t deadline = 0;
  size_t got = 0;
  while (got < 4 && deadline < 5000) {
    int n = client.read(reply + got, 4 - got);
    if (n > 0) got += (size_t)n;
    else { delay(1); deadline++; }
  }
  flexe_wifi_result[4] = (uint32_t)got;
  flexe_wifi_result[5] = fnv1a(reply, 4);
  if (got != 4 || memcmp(reply, ping, 4) != 0) { fail(6); return; }
  flexe_wifi_stage = 4;

  /* Then a transfer long enough to span several segments, checked byte by
   * byte so a truncated or reordered read cannot look plausible. */
  static uint8_t tx[PAYLOAD_LEN];
  static uint8_t rx[PAYLOAD_LEN];
  for (int i = 0; i < PAYLOAD_LEN; i++)
    tx[i] = (uint8_t)((i * 7) ^ (i >> 5) ^ 0x5A);
  size_t sent = 0;
  while (sent < PAYLOAD_LEN) {
    int n = client.write(tx + sent, PAYLOAD_LEN - sent);
    if (n <= 0) { fail(7); return; }
    sent += (size_t)n;
  }
  got = 0;
  deadline = 0;
  while (got < PAYLOAD_LEN && deadline < 20000) {
    int n = client.read(rx + got, PAYLOAD_LEN - got);
    if (n > 0) got += (size_t)n;
    else { delay(1); deadline++; }
  }
  flexe_wifi_result[6] = (uint32_t)got;
  flexe_wifi_result[7] = fnv1a(rx, got);
  flexe_wifi_result[8] = fnv1a(tx, PAYLOAD_LEN);
  if (got != PAYLOAD_LEN) { fail(8); return; }
  for (int i = 0; i < PAYLOAD_LEN; i++)
    if (rx[i] != tx[i]) { fail(9); return; }

  client.stop();
  flexe_wifi_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
