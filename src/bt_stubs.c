/*
 * bt_stubs.c — Bluetooth / NimBLE stubs for ESP32 emulator
 *
 * Provides ESP-IDF BT controller stubs and NimBLE API stubs. Supported stock
 * ROMs retain their real NimBLE scanner and receive synthetic controller
 * events through the firmware's registered callback path.
 */

#include "bt_stubs.h"
#include "guest_call.h"
#include "rom_stubs.h"
#include "memory.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ESP-IDF BT controller status values */
#define ESP_BT_CONTROLLER_STATUS_IDLE    0
#define ESP_BT_CONTROLLER_STATUS_INITED  1
#define ESP_BT_CONTROLLER_STATUS_ENABLED 2

/* BT mode values */
#define ESP_BT_MODE_IDLE  0
#define ESP_BT_MODE_BLE   2
#define ESP_BT_MODE_BTDM  3

/* Fake object pointers (in unused DRAM region) */
#define FAKE_SCAN_PTR       0x3FFB0300u
#define FAKE_SERVER_PTR     0x3FFB0400u
#define FAKE_ADVERTISING_PTR 0x3FFB0500u
#define FAKE_CLIENT_PTR     0x3FFB0600u

/* ESP32 Marauder v1.14 CYD production addresses, verified against each
 * release layout's instruction bytes.  v1.14.2 kept the same image entry but
 * shifted both code and DRAM, so the entry point alone is not an identity. */
#define MARAUDER_V114_ENTRY 0x400831D8u
typedef struct {
    uint32_t scan_start;
    uint32_t scan_stop;
    uint32_t scan_handle_gap;
    uint32_t scan_set_callbacks;
    uint32_t gap_adv_start;
    uint32_t hci_cmd_tx;
    uint32_t conn_can_alloc;
    uint32_t id_use_addr;
    uint32_t hs_enabled_literal;
    uint32_t hs_sync_literal;
    uint32_t hs_public_literal;
    uint32_t ignore_list;
    uint32_t connected_peers;
} marauder_bt_layout_t;

static const marauder_bt_layout_t marauder_v11401_bt = {
    0x401048B4u, 0x401049B4u, 0x40104A9Cu, 0x401DA8D0u,
    0x401096E0u, 0x40110254u, 0x4010FC58u, 0x40110CC0u,
    0x4010B4A8u, 0x4010B4B4u, 0x4010B5DCu,
    0x3FFC9534u, 0x3FFC9540u,
};

static const marauder_bt_layout_t marauder_v11423_bt = {
    0x40104F9Cu, 0x4010509Cu, 0x40105184u, 0x401DB19Cu,
    0x40109DC8u, 0x40110920u, 0x40110324u, 0x4011138Cu,
    0x4010B4DCu, 0x4010B4E8u, 0x4010B60Cu,
    0x3FFC9544u, 0x3FFC9550u,
};

/* A ble_gap_event is 52 bytes in this ESP32 NimBLE build.  Its discovery
 * descriptor begins at +4 and holds a pointer to the advertisement payload.
 * This bounded RTC-fast gap lies between the virtual PHY and WiFi buffers. */
#define BLE_EVENT_SCRATCH_ADDR 0x50001C00u
#define BLE_EVENT_SCRATCH_SIZE 64u
#define BLE_DATA_SCRATCH_ADDR  0x50001C40u
#define BLE_DATA_MAX_LEN       31u
#define BLE_GAP_EVENT_DISC     7u
#define BLE_ADV_NONCONN_IND    3u

/* Bluetooth Core legacy LE controller opcodes. */
#define BLE_HCI_LE_SET_ADV_DATA      0x2008u
#define BLE_HCI_LE_SET_SCAN_RSP_DATA 0x2009u
#define BLE_HCI_LE_SET_ADV_ENABLE    0x200Au
#define BLE_HCI_LE_SET_ADV_PARAMS    0x2006u
#define BLE_HCI_LE_SET_SCAN_PARAMS   0x200Bu
#define BLE_HCI_LE_SET_SCAN_ENABLE   0x200Cu
#define BLE_HCI_ERR_INVALID_PARAMS   0x0012u

/* Synthetic BLE devices */
typedef struct {
    char     name[32];
    uint8_t  addr[6];
    int8_t   rssi;
    uint8_t  addr_type;   /* 0=public, 1=random */
} fake_ble_dev_t;

__attribute__((used)) static const fake_ble_dev_t fake_ble_devs[] = {
    { "MI Band 6",     {0xDE,0x85,0x12,0x34,0x56,0x78}, -52, 1 },
    { "AirPods Pro",   {0x4C,0xAB,0xCD,0xEF,0x01,0x23}, -61, 0 },
    { "Tile Mate",     {0xF4,0x5E,0xAB,0x11,0x22,0x33}, -74, 1 },
    { "JBL FLIP 5",    {0x00,0x1A,0x7D,0x44,0x55,0x66}, -58, 0 },
    { "",              {0x7A,0xBF,0xC2,0x77,0x88,0x99}, -83, 1 },
};
#define FAKE_BLE_DEV_COUNT (sizeof(fake_ble_devs) / sizeof(fake_ble_devs[0]))

struct bt_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;
    bool               event_log;

    /* BT controller state */
    int                bt_status;  /* ESP_BT_CONTROLLER_STATUS_* */
    uint32_t           bt_mode;    /* ESP_BT_MODE_* */

    /* BLE state */
    bool               ble_inited;
    bool               ble_scanning;
    bool               ble_advertising;
    uint8_t            ble_addr[6];
    uint32_t           scan_cb_addr;
    uint32_t           scan_obj_addr;
    uint32_t           gap_handler_addr;
    uint32_t           hs_enabled_literal;
    uint32_t           hs_sync_literal;
    uint32_t           hs_public_literal;
    uint32_t           ignore_list_addr;
    uint32_t           connected_peers_addr;
    xtensa_cpu_t      *scan_cpu;
    bool               production_observer;
    uint8_t            advertisement_data[BLE_DATA_MAX_LEN];
    uint8_t            advertisement_len;
    uint8_t            scan_response_data[BLE_DATA_MAX_LEN];
    uint8_t            scan_response_len;
    bt_advertisement_tx_cb advertisement_tx_cb;
    void              *advertisement_tx_ctx;
    bt_stubs_stats_t    stats;
};

/* ===== Calling convention helpers ===== */

static uint32_t bt_arg(xtensa_cpu_t *cpu, int n)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void bt_return(xtensa_cpu_t *cpu, uint32_t retval)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, retval);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, retval);
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

/* ===== Log helper ===== */

static void bt_log(bt_stubs_t *bt, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    if (bt->event_log)
        fprintf(stderr, "[%10llu] BT    ", (unsigned long long)bt->cpu->cycle_count);
    else
        fprintf(stderr, "[bt] ");
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

/* ===== ESP-IDF BT Controller Stubs ===== */

static void stub_esp_bt_controller_init(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->bt_status = ESP_BT_CONTROLLER_STATUS_INITED;
    bt_log(bt, "esp_bt_controller_init()\n");
    bt_return(cpu, 0);
}

static void stub_esp_bt_controller_deinit(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->bt_status = ESP_BT_CONTROLLER_STATUS_IDLE;
    bt_log(bt, "esp_bt_controller_deinit()\n");
    bt_return(cpu, 0);
}

static void stub_esp_bt_controller_enable(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->bt_mode = bt_arg(cpu, 0);
    bt->bt_status = ESP_BT_CONTROLLER_STATUS_ENABLED;
    bt_log(bt, "esp_bt_controller_enable(mode=%u)\n", bt->bt_mode);
    bt_return(cpu, 0);
}

static void stub_esp_bt_controller_disable(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->bt_status = ESP_BT_CONTROLLER_STATUS_INITED;
    bt_log(bt, "esp_bt_controller_disable()\n");
    bt_return(cpu, 0);
}

static void stub_esp_bt_controller_get_status(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_return(cpu, (uint32_t)bt->bt_status);
}

static void stub_esp_bt_controller_mem_release(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_log(bt, "esp_bt_controller_mem_release()\n");
    bt_return(cpu, 0);
}

static void stub_esp_bt_sleep_disable(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    bt_return(cpu, 0);
}

/* esp_ble_gap_set_rand_addr(addr) */
static void stub_esp_ble_gap_set_rand_addr(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    uint32_t addr_ptr = bt_arg(cpu, 0);
    if (addr_ptr) {
        for (int i = 0; i < 6; i++)
            bt->ble_addr[i] = mem_read8(cpu->mem, addr_ptr + (uint32_t)i);
    }
    bt_log(bt, "esp_ble_gap_set_rand_addr(%02x:%02x:%02x:%02x:%02x:%02x)\n",
           bt->ble_addr[0], bt->ble_addr[1], bt->ble_addr[2],
           bt->ble_addr[3], bt->ble_addr[4], bt->ble_addr[5]);
    bt_return(cpu, 0);
}

/* Generic BT no-op returning ESP_OK */
static void stub_bt_noop(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    bt_return(cpu, 0);
}

/* ===== NimBLE C++ Stubs ===== */

/* NimBLEDevice::init(str) */
static void stub_nimble_device_init(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->ble_inited = true;
    bt_log(bt, "NimBLEDevice::init()\n");
    bt_return(cpu, 0);
}

/* NimBLEDevice::deinit(bool) */
static void stub_nimble_device_deinit(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->ble_inited = false;
    bt_log(bt, "NimBLEDevice::deinit()\n");
    bt_return(cpu, 0);
}

/* NimBLEDevice::getScan() — return fake scan pointer */
static void stub_nimble_get_scan(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_log(bt, "NimBLEDevice::getScan()\n");
    bt_return(cpu, FAKE_SCAN_PTR);
}

/* NimBLEDevice::createServer() — return fake server pointer */
static void stub_nimble_create_server(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_log(bt, "NimBLEDevice::createServer()\n");
    bt_return(cpu, FAKE_SERVER_PTR);
}

/* NimBLEDevice::getAdvertising() — return fake advertising pointer */
static void stub_nimble_get_advertising(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_log(bt, "NimBLEDevice::getAdvertising()\n");
    bt_return(cpu, FAKE_ADVERTISING_PTR);
}

/* NimBLEDevice::createClient() — return fake client pointer */
static void stub_nimble_create_client(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_log(bt, "NimBLEDevice::createClient()\n");
    bt_return(cpu, FAKE_CLIENT_PTR);
}

/* NimBLEScan::start(duration, cb, is_continue) */
static void stub_nimble_scan_start(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->ble_scanning = true;
    uint32_t duration = bt_arg(cpu, 0);
    bt_log(bt, "NimBLEScan::start(duration=%u) — %zu synthetic devices\n",
           duration, FAKE_BLE_DEV_COUNT);
    bt_return(cpu, FAKE_SCAN_PTR); /* returns NimBLEScanResults* */
}

/* NimBLEScan::stop() */
static void stub_nimble_scan_stop(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->ble_scanning = false;
    bt_log(bt, "NimBLEScan::stop()\n");
    bt_return(cpu, 0);
}

/* Production-ROM observers.  Spies inspect the pre-ENTRY CALL8 register
 * window and then allow the genuine NimBLE functions to execute. */
static void spy_nimble_scan_set_callbacks(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->scan_cpu = cpu;
    bt->scan_obj_addr = bt_arg(cpu, 0);
    bt->scan_cb_addr = bt_arg(cpu, 1);
    bt->stats.scan_callback_config_calls++;
    bt_log(bt, "NimBLEScan::setAdvertisedDeviceCallbacks(core=%u, "
               "scan=0x%08x, cb=0x%08x)\n", cpu->core_id,
               bt->scan_obj_addr, bt->scan_cb_addr);
}

static void publish_nimble_controller_state(bt_stubs_t *bt)
{
    xtensa_mem_t *mem = bt->cpu->mem;
    /* A real controller publishes these states and its public identity after
     * host synchronization.  Do not replace an identity chosen by firmware. */
    uint32_t sync_state = mem_read32(mem, bt->hs_sync_literal);
    uint32_t enabled_state = mem_read32(mem, bt->hs_enabled_literal);
    uint32_t public_addr_ptr = mem_read32(mem, bt->hs_public_literal);
    if (sync_state >= 0x3FFB0000u && sync_state < 0x40000000u)
        mem_write8(mem, sync_state, 2);
    if (enabled_state >= 0x3FFB0000u && enabled_state < 0x40000000u)
        mem_write8(mem, enabled_state, 2);
    if (public_addr_ptr < 0x3FFB0000u || public_addr_ptr >= 0x40000000u)
        return;
    bool address_is_zero = true;
    for (uint32_t i = 0; i < 6; i++) {
        if (mem_read8(mem, public_addr_ptr + i) != 0) {
            address_is_zero = false;
            break;
        }
    }
    if (address_is_zero) {
        static const uint8_t default_public_addr[6] = {
            0xFE, 0xCA, 0xEF, 0xBE, 0xAD, 0xDE
        };
        for (uint32_t i = 0; i < sizeof(default_public_addr); i++)
            mem_write8(mem, public_addr_ptr + i, default_public_addr[i]);
    }
}

static void spy_nimble_scan_start(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->scan_cpu = cpu;
    publish_nimble_controller_state(bt);
    bt->scan_obj_addr = bt_arg(cpu, 0);
    bt->ble_scanning = true;
    bt->stats.scan_start_calls++;
    bt_log(bt, "NimBLEScan::start(scan=0x%08x, duration=%u) — observed\n",
           bt->scan_obj_addr, bt_arg(cpu, 1));
}

static void spy_nimble_scan_stop(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->scan_cpu = cpu;
    bt->scan_obj_addr = bt_arg(cpu, 0);
    bt->ble_scanning = false;
    bt->stats.scan_stop_calls++;
    bt_log(bt, "NimBLEScan::stop(scan=0x%08x) — observed\n",
           bt->scan_obj_addr);
}

static bool capture_hci_advertising_data(bt_stubs_t *bt, uint32_t cmd,
                                         uint32_t cmd_len, bool scan_response)
{
    if (!cmd || cmd_len < 1)
        return false;
    uint8_t data_len = mem_read8(bt->cpu->mem, cmd);
    if (data_len > BLE_DATA_MAX_LEN || (uint32_t)data_len + 1u > cmd_len)
        return false;

    uint8_t *dst = scan_response ? bt->scan_response_data :
                                   bt->advertisement_data;
    for (uint32_t i = 0; i < data_len; i++)
        dst[i] = mem_read8(bt->cpu->mem, cmd + 1u + i);
    if (scan_response)
        bt->scan_response_len = data_len;
    else
        bt->advertisement_len = data_len;
    return true;
}

/* Virtualize the controller-facing commands used by legacy scanning and
 * advertising.  This is the lowest stable boundary shared by
 * NimBLEAdvertising's structured and raw-data APIs. */
static int conditional_ble_hs_hci_cmd_tx(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    uint32_t opcode = bt_arg(cpu, 0) & 0xFFFFu;
    uint32_t cmd = bt_arg(cpu, 1);
    uint32_t cmd_len = bt_arg(cpu, 2) & 0xFFu;
    bt->stats.hci_command_calls++;

    if (opcode == BLE_HCI_LE_SET_ADV_DATA) {
        bt->stats.advertising_data_calls++;
        uint32_t result = 0;
        if (!capture_hci_advertising_data(bt, cmd, cmd_len, false)) {
            bt->stats.advertisement_tx_failures++;
            result = BLE_HCI_ERR_INVALID_PARAMS;
        }
        bt_return(cpu, result);
        return 1;
    }
    if (opcode == BLE_HCI_LE_SET_SCAN_RSP_DATA) {
        bt->stats.advertising_scan_response_calls++;
        uint32_t result = 0;
        if (!capture_hci_advertising_data(bt, cmd, cmd_len, true)) {
            bt->stats.advertisement_tx_failures++;
            result = BLE_HCI_ERR_INVALID_PARAMS;
        }
        bt_return(cpu, result);
        return 1;
    }
    if (opcode == BLE_HCI_LE_SET_ADV_PARAMS) {
        bt->stats.advertising_parameters_calls++;
        uint32_t result = cmd && cmd_len >= 15 ? 0 :
                                                   BLE_HCI_ERR_INVALID_PARAMS;
        if (result != 0)
            bt->stats.advertisement_tx_failures++;
        bt_return(cpu, result);
        return 1;
    }
    if (opcode == BLE_HCI_LE_SET_SCAN_PARAMS) {
        bt->stats.hci_scan_parameters_calls++;
        uint32_t result = cmd && cmd_len >= 7 ? 0 :
                                                  BLE_HCI_ERR_INVALID_PARAMS;
        bt_return(cpu, result);
        return 1;
    }
    if (opcode == BLE_HCI_LE_SET_SCAN_ENABLE) {
        uint32_t result = cmd && cmd_len >= 2 ? 0 :
                                                  BLE_HCI_ERR_INVALID_PARAMS;
        if (result == 0 && mem_read8(cpu->mem, cmd) != 0)
            bt->stats.hci_scan_enable_calls++;
        else if (result == 0)
            bt->stats.hci_scan_disable_calls++;
        bt_return(cpu, result);
        return 1;
    }
    if (opcode != BLE_HCI_LE_SET_ADV_ENABLE)
        return 0;
    if (!cmd || cmd_len < 1) {
        bt->stats.advertisement_tx_failures++;
        bt_return(cpu, BLE_HCI_ERR_INVALID_PARAMS);
        return 1;
    }

    bool enable = mem_read8(cpu->mem, cmd) != 0;
    if (!enable) {
        bt->stats.advertising_disable_calls++;
        bt->ble_advertising = false;
        bt_return(cpu, 0);
        return 1;
    }

    bt->stats.advertising_enable_calls++;
    bt->ble_advertising = true;
    bt->stats.advertisement_tx_frames++;
    bt->stats.advertisement_tx_bytes += bt->advertisement_len +
                                         bt->scan_response_len;
    if (bt->advertisement_tx_cb)
        bt->advertisement_tx_cb(bt->advertisement_tx_ctx,
                                bt->advertisement_data,
                                bt->advertisement_len,
                                bt->scan_response_data,
                                bt->scan_response_len);
    if (bt->event_log || bt->stats.advertisement_tx_frames <= 3 ||
        bt->stats.advertisement_tx_frames % 1000 == 0)
        bt_log(bt, "BLE advertising TX (%u-byte adv, %u-byte scan rsp)\n",
               bt->advertisement_len, bt->scan_response_len);
    bt_return(cpu, 0);
    return 1;
}

/* NimBLE rejects connectable advertising when its controller-backed host
 * pools have not been replenished.  Flexe's virtual controller has capacity
 * for a connection, so publish that fact at the same boundary the genuine
 * host uses while leaving the rest of ble_gap_adv_start intact. */
static void stub_ble_hs_conn_can_alloc(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->stats.connection_capacity_queries++;
    bt_return(cpu, 1);
}

static void spy_ble_gap_adv_start(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    publish_nimble_controller_state(bt);
    uint32_t params = bt_arg(cpu, 3);
    bt->stats.gap_advertising_start_calls++;
    bt->stats.last_advertising_own_addr_type = bt_arg(cpu, 0) & 0xFFu;
    bt->stats.last_advertising_duration_ms = bt_arg(cpu, 2);
    if (params) {
        bt->stats.last_advertising_conn_mode = mem_read8(cpu->mem, params);
        bt->stats.last_advertising_disc_mode = mem_read8(cpu->mem,
                                                         params + 1u);
        bt->stats.last_advertising_high_duty =
                mem_read8(cpu->mem, params + 8u) & 1u;
    }
    if (bt->stats.gap_advertising_start_calls <= 3) {
        bt_log(bt, "ble_gap_adv_start(own=%u, duration=%d, conn=%u, "
                   "disc=%u, high_duty=%u) — observed\n",
               bt->stats.last_advertising_own_addr_type,
               (int32_t)bt->stats.last_advertising_duration_ms,
               bt->stats.last_advertising_conn_mode,
               bt->stats.last_advertising_disc_mode,
               bt->stats.last_advertising_high_duty);
    }
}

static void spy_ble_hs_id_use_addr(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    publish_nimble_controller_state(bt);
    bt->stats.identity_address_queries++;
}

/* NimBLEScan::clearResults() */
static void stub_nimble_clear_results(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    bt_return(cpu, 0);
}

/* NimBLEDevice::getInitialized() */
static void stub_nimble_get_initialized(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt_return(cpu, bt->ble_inited ? 1 : 0);
}

/* NimBLEAdvertising::start() */
static void stub_nimble_adv_start(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->ble_advertising = true;
    bt_log(bt, "NimBLEAdvertising::start()\n");
    bt_return(cpu, 1); /* true */
}

/* NimBLEAdvertising::stop() */
static void stub_nimble_adv_stop(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    bt->ble_advertising = false;
    bt_log(bt, "NimBLEAdvertising::stop()\n");
    bt_return(cpu, 1);
}

/* NimBLEAdvertising::setAdvertisementData() and similar config no-ops */
static void stub_nimble_adv_noop(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    bt_return(cpu, 0);
}

/* NimBLEDevice::setMTU */
static void stub_nimble_set_mtu(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    bt_return(cpu, 0);
}

/* ===== NimBLE host stubs ===== */

/* ble_hs_cfg is a global config struct — we just need to let writes to it succeed.
 * ble_svc_gap_init, ble_svc_gatt_init, nimble_port_init, etc. */
static void stub_nimble_host_noop(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    bt_return(cpu, 0);
}

/* ===== Public API ===== */

bt_stubs_t *bt_stubs_create(xtensa_cpu_t *cpu)
{
    bt_stubs_t *bt = calloc(1, sizeof(*bt));
    if (!bt) return NULL;
    bt->cpu = cpu;
    bt->bt_status = ESP_BT_CONTROLLER_STATUS_IDLE;

    /* Default BLE address (locally-administered random) */
    bt->ble_addr[0] = 0xDE; bt->ble_addr[1] = 0xAD;
    bt->ble_addr[2] = 0xBE; bt->ble_addr[3] = 0xEF;
    bt->ble_addr[4] = 0xCA; bt->ble_addr[5] = 0xFE;

    return bt;
}

void bt_stubs_destroy(bt_stubs_t *bt)
{
    free(bt);
}

int bt_stubs_hook_symbols(bt_stubs_t *bt, const elf_symbols_t *syms)
{
    if (!bt || !syms) return 0;

    /* The supported Marauder build runs its real NimBLE host.  Replacing the
     * same functions merely because a companion ELF was supplied would make
     * symbol-assisted runs less faithful than raw stock-ROM runs. */
    if (bt->production_observer)
        return 0;

    esp32_rom_stubs_t *rom = bt->cpu->pc_hook_ctx;
    if (!rom) return 0;
    bt->rom = rom;

    int hooked = 0;

    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        /* ESP-IDF BT controller */
        { "esp_bt_controller_init",        stub_esp_bt_controller_init },
        { "esp_bt_controller_deinit",      stub_esp_bt_controller_deinit },
        { "esp_bt_controller_enable",      stub_esp_bt_controller_enable },
        { "esp_bt_controller_disable",     stub_esp_bt_controller_disable },
        { "esp_bt_controller_get_status",  stub_esp_bt_controller_get_status },
        { "esp_bt_controller_mem_release", stub_esp_bt_controller_mem_release },
        { "esp_bt_controller_shutdown",    stub_bt_noop },
        { "esp_bt_sleep_disable",          stub_esp_bt_sleep_disable },

        /* NimBLE HCI & port (C functions) */
        { "esp_nimble_hci_init",                   stub_bt_noop },
        { "esp_nimble_hci_deinit",                 stub_bt_noop },
        { "esp_nimble_hci_and_controller_deinit",  stub_bt_noop },
        { "nimble_port_init",            stub_nimble_host_noop },
        { "nimble_port_deinit",          stub_nimble_host_noop },
        { "nimble_port_run",             stub_nimble_host_noop },
        { "nimble_port_stop",            stub_nimble_host_noop },
        { "nimble_port_freertos_init",   stub_nimble_host_noop },
        { "nimble_port_freertos_deinit", stub_nimble_host_noop },
        { "nimble_port_get_dflt_eventq", stub_nimble_host_noop },
        { "ble_svc_gap_init",            stub_nimble_host_noop },
        { "ble_svc_gap_device_name_set", stub_nimble_host_noop },
        { "ble_svc_gatt_init",           stub_nimble_host_noop },
        { "ble_store_config_init",       stub_nimble_host_noop },
        { "ble_gatts_start",             stub_nimble_host_noop },
        { "ble_gatts_count_cfg",         stub_nimble_host_noop },
        { "ble_gatts_add_svcs",          stub_nimble_host_noop },
        { "ble_hs_cfg",                  stub_nimble_host_noop },
        { "ble_att_svr_start",           stub_nimble_host_noop },

        /* BLE GAP */
        { "esp_ble_gap_set_rand_addr",     stub_esp_ble_gap_set_rand_addr },
        { "esp_ble_gap_register_callback", stub_bt_noop },
        { "esp_ble_gap_set_device_name",   stub_bt_noop },
        { "esp_ble_gap_config_adv_data",   stub_bt_noop },
        { "esp_ble_gap_start_advertising", stub_bt_noop },
        { "esp_ble_gap_stop_advertising",  stub_bt_noop },
        { "esp_ble_gap_start_scanning",    stub_bt_noop },
        { "esp_ble_gap_stop_scanning",     stub_bt_noop },

        /* BLE GATT (classic ESP-IDF BLE, not NimBLE) */
        { "esp_ble_gatts_register_callback", stub_bt_noop },
        { "esp_ble_gattc_register_callback", stub_bt_noop },
        { "esp_ble_gatts_app_register",      stub_bt_noop },
        { "esp_ble_gattc_app_register",      stub_bt_noop },
        { "esp_bluedroid_init",              stub_bt_noop },
        { "esp_bluedroid_enable",            stub_bt_noop },
        { "esp_bluedroid_disable",           stub_bt_noop },
        { "esp_bluedroid_deinit",            stub_bt_noop },

        /* NimBLE C++ — NimBLEDevice (12 chars) */
        { "_ZN12NimBLEDevice4initERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE",
                                         stub_nimble_device_init },
        { "_ZN12NimBLEDevice6deinitEb",  stub_nimble_device_deinit },
        { "_ZN12NimBLEDevice7getScanEv", stub_nimble_get_scan },
        { "_ZN12NimBLEDevice9getServerEv", stub_nimble_create_server },
        { "_ZN12NimBLEDevice12createServerEv", stub_nimble_create_server },
        { "_ZN12NimBLEDevice14getAdvertisingEv", stub_nimble_get_advertising },
        { "_ZN12NimBLEDevice12createClientEv", stub_nimble_create_client },
        { "_ZN12NimBLEDevice16startAdvertisingEv", stub_nimble_adv_start },
        { "_ZN12NimBLEDevice15stopAdvertisingEv", stub_nimble_adv_stop },
        { "_ZN12NimBLEDevice14getInitializedEv", stub_nimble_get_initialized },
        { "_ZN12NimBLEDevice25setScanDuplicateCacheSizeEt", stub_bt_noop },
        { "_ZN12NimBLEDevice8getPowerE20esp_ble_power_type_t", stub_bt_noop },
        { "_ZN12NimBLEDevice17setScanFilterModeEh", stub_bt_noop },
        { "_ZN12NimBLEDevice6setMTUEt",  stub_nimble_set_mtu },
        { "_ZN12NimBLEDevice9host_taskEPv", stub_nimble_host_noop },
        { "_ZN12NimBLEDevice6onSyncEv",  stub_nimble_host_noop },
        { "_ZN12NimBLEDevice7onResetEi", stub_nimble_host_noop },
        { "_ZN12NimBLEDevice13startSecurityEt", stub_bt_noop },
        { "_ZN12NimBLEDevice18getSecurityPasskeyEv", stub_bt_noop },
        { "_ZN12NimBLEDevice12deleteClientEP12NimBLEClient", stub_bt_noop },
        { "_ZN12NimBLEDevice9isIgnoredERK13NimBLEAddress", stub_bt_noop },

        /* NimBLE C++ — NimBLEScan (10 chars) */
        { "_ZN10NimBLEScan5startEjPFv17NimBLEScanResultsEb", stub_nimble_scan_start },
        { "_ZN10NimBLEScan4stopEv",      stub_nimble_scan_stop },
        { "_ZN10NimBLEScan12clearResultsEv", stub_nimble_clear_results },
        { "_ZN10NimBLEScan28setAdvertisedDeviceCallbacksEP31NimBLEAdvertisedDeviceCallbacksb", stub_bt_noop },
        { "_ZN10NimBLEScan13setActiveScanEb", stub_bt_noop },
        { "_ZN10NimBLEScan11setIntervalEt", stub_bt_noop },
        { "_ZN10NimBLEScan9setWindowEt", stub_bt_noop },
        { "_ZN10NimBLEScan18setDuplicateFilterEb", stub_bt_noop },
        { "_ZN10NimBLEScan13setMaxResultsEh", stub_bt_noop },
        { "_ZN10NimBLEScan10onHostSyncEv", stub_bt_noop },
        { "_ZN10NimBLEScan11onHostResetEv", stub_bt_noop },
        { "_ZN10NimBLEScan5eraseERK13NimBLEAddress", stub_bt_noop },

        /* NimBLE C++ — NimBLEAdvertising (17 chars) */
        { "_ZN17NimBLEAdvertising5startEjPFvPS_E", stub_nimble_adv_start },
        { "_ZN17NimBLEAdvertising4stopEv", stub_nimble_adv_stop },
        { "_ZN17NimBLEAdvertising20setAdvertisementDataER23NimBLEAdvertisementData",
                                         stub_nimble_adv_noop },
        { "_ZN17NimBLEAdvertising5resetEv", stub_nimble_adv_noop },
        { "_ZN17NimBLEAdvertising10onHostSyncEv", stub_nimble_adv_noop },
        { "_ZN17NimBLEAdvertising13advCompleteCBEv", stub_nimble_adv_noop },
        { "_ZN17NimBLEAdvertisingC1Ev", stub_nimble_adv_noop },
        { "_ZN17NimBLEAdvertisingC2Ev", stub_nimble_adv_noop },

        /* NimBLE C++ — NimBLEServer (12 chars) */
        { "_ZN12NimBLEServerC1Ev",       stub_nimble_host_noop },
        { "_ZN12NimBLEServerC2Ev",       stub_nimble_host_noop },
        { "_ZN12NimBLEServer9resetGATTEv", stub_nimble_host_noop },
        { "_ZN12NimBLEServer17getConnectedCountEv", stub_bt_noop },
        { "_ZN12NimBLEServer14getAdvertisingEv", stub_nimble_get_advertising },
        { "_ZN12NimBLEServer17clearIndicateWaitEt", stub_bt_noop },

        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn,
                                   hooks[i].name, bt);
            hooked++;
        }
    }

    if (hooked > 0)
        fprintf(stderr, "[bt] hooked %d BT/BLE symbols\n", hooked);

    return hooked;
}

int bt_stubs_hook_firmware_addrs(bt_stubs_t *bt, uint32_t entry_point)
{
    if (!bt || entry_point != MARAUDER_V114_ENTRY)
        return 0;
    esp32_rom_stubs_t *rom = bt->cpu->pc_hook_ctx;
    if (!rom)
        return 0;

    const marauder_bt_layout_t *layout = NULL;
    rom_firmware_profile_t profile = rom_stubs_identify_firmware(
            rom, entry_point);
    if (profile == ROM_FIRMWARE_MARAUDER_V1140_1)
        layout = &marauder_v11401_bt;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1142_3)
        layout = &marauder_v11423_bt;
    else
        return 0;

    bt->rom = rom;
    bt->production_observer = true;
    bt->gap_handler_addr = layout->scan_handle_gap;
    bt->hs_enabled_literal = layout->hs_enabled_literal;
    bt->hs_sync_literal = layout->hs_sync_literal;
    bt->hs_public_literal = layout->hs_public_literal;
    bt->ignore_list_addr = layout->ignore_list;
    bt->connected_peers_addr = layout->connected_peers;

    /* Register at the first post-ENTRY byte. register_spy also discovers the
     * preceding ENTRY itself, which guarantees pre-window arguments while
     * avoiding accidental attachment to an adjacent tiny function. */
    rom_stubs_register_spy(rom, layout->scan_set_callbacks + 3u,
                           spy_nimble_scan_set_callbacks,
                           "NimBLEScan::setAdvertisedDeviceCallbacks", bt);
    rom_stubs_register_spy(rom, layout->scan_start + 3u,
                           spy_nimble_scan_start, "NimBLEScan::start", bt);
    rom_stubs_register_spy(rom, layout->scan_stop + 3u,
                           spy_nimble_scan_stop, "NimBLEScan::stop", bt);
    rom_stubs_register_spy(rom, layout->gap_adv_start + 3u,
                           spy_ble_gap_adv_start, "ble_gap_adv_start", bt);
    rom_stubs_register_conditional_ctx(
            rom, layout->hci_cmd_tx + 3u,
            conditional_ble_hs_hci_cmd_tx, "ble_hs_hci_cmd_tx", bt);
    rom_stubs_register_ctx(rom, layout->conn_can_alloc + 3u,
                           stub_ble_hs_conn_can_alloc,
                           "ble_hs_conn_can_alloc", bt);
    rom_stubs_register_spy(rom, layout->id_use_addr + 3u,
                           spy_ble_hs_id_use_addr,
                           "ble_hs_id_use_addr", bt);
    fprintf(stderr, "[bt] observing 7 verified production-ROM NimBLE entries\n");
    return 7;
}

void bt_stubs_get_stats(const bt_stubs_t *bt, bt_stubs_stats_t *stats)
{
    if (!stats)
        return;
    if (bt)
        *stats = bt->stats;
    else
        memset(stats, 0, sizeof(*stats));
}

int bt_stubs_inject_advertisement(bt_stubs_t *bt, const uint8_t addr[6],
                                  uint8_t addr_type, int8_t rssi,
                                  const uint8_t *data, size_t len)
{
    if (!bt || !addr || !data || len == 0 || addr_type > 1)
        return -1;
    if (!bt->ble_scanning || bt->scan_obj_addr == 0 ||
        bt->scan_cb_addr == 0 || bt->gap_handler_addr == 0)
        return -2;
    if (len > BLE_DATA_MAX_LEN)
        return -3;

    xtensa_cpu_t *event_cpu = bt->scan_cpu ? bt->scan_cpu : bt->cpu;
    xtensa_mem_t *mem = event_cpu->mem;
    for (uint32_t i = 0; i < BLE_EVENT_SCRATCH_SIZE; i++)
        mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + i, 0);
    for (size_t i = 0; i < len; i++)
        mem_write8(mem, BLE_DATA_SCRATCH_ADDR + (uint32_t)i, data[i]);

    /* ble_gap_event.type */
    mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + 0u, BLE_GAP_EVENT_DISC);
    /* ble_gap_event.disc starts at +4 in the 32-bit ABI. */
    mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + 4u, BLE_ADV_NONCONN_IND);
    mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + 5u, (uint8_t)len);
    mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + 6u, addr_type);
    for (uint32_t i = 0; i < 6; i++)
        mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + 7u + i, addr[i]);
    mem_write8(mem, BLE_EVENT_SCRATCH_ADDR + 13u, (uint8_t)rssi);
    mem_write32(mem, BLE_EVENT_SCRATCH_ADDR + 16u, BLE_DATA_SCRATCH_ADDR);

    if (getenv("FLEXE_BTDBG"))
        fprintf(stderr,
                "[bt] inject scan=0x%08X cb=0x%08X handler=0x%08X "
                "vector={0x%08X,0x%08X,0x%08X} ignore=%u\n",
                bt->scan_obj_addr, bt->scan_cb_addr, bt->gap_handler_addr,
                mem_read32(mem, bt->scan_obj_addr + 0x10u),
                mem_read32(mem, bt->scan_obj_addr + 0x14u),
                mem_read32(mem, bt->scan_obj_addr + 0x18u),
                mem_read8(mem, bt->scan_obj_addr + 0x0Eu));
    if (getenv("FLEXE_BTDBG"))
        fprintf(stderr,
                "[bt] ignore-list={0x%08X,0x%08X,%u} "
                "connected={0x%08X,0x%08X,%u}\n",
                mem_read32(mem, bt->ignore_list_addr),
                mem_read32(mem, bt->ignore_list_addr + 4u),
                mem_read32(mem, bt->ignore_list_addr + 8u),
                mem_read32(mem, bt->connected_peers_addr),
                mem_read32(mem, bt->connected_peers_addr + 4u),
                mem_read32(mem, bt->connected_peers_addr + 8u));

    uint32_t args[] = {BLE_EVENT_SCRATCH_ADDR, bt->scan_obj_addr};
    int result = guest_call8(event_cpu, bt->gap_handler_addr, args, 2,
                             2000000u, NULL);
    if (result != 0) {
        bt->stats.advertisement_callback_failures++;
        return -4;
    }

    bt->stats.advertisement_frames++;
    bt_log(bt, "BLE advertisement(name payload=%zu, rssi=%d) delivered\n",
           len, rssi);
    return 0;
}

void bt_stubs_set_advertisement_tx_callback(bt_stubs_t *bt,
                                             bt_advertisement_tx_cb cb,
                                             void *ctx)
{
    if (!bt)
        return;
    bt->advertisement_tx_cb = cb;
    bt->advertisement_tx_ctx = ctx;
}

void bt_stubs_set_event_log(bt_stubs_t *bt, bool enabled) {
    if (bt) bt->event_log = enabled;
}
