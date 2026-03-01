/*
 * bt_stubs.c — Bluetooth / NimBLE stubs for ESP32 emulator
 *
 * Provides ESP-IDF BT controller stubs and NimBLE API stubs.
 * BLE scans return synthetic device results; advertising and
 * raw TX operations are logged but no-op.
 */

#include "bt_stubs.h"
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

/* Synthetic BLE devices */
typedef struct {
    char     name[32];
    uint8_t  addr[6];
    int8_t   rssi;
    uint8_t  addr_type;   /* 0=public, 1=random */
} fake_ble_dev_t;

static const fake_ble_dev_t fake_ble_devs[] = {
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
        cpu->pc = ar_read(cpu, 0);
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

void bt_stubs_set_event_log(bt_stubs_t *bt, bool enabled) {
    if (bt) bt->event_log = enabled;
}
