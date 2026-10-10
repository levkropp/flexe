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
#define MARAUDER_BOARD_ENTRY 0x400830D0u
#define MARAUDER_V114_ENTRY 0x400831D8u
#define MARAUDER_V1121_CYD2USB_ENTRY 0x40081E90u
#define MESHTASTIC_TBEAM_2726_ENTRY 0x400836F0u
#define MESHTASTIC_TBEAM_2726_HCI_CMD_TX 0x400F1DB0u
#define MESHTASTIC_TBEAM_2726_CONN_CAN_ALLOC 0x400F1D50u
#define MESHTASTIC_TBEAM_2726_SYNC_WAIT 0x4010CDE0u
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

/* NimBLE-Arduino 2.3.8 renamed setAdvertisedDeviceCallbacks() to
 * setScanCallbacks().  The observer contract is otherwise the same.  The
 * state literal slots and all function entries were verified against the
 * official v1.12.1 2-USB release after an exact tagged-source rebuild. */
static const marauder_bt_layout_t marauder_v1121_cyd2usb_bt = {
    0x4010F264u, 0x4010F238u, 0x4010F380u, 0x4010F110u,
    0x40116BF4u, 0x4011B340u, 0x4011ACE4u, 0x4011BFCCu,
    0x401113C8u, 0x401113D4u, 0x401114D8u,
    0, 0,
};

static const marauder_bt_layout_t marauder_v11423_bt = {
    0x40104F9Cu, 0x4010509Cu, 0x40105184u, 0x401DB19Cu,
    0x40109DC8u, 0x40110920u, 0x40110324u, 0x4011138Cu,
    0x4010B4DCu, 0x4010B4E8u, 0x4010B60Cu,
    0x3FFC9544u, 0x3FFC9550u,
};

/* v1.15.x: NimBLE C++ moved +0x2C68, the host stack +0x2BE8, the vtable
 * +0x3058, the literal pool +0xE8/+0xE0, and .bss uniformly +0x188. */
/* v1.14.3 built for the 2432S024 guition board. */
static const marauder_bt_layout_t marauder_guition_bt = {
    0x40104E6Cu, 0x40104F6Cu, 0x40105054u, 0x401DAC60u,
    0x40109C98u, 0x401107A8u, 0x401101ACu, 0x40111214u,
    0x4010B4A0u, 0x4010B4ACu, 0x4010B5D0u,
    0x3FFC9514u, 0x3FFC9520u,
};

/* v1.14.3 built for the 3.5-inch board. */
static const marauder_bt_layout_t marauder_35inch_bt = {
    0x4010506Cu, 0x4010516Cu, 0x40105254u, 0x401DADECu,
    0x40109E98u, 0x40110998u, 0x4011039Cu, 0x40111404u,
    0x4010B4B8u, 0x4010B4C4u, 0x4010B5E8u,
    0x3FFC9654u, 0x3FFC9660u,
};

static const marauder_bt_layout_t marauder_v1151_bt = {
    0x40107C04u, 0x40107D04u, 0x40107DECu, 0x401DE1F4u,
    0x4010F2ECu, 0x40113508u, 0x40112F0Cu, 0x40113F74u,
    0x4010B5C4u, 0x4010B5D0u, 0x4010B6ECu,
    0x3FFC96CCu, 0x3FFC96D8u,
};

/* A ble_gap_event is 52 bytes in this ESP32 NimBLE build.  Its discovery
 * descriptor begins at +4 and holds a pointer to the advertisement payload.
 * This bounded RTC-fast gap lies between the virtual PHY and WiFi buffers. */
/* HCI backend chunks controller payloads through the event scratch so no
 * guest heap call ever runs in borrowed context: heap code with interrupts
 * masked trips the interrupt watchdog under contention. Backend pumps and
 * app-level advertisement injection target different firmware classes, so
 * they never share a pump. */
#define BLE_EVENT_SCRATCH_ADDR 0x50001C00u
#define BLE_EVENT_SCRATCH_SIZE 64u
#define BLE_HCI_CHUNK_ADDR BLE_EVENT_SCRATCH_ADDR
#define BLE_HCI_CHUNK_SIZE BLE_EVENT_SCRATCH_SIZE
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

/* Controller and information commands required by NimBLE host startup. */
#define BLE_HCI_RESET                0x0C03u
#define BLE_HCI_READ_LOCAL_VERSION   0x1001u
#define BLE_HCI_READ_SUPPORTED_CMDS  0x1002u
#define BLE_HCI_READ_LOCAL_FEATURES  0x1003u
#define BLE_HCI_READ_BUFFER_SIZE     0x1005u
#define BLE_HCI_READ_BD_ADDR         0x1009u
#define BLE_HCI_LE_READ_BUFFER_SIZE  0x2002u
#define BLE_HCI_LE_READ_FEATURES     0x2003u
#define BLE_HCI_LE_RAND              0x2018u

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
    bool               virtual_hci_all;
    uint64_t            hci_random_state;
    uint8_t            advertisement_data[BLE_DATA_MAX_LEN];
    uint8_t            advertisement_len;
    uint8_t            scan_response_data[BLE_DATA_MAX_LEN];
    uint8_t            scan_response_len;
    bt_advertisement_tx_cb advertisement_tx_cb;
    void              *advertisement_tx_ctx;
    bt_stubs_stats_t    stats;

    /* External HCI controller backend (--ble-hci tcp:HOST:PORT). When
     * attached, commands forward instead of using the virtual controller. */
    ble_hci_conn_t   *hci_backend;
    /* ESP-IDF VHCI-transport entry points (resolved, never hooked):
     * events go through the raw pool buffer, ACL through rx_acl. */
    uint32_t           hci_alloc_evt_addr;
    uint32_t           hci_to_hs_evt_addr;
    uint32_t           hci_free_evt_addr;
    uint32_t           hci_rx_acl_addr;
    uint32_t           hci_alloc_acl_addr;
    uint32_t           hci_msys_acl_addr;
    uint32_t           hci_append_addr;
    uint32_t           hci_mbuf_free_addr;
    uint32_t           hci_mbuf_len_addr;
    uint32_t           hci_copydata_addr;
    uint32_t           hci_free_chain_addr;
    /* Debug-only pool addresses for non-allocating depth reads. */
    uint32_t           hci_pool_evt_addr;
    uint32_t           hci_pool_evt_lo_addr;
    uint32_t           hci_mpool_acl_addr;
    /* Controller packets that arrived while the guest could not take them.
     * Raw bytes wait here; guest mbufs are only allocated at delivery. */
    ble_hci_packet_t   hci_pending[16];
    unsigned           hci_pending_count;
    uint64_t           hci_dropped_packets;
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

static void write_hci_response(xtensa_cpu_t *cpu, uint32_t rsp,
                               uint32_t rsp_len, const uint8_t *data,
                               size_t data_len)
{
    if (!rsp)
        return;
    for (uint32_t i = 0; i < rsp_len; i++)
        mem_write8(cpu->mem, rsp + i, i < data_len ? data[i] : 0);
}

/* Complete a synchronous NimBLE HCI transaction against the virtual
 * controller.  Keep the advertised capabilities conservative (Bluetooth
 * 4.2 LE, no optional commands) so the genuine host only enables operations
 * implemented at this boundary. */
static void complete_virtual_hci_command(bt_stubs_t *bt, xtensa_cpu_t *cpu,
                                         uint32_t opcode, uint32_t rsp,
                                         uint32_t rsp_len)
{
    static const uint8_t local_version[] = {
        8,          /* Bluetooth Core 4.2 */
        0, 0,       /* HCI revision */
        8,          /* LMP version */
        0xE5, 0x02, /* Espressif Systems company identifier */
        0, 0,       /* LMP subversion */
    };
    static const uint8_t local_features[] = {
        0, 0, 0, 0, 0x60, 0, 0, 0, /* LE supported; BR/EDR unsupported */
    };
    static const uint8_t le_buffer_size[] = {
        0xFB, 0x00, /* 251-byte ACL payload */
        10,         /* ten controller packets */
    };
    static const uint8_t classic_buffer_size[] = {
        0xFB, 0x00, 0, 10, 0, 0, 0,
    };
    uint8_t random_data[8];
    uint64_t random_value;

    switch (opcode) {
    case BLE_HCI_READ_LOCAL_VERSION:
        write_hci_response(cpu, rsp, rsp_len, local_version,
                           sizeof(local_version));
        break;
    case BLE_HCI_READ_LOCAL_FEATURES:
        write_hci_response(cpu, rsp, rsp_len, local_features,
                           sizeof(local_features));
        break;
    case BLE_HCI_LE_READ_BUFFER_SIZE:
        write_hci_response(cpu, rsp, rsp_len, le_buffer_size,
                           sizeof(le_buffer_size));
        break;
    case BLE_HCI_READ_BUFFER_SIZE:
        write_hci_response(cpu, rsp, rsp_len, classic_buffer_size,
                           sizeof(classic_buffer_size));
        break;
    case BLE_HCI_READ_BD_ADDR:
        write_hci_response(cpu, rsp, rsp_len, bt->ble_addr,
                           sizeof(bt->ble_addr));
        break;
    case BLE_HCI_LE_RAND:
        /* xorshift64* gives deterministic but non-degenerate controller
         * entropy.  NimBLE uses this during identity and privacy setup and
         * legitimately retries when a controller returns unusable zeros. */
        random_value = bt->hci_random_state;
        random_value ^= random_value >> 12;
        random_value ^= random_value << 25;
        random_value ^= random_value >> 27;
        bt->hci_random_state = random_value;
        random_value *= UINT64_C(0x2545F4914F6CDD1D);
        for (uint32_t i = 0; i < sizeof(random_data); i++)
            random_data[i] = (uint8_t)(random_value >> (i * 8u));
        write_hci_response(cpu, rsp, rsp_len, random_data,
                           sizeof(random_data));
        break;
    case BLE_HCI_RESET:
    case BLE_HCI_READ_SUPPORTED_CMDS:
    case BLE_HCI_LE_READ_FEATURES:
    default:
        /* Zero is both a valid empty capability set and a deterministic
         * response for commands whose return parameters are unused. */
        write_hci_response(cpu, rsp, rsp_len, NULL, 0);
        break;
    }
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
    uint32_t rsp = bt_arg(cpu, 3);
    uint32_t rsp_len = bt_arg(cpu, 4) & 0xFFu;
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
    if (opcode != BLE_HCI_LE_SET_ADV_ENABLE) {
        if (bt->virtual_hci_all) {
            complete_virtual_hci_command(bt, cpu, opcode, rsp, rsp_len);
            bt->stats.hci_virtual_completions++;
            if (bt->event_log || bt->stats.hci_virtual_completions <= 12)
                bt_log(bt, "HCI command 0x%04x completed by virtual "
                       "controller\n", opcode);
            bt_return(cpu, 0);
            return 1;
        }
        return 0;
    }
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

/* ===== External HCI controller backend (--ble-hci) =====
 *
 * Forwards NimBLE HCI commands to an H4-speaking controller (Bumble) and
 * injects its events/ACL back into the genuine host. Command transactions
 * stay synchronous like the intercepted ble_hs_hci_cmd_tx contract: the
 * forwarder round-trips the matching Command Complete/Status inline.
 * Anything else the controller emits while waiting is retained for the
 * asynchronous pump, which delivers at a safe inter-batch boundary using
 * the host's own transport allocators, so buffer ownership always matches
 * what the guest frees. */

#define BLE_HCI_BACKEND_SYNC_TIMEOUT_MS 5000
#define BLE_HCI_EVCODE_COMMAND_COMPLETE 0x0Eu
#define BLE_HCI_EVCODE_COMMAND_STATUS 0x0Fu
#define BLE_HCI_ERR_HW_FAILURE 0x03u

static void bt_hci_queue_async(bt_stubs_t *bt, const ble_hci_packet_t *packet)
{
    if (packet->type == BLE_HCI_H4_EVT)
        bt->stats.hci_forwarded_events++;
    else if (packet->type == BLE_HCI_H4_ACL)
        bt->stats.hci_forwarded_acl++;
    if (bt->hci_pending_count >=
        sizeof(bt->hci_pending) / sizeof(bt->hci_pending[0])) {
        bt->hci_dropped_packets++;
        bt->stats.hci_injection_failures++;
        return;
    }
    bt->hci_pending[bt->hci_pending_count++] = *packet;
}

void bt_stubs_set_hci_backend(bt_stubs_t *bt, ble_hci_conn_t *conn)
{
    if (!bt) return;
    bt->hci_backend = conn;
}

/* Copy controller return parameters into the caller's response buffer,
 * zero-padding short replies like the virtual controller does. */
static void bt_hci_fill_response(xtensa_cpu_t *cpu, uint32_t rsp,
                                  uint32_t rsp_len, const uint8_t *params,
                                  size_t params_len)
{
    for (uint32_t i = 0u; i < rsp_len; i++)
        mem_write8(cpu->mem, rsp + i,
                   i < params_len ? params[i] : 0u);
}

static bool bt_hci_is_sync_ack(const ble_hci_packet_t *packet,
                               uint16_t opcode, uint8_t *status_out,
                               const uint8_t **params_out,
                               size_t *params_len_out)
{
    if (packet->type != BLE_HCI_H4_EVT || packet->payload_len < 2u)
        return false;
    uint8_t code = packet->payload[0];
    if (code == BLE_HCI_EVCODE_COMMAND_COMPLETE &&
        packet->payload_len >= 6u) {
        uint16_t echoed = (uint16_t)packet->payload[3] |
                          ((uint16_t)packet->payload[4] << 8);
        if (echoed != opcode) return false;
        *status_out = packet->payload[5];
        *params_out = packet->payload + 6u;
        *params_len_out = packet->payload[1] >= 4u ?
            (size_t)packet->payload[1] - 4u : 0u;
        return true;
    }
    if (code == BLE_HCI_EVCODE_COMMAND_STATUS &&
        packet->payload_len >= 6u) {
        uint16_t echoed = (uint16_t)packet->payload[4] |
                          ((uint16_t)packet->payload[5] << 8);
        if (echoed != opcode) return false;
        *status_out = packet->payload[2];
        *params_out = NULL;
        *params_len_out = 0u;
        return true;
    }
    return false;
}

static int forward_ble_hs_hci_cmd_tx(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    if (!bt->hci_backend) return 0; /* no backend: run the guest original */
    uint32_t opcode = bt_arg(cpu, 0) & 0xFFFFu;
    uint32_t cmd = bt_arg(cpu, 1);
    uint32_t cmd_len = bt_arg(cpu, 2) & 0xFFu;
    uint32_t rsp = bt_arg(cpu, 3);
    uint32_t rsp_len = bt_arg(cpu, 4) & 0xFFu;
    bt->stats.hci_command_calls++;
    bt->stats.hci_forwarded_commands++;

    uint8_t params[255];
    for (uint32_t i = 0u; i < cmd_len; i++)
        params[i] = mem_read8(cpu->mem, cmd + i);
    if (ble_hci_send_cmd(bt->hci_backend, (uint16_t)opcode, params,
                         cmd_len) != 0) {
        bt->stats.hci_backend_timeouts++;
        bt_log(bt, "HCI forward 0x%04x: transport send failed\n", opcode);
        bt_return(cpu, BLE_HCI_ERR_HW_FAILURE);
        return 1;
    }

    int waited = 0;
    for (;;) {
        ble_hci_packet_t packet;
        int rc = ble_hci_recv(bt->hci_backend, &packet, 50);
        if (rc < 0) {
            bt->stats.hci_backend_timeouts++;
            bt_log(bt, "HCI forward 0x%04x: transport lost\n", opcode);
            bt_return(cpu, BLE_HCI_ERR_HW_FAILURE);
            return 1;
        }
        if (rc == 0) {
            waited += 50;
            if (waited >= BLE_HCI_BACKEND_SYNC_TIMEOUT_MS) {
                bt->stats.hci_backend_timeouts++;
                bt_log(bt, "HCI forward 0x%04x: ack timeout\n", opcode);
                bt_return(cpu, BLE_HCI_ERR_HW_FAILURE);
                return 1;
            }
            continue;
        }
        uint8_t status = 0u;
        const uint8_t *retparams = NULL;
        size_t retparams_len = 0u;
        if (bt_hci_is_sync_ack(&packet, (uint16_t)opcode, &status,
                               &retparams, &retparams_len)) {
            if (status == 0u && retparams)
                bt_hci_fill_response(cpu, rsp, rsp_len, retparams,
                                     retparams_len);
            bt_return(cpu, status);
            return 1;
        }
        /* Not our ack: a genuinely asynchronous controller event. */
        bt_hci_queue_async(bt, &packet);
    }
}

/* Deliver one controller packet through ESP-IDF's own VHCI-transport
 * entries: events land in a transport pool buffer handed to to_hs_evt,
 * ACL lands in a host msys mbuf handed to ble_hs_rx_data. This mirrors
 * host_rcv_pkt, so buffer ownership matches what the guest frees. (ACL
 * uses the msys pool rather than the dedicated 24-buffer transport pool,
 * which was observed to drain under a long GATT session while msys keeps
 * cycling; the FROM_LL flag it skips only feeds disabled IPC flow
 * control.) Returns 0 when delivered, -1 when the guest cannot take it
 * yet (the packet stays queued for a later pump), -2 when it can never be
 * delivered. */
static int bt_hci_deliver(bt_stubs_t *bt, xtensa_cpu_t *cpu,
                          xtensa_cpu_t *peer, const ble_hci_packet_t *packet)
{
    if (packet->type != BLE_HCI_H4_EVT &&
        packet->type != BLE_HCI_H4_ACL)
        return -2; /* SCO/ISO have no host consumer. */
    if (!guest_call_injection_is_quiescent(cpu, peer)) {
        /* The NimBLE host task can spin on one core while the other parks
         * in WAITI; borrow whichever core is quiescent. */
        if (peer && peer != cpu &&
            guest_call_injection_is_quiescent(peer, cpu)) {
            cpu = peer;
            peer = NULL;
        } else {
            if (getenv("FLEXE_BTDBG")) {
                fprintf(stderr,
                        "[bt] hci deliver deferred "
                        "(cycle=%llu pc=0x%08x run=%d halt=%d exc=%d ingc=%u il=%u excm=%d)\n",
                        (unsigned long long)cpu->cycle_count, cpu->pc,
                        cpu->running, cpu->halted, cpu->exception,
                        cpu->in_guest_call, XT_PS_INTLEVEL(cpu->ps),
                        XT_PS_EXCM(cpu->ps) != 0);
                fflush(stderr);
            }
            return -1;
        }
    }
    if (packet->type == BLE_HCI_H4_EVT) {
        if (bt->hci_alloc_evt_addr == 0u || bt->hci_to_hs_evt_addr == 0u) {
            if (getenv("FLEXE_BTDBG")) {
                fprintf(stderr, "[bt] hci deliver missing evt symbols\n");
                fflush(stderr);
            }
            return -2;
        }
        uint32_t alloc_arg = 0u;
        uint32_t evbuf = 0u;
        int alloc_rc =
            guest_call8(cpu, bt->hci_alloc_evt_addr, &alloc_arg, 1,
                        2000000u, &evbuf);
        if (getenv("FLEXE_BTDBG")) {
            fprintf(stderr, "[bt] hci deliver evt alloc=%d evbuf=0x%08x\n",
                    alloc_rc, evbuf);
            fflush(stderr);
        }
        if (alloc_rc != 0 || evbuf == 0u)
            return -1;
        for (size_t i = 0u; i < packet->payload_len; i++)
            mem_write8(cpu->mem, evbuf + (uint32_t)i,
                       packet->payload[i]);
        uint32_t rc = 1u;
        int delivered =
            guest_call8(cpu, bt->hci_to_hs_evt_addr, &evbuf, 1, 2000000u,
                        &rc);
        (void)rc;
        if (getenv("FLEXE_BTDBG"))
            fprintf(stderr, "[bt] hci deliver evt len=%zu -> %s\n",
                    packet->payload_len,
                    delivered == 0 ? "ok" : "guest-busy");
        fflush(stderr);
        if (delivered != 0) {
            /* Host did not take ownership: release the pool buffer so a
             * later retry can allocate again. Without this the 30-buffer
             * event pool drains and every later delivery fails. */
            if (bt->hci_free_evt_addr != 0u) {
                uint32_t free_arg[1] = {evbuf};
                guest_call8(cpu, bt->hci_free_evt_addr, free_arg, 1,
                            2000000u, NULL);
            } else {
                bt->stats.hci_injection_failures++;
            }
            return -1;
        }
        bt->stats.hci_injected_events++;
        return 0;
    }
    if (bt->hci_rx_acl_addr == 0u || bt->hci_alloc_acl_addr == 0u ||
        bt->hci_append_addr == 0u || bt->hci_mbuf_free_addr == 0u)
        return -2;
    /* Pool mbuf plus chunked append through static scratch: no guest heap
     * call runs in borrowed context. Prefer the host msys ACL pool: the
     * dedicated transport pool has been observed to drain under a long
     * GATT session while msys keeps cycling (same free path either way).
     * Fall back to the transport allocator when msys is unavailable. */
    uint32_t om = 0u;
    int alloc_rc = -1;
    if (bt->hci_msys_acl_addr != 0u)
        alloc_rc = guest_call8(cpu, bt->hci_msys_acl_addr, NULL, 0,
                               2000000u, &om);
    if (alloc_rc != 0 || om == 0u)
        alloc_rc = guest_call8(cpu, bt->hci_alloc_acl_addr, NULL, 0,
                               2000000u, &om);
    if (getenv("FLEXE_BTDBG")) {
        fprintf(stderr, "[bt] hci deliver acl alloc=%d om=0x%08x\n",
                alloc_rc, om);
        fflush(stderr);
    }
    if (alloc_rc != 0 || om == 0u)
        return -1;
    size_t off = 0u;
    while (off < packet->payload_len) {
        size_t chunk = packet->payload_len - off;
        if (chunk > BLE_HCI_CHUNK_SIZE) chunk = BLE_HCI_CHUNK_SIZE;
        for (size_t i = 0u; i < chunk; i++)
            mem_write8(cpu->mem,
                       BLE_HCI_CHUNK_ADDR + (uint32_t)i,
                       packet->payload[off + i]);
        uint32_t append_args[3] = {om, BLE_HCI_CHUNK_ADDR,
                                   (uint32_t)chunk};
        uint32_t append_rc = 1u;
        if (guest_call8(cpu, bt->hci_append_addr, append_args, 3,
                        2000000u, &append_rc) != 0 ||
            append_rc != 0u) {
            uint32_t free_one[1] = {om};
            guest_call8(cpu, bt->hci_mbuf_free_addr, free_one, 1,
                        2000000u, NULL);
            return -1;
        }
        off += chunk;
    }
    uint32_t rx_args[2] = {om, 0u};
    uint32_t rc = 0u;
    int call = guest_call8(cpu, bt->hci_rx_acl_addr, rx_args, 2, 2000000u,
                           &rc);
    (void)rc;
    if (getenv("FLEXE_BTDBG")) {
        unsigned acl_free = 9999u;
        if (bt->hci_mpool_acl_addr != 0u) {
            uint32_t ext =
                (uint32_t)mem_read8(cpu->mem,
                    bt->hci_mpool_acl_addr + 4u) |
                ((uint32_t)mem_read8(cpu->mem,
                    bt->hci_mpool_acl_addr + 5u) << 8) |
                ((uint32_t)mem_read8(cpu->mem,
                    bt->hci_mpool_acl_addr + 6u) << 16) |
                ((uint32_t)mem_read8(cpu->mem,
                    bt->hci_mpool_acl_addr + 7u) << 24);
            if (ext != 0u)
                acl_free = (unsigned)mem_read8(cpu->mem, ext + 6u) |
                           ((unsigned)mem_read8(cpu->mem, ext + 7u) << 8);
        }
        fprintf(stderr, "[bt] hci deliver acl len=%zu -> %s aclfree=%u\n",
                packet->payload_len, call == 0 ? "ok" : "guest-busy",
                acl_free);
        fflush(stderr);
    }
    if (call != 0) {
        uint32_t free_one[1] = {om};
        guest_call8(cpu, bt->hci_mbuf_free_addr, free_one, 1, 2000000u,
                    NULL);
        return -1;
    }
    bt->stats.hci_injected_events++;
    return 0;
}

void bt_stubs_hci_pump(bt_stubs_t *bt, xtensa_cpu_t *peer)
{
    if (!bt || !bt->hci_backend) return;
    xtensa_cpu_t *cpu = bt->cpu;
    if (getenv("FLEXE_BTDBG")) {
        static unsigned long pumps = 0u;
        if ((++pumps % 5000u) == 0u) {
            /* Counter-only: allocating pool buffers here to probe would
             * consume the very pool the pump needs (the evt probe was
             * never freed and drained the 30-buffer pool). Depth is read
             * straight from guest RAM: no guest call, no leak. */
            unsigned evt_free = 9999u, evt_lo_free = 9999u, acl_free = 9999u;
            if (bt->hci_pool_evt_addr != 0u)
                evt_free = (unsigned)mem_read8(cpu->mem,
                           bt->hci_pool_evt_addr + 6u) |
                           ((unsigned)mem_read8(cpu->mem,
                           bt->hci_pool_evt_addr + 7u) << 8);
            if (bt->hci_pool_evt_lo_addr != 0u)
                evt_lo_free = (unsigned)mem_read8(cpu->mem,
                           bt->hci_pool_evt_lo_addr + 6u) |
                           ((unsigned)mem_read8(cpu->mem,
                           bt->hci_pool_evt_lo_addr + 7u) << 8);
            if (bt->hci_mpool_acl_addr != 0u) {
                uint32_t ext = (uint32_t)mem_read8(cpu->mem,
                               bt->hci_mpool_acl_addr + 4u) |
                               ((uint32_t)mem_read8(cpu->mem,
                               bt->hci_mpool_acl_addr + 5u) << 8) |
                               ((uint32_t)mem_read8(cpu->mem,
                               bt->hci_mpool_acl_addr + 6u) << 16) |
                               ((uint32_t)mem_read8(cpu->mem,
                               bt->hci_mpool_acl_addr + 7u) << 24);
                if (ext != 0u)
                    acl_free = (unsigned)mem_read8(cpu->mem, ext + 6u) |
                               ((unsigned)mem_read8(cpu->mem, ext + 7u)
                                << 8);
            }
            fprintf(stderr,
                    "[bt] hci pump alive #%lu queued=%u "
                    "evt(free=%u lo=%u acl=%u) "
                    "c0(halt=%d) c1(halt=%d)\n",
                    pumps, bt->hci_pending_count, evt_free, evt_lo_free,
                    acl_free, bt->cpu->halted, peer ? peer->halted : -1);
            fflush(stderr);
        }
    }
    /* Deliver already-queued packets before reading new ones. */
    unsigned i = 0u;
    while (i < bt->hci_pending_count) {
        int delivered = bt_hci_deliver(bt, cpu, peer, &bt->hci_pending[i]);
        if (delivered == -2) {
            bt->stats.hci_injection_failures++;
        } else if (delivered != 0) {
            break;
        }
        i++;
    }
    if (i != 0u) {
        memmove(bt->hci_pending, bt->hci_pending + i,
                (bt->hci_pending_count - i) * sizeof(bt->hci_pending[0]));
        bt->hci_pending_count -= i;
    }
    for (;;) {
        ble_hci_packet_t packet;
        int rc = ble_hci_recv(bt->hci_backend, &packet, 0);
        if (rc <= 0) return;
        if (getenv("FLEXE_BTDBG")) {
            char hex[129];
            size_t show = packet.payload_len < 64u ?
                packet.payload_len : 64u;
            for (size_t i = 0u; i < show; i++)
                snprintf(hex + 2u * i, 3u, "%02x",
                         packet.payload[i]);
            hex[2u * show] = '\0';
            fprintf(stderr, "[bt] hci pump got type=%u len=%zu %s\n",
                    packet.type, packet.payload_len, hex);
            fflush(stderr);
        }
        if (packet.type == BLE_HCI_H4_EVT)
            bt->stats.hci_forwarded_events++;
        else if (packet.type == BLE_HCI_H4_ACL)
            bt->stats.hci_forwarded_acl++;
        int delivered = bt_hci_deliver(bt, cpu, peer, &packet);
        if (delivered == -2)
            bt->stats.hci_injection_failures++;
        else if (delivered != 0)
            bt_hci_queue_async(bt, &packet);
    }
}

/* Forward one host-to-controller ACL packet: copy the mbuf chain out in
 * static-scratch chunks (no guest heap in borrowed context), emit it as
 * H4, then release the chain. The transport owns the chain, so every exit
 * frees it. */
static int forward_ble_hci_acl_tx(xtensa_cpu_t *cpu, void *ctx)
{
    bt_stubs_t *bt = ctx;
    if (!bt->hci_backend) return 0; /* no backend: run the guest original */
    if (bt->hci_mbuf_len_addr == 0u || bt->hci_copydata_addr == 0u ||
        bt->hci_free_chain_addr == 0u)
        return 0;
    uint32_t om = bt_arg(cpu, 0);
    if (om == 0u) return 0;
    bt->stats.hci_command_calls++;
    if (getenv("FLEXE_BTDBG")) {
        fprintf(stderr, "[bt] hci acl TX om=0x%08x\n", om);
        fflush(stderr);
    }

    uint32_t total = 0u;
    int len_rc = guest_call8(cpu, bt->hci_mbuf_len_addr, &om, 1, 2000000u,
                             &total);
    if (getenv("FLEXE_BTDBG")) {
        fprintf(stderr, "[bt] hci acl TX len=%d total=%u\n", len_rc, total);
        fflush(stderr);
    }
    if (len_rc != 0 ||
        total == 0u || total > BLE_HCI_MAX_PACKET) {
        uint32_t chain[1] = {om};
        guest_call8(cpu, bt->hci_free_chain_addr, chain, 1, 2000000u,
                    NULL);
        bt->stats.hci_injection_failures++;
        return 1;
    }
    static uint8_t flat[BLE_HCI_MAX_PACKET];
    size_t off = 0u;
    uint32_t copy_rc = 1u;
    while (off < total) {
        size_t chunk = total - off;
        if (chunk > BLE_HCI_CHUNK_SIZE) chunk = BLE_HCI_CHUNK_SIZE;
        uint32_t copy_args[4] = {om, (uint32_t)off, (uint32_t)chunk,
                                 BLE_HCI_CHUNK_ADDR};
        if (guest_call8(cpu, bt->hci_copydata_addr, copy_args, 4,
                        2000000u, &copy_rc) != 0 ||
            copy_rc != 0)
            break;
        for (size_t i = 0u; i < chunk; i++)
            flat[off + i] =
                mem_read8(cpu->mem, BLE_HCI_CHUNK_ADDR + (uint32_t)i);
        off += chunk;
    }
    int rc = 1;
    if (off == total &&
        ble_hci_send_raw(bt->hci_backend, BLE_HCI_H4_ACL, flat,
                         total) == 0) {
        if (getenv("FLEXE_BTDBG")) {
            char hex[65];
            size_t show = total < 32u ? total : 32u;
            for (size_t i = 0u; i < show; i++)
                snprintf(hex + 2u * i, 3u, "%02x", flat[i]);
            hex[2u * show] = '\0';
            fprintf(stderr, "[bt] hci acl TX total=%u head=%s\n",
                    total, hex);
            fflush(stderr);
        }
        bt->stats.hci_forwarded_acl++;
        rc = 0;
    }
    {
        uint32_t chain[1] = {om};
        uint32_t free_rc = 0xFFFFFFFFu;
        int free_call = guest_call8(cpu, bt->hci_free_chain_addr, chain, 1,
                                    2000000u, &free_rc);
        if (getenv("FLEXE_BTDBG")) {
            unsigned acl_free = 9999u;
            if (bt->hci_mpool_acl_addr != 0u) {
                uint32_t ext =
                    (uint32_t)mem_read8(cpu->mem,
                        bt->hci_mpool_acl_addr + 4u) |
                    ((uint32_t)mem_read8(cpu->mem,
                        bt->hci_mpool_acl_addr + 5u) << 8) |
                    ((uint32_t)mem_read8(cpu->mem,
                        bt->hci_mpool_acl_addr + 6u) << 16) |
                    ((uint32_t)mem_read8(cpu->mem,
                        bt->hci_mpool_acl_addr + 7u) << 24);
                if (ext != 0u)
                    acl_free = (unsigned)mem_read8(cpu->mem, ext + 6u) |
                               ((unsigned)mem_read8(cpu->mem, ext + 7u)
                                << 8);
            }
            fprintf(stderr,
                    "[bt] hci acl TX free call=%d rc=%u aclfree=%u\n",
                    free_call, free_rc, acl_free);
            fflush(stderr);
        }
    }
    if (rc != 0) bt->stats.hci_injection_failures++;
    bt_return(cpu, (uint32_t)rc);
    return 1;
}

int bt_stubs_hook_hci_transport(bt_stubs_t *bt, const elf_symbols_t *syms)
{
    if (!bt || !syms) return 0;
    esp32_rom_stubs_t *rom = bt->cpu->pc_hook_ctx;
    if (!rom) return 0;
    bt->rom = rom;

    struct {
        const char *name;
        uint32_t *slot;
    } resolved[] = {
        {"ble_transport_alloc_evt", &bt->hci_alloc_evt_addr},
        {"ble_transport_to_hs_evt", &bt->hci_to_hs_evt_addr},
        {"ble_transport_free", &bt->hci_free_evt_addr},
        {"ble_hs_rx_data", &bt->hci_rx_acl_addr},
        {"ble_transport_alloc_acl_from_ll", &bt->hci_alloc_acl_addr},
        {"ble_hs_mbuf_acl_pkt", &bt->hci_msys_acl_addr},
        {"os_mbuf_append", &bt->hci_append_addr},
        {"os_mbuf_free", &bt->hci_mbuf_free_addr},
        {"os_mbuf_len", &bt->hci_mbuf_len_addr},
        {"os_mbuf_copydata", &bt->hci_copydata_addr},
        {"os_mbuf_free_chain", &bt->hci_free_chain_addr},
        {"pool_evt", &bt->hci_pool_evt_addr},
        {"pool_evt_lo", &bt->hci_pool_evt_lo_addr},
        {"mpool_acl", &bt->hci_mpool_acl_addr},
    };
    int found = 0;
    for (unsigned i = 0u;
         i < sizeof(resolved) / sizeof(resolved[0]); i++) {
        if (elf_symbols_find(syms, resolved[i].name,
                             resolved[i].slot) == 0 &&
            *resolved[i].slot != 0u)
            found++;
        else
            *resolved[i].slot = 0u;
    }
    /* The event entry is spelled _impl in the symbol table; the public
     * name is a link-time alias. rx_evt takes the same buffer shape. */
    static const char *const evt_names[] = {
        "ble_transport_to_hs_evt",
        "ble_transport_to_hs_evt_impl",
        "ble_hs_hci_rx_evt",
    };
    for (unsigned i = 0u;
         i < sizeof(evt_names) / sizeof(evt_names[0]); i++) {
        uint32_t addr = 0u;
        if (elf_symbols_find(syms, evt_names[i], &addr) == 0 &&
            addr != 0u) {
            if (bt->hci_to_hs_evt_addr == 0u) {
                bt->hci_to_hs_evt_addr = addr;
                found++;
            }
            break;
        }
    }
    if (bt->hci_to_hs_evt_addr == 0u)
        bt->hci_to_hs_evt_addr = 0u;
    uint32_t cmd_tx = 0u;
    if (elf_symbols_find(syms, "ble_hs_hci_cmd_tx", &cmd_tx) != 0 ||
        cmd_tx == 0u)
        return 0;
    if (getenv("FLEXE_BTDBG")) {
        fprintf(stderr,
                "[bt] hci addrs cmd_tx=0x%08x rx_acl=0x%08x alloc_evt=0x%08x "
                "to_hs=0x%08x mlen=0x%08x "
                "copy=0x%08x freechain=0x%08x\n",
                cmd_tx, bt->hci_rx_acl_addr, bt->hci_alloc_evt_addr,
                bt->hci_to_hs_evt_addr, bt->hci_mbuf_len_addr,
                bt->hci_copydata_addr, bt->hci_free_chain_addr);
        fflush(stderr);
    }
    rom_stubs_register_conditional_ctx(rom, cmd_tx,
                                       forward_ble_hs_hci_cmd_tx,
                                       "ble_hs_hci_cmd_tx", bt);
    uint32_t acl_tx = 0u;
    if (elf_symbols_find(syms, "ble_transport_to_ll_acl_impl",
                         &acl_tx) == 0 &&
        acl_tx != 0u)
        rom_stubs_register_conditional_ctx(rom, acl_tx,
                                           forward_ble_hci_acl_tx,
                                           "ble_transport_to_ll_acl", bt);
    else
        bt_log(bt, "HCI transport: no ACL TX symbol; host-to-controller "
                   "ACL stays unforwarded\n");
    fprintf(stderr,
            "[bt] HCI transport: forwarding to external controller "
            "(%d/%u support symbols)\n",
            found,
            (unsigned)(sizeof(resolved) / sizeof(resolved[0])));
    return 1 + found;
}

/* ===== Public API ===== */

bt_stubs_t *bt_stubs_create(xtensa_cpu_t *cpu)
{
    bt_stubs_t *bt = calloc(1, sizeof(*bt));
    if (!bt) return NULL;
    bt->cpu = cpu;
    bt->bt_status = ESP_BT_CONTROLLER_STATUS_IDLE;
    bt->hci_random_state = UINT64_C(0x6A09E667F3BCC909);

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

    /* Supported production builds run their real NimBLE host.  Replacing the
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

static bool firmware_bytes_match(xtensa_mem_t *mem, uint32_t addr,
                                 const uint8_t *expected, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        if (mem_read8(mem, addr + (uint32_t)i) != expected[i])
            return false;
    }
    return true;
}

static bool is_meshtastic_tbeam_2726(bt_stubs_t *bt, uint32_t entry_point)
{
    static const uint8_t hci_cmd_tx[] = {
        0x36, 0x61, 0x00, 0x71, 0x83, 0x7F, 0x70, 0xA7,
        0x20, 0x25, 0xEF, 0xE0, 0x40, 0xC4, 0x20, 0xBD,
    };
    static const uint8_t sync_wait[] = {
        0x21, 0x30, 0x17, 0x32, 0x02, 0x00, 0x39, 0x61,
        0x56, 0xC3, 0x0C, 0x81, 0x2D, 0x14, 0xE0, 0x08,
        0x00, 0x86, 0xFB, 0xFF, 0x00,
    };
    xtensa_mem_t *mem = bt->cpu->mem;
    return entry_point == MESHTASTIC_TBEAM_2726_ENTRY &&
           firmware_bytes_match(mem, MESHTASTIC_TBEAM_2726_HCI_CMD_TX,
                                hci_cmd_tx, sizeof(hci_cmd_tx)) &&
           firmware_bytes_match(mem, MESHTASTIC_TBEAM_2726_SYNC_WAIT,
                                sync_wait, sizeof(sync_wait));
}

int bt_stubs_hook_firmware_addrs(bt_stubs_t *bt, uint32_t entry_point)
{
    if (!bt || !bt->cpu || !bt->cpu->mem)
        return 0;
    esp32_rom_stubs_t *rom = bt->cpu->pc_hook_ctx;
    if (!rom)
        return 0;

    if (is_meshtastic_tbeam_2726(bt, entry_point)) {
        bt->rom = rom;
        bt->production_observer = true;
        bt->virtual_hci_all = true;
        rom_stubs_register_conditional_ctx(
                rom, MESHTASTIC_TBEAM_2726_HCI_CMD_TX + 3u,
                conditional_ble_hs_hci_cmd_tx, "ble_hs_hci_cmd_tx", bt);
        rom_stubs_register_ctx(rom,
                MESHTASTIC_TBEAM_2726_CONN_CAN_ALLOC + 3u,
                stub_ble_hs_conn_can_alloc, "ble_hs_conn_can_alloc", bt);
        fprintf(stderr, "[bt] attached Meshtastic T-Beam virtual "
                        "NimBLE controller\n");
        return 2;
    }

    /* v1.14/v1.15 for the 2432S028 share one entry point; the other CYD
     * boards are a separate link with their own. */
    if (entry_point != MARAUDER_V114_ENTRY &&
        entry_point != MARAUDER_BOARD_ENTRY &&
        entry_point != MARAUDER_V1121_CYD2USB_ENTRY)
        return 0;

    const marauder_bt_layout_t *layout = NULL;
    rom_firmware_profile_t profile = rom_stubs_identify_firmware(
            rom, entry_point);
    if (profile == ROM_FIRMWARE_MARAUDER_V1121_CYD2USB)
        layout = &marauder_v1121_cyd2usb_bt;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1140_1)
        layout = &marauder_v11401_bt;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1142_3)
        layout = &marauder_v11423_bt;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1151)
        layout = &marauder_v1151_bt;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1143_GUITION)
        layout = &marauder_guition_bt;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1143_35INCH)
        layout = &marauder_35inch_bt;
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
    if (getenv("FLEXE_BTDBG") && bt->ignore_list_addr &&
        bt->connected_peers_addr)
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
