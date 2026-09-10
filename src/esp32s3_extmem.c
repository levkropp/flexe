#include "esp32s3_extmem.h"
#include "xtensa.h"

#include <stdbool.h>
#include <stdlib.h>

#define EXTMEM_REGISTER_BYTES 0x400u
#define EXTMEM_REGISTER_WORDS (EXTMEM_REGISTER_BYTES / sizeof(uint32_t))

/* Offsets and reset values are from ESP32-S3 extmem_reg.h. Keep this module
 * self-contained: these are properties of the S3 EXTMEM IP, while its base
 * and aperture live in the target descriptor. */
#define DCACHE_CTRL_OFF             0x000u
#define DCACHE_CTRL1_OFF            0x004u
#define DCACHE_TAG_POWER_OFF        0x008u
#define DCACHE_LOCK_CTRL_OFF        0x01Cu
#define DCACHE_SYNC_CTRL_OFF        0x028u
#define DCACHE_SYNC_ADDR_OFF        0x02Cu
#define DCACHE_SYNC_SIZE_OFF        0x030u
#define DCACHE_OCCUPY_CTRL_OFF      0x034u
#define DCACHE_PRELOAD_CTRL_OFF     0x040u
#define DCACHE_AUTOLOAD_CTRL_OFF    0x04Cu
#define ICACHE_CTRL_OFF             0x060u
#define ICACHE_CTRL1_OFF            0x064u
#define ICACHE_TAG_POWER_OFF        0x068u
#define ICACHE_LOCK_CTRL_OFF        0x07Cu
#define ICACHE_SYNC_CTRL_OFF        0x088u
#define ICACHE_SYNC_ADDR_OFF        0x08Cu
#define ICACHE_SYNC_SIZE_OFF        0x090u
#define ICACHE_PRELOAD_CTRL_OFF     0x094u
#define ICACHE_AUTOLOAD_CTRL_OFF    0x0A0u
#define IBUS_FLASH_START_OFF        0x0B4u
#define IBUS_FLASH_END_OFF          0x0B8u
#define DBUS_FLASH_START_OFF        0x0BCu
#define DBUS_FLASH_END_OFF          0x0C0u
#define CORE0_DBUS_REJECT_ADDR_OFF  0x104u
#define CORE0_IBUS_REJECT_ADDR_OFF  0x10Cu
#define CORE1_DBUS_REJECT_ADDR_OFF  0x114u
#define CORE1_IBUS_REJECT_ADDR_OFF  0x11Cu
#define CACHE_MMU_POWER_OFF         0x12Cu
#define CACHE_STATE_OFF             0x130u
#define CACHE_CRYPT_CLOCK_OFF       0x138u
#define CACHE_CONF_MISC_OFF         0x14Cu
#define DCACHE_FREEZE_OFF           0x150u
#define ICACHE_FREEZE_OFF           0x154u
#define ICACHE_ATOMIC_OFF           0x158u
#define DCACHE_ATOMIC_OFF           0x15Cu
#define CLOCK_GATE_OFF              0x164u
#define DATE_OFF                    0x3FCu

#define CACHE_ENABLE               (1u << 0)
#define CACHE_BUSES_SHUT           0x3u
#define CACHE_LOCK_DONE            (1u << 2)
#define DCACHE_SYNC_COMMANDS       0x7u
#define DCACHE_SYNC_DONE           (1u << 3)
#define ICACHE_SYNC_COMMAND        (1u << 0)
#define ICACHE_SYNC_DONE           (1u << 1)
#define CACHE_OPERATION_ENABLE     (1u << 0)
#define CACHE_OPERATION_DONE       (1u << 1)
#define CACHE_OPERATION_ORDER      (1u << 2)
#define CACHE_AUTOLOAD_DONE        (1u << 3)
#define CACHE_AUTOLOAD_CLEAR       (1u << 9)
#define CACHE_FREEZE_DONE          (1u << 2)
#define CACHE_IDLE_STATE           0x001001u
#define EXTMEM_DATE_RESET          0x02012310u

struct flexe_esp32s3_extmem {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    xtensa_cpu_t *cpu[2];
    uint32_t reg[EXTMEM_REGISTER_WORDS];
};

static bool extmem_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target ||
        !(target->capabilities & FLEXE_TARGET_CAP_ESP32S3_EXTMEM) ||
        target->cache_control_size < EXTMEM_REGISTER_BYTES ||
        (target->cache_control_base & 0xFFFu) != 0 ||
        target->cache_control_base < target->peripheral_start ||
        target->cache_control_base >= target->peripheral_end)
        return false;
    return target->cache_control_size <=
           target->peripheral_end - target->cache_control_base;
}

static uint32_t *extmem_reg(flexe_esp32s3_extmem_t *extmem, uint32_t off)
{
    if ((off & 3u) != 0 || off >= EXTMEM_REGISTER_BYTES) return NULL;
    return &extmem->reg[off / 4u];
}

static void extmem_invalidate_code(flexe_esp32s3_extmem_t *extmem,
                                   uint32_t addr, uint32_t block_count)
{
    uint32_t ctrl = extmem->reg[ICACHE_CTRL_OFF / 4u];
    uint32_t line_size = (ctrl & (1u << 3)) ? 32u : 16u;
    uint64_t bytes64 = (uint64_t)block_count * line_size;
    size_t bytes = bytes64 > SIZE_MAX ? SIZE_MAX : (size_t)bytes64;
    if (bytes == 0) return;

    xtensa_cpu_t *cpu0 = extmem->cpu[0];
    xtensa_cpu_t *cpu1 = extmem->cpu[1];
    if (cpu0) xtensa_invalidate_code(cpu0, addr, bytes);
    if (!cpu1) return;
    if (cpu0 && cpu1->predecode == cpu0->predecode &&
        cpu1->code_invalidate == cpu0->code_invalidate &&
        cpu1->code_invalidate_ctx == cpu0->code_invalidate_ctx)
        return;
    xtensa_invalidate_code(cpu1, addr, bytes);
}

static uint32_t extmem_read(void *ctx, uint32_t addr)
{
    flexe_esp32s3_extmem_t *extmem = ctx;
    uint32_t off = addr - extmem->target->cache_control_base;
    uint32_t *reg = extmem_reg(extmem, off);
    return reg ? *reg : 0;
}

static void extmem_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_esp32s3_extmem_t *extmem = ctx;
    uint32_t off = addr - extmem->target->cache_control_base;
    uint32_t *reg = extmem_reg(extmem, off);
    if (!reg) return;

    switch (off) {
    case DCACHE_CTRL_OFF:
        *reg = value & 0x1Du;
        return;
    case ICACHE_CTRL_OFF:
        *reg = value & 0x0Fu;
        return;
    case DCACHE_CTRL1_OFF:
    case ICACHE_CTRL1_OFF:
        *reg = value & CACHE_BUSES_SHUT;
        return;
    case DCACHE_TAG_POWER_OFF:
    case ICACHE_TAG_POWER_OFF:
    case CACHE_MMU_POWER_OFF:
        *reg = value & 0x7u;
        return;
    case DCACHE_LOCK_CTRL_OFF:
    case ICACHE_LOCK_CTRL_OFF:
        /* Fast mode completes lock/unlock synchronously. Command bits are
         * hardware-cleared and DONE remains observable to polling code. */
        *reg = CACHE_LOCK_DONE;
        return;
    case DCACHE_SYNC_CTRL_OFF:
        *reg = DCACHE_SYNC_DONE;
        return;
    case ICACHE_SYNC_CTRL_OFF:
        if (value & ICACHE_SYNC_COMMAND)
            extmem_invalidate_code(
                extmem, extmem->reg[ICACHE_SYNC_ADDR_OFF / 4u],
                extmem->reg[ICACHE_SYNC_SIZE_OFF / 4u] & 0x7FFFFFu);
        *reg = ICACHE_SYNC_DONE;
        return;
    case DCACHE_OCCUPY_CTRL_OFF:
        *reg = CACHE_OPERATION_DONE;
        return;
    case DCACHE_PRELOAD_CTRL_OFF:
    case ICACHE_PRELOAD_CTRL_OFF:
        *reg = (value & CACHE_OPERATION_ORDER) | CACHE_OPERATION_DONE;
        return;
    case DCACHE_AUTOLOAD_CTRL_OFF:
    case ICACHE_AUTOLOAD_CTRL_OFF:
        /* BUFFER_CLEAR is a pulse. The autoload enable/configuration remains
         * readable; DONE is immediate in the functional fast model. */
        *reg = (value & ~CACHE_AUTOLOAD_CLEAR) | CACHE_AUTOLOAD_DONE;
        return;
    case DCACHE_FREEZE_OFF:
    case ICACHE_FREEZE_OFF:
        *reg = (value & 0x3u) | CACHE_FREEZE_DONE;
        return;
    case CACHE_STATE_OFF:
    case CORE0_DBUS_REJECT_ADDR_OFF:
    case CORE0_IBUS_REJECT_ADDR_OFF:
    case CORE1_DBUS_REJECT_ADDR_OFF:
    case CORE1_IBUS_REJECT_ADDR_OFF:
        return; /* read-only */
    case DATE_OFF:
        *reg = value & 0x0FFFFFFFu;
        return;
    default:
        *reg = value;
        return;
    }
}

flexe_esp32s3_extmem_t *flexe_esp32s3_extmem_create(xtensa_mem_t *mem)
{
    const flexe_target_desc_t *target = mem_target(mem);
    if (!mem || !extmem_geometry_valid(target)) return NULL;

    flexe_esp32s3_extmem_t *extmem = calloc(1, sizeof(*extmem));
    if (!extmem) return NULL;
    extmem->mem = mem;
    extmem->target = target;

    /* Documented hardware reset state for values software commonly probes. */
    extmem->reg[DCACHE_CTRL1_OFF / 4u] = CACHE_BUSES_SHUT;
    extmem->reg[ICACHE_CTRL1_OFF / 4u] = CACHE_BUSES_SHUT;
    extmem->reg[DCACHE_TAG_POWER_OFF / 4u] = 0x5u;
    extmem->reg[ICACHE_TAG_POWER_OFF / 4u] = 0x5u;
    extmem->reg[DCACHE_LOCK_CTRL_OFF / 4u] = CACHE_LOCK_DONE;
    extmem->reg[ICACHE_LOCK_CTRL_OFF / 4u] = CACHE_LOCK_DONE;
    extmem->reg[DCACHE_SYNC_CTRL_OFF / 4u] = CACHE_OPERATION_ENABLE;
    extmem->reg[ICACHE_SYNC_CTRL_OFF / 4u] = CACHE_OPERATION_ENABLE;
    extmem->reg[DCACHE_OCCUPY_CTRL_OFF / 4u] = CACHE_OPERATION_DONE;
    extmem->reg[DCACHE_PRELOAD_CTRL_OFF / 4u] = CACHE_OPERATION_DONE;
    extmem->reg[ICACHE_PRELOAD_CTRL_OFF / 4u] = CACHE_OPERATION_DONE;
    extmem->reg[DCACHE_AUTOLOAD_CTRL_OFF / 4u] = CACHE_AUTOLOAD_DONE;
    extmem->reg[ICACHE_AUTOLOAD_CTRL_OFF / 4u] = CACHE_AUTOLOAD_DONE;
    extmem->reg[IBUS_FLASH_START_OFF / 4u] = 0x44000000u;
    extmem->reg[IBUS_FLASH_END_OFF / 4u] = 0x47FFFFFFu;
    extmem->reg[CORE0_DBUS_REJECT_ADDR_OFF / 4u] = UINT32_MAX;
    extmem->reg[CORE0_IBUS_REJECT_ADDR_OFF / 4u] = UINT32_MAX;
    extmem->reg[CORE1_DBUS_REJECT_ADDR_OFF / 4u] = UINT32_MAX;
    extmem->reg[CORE1_IBUS_REJECT_ADDR_OFF / 4u] = UINT32_MAX;
    extmem->reg[CACHE_MMU_POWER_OFF / 4u] = 0x5u;
    extmem->reg[CACHE_STATE_OFF / 4u] = CACHE_IDLE_STATE;
    extmem->reg[CACHE_CRYPT_CLOCK_OFF / 4u] = 0x7u;
    extmem->reg[CACHE_CONF_MISC_OFF / 4u] = 0x7u;
    extmem->reg[DCACHE_FREEZE_OFF / 4u] = CACHE_FREEZE_DONE;
    extmem->reg[ICACHE_FREEZE_OFF / 4u] = CACHE_FREEZE_DONE;
    extmem->reg[ICACHE_ATOMIC_OFF / 4u] = 1u;
    extmem->reg[DCACHE_ATOMIC_OFF / 4u] = 1u;
    extmem->reg[CLOCK_GATE_OFF / 4u] = 1u;
    extmem->reg[DATE_OFF / 4u] = EXTMEM_DATE_RESET;

    if (mem_register_mmio_range(mem, target->cache_control_base,
                                target->cache_control_size,
                                extmem_read, extmem_write, extmem) != 0) {
        free(extmem);
        return NULL;
    }
    return extmem;
}

void flexe_esp32s3_extmem_destroy(flexe_esp32s3_extmem_t *extmem)
{
    if (!extmem) return;
    (void)mem_register_mmio_range(extmem->mem,
                                  extmem->target->cache_control_base,
                                  extmem->target->cache_control_size,
                                  NULL, NULL, NULL);
    free(extmem);
}

void flexe_esp32s3_extmem_application_handoff(
    flexe_esp32s3_extmem_t *extmem)
{
    if (!extmem) return;
    extmem->reg[DCACHE_CTRL_OFF / 4u] |= CACHE_ENABLE;
    extmem->reg[ICACHE_CTRL_OFF / 4u] |= CACHE_ENABLE;
    extmem->reg[DCACHE_CTRL1_OFF / 4u] = 0;
    extmem->reg[ICACHE_CTRL1_OFF / 4u] = 0;
    extmem->reg[DCACHE_SYNC_CTRL_OFF / 4u] = DCACHE_SYNC_DONE;
    extmem->reg[ICACHE_SYNC_CTRL_OFF / 4u] = ICACHE_SYNC_DONE;
    extmem->reg[IBUS_FLASH_START_OFF / 4u] = extmem->target->irom_start;
    extmem->reg[IBUS_FLASH_END_OFF / 4u] = extmem->target->irom_end - 1u;
    extmem->reg[DBUS_FLASH_START_OFF / 4u] = extmem->target->drom_start;
    extmem->reg[DBUS_FLASH_END_OFF / 4u] = extmem->target->drom_end - 1u;
}

void flexe_esp32s3_extmem_attach_cpus(flexe_esp32s3_extmem_t *extmem,
                                      xtensa_cpu_t *cpu0,
                                      xtensa_cpu_t *cpu1)
{
    if (!extmem) return;
    extmem->cpu[0] = cpu0;
    extmem->cpu[1] = cpu1;
}
