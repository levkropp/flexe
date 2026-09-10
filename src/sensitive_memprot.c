#include "sensitive_memprot.h"

#include <stdbool.h>
#include <stdlib.h>

/* Offsets and field masks are from ESP32-S3 sensitive_reg.h. This first
 * functional model covers the complete CPU SRAM/PIF protection register
 * interval. It records the policy, masks reserved bits, and honors write-once
 * configuration locks. Access-fault generation will consume the same state in
 * timed/cycle modes; status registers remain read-only zero until then. */
#define MEMPROT_FIRST_OFF 0x0C0u
#define MEMPROT_LAST_OFF  0x278u
#define MEMPROT_WORDS \
    (((MEMPROT_LAST_OFF - MEMPROT_FIRST_OFF) / sizeof(uint32_t)) + 1u)
#define NO_LOCK UINT32_MAX

struct flexe_sensitive_memprot {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t reg[MEMPROT_WORDS];
};

static bool memprot_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_SENSITIVE_MEMPROT_V1))
        return false;
    const flexe_sensitive_memprot_desc_t *desc =
        &target->sensitive_memprot;
    return (desc->base & 0xFFFu) == 0u &&
           desc->register_size >= MEMPROT_LAST_OFF + sizeof(uint32_t) &&
           desc->base >= target->peripheral_start &&
           desc->base < target->peripheral_end &&
           desc->register_size <= target->peripheral_end - desc->base;
}

static uint32_t *memprot_reg(flexe_sensitive_memprot_t *memprot,
                             uint32_t off)
{
    if ((off & 3u) != 0u || off < MEMPROT_FIRST_OFF ||
        off > MEMPROT_LAST_OFF)
        return NULL;
    return &memprot->reg[(off - MEMPROT_FIRST_OFF) / sizeof(uint32_t)];
}

static uint32_t writable_mask(uint32_t off)
{
    switch (off) {
    case 0x0C0u: case 0x0D8u: case 0x0E4u: case 0x0F0u:
    case 0x0FCu: case 0x104u: case 0x114u: case 0x124u:
    case 0x160u: case 0x19Cu: case 0x1B8u: case 0x1C8u:
    case 0x1D0u: case 0x20Cu: case 0x248u: case 0x264u:
    case 0x274u:
        return 0x00000001u;

    case 0x0C4u: case 0x0C8u: case 0x0CCu: case 0x0D0u:
    case 0x0D4u: case 0x148u: case 0x150u: case 0x158u:
    case 0x164u: case 0x168u: case 0x1F4u: case 0x1FCu:
    case 0x204u: case 0x210u: case 0x214u: case 0x270u:
        return 0x003FFFFFu;

    case 0x0DCu: case 0x0E0u:
        return 0x001FFFFFu;

    case 0x0E8u: case 0x0F4u: case 0x108u: case 0x118u:
    case 0x1A0u: case 0x1ACu: case 0x24Cu: case 0x258u:
        return 0x00000003u;

    case 0x100u:
        return 0x0FFFFFFFu;

    case 0x128u: case 0x138u: case 0x1D4u: case 0x1E4u:
        return 0xFF33CFFFu;
    case 0x12Cu: case 0x13Cu: case 0x1D8u: case 0x1E8u:
        return 0xFFCFFFF3u;
    case 0x130u: case 0x140u: case 0x1DCu: case 0x1ECu:
        return 0x3CC3FFFFu;
    case 0x134u: case 0x144u: case 0x1E0u: case 0x1F0u:
        return UINT32_MAX;

    case 0x14Cu: case 0x154u: case 0x15Cu: case 0x1F8u:
    case 0x200u: case 0x208u:
        return 0x00000FFFu;

    case 0x16Cu: case 0x170u: case 0x174u: case 0x178u:
    case 0x17Cu: case 0x180u: case 0x184u: case 0x188u:
    case 0x18Cu: case 0x190u: case 0x194u: case 0x198u:
    case 0x218u: case 0x21Cu: case 0x220u: case 0x224u:
    case 0x228u: case 0x22Cu: case 0x230u: case 0x234u:
    case 0x238u: case 0x23Cu: case 0x240u: case 0x244u:
        return 0x3FFFFFFFu;

    case 0x1BCu: case 0x268u:
        return 0x00000001u;
    case 0x1C0u: case 0x26Cu:
        return 0x00FFFFFFu;
    case 0x1C4u:
        return 0x003FFFFFu;
    case 0x1CCu: case 0x278u:
        return 0x00000001u;

    default:
        return 0u; /* Hardware-owned violation status. */
    }
}

static bool lock_register(uint32_t off)
{
    switch (off) {
    case 0x0C0u: case 0x0D8u: case 0x0E4u: case 0x0F0u:
    case 0x0FCu: case 0x104u: case 0x114u: case 0x124u:
    case 0x160u: case 0x19Cu: case 0x1B8u: case 0x1C8u:
    case 0x1D0u: case 0x20Cu: case 0x248u: case 0x264u:
    case 0x274u:
        return true;
    default:
        return false;
    }
}

static uint32_t protecting_lock(uint32_t off)
{
    if (off >= 0x0C4u && off <= 0x0D4u) return 0x0C0u;
    if (off == 0x0DCu || off == 0x0E0u) return 0x0D8u;
    if (off == 0x0E8u) return 0x0E4u;
    if (off == 0x0F4u) return 0x0F0u;
    if (off == 0x100u) return 0x0FCu;
    if (off == 0x108u) return 0x104u;
    if (off == 0x118u) return 0x114u;
    if (off >= 0x128u && off <= 0x15Cu) return 0x124u;
    if (off >= 0x164u && off <= 0x198u) return 0x160u;
    if (off >= 0x1A0u && off <= 0x1B4u) return 0x19Cu;
    if (off >= 0x1BCu && off <= 0x1C4u) return 0x1B8u;
    if (off == 0x1CCu) return 0x1C8u;
    if (off >= 0x1D4u && off <= 0x208u) return 0x1D0u;
    if (off >= 0x210u && off <= 0x244u) return 0x20Cu;
    if (off >= 0x24Cu && off <= 0x260u) return 0x248u;
    if (off >= 0x268u && off <= 0x270u) return 0x264u;
    if (off == 0x278u) return 0x274u;
    return NO_LOCK;
}

static uint32_t memprot_read(void *ctx, uint32_t addr)
{
    flexe_sensitive_memprot_t *memprot = ctx;
    uint32_t off = addr - memprot->target->sensitive_memprot.base;
    uint32_t *reg = memprot_reg(memprot, off);
    if (reg) return *reg;
    return memprot->fallback_read
        ? memprot->fallback_read(memprot->fallback_ctx, addr) : 0u;
}

static void memprot_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_sensitive_memprot_t *memprot = ctx;
    uint32_t off = addr - memprot->target->sensitive_memprot.base;
    uint32_t *reg = memprot_reg(memprot, off);
    if (!reg) {
        if (memprot->fallback_write)
            memprot->fallback_write(memprot->fallback_ctx, addr, value);
        return;
    }

    uint32_t mask = writable_mask(off);
    if (mask == 0u) return;
    if (lock_register(off)) {
        *reg |= value & mask; /* Lock bits cannot be cleared until reset. */
        return;
    }

    uint32_t lock_off = protecting_lock(off);
    if (lock_off != NO_LOCK) {
        uint32_t *lock = memprot_reg(memprot, lock_off);
        if (lock && (*lock & 1u) != 0u) return;
    }
    *reg = (*reg & ~mask) | (value & mask);
}

flexe_sensitive_memprot_t *flexe_sensitive_memprot_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!memprot_geometry_valid(target)) return NULL;

    flexe_sensitive_memprot_t *memprot = calloc(1, sizeof(*memprot));
    if (!memprot) return NULL;
    memprot->mem = mem;
    memprot->target = target;
    memprot->fallback_read = fallback_read;
    memprot->fallback_write = fallback_write;
    memprot->fallback_ctx = fallback_ctx;

    /* Both exception-loop override controls reset enabled. */
    *memprot_reg(memprot, 0x1CCu) = 1u;
    *memprot_reg(memprot, 0x278u) = 1u;

    const flexe_sensitive_memprot_desc_t *desc =
        &target->sensitive_memprot;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                memprot_read, memprot_write, memprot) != 0) {
        free(memprot);
        return NULL;
    }
    return memprot;
}

void flexe_sensitive_memprot_destroy(flexe_sensitive_memprot_t *memprot)
{
    if (!memprot) return;
    const flexe_sensitive_memprot_desc_t *desc =
        &memprot->target->sensitive_memprot;
    (void)mem_register_mmio_range(memprot->mem, desc->base,
                                  desc->register_size, NULL, NULL, NULL);
    free(memprot);
}
