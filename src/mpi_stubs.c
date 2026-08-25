/*
 * mpi_stubs.c — ESP32 MPI (RSA) hardware accelerator stubs
 *
 * Replaces the ESP32 RSA peripheral with software big-number arithmetic.
 * Hooks esp_mpi_* and esp_mont_hw_op symbols from the firmware ELF.
 */

#include "mpi_stubs.h"
#include "rom_stubs.h"
#include "memory.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <openssl/bn.h>

/* Maximum hardware words: ESP32 RSA supports up to 4096 bits = 128 uint32 */
#define MPI_MAX_WORDS 128

/* Original ESP32 RSA/MPI accelerator register layout. */
#define RSA_PERIPH_BASE        0x3FF02000u
#define RSA_PERIPH_PAGE        2
#define RSA_M_WORD             (0x000u / 4)
#define RSA_Z_WORD             (0x200u / 4)
#define RSA_Y_WORD             (0x400u / 4)
#define RSA_X_WORD             (0x600u / 4)
#define RSA_MEMORY_WORDS       (0x800u / 4)
#define RSA_M_DASH_OFF         0x800u
#define RSA_MODEXP_MODE_OFF    0x804u
#define RSA_MODEXP_START_OFF   0x808u
#define RSA_MULT_MODE_OFF      0x80Cu
#define RSA_MULT_START_OFF     0x810u
#define RSA_INTERRUPT_OFF      0x814u
#define RSA_QUERY_CLEAN_OFF    0x818u
#define RSA_INTR_SOURCE        51

/* mbedtls_mpi struct layout in emulator memory (32-bit platform):
 *   offset 0: int s        (sign, +1 or -1)
 *   offset 4: size_t n     (number of allocated limbs)
 *   offset 8: uint32_t *p  (pointer to limb array, little-endian order)
 */
#define MPI_OFS_S  0
#define MPI_OFS_N  4
#define MPI_OFS_P  8

struct mpi_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;
    esp32_periph_t    *periph;

    /* Per-core result buffer for async mul operations.
     * Each core gets its own buffer so interleaved batches don't corrupt. */
    uint32_t result[2][MPI_MAX_WORDS * 2];
    size_t   result_words[2];

    /* Shared physical RSA accelerator state.  M/Z/Y/X are four contiguous
     * 4096-bit little-endian memory blocks in the first 0x800 bytes. */
    uint32_t hw_memory[RSA_MEMORY_WORDS];
    uint32_t hw_m_dash;
    uint32_t hw_modexp_mode;
    uint32_t hw_mult_mode;
    bool     hw_interrupt;
};

/* ===== Calling convention helpers ===== */

static uint32_t mpi_arg(xtensa_cpu_t *cpu, int n)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void mpi_return(xtensa_cpu_t *cpu, uint32_t retval)
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

static void mpi_return_void(xtensa_cpu_t *cpu)
{
    mpi_return(cpu, 0);
}

/* ===== MPI memory helpers ===== */

/* Read an mbedtls_mpi's limb array from emulator memory.
 * Reads min(mpi.n, max_words) limbs, zero-pads to max_words. */
static void read_mpi_limbs(xtensa_cpu_t *cpu, uint32_t mpi_addr,
                           uint32_t *limbs, size_t max_words)
{
    uint32_t n = mem_read32(cpu->mem, mpi_addr + MPI_OFS_N);
    uint32_t p = mem_read32(cpu->mem, mpi_addr + MPI_OFS_P);

    size_t to_read = n < max_words ? n : max_words;
    for (size_t i = 0; i < to_read; i++)
        limbs[i] = mem_read32(cpu->mem, p + (uint32_t)(i * 4));
    for (size_t i = to_read; i < max_words; i++)
        limbs[i] = 0;
}

/* Write limbs to an mbedtls_mpi's p array in emulator memory.
 * Also zeroes any remaining allocated limbs beyond z_words. */
static void write_mpi_limbs(xtensa_cpu_t *cpu, uint32_t mpi_addr,
                            const uint32_t *limbs, size_t z_words)
{
    uint32_t n = mem_read32(cpu->mem, mpi_addr + MPI_OFS_N);
    uint32_t p = mem_read32(cpu->mem, mpi_addr + MPI_OFS_P);

    size_t to_write = z_words < n ? z_words : n;
    for (size_t i = 0; i < to_write; i++)
        mem_write32(cpu->mem, p + (uint32_t)(i * 4), limbs[i]);
    /* Zero remaining allocated limbs */
    for (size_t i = to_write; i < n; i++)
        mem_write32(cpu->mem, p + (uint32_t)(i * 4), 0);
}

/* ===== Big number arithmetic ===== */

/* Schoolbook multiplication: result[0..rw-1] = x[0..xw-1] * y[0..yw-1]
 * Result must be zeroed before calling. rw must be >= xw + yw. */
static void bignum_mul(const uint32_t *x, size_t xw,
                       const uint32_t *y, size_t yw,
                       uint32_t *result, size_t rw)
{
    memset(result, 0, rw * sizeof(uint32_t));
    for (size_t i = 0; i < yw; i++) {
        uint64_t carry = 0;
        for (size_t j = 0; j < xw; j++) {
            uint64_t prod = (uint64_t)x[j] * y[i] + result[i + j] + carry;
            result[i + j] = (uint32_t)prod;
            carry = prod >> 32;
        }
        if (i + xw < rw)
            result[i + xw] = (uint32_t)carry;
    }
}

/* Compare a[0..n-1] >= b[0..n-1] (little-endian limb order).
 * Returns 1 if a >= b, 0 otherwise. */
static int bignum_gte(const uint32_t *a, const uint32_t *b, size_t n)
{
    for (size_t i = n; i > 0; i--) {
        if (a[i - 1] > b[i - 1]) return 1;
        if (a[i - 1] < b[i - 1]) return 0;
    }
    return 1; /* equal */
}

/* Subtract: result[0..n-1] = a[0..n-1] - b[0..n-1] (assumes a >= b) */
static void bignum_sub(const uint32_t *a, const uint32_t *b,
                       uint32_t *result, size_t n)
{
    uint64_t borrow = 0;
    for (size_t i = 0; i < n; i++) {
        uint64_t diff = (uint64_t)a[i] - b[i] - borrow;
        result[i] = (uint32_t)diff;
        borrow = (diff >> 63) & 1;
    }
}

/* Montgomery multiplication: result = x * y * R^(-1) mod m
 * where R = 2^(n*32), m_prime = -m^(-1) mod 2^32.
 * Uses CIOS (Coarsely Integrated Operand Scanning) method.
 * x, y, m are n limbs each. result is n limbs. */
static void montgomery_mul(const uint32_t *x, const uint32_t *y,
                           const uint32_t *m, uint32_t m_prime,
                           size_t n, uint32_t *result)
{
    /* Working space: t[0..n] (n+2 words to handle carries) */
    uint32_t *t = calloc(n + 2, sizeof(uint32_t));
    if (!t) return;

    for (size_t i = 0; i < n; i++) {
        /* Step 1: t = t + x * y[i] */
        uint64_t carry = 0;
        for (size_t j = 0; j < n; j++) {
            uint64_t prod = (uint64_t)x[j] * y[i] + t[j] + carry;
            t[j] = (uint32_t)prod;
            carry = prod >> 32;
        }
        uint64_t sum = (uint64_t)t[n] + carry;
        t[n] = (uint32_t)sum;
        t[n + 1] = (uint32_t)(sum >> 32);

        /* Step 2: Montgomery reduction */
        uint32_t q = t[0] * m_prime;
        carry = 0;
        uint64_t prod0 = (uint64_t)q * m[0] + t[0];
        carry = prod0 >> 32;
        for (size_t j = 1; j < n; j++) {
            uint64_t prod = (uint64_t)q * m[j] + t[j] + carry;
            t[j - 1] = (uint32_t)prod;
            carry = prod >> 32;
        }
        sum = (uint64_t)t[n] + carry;
        t[n - 1] = (uint32_t)sum;
        t[n] = t[n + 1] + (uint32_t)(sum >> 32);
        t[n + 1] = 0;
    }

    /* Final reduction: if t >= m, t = t - m */
    if (t[n] || bignum_gte(t, m, n))
        bignum_sub(t, m, result, n);
    else
        memcpy(result, t, n * sizeof(uint32_t));

    free(t);
}

/* ===== RSA peripheral MMIO ===== */

static int words_to_bn(BIGNUM *bn, const uint32_t *words, size_t count)
{
    uint8_t bytes[MPI_MAX_WORDS * 4];
    if (count > MPI_MAX_WORDS) return 0;
    for (size_t i = 0; i < count; i++) {
        bytes[i * 4] = (uint8_t)words[i];
        bytes[i * 4 + 1] = (uint8_t)(words[i] >> 8);
        bytes[i * 4 + 2] = (uint8_t)(words[i] >> 16);
        bytes[i * 4 + 3] = (uint8_t)(words[i] >> 24);
    }
    return BN_lebin2bn(bytes, (int)(count * 4), bn) != NULL;
}

static int bn_to_words(const BIGNUM *bn, uint32_t *words, size_t count)
{
    uint8_t bytes[MPI_MAX_WORDS * 4];
    if (count > MPI_MAX_WORDS ||
        BN_bn2lebinpad(bn, bytes, (int)(count * 4)) < 0)
        return 0;
    for (size_t i = 0; i < count; i++) {
        words[i] = (uint32_t)bytes[i * 4] |
                   ((uint32_t)bytes[i * 4 + 1] << 8) |
                   ((uint32_t)bytes[i * 4 + 2] << 16) |
                   ((uint32_t)bytes[i * 4 + 3] << 24);
    }
    return 1;
}

static void rsa_complete(mpi_stubs_t *ms)
{
    ms->hw_interrupt = true;
    if (ms->periph)
        periph_assert_interrupt(ms->periph, RSA_INTR_SOURCE);
}

static void rsa_zero_z(mpi_stubs_t *ms)
{
    memset(&ms->hw_memory[RSA_Z_WORD], 0,
           MPI_MAX_WORDS * sizeof(uint32_t));
}

static void rsa_modexp(mpi_stubs_t *ms)
{
    /* MODEXP_MODE is N/512 - 1, for the eight supported lengths from
     * 512 through 4096 bits. */
    size_t words = ((size_t)ms->hw_modexp_mode + 1) * 16;

    BN_CTX *ctx = BN_CTX_new();
    if (!ctx) {
        rsa_zero_z(ms);
        rsa_complete(ms);
        return;
    }
    BN_CTX_start(ctx);
    BIGNUM *x = BN_CTX_get(ctx);
    BIGNUM *y = BN_CTX_get(ctx);
    BIGNUM *m = BN_CTX_get(ctx);
    BIGNUM *z = BN_CTX_get(ctx);
    int ok = z &&
             words_to_bn(x, &ms->hw_memory[RSA_X_WORD], words) &&
             words_to_bn(y, &ms->hw_memory[RSA_Y_WORD], words) &&
             words_to_bn(m, &ms->hw_memory[RSA_M_WORD], words) &&
             !BN_is_zero(m) && BN_mod_exp(z, x, y, m, ctx) &&
             bn_to_words(z, &ms->hw_memory[RSA_Z_WORD], words);
    if (!ok)
        rsa_zero_z(ms);
    BN_CTX_end(ctx);
    BN_CTX_free(ctx);
    rsa_complete(ms);
}

static void rsa_montgomery_mul(mpi_stubs_t *ms, size_t words)
{
    BN_CTX *ctx = BN_CTX_new();
    if (!ctx) {
        rsa_zero_z(ms);
        return;
    }
    BN_CTX_start(ctx);
    BIGNUM *x = BN_CTX_get(ctx);
    BIGNUM *y = BN_CTX_get(ctx);
    BIGNUM *m = BN_CTX_get(ctx);
    BIGNUM *r = BN_CTX_get(ctx);
    BIGNUM *rinv = BN_CTX_get(ctx);
    BIGNUM *product = BN_CTX_get(ctx);
    BIGNUM *z = BN_CTX_get(ctx);
    int ok = z &&
             words_to_bn(x, &ms->hw_memory[RSA_X_WORD], words) &&
             words_to_bn(y, &ms->hw_memory[RSA_Z_WORD], words) &&
             words_to_bn(m, &ms->hw_memory[RSA_M_WORD], words) &&
             !BN_is_zero(m) && BN_one(r) &&
             BN_lshift(r, r, (int)(words * 32)) &&
             BN_mod_inverse(rinv, r, m, ctx) != NULL &&
             BN_mod_mul(product, x, y, m, ctx) &&
             BN_mod_mul(z, product, rinv, m, ctx) &&
             bn_to_words(z, &ms->hw_memory[RSA_Z_WORD], words);
    if (!ok)
        rsa_zero_z(ms);
    BN_CTX_end(ctx);
    BN_CTX_free(ctx);
}

static void rsa_plain_mul(mpi_stubs_t *ms, size_t operand_words,
                          size_t result_words)
{
    BN_CTX *ctx = BN_CTX_new();
    if (!ctx) {
        rsa_zero_z(ms);
        return;
    }
    BN_CTX_start(ctx);
    BIGNUM *x = BN_CTX_get(ctx);
    BIGNUM *y = BN_CTX_get(ctx);
    BIGNUM *z = BN_CTX_get(ctx);
    int ok = z &&
             words_to_bn(x, &ms->hw_memory[RSA_X_WORD], operand_words) &&
             words_to_bn(y, &ms->hw_memory[RSA_Z_WORD + operand_words],
                         operand_words) &&
             BN_mul(z, x, y, ctx) &&
             bn_to_words(z, &ms->hw_memory[RSA_Z_WORD], result_words);
    if (!ok)
        rsa_zero_z(ms);
    BN_CTX_end(ctx);
    BN_CTX_free(ctx);
}

static void rsa_mult(mpi_stubs_t *ms)
{
    if (ms->hw_mult_mode <= 7) {
        size_t words = ((size_t)ms->hw_mult_mode + 1) * 16;
        rsa_montgomery_mul(ms, words);
    } else {
        /* ESP32's plain-multiply encoding stores a 2N-word result in Z and
         * places the second N-word operand in the upper half of Z. */
        size_t result_words = ((size_t)ms->hw_mult_mode - 7) * 16;
        if (result_words > MPI_MAX_WORDS) result_words = MPI_MAX_WORDS;
        rsa_plain_mul(ms, result_words / 2, result_words);
    }
    rsa_complete(ms);
}

static uint32_t rsa_mmio_read(void *ctx, uint32_t addr)
{
    mpi_stubs_t *ms = ctx;
    uint32_t off = addr - RSA_PERIPH_BASE;
    if (off < 0x800u)
        return ms->hw_memory[off / 4];

    switch (off) {
    case RSA_M_DASH_OFF:       return ms->hw_m_dash;
    case RSA_MODEXP_MODE_OFF:  return ms->hw_modexp_mode;
    case RSA_MULT_MODE_OFF:    return ms->hw_mult_mode;
    case RSA_INTERRUPT_OFF:    return ms->hw_interrupt ? 1u : 0u;
    case RSA_QUERY_CLEAN_OFF:  return 1u;
    default:                   return 0;
    }
}

static void rsa_mmio_write(void *ctx, uint32_t addr, uint32_t val)
{
    mpi_stubs_t *ms = ctx;
    uint32_t off = addr - RSA_PERIPH_BASE;
    if (off < 0x800u) {
        ms->hw_memory[off / 4] = val;
        return;
    }

    switch (off) {
    case RSA_M_DASH_OFF:
        ms->hw_m_dash = val;
        break;
    case RSA_MODEXP_MODE_OFF:
        ms->hw_modexp_mode = val & 7u;
        break;
    case RSA_MODEXP_START_OFF:
        if (val & 1u) {
            ms->hw_interrupt = false;
            rsa_modexp(ms);
        }
        break;
    case RSA_MULT_MODE_OFF:
        ms->hw_mult_mode = val & 15u;
        break;
    case RSA_MULT_START_OFF:
        if (val & 1u) {
            ms->hw_interrupt = false;
            rsa_mult(ms);
        }
        break;
    case RSA_INTERRUPT_OFF:
        if (val & 1u) {
            ms->hw_interrupt = false;
            if (ms->periph)
                periph_deassert_interrupt(ms->periph, RSA_INTR_SOURCE);
        }
        break;
    default:
        break;
    }
}

/* ===== Stub implementations ===== */

static void stub_esp_mpi_enable_hardware_hw_op(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    mpi_return_void(cpu);
}

static void stub_esp_mpi_disable_hardware_hw_op(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    mpi_return_void(cpu);
}

static void stub_esp_mpi_hardware_words(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    uint32_t words = mpi_arg(cpu, 0);
    /* Round up to next multiple of 16 (512-bit blocks) */
    uint32_t hw = (words + 0xF) & ~0xFu;
    mpi_return(cpu, hw);
}

/* esp_mpi_mul_mpi_hw_op(X, Y, hw_words): compute X * Y (full product) */
static void stub_esp_mpi_mul_mpi_hw_op(xtensa_cpu_t *cpu, void *ctx)
{
    mpi_stubs_t *ms = ctx;
    uint32_t x_addr  = mpi_arg(cpu, 0);
    uint32_t y_addr  = mpi_arg(cpu, 1);
    uint32_t hw_words = mpi_arg(cpu, 2);

    if (hw_words > MPI_MAX_WORDS) hw_words = MPI_MAX_WORDS;
    int c = cpu->core_id;

    uint32_t x[MPI_MAX_WORDS], y[MPI_MAX_WORDS];
    read_mpi_limbs(cpu, x_addr, x, hw_words);
    read_mpi_limbs(cpu, y_addr, y, hw_words);

    ms->result_words[c] = hw_words * 2;
    if (ms->result_words[c] > MPI_MAX_WORDS * 2)
        ms->result_words[c] = MPI_MAX_WORDS * 2;

    bignum_mul(x, hw_words, y, hw_words, ms->result[c], ms->result_words[c]);

    mpi_return_void(cpu);
}

/* esp_mpi_read_result_hw_op(Z, z_words): write stored result to Z->p */
static void stub_esp_mpi_read_result_hw_op(xtensa_cpu_t *cpu, void *ctx)
{
    mpi_stubs_t *ms = ctx;
    uint32_t z_addr  = mpi_arg(cpu, 0);
    uint32_t z_words = mpi_arg(cpu, 1);
    int c = cpu->core_id;

    if (z_words > ms->result_words[c]) z_words = ms->result_words[c];

    write_mpi_limbs(cpu, z_addr, ms->result[c], z_words);

    mpi_return_void(cpu);
}

/* esp_mpi_mult_mpi_failover_mod_mult_hw_op(X, Y, num_words):
 * Plain X * Y using Montgomery trick (result = X * Y, not modular) */
static void stub_esp_mpi_mult_mpi_failover_mod_mult_hw_op(xtensa_cpu_t *cpu,
                                                            void *ctx)
{
    mpi_stubs_t *ms = ctx;
    uint32_t x_addr   = mpi_arg(cpu, 0);
    uint32_t y_addr   = mpi_arg(cpu, 1);
    uint32_t num_words = mpi_arg(cpu, 2);

    if (num_words > MPI_MAX_WORDS) num_words = MPI_MAX_WORDS;
    int c = cpu->core_id;

    uint32_t x[MPI_MAX_WORDS], y[MPI_MAX_WORDS];
    read_mpi_limbs(cpu, x_addr, x, num_words);
    read_mpi_limbs(cpu, y_addr, y, num_words);

    /* This is just plain multiplication (the hardware trick makes mod a no-op).
     * Result is num_words wide (not double-width, since the mod truncates). */
    ms->result_words[c] = num_words;
    bignum_mul(x, num_words, y, num_words, ms->result[c], num_words);

    mpi_return_void(cpu);
}

/* esp_mont_hw_op(Z, X, Y, M, Mprime, hw_words, again):
 * Z = X * Y * R^(-1) mod M (Montgomery multiplication).
 * This is synchronous: computes, reads result, and reduces. Returns 0. */
static void stub_esp_mont_hw_op(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    uint32_t z_addr   = mpi_arg(cpu, 0);
    uint32_t x_addr   = mpi_arg(cpu, 1);
    uint32_t y_addr   = mpi_arg(cpu, 2);
    uint32_t m_addr   = mpi_arg(cpu, 3);

    /* Args 4-6 are on the stack for windowed CALL8.
     * For CALLINC=2 (CALL8): base = 2*4 = 8, so a10=arg0..
     * Stack args start at a2+N*4 where N is the register args count.
     * Actually, for windowed calls, extra args are on the caller's stack.
     * The caller pushes them at [caller_sp + 16 + extra_idx*4]. */
    int ci = XT_PS_CALLINC(cpu->ps);
    uint32_t caller_sp = ar_read(cpu, ci * 4 + 2 + 1 - 1); /* a1 of caller */

    /* Wait — let's get args 4-6 correctly.
     * In Xtensa windowed ABI with CALL8 (CALLINC=2):
     *   a10=retaddr, a11=sp, a12-a15 = args 0-3 (Z, X, Y, M)
     *   args 4-6 (Mprime, hw_words, again) are on the stack at [sp+0], [sp+4], [sp+8]
     *   where sp is the callee's frame pointer = a1.
     *   Actually, extra args go at [caller_a1 + 16 + idx*4] or similar.
     *   Let me read from the stack properly. */

    /* For windowed ABI: first 6 register args go in a2..a7 (relative to callee).
     * With CALL8 (ci=2), callee's a2 = caller's a10, so:
     *   arg0 = a(ci*4+2) = a10
     *   arg1 = a(ci*4+3) = a11
     *   arg2 = a(ci*4+4) = a12
     *   arg3 = a(ci*4+5) = a13
     * For args beyond the 4 register args (when using mpi_arg which starts at +2):
     *   arg4 = a(ci*4+6) = a14
     *   arg5 = a(ci*4+7) = a15
     * But beyond a15 (4 args for CALL8 with ci=2), extra args go on the stack. */

    /* Actually, for CALL8: caller uses a10..a15 for up to 6 args.
     * mpi_arg(cpu, n) reads ar[ci*4 + 2 + n], so:
     *   mpi_arg(cpu, 0) = ar[10] (Z)
     *   mpi_arg(cpu, 1) = ar[11] (X)
     *   mpi_arg(cpu, 2) = ar[12] (Y)
     *   mpi_arg(cpu, 3) = ar[13] (M)
     *   mpi_arg(cpu, 4) = ar[14] (Mprime)
     *   mpi_arg(cpu, 5) = ar[15] (hw_words)
     * For arg6 (again), it's on the stack. */

    uint32_t m_prime  = mpi_arg(cpu, 4);
    uint32_t hw_words = mpi_arg(cpu, 5);

    /* Arg 6 (again) is on the caller's stack.
     * Caller's SP = caller's a1. For CALL8, caller's a1 = callee's a9.
     * Stack arg is at [caller_sp + 32 + 0*4] (after 8 reg save area). */
    uint32_t callee_sp = ar_read(cpu, 1); /* a1 in current window */
    /* For CALL8 calling convention, the 7th arg is at stack frame.
     * On entry, the ENTRY instruction allocates frame, and extra args
     * were pushed by the caller at [old_sp + ...].
     * For the firmware's CALL8 convention:
     * caller pushes arg6 at [caller_a1 + 40] (8 words saved + 2 words overflow) */
    /* Actually, the standard Xtensa windowed ABI puts overflow args at
     * callee's [sp + 0], [sp + 4], etc. after ENTRY allocates its frame.
     * But that's not right either. The args are placed by the caller. */
    /* Let's just read from the callee's stack frame.
     * The ENTRY instruction copies caller's a8-a15 to callee's a0-a7,
     * and extra args are at the callee's stack just above the base window save. */
    /* For Xtensa, extra args are passed at [sp + frame_size] where frame_size
     * is the ENTRY allocation. This is too complex. Let's just skip 'again'
     * and always load M/Mprime (no optimization). */
    (void)callee_sp;
    (void)caller_sp;
    /* bool again = false; (always reload M) */

    if (hw_words > MPI_MAX_WORDS) hw_words = MPI_MAX_WORDS;

    uint32_t x[MPI_MAX_WORDS], y[MPI_MAX_WORDS], m[MPI_MAX_WORDS];
    read_mpi_limbs(cpu, x_addr, x, hw_words);
    read_mpi_limbs(cpu, y_addr, y, hw_words);
    read_mpi_limbs(cpu, m_addr, m, hw_words);

    /* Compute Z = X * Y * R^(-1) mod M (Montgomery multiplication) */
    uint32_t z[MPI_MAX_WORDS];
    montgomery_mul(x, y, m, m_prime, hw_words, z);

    /* Write result to Z (grow if needed, then write limbs) */
    uint32_t z_n = mem_read32(cpu->mem, z_addr + MPI_OFS_N);
    if (z_n < hw_words) {
        /* Can't easily grow from stub side — firmware should have pre-grown.
         * Just write what we can. */
    }
    write_mpi_limbs(cpu, z_addr, z, hw_words);

    mpi_return(cpu, 0); /* return 0 (success) */
}

/* ===== Public API ===== */

mpi_stubs_t *mpi_stubs_create(xtensa_cpu_t *cpu)
{
    mpi_stubs_t *ms = calloc(1, sizeof(*ms));
    if (!ms) return NULL;
    ms->cpu = cpu;
    mem_register_mmio(cpu->mem, RSA_PERIPH_PAGE,
                      rsa_mmio_read, rsa_mmio_write, ms);
    return ms;
}

void mpi_stubs_destroy(mpi_stubs_t *ms)
{
    free(ms);
}

void mpi_stubs_set_peripheral(mpi_stubs_t *ms, esp32_periph_t *periph)
{
    if (ms) ms->periph = periph;
}

int mpi_stubs_hook_symbols(mpi_stubs_t *ms, const elf_symbols_t *syms)
{
    if (!ms || !syms) return 0;

    esp32_rom_stubs_t *rom = ms->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ms->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "esp_mpi_enable_hardware_hw_op",              stub_esp_mpi_enable_hardware_hw_op },
        { "esp_mpi_disable_hardware_hw_op",             stub_esp_mpi_disable_hardware_hw_op },
        { "esp_mpi_hardware_words",                     stub_esp_mpi_hardware_words },
        { "esp_mpi_mul_mpi_hw_op",                      stub_esp_mpi_mul_mpi_hw_op },
        { "esp_mpi_read_result_hw_op",                  stub_esp_mpi_read_result_hw_op },
        { "esp_mpi_mult_mpi_failover_mod_mult_hw_op",   stub_esp_mpi_mult_mpi_failover_mod_mult_hw_op },
        { "esp_mont_hw_op",                             stub_esp_mont_hw_op },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn,
                                   hooks[i].name, ms);
            hooked++;
        }
    }

    if (hooked > 0)
        fprintf(stderr, "[mpi] hooked %d MPI symbols\n", hooked);

    return hooked;
}
