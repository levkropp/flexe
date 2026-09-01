/* Compute benchmark fixture.
 *
 * A fixed, deterministic amount of integer work per round, so the host
 * harness can report emulated MIPS against identical work on every run and
 * on both engines. The kernels are deliberately different shapes: a
 * dependent scalar chain, strided array traffic, the rotate/xor mixing a
 * SHA-256 round is made of, and byte-wide load/store. Between them they
 * cover the instruction mix the JIT actually has to be good at.
 *
 * Every kernel feeds its result into the next, so a single wrong instruction
 * anywhere changes flexe_bench_checksum. The interpreter and the JIT must
 * agree on it exactly.
 */
#include <Arduino.h>

/* Markers the host harness resolves by ELF symbol. */
volatile uint32_t flexe_bench_stage = 0;     /* 1 once setup() has run */
volatile uint32_t flexe_bench_rounds = 0;    /* completed rounds */
volatile uint32_t flexe_bench_checksum = 0;  /* running result */
volatile uint32_t flexe_bench_kernel[4];     /* per-kernel result */
/* Host-set round limit. Once reached the fixture stops mutating its result
 * state, so the host can read a checksum that corresponds to exactly N rounds
 * no matter where its own batch boundary happened to fall. Without this the
 * two engines can stop on different rounds and disagree for no good reason. */
volatile uint32_t flexe_bench_limit = 0;

#define BENCH_WORDS 256u
static uint32_t buf[BENCH_WORDS];

/* K1: dependent integer chain. Latency bound, no memory traffic -- measures
 * raw dispatch and ALU throughput. */
static uint32_t kernel_mix(uint32_t acc) {
  for (uint32_t i = 0; i < 4096u; ++i) {
    acc = acc * 1664525u + 1013904223u;
    acc ^= acc >> 13;
    acc = (acc << 5) | (acc >> 27);
  }
  return acc;
}

/* K2: strided 32-bit load/store over a small array. */
static uint32_t kernel_array(uint32_t acc) {
  for (uint32_t i = 0; i < BENCH_WORDS; ++i) buf[i] += acc ^ i;
  for (uint32_t i = 0; i < BENCH_WORDS; ++i)
    acc += buf[(i * 7u) & (BENCH_WORDS - 1u)];
  return acc;
}

static inline uint32_t rotr(uint32_t x, uint32_t n) {
  return (x >> n) | (x << (32u - n));
}

/* K3: the rotate/xor/majority mixing of a SHA-256 round -- the inner loop
 * NerdMiner spends its time in. */
static uint32_t kernel_sha_round(uint32_t acc) {
  uint32_t a = acc, b = 0x6A09E667u, c = 0xBB67AE85u, d = 0x3C6EF372u;
  for (uint32_t i = 0; i < 1024u; ++i) {
    uint32_t s1 = rotr(a, 6) ^ rotr(a, 11) ^ rotr(a, 25);
    uint32_t ch = (a & b) ^ (~a & c);
    uint32_t t1 = d + s1 + ch + i;
    uint32_t s0 = rotr(t1, 2) ^ rotr(t1, 13) ^ rotr(t1, 22);
    uint32_t maj = (t1 & a) ^ (t1 & b) ^ (a & b);
    d = c; c = b; b = a; a = t1 + s0 + maj;
  }
  return a;
}

/* K4: byte-wide load/store, which lowers to L8UI/S8I rather than L32I/S32I. */
static uint32_t kernel_bytes(uint32_t acc) {
  uint8_t *p = (uint8_t *)buf;
  for (uint32_t i = 0; i < BENCH_WORDS * 4u; ++i) {
    p[i] = (uint8_t)(p[i] * 31u + (uint8_t)acc);
    acc += p[i];
  }
  return acc;
}

void setup() {
  for (uint32_t i = 0; i < BENCH_WORDS; ++i) buf[i] = i * 2654435761u;
  flexe_bench_stage = 1;
}

void loop() {
  if (flexe_bench_limit != 0u && flexe_bench_rounds >= flexe_bench_limit) {
    delay(1);
    return;
  }
  uint32_t acc = flexe_bench_checksum ? flexe_bench_checksum : 0x12345678u;
  acc = kernel_mix(acc);       flexe_bench_kernel[0] = acc;
  acc = kernel_array(acc);     flexe_bench_kernel[1] = acc;
  acc = kernel_sha_round(acc); flexe_bench_kernel[2] = acc;
  acc = kernel_bytes(acc);     flexe_bench_kernel[3] = acc;
  flexe_bench_checksum = acc;
  flexe_bench_rounds = flexe_bench_rounds + 1u;
}
