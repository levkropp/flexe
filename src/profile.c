/* Interpreter sampling profiler.
 *
 * Answers "where is the interpreter actually spending its instructions",
 * which is the question that matters once a workload is mostly interpreted.
 * It found that NerdMiner spends over a tenth of its interpreted
 * instructions inside one spinlock acquire loop.
 *
 * Kept in its own translation unit on purpose. An earlier version lived in
 * xtensa.c behind the same compile-time switch, and *even fully compiled
 * out* it cost 4% on both stock ROMs -- the 48 bytes it added shifted the
 * dispatch loop's code layout. Measuring that took an interleaved A/B,
 * because a straight before/after was swamped by machine drift.
 *
 * Build with -DFLEXE_PROFILE=ON, run with FLEXE_PROFILE=1.
 */
#include "xtensa.h"

#include <stdio.h>
#include <stdlib.h>

#if FLEXE_PROFILE_BUILD

#define PROF_BUCKETS 4096u
static uint32_t g_prof_pc[PROF_BUCKETS];
/* Second histogram keyed by 1 KB region. Individual PCs are useless for
 * diffuse code -- unrolled hashing or a large state machine spreads samples
 * so thinly that no single PC stands out -- but the region they live in
 * still identifies the function. */
static uint32_t g_prof_rgn[PROF_BUCKETS];
static uint64_t g_prof_rgn_hits[PROF_BUCKETS];
static uint64_t g_prof_hits[PROF_BUCKETS];
static uint64_t g_prof_total;
static uint32_t g_prof_tick;
static int      g_prof_on;

/* Optional filter: only sample while the stack pointer is inside this 4 KB
 * region, i.e. only while one particular FreeRTOS task is running. That turns
 * a whole-system profile into a per-task one, which is the only way to ask
 * "what is *that* task doing" in a symbol-less ROM running its own
 * scheduler. Set FLEXE_PROFILE_SP=0x3FFD7000. */
static uint32_t g_prof_sp_filter;

void xtensa_profile_init(void)
{
    const char *e = getenv("FLEXE_PROFILE");
    g_prof_on = e ? atoi(e) : 0;
    const char *f = getenv("FLEXE_PROFILE_SP");
    g_prof_sp_filter = f ? (uint32_t)strtoul(f, NULL, 0) & ~0xFFFu : 0;
}

/* Sampled every 1024th dispatch: frequent enough to rank the hot PCs,
 * infrequent enough that the sampling itself does not distort them. */
/* Stack-pointer histogram. Each FreeRTOS task runs on its own stack, so
 * distinct a1 regions are distinct tasks -- which identifies who is consuming
 * the machine even in a symbol-less ROM running its own scheduler, where
 * there is no task list to read. */
static uint32_t g_prof_sp[PROF_BUCKETS];
static uint64_t g_prof_sp_hits[PROF_BUCKETS];

void xtensa_profile_tick_sp(uint32_t pc, uint32_t sp)
{
    if (g_prof_sp_filter && (sp & ~0xFFFu) != g_prof_sp_filter) {
        g_prof_tick++;   /* keep the sampling cadence honest */
        return;
    }
    xtensa_profile_tick(pc);
    if (!g_prof_on || (g_prof_tick & 1023u) != 0) return;
    uint32_t r = sp & ~0xFFFu;
    uint32_t h = (r >> 12) * 2654435761u;
    for (unsigned i = 0; i < 8; i++) {
        uint32_t k = (h + i) & (PROF_BUCKETS - 1);
        if (g_prof_sp_hits[k] == 0 || g_prof_sp[k] == r) {
            g_prof_sp[k] = r;
            g_prof_sp_hits[k]++;
            return;
        }
    }
}

void xtensa_profile_tick(uint32_t pc)
{
    if (!g_prof_on || (++g_prof_tick & 1023u) != 0) return;
    g_prof_total++;
    uint32_t h = (pc >> 2) * 2654435761u;
    for (unsigned i = 0; i < 8; i++) {
        uint32_t k = (h + i) & (PROF_BUCKETS - 1);
        if (g_prof_hits[k] == 0 || g_prof_pc[k] == pc) {
            g_prof_pc[k] = pc;
            g_prof_hits[k]++;
            break;
        }
    }
    uint32_t r = pc & ~0x3FFu;
    uint32_t rh = (r >> 10) * 2654435761u;
    for (unsigned i = 0; i < 8; i++) {
        uint32_t k = (rh + i) & (PROF_BUCKETS - 1);
        if (g_prof_rgn_hits[k] == 0 || g_prof_rgn[k] == r) {
            g_prof_rgn[k] = r;
            g_prof_rgn_hits[k]++;
            return;
        }
    }
}

void xtensa_profile_report(void)
{
    if (!g_prof_on || g_prof_total == 0) return;
    fprintf(stderr, "[profile] %llu samples, top PCs:\n",
            (unsigned long long)g_prof_total);
    for (int n = 0; n < 25; n++) {
        unsigned best = 0;
        for (unsigned i = 1; i < PROF_BUCKETS; i++)
            if (g_prof_hits[i] > g_prof_hits[best]) best = i;
        if (g_prof_hits[best] == 0) break;
        fprintf(stderr, "  %2d. pc=0x%08X  %6.2f%%  (%llu)\n", n + 1,
                g_prof_pc[best],
                100.0 * (double)g_prof_hits[best] / (double)g_prof_total,
                (unsigned long long)g_prof_hits[best]);
        g_prof_hits[best] = 0;
    }
    fprintf(stderr, "[profile] top 1 KB regions:\n");
    for (int n = 0; n < 12; n++) {
        unsigned best = 0;
        for (unsigned i = 1; i < PROF_BUCKETS; i++)
            if (g_prof_rgn_hits[i] > g_prof_rgn_hits[best]) best = i;
        if (g_prof_rgn_hits[best] == 0) break;
        fprintf(stderr, "  %2d. 0x%08X..0x%08X  %6.2f%%\n", n + 1,
                g_prof_rgn[best], g_prof_rgn[best] + 0x3FF,
                100.0 * (double)g_prof_rgn_hits[best] / (double)g_prof_total);
        g_prof_rgn_hits[best] = 0;
    }
    fprintf(stderr, "[profile] top task stacks (4 KB regions of a1):\n");
    for (int n = 0; n < 10; n++) {
        unsigned best = 0;
        for (unsigned i = 1; i < PROF_BUCKETS; i++)
            if (g_prof_sp_hits[i] > g_prof_sp_hits[best]) best = i;
        if (g_prof_sp_hits[best] == 0) break;
        fprintf(stderr, "  %2d. sp 0x%08X..0x%08X  %6.2f%%\n", n + 1,
                g_prof_sp[best], g_prof_sp[best] + 0xFFF,
                100.0 * (double)g_prof_sp_hits[best] / (double)g_prof_total);
        g_prof_sp_hits[best] = 0;
    }
}

#else
void xtensa_profile_report(void) { }
#endif /* FLEXE_PROFILE_BUILD */
