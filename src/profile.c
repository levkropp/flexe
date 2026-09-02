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
static uint64_t g_prof_hits[PROF_BUCKETS];
static uint64_t g_prof_total;
static uint32_t g_prof_tick;
static int      g_prof_on;

void xtensa_profile_init(void)
{
    const char *e = getenv("FLEXE_PROFILE");
    g_prof_on = e ? atoi(e) : 0;
}

/* Sampled every 1024th dispatch: frequent enough to rank the hot PCs,
 * infrequent enough that the sampling itself does not distort them. */
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
}

#else
void xtensa_profile_report(void) { }
#endif /* FLEXE_PROFILE_BUILD */
