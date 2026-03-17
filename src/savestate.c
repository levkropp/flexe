#include "savestate.h"
#include "xtensa.h"
#include "memory.h"
#include "freertos_stubs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

/* Memory region sizes (from memory.c) */
#define SRAM_SIZE       (704 * 1024)
#define ROM_SIZE        (384 * 1024)
#define FLASH_SIZE      (4 * 1024 * 1024)
#define PSRAM_SIZE      (4 * 1024 * 1024)
#define RTC_DRAM_SIZE   (8 * 1024)
#define RTC_FAST_SIZE   (8 * 1024)
#define RTC_SLOW_SIZE   (8 * 1024)

/* Helper macros for safe file I/O */
#define WRITE_OR_FAIL(f, ptr, size, msg) \
    do { \
        if (fwrite(ptr, 1, size, f) != size) { \
            fprintf(stderr, "savestate_save: failed to write %s: %s\n", msg, strerror(errno)); \
            fclose(f); \
            return -1; \
        } \
    } while (0)

#define READ_OR_FAIL(f, ptr, size, msg) \
    do { \
        if (fread(ptr, 1, size, f) != size) { \
            fprintf(stderr, "savestate_restore: failed to read %s: %s\n", msg, strerror(errno)); \
            fclose(f); \
            return -1; \
        } \
    } while (0)

/*
 * Save complete emulator state to file.
 */
int savestate_save(xtensa_cpu_t *cpu, freertos_stubs_t *frt, const char *path, const char *description) {
    if (!cpu || !path) {
        fprintf(stderr, "savestate_save: null argument\n");
        return -1;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "savestate_save: failed to open %s: %s\n", path, strerror(errno));
        return -1;
    }

    /* Prepare header */
    savestate_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = SAVESTATE_MAGIC;
    hdr.version = SAVESTATE_VERSION;
    hdr.cycle_count = cpu->cycle_count;
    hdr.real_cycles = cpu->cycle_count; /* Same for now (no delay stub acceleration) */
    hdr.timestamp = (uint64_t)time(NULL);

    if (description) {
        strncpy(hdr.description, description, sizeof(hdr.description) - 1);
        hdr.description[sizeof(hdr.description) - 1] = '\0';
    } else {
        snprintf(hdr.description, sizeof(hdr.description), "checkpoint-%llu",
                 (unsigned long long)hdr.cycle_count);
    }

    hdr.iram_size = SRAM_SIZE;
    hdr.dram_size = SRAM_SIZE;  /* SRAM is used for both IRAM and DRAM */
    hdr.flash_size = FLASH_SIZE;
    hdr.psram_size = PSRAM_SIZE;
    hdr.compressed = 0;
    hdr.differential = 0;

    /* Write header */
    WRITE_OR_FAIL(f, &hdr, sizeof(hdr), "header");

    /* === CPU State === */
    /* General-purpose registers (64 physical registers) */
    WRITE_OR_FAIL(f, cpu->ar, sizeof(cpu->ar), "AR registers");

    /* Program counter */
    WRITE_OR_FAIL(f, &cpu->pc, sizeof(cpu->pc), "PC");

    /* Special registers */
    WRITE_OR_FAIL(f, &cpu->sar, sizeof(cpu->sar), "SAR");
    WRITE_OR_FAIL(f, &cpu->lbeg, sizeof(cpu->lbeg), "LBEG");
    WRITE_OR_FAIL(f, &cpu->lend, sizeof(cpu->lend), "LEND");
    WRITE_OR_FAIL(f, &cpu->lcount, sizeof(cpu->lcount), "LCOUNT");
    WRITE_OR_FAIL(f, &cpu->windowbase, sizeof(cpu->windowbase), "WINDOWBASE");
    WRITE_OR_FAIL(f, &cpu->windowstart, sizeof(cpu->windowstart), "WINDOWSTART");
    WRITE_OR_FAIL(f, &cpu->ps, sizeof(cpu->ps), "PS");
    WRITE_OR_FAIL(f, &cpu->vecbase, sizeof(cpu->vecbase), "VECBASE");
    WRITE_OR_FAIL(f, &cpu->exccause, sizeof(cpu->exccause), "EXCCAUSE");
    WRITE_OR_FAIL(f, &cpu->excvaddr, sizeof(cpu->excvaddr), "EXCVADDR");
    WRITE_OR_FAIL(f, cpu->epc, sizeof(cpu->epc), "EPC");
    WRITE_OR_FAIL(f, cpu->eps, sizeof(cpu->eps), "EPS");
    WRITE_OR_FAIL(f, cpu->excsave, sizeof(cpu->excsave), "EXCSAVE");
    WRITE_OR_FAIL(f, &cpu->depc, sizeof(cpu->depc), "DEPC");
    WRITE_OR_FAIL(f, &cpu->ccount, sizeof(cpu->ccount), "CCOUNT");
    WRITE_OR_FAIL(f, cpu->ccompare, sizeof(cpu->ccompare), "CCOMPARE");
    WRITE_OR_FAIL(f, &cpu->next_timer_event, sizeof(cpu->next_timer_event), "NEXT_TIMER_EVENT");
    WRITE_OR_FAIL(f, &cpu->scompare1, sizeof(cpu->scompare1), "SCOMPARE1");
    WRITE_OR_FAIL(f, cpu->misc, sizeof(cpu->misc), "MISC");
    WRITE_OR_FAIL(f, &cpu->litbase, sizeof(cpu->litbase), "LITBASE");
    WRITE_OR_FAIL(f, &cpu->atomctl, sizeof(cpu->atomctl), "ATOMCTL");
    WRITE_OR_FAIL(f, &cpu->memctl, sizeof(cpu->memctl), "MEMCTL");
    WRITE_OR_FAIL(f, &cpu->icount, sizeof(cpu->icount), "ICOUNT");
    WRITE_OR_FAIL(f, &cpu->icountlevel, sizeof(cpu->icountlevel), "ICOUNTLEVEL");
    WRITE_OR_FAIL(f, &cpu->debugcause, sizeof(cpu->debugcause), "DEBUGCAUSE");
    WRITE_OR_FAIL(f, &cpu->ibreakenable, sizeof(cpu->ibreakenable), "IBREAKENABLE");
    WRITE_OR_FAIL(f, cpu->ibreaka, sizeof(cpu->ibreaka), "IBREAKA");
    WRITE_OR_FAIL(f, cpu->dbreaka, sizeof(cpu->dbreaka), "DBREAKA");
    WRITE_OR_FAIL(f, cpu->dbreakc, sizeof(cpu->dbreakc), "DBREAKC");
    WRITE_OR_FAIL(f, &cpu->configid0, sizeof(cpu->configid0), "CONFIGID0");
    WRITE_OR_FAIL(f, &cpu->configid1, sizeof(cpu->configid1), "CONFIGID1");
    WRITE_OR_FAIL(f, &cpu->prid, sizeof(cpu->prid), "PRID");
    WRITE_OR_FAIL(f, &cpu->core_id, sizeof(cpu->core_id), "CORE_ID");
    WRITE_OR_FAIL(f, &cpu->intenable, sizeof(cpu->intenable), "INTENABLE");
    WRITE_OR_FAIL(f, &cpu->interrupt, sizeof(cpu->interrupt), "INTERRUPT");
    WRITE_OR_FAIL(f, &cpu->cpenable, sizeof(cpu->cpenable), "CPENABLE");

    /* Boolean and MAC16 state */
    WRITE_OR_FAIL(f, &cpu->br, sizeof(cpu->br), "BR");
    WRITE_OR_FAIL(f, &cpu->acclo, sizeof(cpu->acclo), "ACCLO");
    WRITE_OR_FAIL(f, &cpu->acchi, sizeof(cpu->acchi), "ACCHI");
    WRITE_OR_FAIL(f, cpu->mr, sizeof(cpu->mr), "MR");

    /* Floating-point state */
    WRITE_OR_FAIL(f, &cpu->fcr, sizeof(cpu->fcr), "FCR");
    WRITE_OR_FAIL(f, &cpu->fsr, sizeof(cpu->fsr), "FSR");
    WRITE_OR_FAIL(f, cpu->fr, sizeof(cpu->fr), "FR");

    /* Interrupt configuration */
    WRITE_OR_FAIL(f, cpu->int_level, sizeof(cpu->int_level), "INT_LEVEL");

    /* Window spill state */
    WRITE_OR_FAIL(f, cpu->spill_stack, sizeof(cpu->spill_stack), "SPILL_STACK");
    WRITE_OR_FAIL(f, cpu->spill_base, sizeof(cpu->spill_base), "SPILL_BASE");
    WRITE_OR_FAIL(f, cpu->spill_shadow, sizeof(cpu->spill_shadow), "SPILL_SHADOW");
    WRITE_OR_FAIL(f, &cpu->spill_verify, sizeof(cpu->spill_verify), "SPILL_VERIFY");

    /* Execution state */
    WRITE_OR_FAIL(f, &cpu->running, sizeof(cpu->running), "RUNNING");
    WRITE_OR_FAIL(f, &cpu->exception, sizeof(cpu->exception), "EXCEPTION");
    WRITE_OR_FAIL(f, &cpu->debug_break, sizeof(cpu->debug_break), "DEBUG_BREAK");
    WRITE_OR_FAIL(f, &cpu->halted, sizeof(cpu->halted), "HALTED");
    WRITE_OR_FAIL(f, &cpu->cycle_count, sizeof(cpu->cycle_count), "CYCLE_COUNT");
    WRITE_OR_FAIL(f, &cpu->virtual_time_us, sizeof(cpu->virtual_time_us), "VIRTUAL_TIME_US");

    /* === Memory Regions === */
    if (cpu->mem) {
        /* SRAM (704KB) */
        if (cpu->mem->sram) {
            WRITE_OR_FAIL(f, cpu->mem->sram, SRAM_SIZE, "SRAM");
        } else {
            fprintf(stderr, "savestate_save: SRAM is NULL\n");
            fclose(f);
            return -1;
        }

        /* ROM (384KB) - Optional: skip if not needed for restore */
        if (cpu->mem->rom) {
            WRITE_OR_FAIL(f, cpu->mem->rom, ROM_SIZE, "ROM");
        } else {
            fprintf(stderr, "savestate_save: ROM is NULL\n");
            fclose(f);
            return -1;
        }

        /* Flash data (4MB) */
        if (cpu->mem->flash_data) {
            WRITE_OR_FAIL(f, cpu->mem->flash_data, FLASH_SIZE, "FLASH_DATA");
        } else {
            fprintf(stderr, "savestate_save: FLASH_DATA is NULL\n");
            fclose(f);
            return -1;
        }

        /* Flash instruction (4MB) */
        if (cpu->mem->flash_insn) {
            WRITE_OR_FAIL(f, cpu->mem->flash_insn, FLASH_SIZE, "FLASH_INSN");
        } else {
            fprintf(stderr, "savestate_save: FLASH_INSN is NULL\n");
            fclose(f);
            return -1;
        }

        /* PSRAM (4MB) */
        if (cpu->mem->psram) {
            WRITE_OR_FAIL(f, cpu->mem->psram, PSRAM_SIZE, "PSRAM");
        } else {
            fprintf(stderr, "savestate_save: PSRAM is NULL\n");
            fclose(f);
            return -1;
        }

        /* RTC memory regions */
        if (cpu->mem->rtc_dram) {
            WRITE_OR_FAIL(f, cpu->mem->rtc_dram, RTC_DRAM_SIZE, "RTC_DRAM");
        }
        if (cpu->mem->rtc_fast) {
            WRITE_OR_FAIL(f, cpu->mem->rtc_fast, RTC_FAST_SIZE, "RTC_FAST");
        }
        if (cpu->mem->rtc_slow) {
            WRITE_OR_FAIL(f, cpu->mem->rtc_slow, RTC_SLOW_SIZE, "RTC_SLOW");
        }
    } else {
        fprintf(stderr, "savestate_save: memory subsystem is NULL\n");
        fclose(f);
        return -1;
    }

    /* === FreeRTOS State === */
    if (frt) {
        /* Marker to indicate FreeRTOS data follows */
        uint32_t freertos_marker = 0xFEEDC0DE;
        WRITE_OR_FAIL(f, &freertos_marker, sizeof(freertos_marker), "FREERTOS_MARKER");

        /* Save FreeRTOS state */
        if (freertos_stubs_save_state(frt, f) != 0) {
            fprintf(stderr, "savestate_save: failed to save FreeRTOS state\n");
            fclose(f);
            return -1;
        }
    } else {
        /* No FreeRTOS - write null marker */
        uint32_t null_marker = 0;
        WRITE_OR_FAIL(f, &null_marker, sizeof(null_marker), "NULL_FREERTOS_MARKER");
    }

    fclose(f);
    return 0;
}

/*
 * Restore emulator state from checkpoint file.
 */
int savestate_restore(xtensa_cpu_t *cpu, freertos_stubs_t *frt, const char *path) {
    if (!cpu || !path) {
        fprintf(stderr, "savestate_restore: null argument\n");
        return -1;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "savestate_restore: failed to open %s: %s\n", path, strerror(errno));
        return -1;
    }

    /* Read and validate header */
    savestate_header_t hdr;
    READ_OR_FAIL(f, &hdr, sizeof(hdr), "header");

    if (hdr.magic != SAVESTATE_MAGIC) {
        fprintf(stderr, "savestate_restore: invalid magic number (expected 0x%X, got 0x%X)\n",
                SAVESTATE_MAGIC, hdr.magic);
        fclose(f);
        return -1;
    }

    if (hdr.version != SAVESTATE_VERSION) {
        fprintf(stderr, "savestate_restore: version mismatch (expected %u, got %u)\n",
                SAVESTATE_VERSION, hdr.version);
        fclose(f);
        return -1;
    }

    /* Validate memory sizes */
    if (hdr.iram_size != SRAM_SIZE || hdr.dram_size != SRAM_SIZE ||
        hdr.flash_size != FLASH_SIZE || hdr.psram_size != PSRAM_SIZE) {
        fprintf(stderr, "savestate_restore: memory size mismatch\n");
        fprintf(stderr, "  Expected: SRAM=%u, FLASH=%u, PSRAM=%u\n",
                SRAM_SIZE, FLASH_SIZE, PSRAM_SIZE);
        fprintf(stderr, "  Got:      IRAM=%u, DRAM=%u, FLASH=%u, PSRAM=%u\n",
                hdr.iram_size, hdr.dram_size, hdr.flash_size, hdr.psram_size);
        fclose(f);
        return -1;
    }

    fprintf(stderr, "Restoring checkpoint: %s\n", hdr.description);
    fprintf(stderr, "  Saved at:     %llu (Unix timestamp)\n", (unsigned long long)hdr.timestamp);
    fprintf(stderr, "  Cycle count:  %llu\n", (unsigned long long)hdr.cycle_count);

    /* === Restore CPU State === */
    READ_OR_FAIL(f, cpu->ar, sizeof(cpu->ar), "AR registers");
    READ_OR_FAIL(f, &cpu->pc, sizeof(cpu->pc), "PC");
    READ_OR_FAIL(f, &cpu->sar, sizeof(cpu->sar), "SAR");
    READ_OR_FAIL(f, &cpu->lbeg, sizeof(cpu->lbeg), "LBEG");
    READ_OR_FAIL(f, &cpu->lend, sizeof(cpu->lend), "LEND");
    READ_OR_FAIL(f, &cpu->lcount, sizeof(cpu->lcount), "LCOUNT");
    READ_OR_FAIL(f, &cpu->windowbase, sizeof(cpu->windowbase), "WINDOWBASE");
    READ_OR_FAIL(f, &cpu->windowstart, sizeof(cpu->windowstart), "WINDOWSTART");
    READ_OR_FAIL(f, &cpu->ps, sizeof(cpu->ps), "PS");
    READ_OR_FAIL(f, &cpu->vecbase, sizeof(cpu->vecbase), "VECBASE");
    READ_OR_FAIL(f, &cpu->exccause, sizeof(cpu->exccause), "EXCCAUSE");
    READ_OR_FAIL(f, &cpu->excvaddr, sizeof(cpu->excvaddr), "EXCVADDR");
    READ_OR_FAIL(f, cpu->epc, sizeof(cpu->epc), "EPC");
    READ_OR_FAIL(f, cpu->eps, sizeof(cpu->eps), "EPS");
    READ_OR_FAIL(f, cpu->excsave, sizeof(cpu->excsave), "EXCSAVE");
    READ_OR_FAIL(f, &cpu->depc, sizeof(cpu->depc), "DEPC");
    READ_OR_FAIL(f, &cpu->ccount, sizeof(cpu->ccount), "CCOUNT");
    READ_OR_FAIL(f, cpu->ccompare, sizeof(cpu->ccompare), "CCOMPARE");
    READ_OR_FAIL(f, &cpu->next_timer_event, sizeof(cpu->next_timer_event), "NEXT_TIMER_EVENT");
    READ_OR_FAIL(f, &cpu->scompare1, sizeof(cpu->scompare1), "SCOMPARE1");
    READ_OR_FAIL(f, cpu->misc, sizeof(cpu->misc), "MISC");
    READ_OR_FAIL(f, &cpu->litbase, sizeof(cpu->litbase), "LITBASE");
    READ_OR_FAIL(f, &cpu->atomctl, sizeof(cpu->atomctl), "ATOMCTL");
    READ_OR_FAIL(f, &cpu->memctl, sizeof(cpu->memctl), "MEMCTL");
    READ_OR_FAIL(f, &cpu->icount, sizeof(cpu->icount), "ICOUNT");
    READ_OR_FAIL(f, &cpu->icountlevel, sizeof(cpu->icountlevel), "ICOUNTLEVEL");
    READ_OR_FAIL(f, &cpu->debugcause, sizeof(cpu->debugcause), "DEBUGCAUSE");
    READ_OR_FAIL(f, &cpu->ibreakenable, sizeof(cpu->ibreakenable), "IBREAKENABLE");
    READ_OR_FAIL(f, cpu->ibreaka, sizeof(cpu->ibreaka), "IBREAKA");
    READ_OR_FAIL(f, cpu->dbreaka, sizeof(cpu->dbreaka), "DBREAKA");
    READ_OR_FAIL(f, cpu->dbreakc, sizeof(cpu->dbreakc), "DBREAKC");
    READ_OR_FAIL(f, &cpu->configid0, sizeof(cpu->configid0), "CONFIGID0");
    READ_OR_FAIL(f, &cpu->configid1, sizeof(cpu->configid1), "CONFIGID1");
    READ_OR_FAIL(f, &cpu->prid, sizeof(cpu->prid), "PRID");
    READ_OR_FAIL(f, &cpu->core_id, sizeof(cpu->core_id), "CORE_ID");
    READ_OR_FAIL(f, &cpu->intenable, sizeof(cpu->intenable), "INTENABLE");
    READ_OR_FAIL(f, &cpu->interrupt, sizeof(cpu->interrupt), "INTERRUPT");
    READ_OR_FAIL(f, &cpu->cpenable, sizeof(cpu->cpenable), "CPENABLE");

    READ_OR_FAIL(f, &cpu->br, sizeof(cpu->br), "BR");
    READ_OR_FAIL(f, &cpu->acclo, sizeof(cpu->acclo), "ACCLO");
    READ_OR_FAIL(f, &cpu->acchi, sizeof(cpu->acchi), "ACCHI");
    READ_OR_FAIL(f, cpu->mr, sizeof(cpu->mr), "MR");

    READ_OR_FAIL(f, &cpu->fcr, sizeof(cpu->fcr), "FCR");
    READ_OR_FAIL(f, &cpu->fsr, sizeof(cpu->fsr), "FSR");
    READ_OR_FAIL(f, cpu->fr, sizeof(cpu->fr), "FR");

    READ_OR_FAIL(f, cpu->int_level, sizeof(cpu->int_level), "INT_LEVEL");

    READ_OR_FAIL(f, cpu->spill_stack, sizeof(cpu->spill_stack), "SPILL_STACK");
    READ_OR_FAIL(f, cpu->spill_base, sizeof(cpu->spill_base), "SPILL_BASE");
    READ_OR_FAIL(f, cpu->spill_shadow, sizeof(cpu->spill_shadow), "SPILL_SHADOW");
    READ_OR_FAIL(f, &cpu->spill_verify, sizeof(cpu->spill_verify), "SPILL_VERIFY");

    READ_OR_FAIL(f, &cpu->running, sizeof(cpu->running), "RUNNING");
    READ_OR_FAIL(f, &cpu->exception, sizeof(cpu->exception), "EXCEPTION");
    READ_OR_FAIL(f, &cpu->debug_break, sizeof(cpu->debug_break), "DEBUG_BREAK");
    READ_OR_FAIL(f, &cpu->halted, sizeof(cpu->halted), "HALTED");
    READ_OR_FAIL(f, &cpu->cycle_count, sizeof(cpu->cycle_count), "CYCLE_COUNT");
    READ_OR_FAIL(f, &cpu->virtual_time_us, sizeof(cpu->virtual_time_us), "VIRTUAL_TIME_US");

    /* === Restore Memory Regions === */
    if (!cpu->mem) {
        fprintf(stderr, "savestate_restore: memory subsystem is NULL\n");
        fclose(f);
        return -1;
    }

    READ_OR_FAIL(f, cpu->mem->sram, SRAM_SIZE, "SRAM");
    READ_OR_FAIL(f, cpu->mem->rom, ROM_SIZE, "ROM");
    READ_OR_FAIL(f, cpu->mem->flash_data, FLASH_SIZE, "FLASH_DATA");
    READ_OR_FAIL(f, cpu->mem->flash_insn, FLASH_SIZE, "FLASH_INSN");
    READ_OR_FAIL(f, cpu->mem->psram, PSRAM_SIZE, "PSRAM");
    READ_OR_FAIL(f, cpu->mem->rtc_dram, RTC_DRAM_SIZE, "RTC_DRAM");
    READ_OR_FAIL(f, cpu->mem->rtc_fast, RTC_FAST_SIZE, "RTC_FAST");
    READ_OR_FAIL(f, cpu->mem->rtc_slow, RTC_SLOW_SIZE, "RTC_SLOW");

    /* === Restore FreeRTOS State === */
    uint32_t freertos_marker;
    READ_OR_FAIL(f, &freertos_marker, sizeof(freertos_marker), "FREERTOS_MARKER");

    if (freertos_marker == 0xFEEDC0DE) {
        /* FreeRTOS state present */
        if (!frt) {
            fprintf(stderr, "savestate_restore: checkpoint has FreeRTOS state but no frt provided\n");
            fclose(f);
            return -1;
        }

        if (freertos_stubs_restore_state(frt, f) != 0) {
            fprintf(stderr, "savestate_restore: failed to restore FreeRTOS state\n");
            fclose(f);
            return -1;
        }
    } else if (freertos_marker == 0) {
        /* No FreeRTOS state in checkpoint */
        if (frt) {
            fprintf(stderr, "savestate_restore: warning - checkpoint has no FreeRTOS state but frt provided\n");
        }
    } else {
        fprintf(stderr, "savestate_restore: invalid FreeRTOS marker (expected 0xFEEDC0DE or 0, got 0x%X)\n",
                freertos_marker);
        fclose(f);
        return -1;
    }

    fclose(f);
    fprintf(stderr, "Checkpoint restored successfully\n");
    return 0;
}

/*
 * List available checkpoints in directory.
 * (Placeholder implementation - just returns empty list for now)
 */
int savestate_list(const char *dir, char ***out_paths, int *out_count) {
    (void)dir;
    if (out_paths) *out_paths = NULL;
    if (out_count) *out_count = 0;
    /* TODO: Implement directory scanning for .sav files */
    return 0;
}
