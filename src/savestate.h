#ifndef SAVESTATE_H
#define SAVESTATE_H

#include <stdint.h>

/* Forward declarations */
typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct freertos_stubs freertos_stubs_t;

/* File format version (increment when structure changes) */
#define SAVESTATE_VERSION 1

/* Savestate file magic (ASCII: "XTST") */
#define SAVESTATE_MAGIC 0x54535458

/* Savestate header structure */
typedef struct {
    uint32_t magic;           /* SAVESTATE_MAGIC */
    uint32_t version;         /* SAVESTATE_VERSION */
    uint64_t cycle_count;     /* Virtual cycles at checkpoint */
    uint64_t real_cycles;     /* Real cycles (includes delay stubs) - same as cycle_count for now */
    uint64_t timestamp;       /* Unix timestamp of save */
    char description[256];    /* Human-readable checkpoint name */

    /* Memory region sizes (for validation and allocation) */
    uint32_t iram_size;       /* IRAM size in bytes */
    uint32_t dram_size;       /* DRAM size in bytes */
    uint32_t flash_size;      /* Flash data size in bytes */
    uint32_t psram_size;      /* PSRAM size in bytes */

    /* Compression/optimization flags (for future use) */
    uint8_t compressed;       /* 0=raw, 1=zlib (reserved) */
    uint8_t differential;     /* 0=full, 1=diff from previous (reserved) */
    uint8_t reserved[6];      /* Reserved for future use */
} savestate_header_t;

/*
 * Save complete emulator state to file.
 *
 * Args:
 *   cpu: CPU state to save
 *   frt: FreeRTOS stubs (can be NULL if not using FreeRTOS)
 *   path: Output file path
 *   description: Human-readable checkpoint description (max 255 chars)
 *
 * Returns:
 *   0 on success, -1 on error (check errno or stderr for details)
 *
 * File format:
 *   1. savestate_header_t
 *   2. CPU state (registers, special registers, window state)
 *   3. Memory regions (SRAM, ROM, Flash, PSRAM)
 *   4. FreeRTOS state (tasks, queues, scheduler)
 *   5. Peripheral state (future - not yet implemented)
 */
int savestate_save(xtensa_cpu_t *cpu, freertos_stubs_t *frt, const char *path, const char *description);

/*
 * Restore emulator state from checkpoint file.
 *
 * Args:
 *   cpu: CPU state to restore into (must be pre-allocated)
 *   frt: FreeRTOS stubs (can be NULL if not using FreeRTOS)
 *   path: Input checkpoint file path
 *
 * Returns:
 *   0 on success, -1 on error
 *
 * Note: This function validates the magic number, version, and checksums.
 *       On error, the CPU state is left in an undefined state.
 */
int savestate_restore(xtensa_cpu_t *cpu, freertos_stubs_t *frt, const char *path);

/*
 * List available checkpoints in directory.
 *
 * Args:
 *   dir: Directory to search for .sav files
 *   out_paths: Output array of checkpoint file paths (caller must free)
 *   out_count: Number of checkpoints found
 *
 * Returns:
 *   0 on success, -1 on error
 *
 * Note: Caller must free out_paths array and each string in it.
 */
int savestate_list(const char *dir, char ***out_paths, int *out_count);

#endif /* SAVESTATE_H */
