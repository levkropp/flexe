#ifndef ROM_ELF_H
#define ROM_ELF_H

#include "memory.h"

#include <stdint.h>

typedef struct {
    int      result;              /* 0 = success */
    unsigned sections_loaded;     /* Immutable code/rodata sections */
    uint32_t bytes_loaded;
    unsigned data_images_loaded;  /* Writable ROM startup images installed */
    uint32_t data_image_bytes;
    unsigned interface_sections_loaded; /* ROM ABI tables installed in SRAM */
    uint32_t interface_bytes_loaded;
    char     error[256];
} rom_elf_load_result_t;

/* Load an Espressif Xtensa ROM ELF into the target's on-chip ROM apertures.
 *
 * Executable and read-only allocated sections are copied to their guest
 * addresses. Writable .data_* bytes are copied both to the ROM load-image
 * addresses encoded by the ROM linker's startup descriptors and to their
 * live SRAM addresses when the target selects direct ROM-data initialization.
 * That capability is used when Flexe hands control directly to an application
 * after the real ROM startup copy would have run. Targets whose established
 * bootstrap model initializes ROM state elsewhere retain their existing live
 * SRAM contents. Accepted addresses come from mem's versioned target
 * descriptor, not from hard-coded ESP32 ranges. */
rom_elf_load_result_t rom_elf_load(xtensa_mem_t *mem, const char *path);

#endif /* ROM_ELF_H */
