#ifndef ROM_ELF_H
#define ROM_ELF_H

#include "memory.h"

#include <stdint.h>

typedef struct {
    int      result;              /* 0 = success */
    unsigned sections_loaded;     /* Immutable code/rodata sections */
    uint32_t bytes_loaded;
    unsigned data_images_loaded;  /* Writable initializers mirrored into ROM */
    uint32_t data_image_bytes;
    char     error[256];
} rom_elf_load_result_t;

/* Load an Espressif ESP32 ROM ELF into the on-chip ROM address spaces.
 *
 * Executable and read-only allocated sections are copied to their guest
 * addresses. Writable .data_* sections are not installed into live DRAM:
 * their bytes are copied to the ROM load-image addresses encoded by the ROM
 * linker's startup descriptors, just as they reside in silicon before the
 * boot code initializes DRAM. */
rom_elf_load_result_t rom_elf_load(xtensa_mem_t *mem, const char *path);

#endif /* ROM_ELF_H */
