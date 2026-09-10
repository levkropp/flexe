/* Target-described ESP32-family SPI memory controllers and attached NOR flash. */
#ifndef FLEXE_SPI_MEM_H
#define FLEXE_SPI_MEM_H

#include "memory.h"

typedef struct flexe_spi_mem flexe_spi_mem_t;

typedef void (*flexe_spi_mem_flash_changed_fn)(void *ctx,
                                                uint32_t offset,
                                                uint32_t size);

/* Register every controller instance described by mem's target. Reserved
 * offsets and unsupported commands are reported through the fallback hooks. */
flexe_spi_mem_t *flexe_spi_mem_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_spi_mem_flash_changed_fn flash_changed, void *flash_changed_ctx);
void flexe_spi_mem_destroy(flexe_spi_mem_t *spi_mem);

#endif /* FLEXE_SPI_MEM_H */
