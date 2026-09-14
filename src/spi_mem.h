/* Target-described ESP32-family SPI memory controllers and attached NOR flash. */
#ifndef FLEXE_SPI_MEM_H
#define FLEXE_SPI_MEM_H

#include <stdbool.h>
#include "memory.h"

typedef struct flexe_spi_mem flexe_spi_mem_t;

/* The NOR is an external chip, separate from the SoC's SPI controllers.
 * A warm SoC reset leaves its command state intact; deep sleep removes flash
 * power and retains only profiled nonvolatile status. */
typedef struct {
    uint8_t status[3];
    bool reset_armed;
    bool powered_down;
    bool address_4byte;
} flexe_spi_mem_nor_state_t;

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
void flexe_spi_mem_nor_snapshot(const flexe_spi_mem_t *spi_mem,
                                flexe_spi_mem_nor_state_t *out);
/* power_cycle=false for an SoC-only restart, true after deep sleep. */
void flexe_spi_mem_nor_restore(flexe_spi_mem_t *spi_mem,
                               const flexe_spi_mem_nor_state_t *state,
                               bool power_cycle);

#endif /* FLEXE_SPI_MEM_H */
