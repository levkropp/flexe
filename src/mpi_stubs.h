/*
 * mpi_stubs.h — ESP32 MPI (RSA) hardware accelerator stubs
 *
 * Software big-number multiplication and Montgomery multiplication
 * replacing the ESP32 RSA peripheral for TLS certificate verification.
 */

#ifndef MPI_STUBS_H
#define MPI_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct mpi_stubs mpi_stubs_t;

mpi_stubs_t *mpi_stubs_create(xtensa_cpu_t *cpu);
void         mpi_stubs_destroy(mpi_stubs_t *ms);
int          mpi_stubs_hook_symbols(mpi_stubs_t *ms, const elf_symbols_t *syms);

#endif
