#include "flash_mmu.h"
#include "xtensa.h"

#include <stdlib.h>

#define HOST_PAGE_SIZE 4096u

struct flexe_flash_mmu {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    xtensa_cpu_t *cpu[2];
    uint32_t *entry;
};

static bool flexe_flash_mmu_geometry_valid(
        const flexe_target_desc_t *target) {
    if (!target) return false;
    const flexe_flash_mmu_desc_t *desc = &target->flash_mmu;
    uint64_t window_size = (uint64_t)desc->entry_count * desc->page_size;
    return desc->shared_instruction_data && desc->table_base[0] != 0 &&
           desc->table_base[1] == 0 && desc->entry_count != 0 &&
           desc->page_size >= HOST_PAGE_SIZE &&
           (desc->page_size & (desc->page_size - 1u)) == 0 &&
           (desc->page_size & (HOST_PAGE_SIZE - 1u)) == 0 &&
           target->drom_end > target->drom_start &&
           target->irom_end > target->irom_start &&
           window_size == target->drom_end - target->drom_start &&
           window_size == target->irom_end - target->irom_start &&
           desc->table_base[0] >= target->peripheral_start &&
           desc->table_base[0] < target->peripheral_end &&
           (uint64_t)desc->table_base[0] + desc->entry_count * 4u <=
               target->peripheral_end;
}

static void flexe_flash_mmu_invalidate_code(flexe_flash_mmu_t *mmu,
                                            uint32_t addr,
                                            uint32_t size) {
    xtensa_cpu_t *cpu0 = mmu->cpu[0];
    xtensa_cpu_t *cpu1 = mmu->cpu[1];
    if (cpu0) xtensa_invalidate_code(cpu0, addr, size);
    if (!cpu1) return;

    /* Sessions share their predecode table and JIT instance. Embedders may
     * attach independent engines, in which case both require invalidation. */
    if (cpu0 && cpu1->predecode == cpu0->predecode &&
        cpu1->code_invalidate == cpu0->code_invalidate &&
        cpu1->code_invalidate_ctx == cpu0->code_invalidate_ctx)
        return;
    xtensa_invalidate_code(cpu1, addr, size);
}

static void flexe_flash_mmu_apply(flexe_flash_mmu_t *mmu, uint32_t index) {
    const flexe_target_desc_t *target = mmu->target;
    const flexe_flash_mmu_desc_t *desc = &target->flash_mmu;
    uint32_t value = mmu->entry[index];
    uint32_t physical = (value & desc->physical_page_mask) * desc->page_size;
    uint32_t drom = target->drom_start + index * desc->page_size;
    uint32_t irom = target->irom_start + index * desc->page_size;
    bool invalid = (value & desc->invalid_mask) != 0;
    bool psram = desc->target_mask != 0 &&
                 (value & desc->target_mask) != 0;

    flexe_mem_backing_t data_backing = psram ? FLEXE_MEM_PSRAM :
                                               FLEXE_MEM_FLASH_DATA;
    flexe_mem_backing_t insn_backing = psram ? FLEXE_MEM_PSRAM :
                                               FLEXE_MEM_FLASH_INSN;
    uint32_t data_size = mem_backing_size(mmu->mem, data_backing);
    uint32_t insn_size = mem_backing_size(mmu->mem, insn_backing);
    bool mapped = !invalid && physical <= data_size &&
                  desc->page_size <= data_size - physical &&
                  physical <= insn_size &&
                  desc->page_size <= insn_size - physical;

    if (mapped) {
        if (mem_map_backing_range(mmu->mem, drom, data_backing, physical,
                                  desc->page_size) != 0 ||
            mem_map_backing_range(mmu->mem, irom, insn_backing, physical,
                                  desc->page_size) != 0)
            mapped = false;
    }
    if (!mapped) {
        (void)mem_unmap_range(mmu->mem, drom, desc->page_size);
        (void)mem_unmap_range(mmu->mem, irom, desc->page_size);
    }
    flexe_flash_mmu_invalidate_code(mmu, irom, desc->page_size);
}

static uint32_t flexe_flash_mmu_read(void *ctx, uint32_t addr) {
    flexe_flash_mmu_t *mmu = ctx;
    const flexe_flash_mmu_desc_t *desc = &mmu->target->flash_mmu;
    uint32_t offset = addr - desc->table_base[0];
    if ((offset & 3u) != 0 || offset / 4u >= desc->entry_count)
        return 0;
    return mmu->entry[offset / 4u];
}

static void flexe_flash_mmu_write(void *ctx, uint32_t addr, uint32_t value) {
    flexe_flash_mmu_t *mmu = ctx;
    const flexe_flash_mmu_desc_t *desc = &mmu->target->flash_mmu;
    uint32_t offset = addr - desc->table_base[0];
    if ((offset & 3u) != 0 || offset / 4u >= desc->entry_count)
        return;

    uint32_t index = offset / 4u;
    uint32_t value_mask = desc->physical_page_mask | desc->invalid_mask |
                          desc->target_mask;
    value &= value_mask;
    if (mmu->entry[index] == value) return;
    mmu->entry[index] = value;
    flexe_flash_mmu_apply(mmu, index);
}

flexe_flash_mmu_t *flexe_flash_mmu_create(xtensa_mem_t *mem) {
    const flexe_target_desc_t *target = mem_target(mem);
    if (!mem || !flexe_flash_mmu_geometry_valid(target)) return NULL;

    flexe_flash_mmu_t *mmu = calloc(1, sizeof(*mmu));
    if (!mmu) return NULL;
    mmu->entry = malloc((size_t)target->flash_mmu.entry_count *
                        sizeof(*mmu->entry));
    if (!mmu->entry) {
        free(mmu);
        return NULL;
    }
    mmu->mem = mem;
    mmu->target = target;

    for (uint32_t i = 0; i < target->flash_mmu.entry_count; i++)
        mmu->entry[i] = target->flash_mmu.invalid_entry;
    if (mem_unmap_range(mem, target->drom_start,
                        target->drom_end - target->drom_start) != 0 ||
        mem_unmap_range(mem, target->irom_start,
                        target->irom_end - target->irom_start) != 0 ||
        mem_register_mmio_range(
            mem, target->flash_mmu.table_base[0],
            (uint32_t)target->flash_mmu.entry_count * 4u,
            flexe_flash_mmu_read, flexe_flash_mmu_write, mmu) != 0) {
        free(mmu->entry);
        free(mmu);
        return NULL;
    }
    return mmu;
}

void flexe_flash_mmu_destroy(flexe_flash_mmu_t *mmu) {
    if (!mmu) return;
    const flexe_flash_mmu_desc_t *desc = &mmu->target->flash_mmu;
    (void)mem_register_mmio_range(mmu->mem, desc->table_base[0],
                                  (uint32_t)desc->entry_count * 4u,
                                  NULL, NULL, NULL);
    free(mmu->entry);
    free(mmu);
}

void flexe_flash_mmu_attach_cpus(flexe_flash_mmu_t *mmu,
                                 xtensa_cpu_t *cpu0,
                                 xtensa_cpu_t *cpu1) {
    if (!mmu) return;
    mmu->cpu[0] = cpu0;
    mmu->cpu[1] = cpu1;
}
