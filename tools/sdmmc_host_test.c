/* Run Espressif's compiled SDMMC host driver against Flexe's target-described
 * controller model. The guest owns command sequencing, ISR delivery, queues,
 * GPIO routing, and DesignWare IDMAC descriptors; the host owns only media. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x53444D4Du
#define CLASSIC_MAX_CYCLES UINT64_C(180000000)
#define S3_MAX_CYCLES UINT64_C(3000000000)
#define RESULT_COUNT   24u
#define CARD_SECTORS   128u
#define SECTOR_BYTES   512u
#define MULTI_BYTES    (20u * 1024u)

volatile int emu_app_running = 1;

typedef struct {
    uint8_t data[CARD_SECTORS][SECTOR_BYTES];
    size_t read_calls;
    size_t read_blocks;
    size_t write_calls;
    size_t write_blocks;
} card_image_t;

static uint8_t card_pattern(uint32_t sector, size_t offset) {
    return (uint8_t)(0x61u ^ (sector * 7u + offset));
}

static int card_read(void *opaque, uint32_t first, uint8_t *data,
                     size_t count) {
    card_image_t *card = opaque;
    if (!card || !data || first > CARD_SECTORS ||
        count > CARD_SECTORS - first)
        return -1;
    memcpy(data, card->data[first], count * SECTOR_BYTES);
    card->read_calls++;
    card->read_blocks += count;
    return 0;
}

static int card_write(void *opaque, uint32_t first, const uint8_t *data,
                      size_t count) {
    card_image_t *card = opaque;
    if (!card || !data || first > CARD_SECTORS ||
        count > CARD_SECTORS - first)
        return -1;
    memcpy(card->data[first], data, count * SECTOR_BYTES);
    card->write_calls++;
    card->write_blocks += count;
    return 0;
}

static bool writes_match(const card_image_t *card) {
    for (size_t index = 0; index < SECTOR_BYTES; ++index) {
        if (card->data[5][index] != (uint8_t)(0xC3u ^ index))
            return false;
    }
    for (size_t index = 0; index < MULTI_BYTES; ++index) {
        size_t sector = 64u + index / SECTOR_BYTES;
        size_t offset = index % SECTOR_BYTES;
        if (card->data[sector][offset] !=
            (uint8_t)(0x96u ^ (index * 3u)))
            return false;
    }
    return true;
}

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    int positional = argc - argi;
    if (positional != 2 && positional != 3) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf "
                "[ROM.elf]\n",
                argv[0]);
        return 2;
    }
    bool s3 = positional == 3;
    flexe_target_id_t target_id = s3 ? FLEXE_TARGET_ESP32S3 :
                                      FLEXE_TARGET_ESP32;
    const flexe_target_desc_t *target = flexe_target_by_id(target_id);

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0;
    uint32_t result_addr = 0;
    uint32_t descriptor_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_sdmmc_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_sdmmc_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }
    (void)elf_symbols_find(symbols, "s_dma_desc", &descriptor_addr);

    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = s3 ? argv[argi + 2] : NULL,
        .native_freertos = s3,
        .disable_jit = disable_jit,
        .target = target_id,
        .unhandled_audit = s3,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    static card_image_t card;
    for (uint32_t sector = 0; sector < CARD_SECTORS; ++sector) {
        for (size_t offset = 0; offset < SECTOR_BYTES; ++offset)
            card.data[sector][offset] = card_pattern(sector, offset);
    }
    esp32_periph_t *periph = flexe_session_periph(session);
    if (periph_sdmmc_attach_card(periph, 1, 4096u,
                                 card_read, card_write, &card) != 0) {
        fprintf(stderr, "error: could not attach virtual SDHC media\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint64_t max_cycles = s3 ? S3_MAX_CYCLES : CLASSIC_MAX_CYCLES;
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    while (cpu->cycle_count < max_cycles) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[sdmmc-fixture] engine=%s stage=0x%08X cycles=%llu "
                    "pc=0x%08X\n",
                    flexe_session_jit(session) ? "jit" : "interp", stage,
                    (unsigned long long)cpu->cycle_count, cpu->pc);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u ||
            !cpu->running || (cpu1 && cpu1->debug_break))
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t result[RESULT_COUNT];
    for (unsigned index = 0; index < RESULT_COUNT; ++index)
        result[index] = mem_read32(mem, result_addr + index * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));
    bool media_ok = writes_match(&card);
    int clock_route = s3 ? periph_gpio_out_signal(periph, 14) : -1;
    int command_route = s3 ? periph_gpio_out_signal(periph, 15) : -1;
    int data_route = s3 ? periph_gpio_out_signal(periph, 2) : -1;
    uint64_t jit_instructions = 0u;
    jit_state_t *jit = flexe_session_jit(session);
    if (jit) jit_instructions = jit_get_stats(jit)->insns_jitted;

    printf("target=%s engine=%s stage=0x%08X result=",
           target->name,
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned index = 0; index < RESULT_COUNT; ++index)
        printf("%s0x%08X", index ? "/" : "", result[index]);
    printf(" reads=%zu/%zu writes=%zu/%zu media=%d "
           "routes=%d,%d,%d unhandled=%d unregistered=%d "
           "cycles=%llu jit_insns=%llu\n",
           card.read_calls, card.read_blocks,
           card.write_calls, card.write_blocks, media_ok,
           clock_route, command_route, data_route,
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count,
           (unsigned long long)jit_instructions);

    if (stage != SUCCESS_MARKER) {
        const uint32_t base = target->sdmmc_host.base;
        fprintf(stderr,
                "[sdmmc-fixture] ctrl=%08X mask=%08X mint=%08X "
                "raw=%08X status=%08X idsts=%08X idinten=%08X "
                "bmod=%08X dbaddr=%08X dscaddr=%08X bufaddr=%08X\n",
                mem_read32(mem, base + 0x00u),
                mem_read32(mem, base + 0x24u),
                mem_read32(mem, base + 0x40u),
                mem_read32(mem, base + 0x44u),
                mem_read32(mem, base + 0x48u),
                mem_read32(mem, base + 0x8Cu),
                mem_read32(mem, base + 0x90u),
                mem_read32(mem, base + 0x80u),
                mem_read32(mem, base + 0x88u),
                mem_read32(mem, base + 0x94u),
                mem_read32(mem, base + 0xA0u));
        for (unsigned desc = 0; descriptor_addr != 0u && desc < 4u; ++desc) {
            uint32_t addr = descriptor_addr + desc * 16u;
            fprintf(stderr,
                    "[sdmmc-fixture] desc%u@%08X=%08X/%08X/%08X/%08X\n",
                    desc, addr, mem_read32(mem, addr),
                    mem_read32(mem, addr + 4u),
                    mem_read32(mem, addr + 8u),
                    mem_read32(mem, addr + 12u));
        }
    }

    bool results_ok = true;
    for (unsigned index = 0; index < RESULT_COUNT; ++index) {
        if (index == 5u || index == 8u || index == 10u || index == 12u ||
            index == 14u || index == 17u || index == 20u)
            continue;
        if (result[index] != 0u)
            results_ok = false;
    }
    int ok = stage == SUCCESS_MARKER && results_ok &&
             result[5] == 0x1AAu &&
             (result[8] & 0xC0000000u) == 0xC0000000u &&
             result[10] != 0u && result[12] != 0u && result[14] == 1u &&
             result[17] == 1u && result[20] == 1u &&
             card.read_calls == 2u && card.read_blocks == 41u &&
             card.write_calls == 2u && card.write_blocks == 41u &&
             media_ok && unhandled == 0 && unregistered == 0 &&
             (!s3 || (clock_route == 173 && command_route == 179 &&
                      data_route == 213)) &&
             (disable_jit || jit_instructions != 0u);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
