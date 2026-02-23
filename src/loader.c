#include "loader.h"
#include <stdio.h>
#include <string.h>

load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path) {
    load_result_t res = {0};

    if (!path) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "NULL path");
        return res;
    }

    if (!mem) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "NULL memory");
        return res;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Cannot open file: %s", path);
        return res;
    }

    /* Read magic byte */
    uint8_t magic;
    if (fread(&magic, 1, 1, f) != 1 || magic != 0xE9) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Bad magic: 0x%02X (expected 0xE9)", magic);
        fclose(f);
        return res;
    }

    /* Stub: full parsing deferred to M3 */
    res.entry_point = 0;
    res.segment_count = 0;
    res.result = 0;

    fclose(f);
    return res;
}
