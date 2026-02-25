#include "display_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>

/* Display dimensions */
#define DISP_W 320
#define DISP_H 240

/* Embedded 8x16 VGA font (printable ASCII 32-126) */
#define FONT_W      8
#define FONT_H      16
#define FONT_FIRST  32
#define FONT_LAST   126

static const uint8_t font_8x16[95][16] = {
    /* 32 ' ' */ { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 33 '!' */ { 0x00,0x00,0x18,0x3C,0x3C,0x3C,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00 },
    /* 34 '"' */ { 0x00,0x66,0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 35 '#' */ { 0x00,0x00,0x00,0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 36 '$' */ { 0x18,0x18,0x7C,0xC6,0xC2,0xC0,0x7C,0x06,0x06,0x86,0xC6,0x7C,0x18,0x18,0x00,0x00 },
    /* 37 '%' */ { 0x00,0x00,0x00,0x00,0xC2,0xC6,0x0C,0x18,0x30,0x60,0xC6,0x86,0x00,0x00,0x00,0x00 },
    /* 38 '&' */ { 0x00,0x00,0x38,0x6C,0x6C,0x38,0x76,0xDC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00 },
    /* 39 ''' */ { 0x00,0x30,0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 40 '(' */ { 0x00,0x00,0x0C,0x18,0x30,0x30,0x30,0x30,0x30,0x30,0x18,0x0C,0x00,0x00,0x00,0x00 },
    /* 41 ')' */ { 0x00,0x00,0x30,0x18,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x18,0x30,0x00,0x00,0x00,0x00 },
    /* 42 '*' */ { 0x00,0x00,0x00,0x00,0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 43 '+' */ { 0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 44 ',' */ { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x30,0x00,0x00,0x00 },
    /* 45 '-' */ { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 46 '.' */ { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00 },
    /* 47 '/' */ { 0x00,0x00,0x00,0x00,0x02,0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x00 },
    /* 48 '0' */ { 0x00,0x00,0x7C,0xC6,0xC6,0xCE,0xDE,0xF6,0xE6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 49 '1' */ { 0x00,0x00,0x18,0x38,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00 },
    /* 50 '2' */ { 0x00,0x00,0x7C,0xC6,0x06,0x0C,0x18,0x30,0x60,0xC0,0xC6,0xFE,0x00,0x00,0x00,0x00 },
    /* 51 '3' */ { 0x00,0x00,0x7C,0xC6,0x06,0x06,0x3C,0x06,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 52 '4' */ { 0x00,0x00,0x0C,0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C,0x0C,0x1E,0x00,0x00,0x00,0x00 },
    /* 53 '5' */ { 0x00,0x00,0xFE,0xC0,0xC0,0xC0,0xFC,0x06,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 54 '6' */ { 0x00,0x00,0x38,0x60,0xC0,0xC0,0xFC,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 55 '7' */ { 0x00,0x00,0xFE,0xC6,0x06,0x06,0x0C,0x18,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00 },
    /* 56 '8' */ { 0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7C,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 57 '9' */ { 0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7E,0x06,0x06,0x06,0x0C,0x78,0x00,0x00,0x00,0x00 },
    /* 58 ':' */ { 0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00 },
    /* 59 ';' */ { 0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00 },
    /* 60 '<' */ { 0x00,0x00,0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00 },
    /* 61 '=' */ { 0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 62 '>' */ { 0x00,0x00,0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00 },
    /* 63 '?' */ { 0x00,0x00,0x7C,0xC6,0xC6,0x0C,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00 },
    /* 64 '@' */ { 0x00,0x00,0x00,0x7C,0xC6,0xC6,0xDE,0xDE,0xDE,0xDC,0xC0,0x7C,0x00,0x00,0x00,0x00 },
    /* 65 'A' */ { 0x00,0x00,0x10,0x38,0x6C,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00 },
    /* 66 'B' */ { 0x00,0x00,0xFC,0x66,0x66,0x66,0x7C,0x66,0x66,0x66,0x66,0xFC,0x00,0x00,0x00,0x00 },
    /* 67 'C' */ { 0x00,0x00,0x3C,0x66,0xC2,0xC0,0xC0,0xC0,0xC0,0xC2,0x66,0x3C,0x00,0x00,0x00,0x00 },
    /* 68 'D' */ { 0x00,0x00,0xF8,0x6C,0x66,0x66,0x66,0x66,0x66,0x66,0x6C,0xF8,0x00,0x00,0x00,0x00 },
    /* 69 'E' */ { 0x00,0x00,0xFE,0x66,0x62,0x68,0x78,0x68,0x60,0x62,0x66,0xFE,0x00,0x00,0x00,0x00 },
    /* 70 'F' */ { 0x00,0x00,0xFE,0x66,0x62,0x68,0x78,0x68,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00 },
    /* 71 'G' */ { 0x00,0x00,0x3C,0x66,0xC2,0xC0,0xC0,0xDE,0xC6,0xC6,0x66,0x3A,0x00,0x00,0x00,0x00 },
    /* 72 'H' */ { 0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00 },
    /* 73 'I' */ { 0x00,0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00 },
    /* 74 'J' */ { 0x00,0x00,0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0xCC,0xCC,0xCC,0x78,0x00,0x00,0x00,0x00 },
    /* 75 'K' */ { 0x00,0x00,0xE6,0x66,0x66,0x6C,0x78,0x78,0x6C,0x66,0x66,0xE6,0x00,0x00,0x00,0x00 },
    /* 76 'L' */ { 0x00,0x00,0xF0,0x60,0x60,0x60,0x60,0x60,0x60,0x62,0x66,0xFE,0x00,0x00,0x00,0x00 },
    /* 77 'M' */ { 0x00,0x00,0xC6,0xEE,0xFE,0xFE,0xD6,0xC6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00 },
    /* 78 'N' */ { 0x00,0x00,0xC6,0xE6,0xF6,0xFE,0xDE,0xCE,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00 },
    /* 79 'O' */ { 0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 80 'P' */ { 0x00,0x00,0xFC,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00 },
    /* 81 'Q' */ { 0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x0C,0x0E,0x00,0x00 },
    /* 82 'R' */ { 0x00,0x00,0xFC,0x66,0x66,0x66,0x7C,0x6C,0x66,0x66,0x66,0xE6,0x00,0x00,0x00,0x00 },
    /* 83 'S' */ { 0x00,0x00,0x7C,0xC6,0xC6,0x60,0x38,0x0C,0x06,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 84 'T' */ { 0x00,0x00,0xFF,0xDB,0x99,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00 },
    /* 85 'U' */ { 0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /* 86 'V' */ { 0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00 },
    /* 87 'W' */ { 0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xD6,0xD6,0xD6,0xFE,0xEE,0x6C,0x00,0x00,0x00,0x00 },
    /* 88 'X' */ { 0x00,0x00,0xC6,0xC6,0x6C,0x7C,0x38,0x38,0x7C,0x6C,0xC6,0xC6,0x00,0x00,0x00,0x00 },
    /* 89 'Y' */ { 0x00,0x00,0xCC,0xCC,0xCC,0xCC,0x78,0x30,0x30,0x30,0x30,0x78,0x00,0x00,0x00,0x00 },
    /* 90 'Z' */ { 0x00,0x00,0xFE,0xC6,0x86,0x0C,0x18,0x30,0x60,0xC2,0xC6,0xFE,0x00,0x00,0x00,0x00 },
    /* 91 '[' */ { 0x00,0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00,0x00,0x00 },
    /* 92 '\' */ { 0x00,0x00,0x00,0x80,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x06,0x02,0x00,0x00,0x00,0x00 },
    /* 93 ']' */ { 0x00,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00,0x00,0x00 },
    /* 94 '^' */ { 0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 95 '_' */ { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00 },
    /* 96 '`' */ { 0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    /* 97 'a' */ { 0x00,0x00,0x00,0x00,0x00,0x78,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00 },
    /* 98 'b' */ { 0x00,0x00,0xE0,0x60,0x60,0x78,0x6C,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00 },
    /* 99 'c' */ { 0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0xC0,0xC0,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /*100 'd' */ { 0x00,0x00,0x1C,0x0C,0x0C,0x3C,0x6C,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00 },
    /*101 'e' */ { 0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0xFE,0xC0,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /*102 'f' */ { 0x00,0x00,0x38,0x6C,0x64,0x60,0xF0,0x60,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00 },
    /*103 'g' */ { 0x00,0x00,0x00,0x00,0x00,0x76,0xCC,0xCC,0xCC,0xCC,0xCC,0x7C,0x0C,0xCC,0x78,0x00 },
    /*104 'h' */ { 0x00,0x00,0xE0,0x60,0x60,0x6C,0x76,0x66,0x66,0x66,0x66,0xE6,0x00,0x00,0x00,0x00 },
    /*105 'i' */ { 0x00,0x00,0x18,0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00 },
    /*106 'j' */ { 0x00,0x00,0x06,0x06,0x00,0x0E,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x66,0x3C,0x00 },
    /*107 'k' */ { 0x00,0x00,0xE0,0x60,0x60,0x66,0x6C,0x78,0x78,0x6C,0x66,0xE6,0x00,0x00,0x00,0x00 },
    /*108 'l' */ { 0x00,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00 },
    /*109 'm' */ { 0x00,0x00,0x00,0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xD6,0xD6,0xC6,0x00,0x00,0x00,0x00 },
    /*110 'n' */ { 0x00,0x00,0x00,0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00 },
    /*111 'o' */ { 0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /*112 'p' */ { 0x00,0x00,0x00,0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00 },
    /*113 'q' */ { 0x00,0x00,0x00,0x00,0x00,0x76,0xCC,0xCC,0xCC,0xCC,0xCC,0x7C,0x0C,0x0C,0x1E,0x00 },
    /*114 'r' */ { 0x00,0x00,0x00,0x00,0x00,0xDC,0x76,0x66,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00 },
    /*115 's' */ { 0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0x60,0x38,0x0C,0xC6,0x7C,0x00,0x00,0x00,0x00 },
    /*116 't' */ { 0x00,0x00,0x10,0x30,0x30,0xFC,0x30,0x30,0x30,0x30,0x36,0x1C,0x00,0x00,0x00,0x00 },
    /*117 'u' */ { 0x00,0x00,0x00,0x00,0x00,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00 },
    /*118 'v' */ { 0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x00,0x00,0x00,0x00 },
    /*119 'w' */ { 0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xD6,0xD6,0xD6,0xFE,0x6C,0x00,0x00,0x00,0x00 },
    /*120 'x' */ { 0x00,0x00,0x00,0x00,0x00,0xC6,0x6C,0x38,0x38,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00 },
    /*121 'y' */ { 0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7E,0x06,0x0C,0xF8,0x00 },
    /*122 'z' */ { 0x00,0x00,0x00,0x00,0x00,0xFE,0xCC,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00 },
    /*123 '{' */ { 0x00,0x00,0x0E,0x18,0x18,0x18,0x70,0x18,0x18,0x18,0x18,0x0E,0x00,0x00,0x00,0x00 },
    /*124 '|' */ { 0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00 },
    /*125 '}' */ { 0x00,0x00,0x70,0x18,0x18,0x18,0x0E,0x18,0x18,0x18,0x18,0x70,0x00,0x00,0x00,0x00 },
    /*126 '~' */ { 0x00,0x00,0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
};

struct display_stubs {
    xtensa_cpu_t       *cpu;
    esp32_rom_stubs_t  *rom;
    uint16_t           *framebuf;
    pthread_mutex_t    *framebuf_mtx;
    int                 width;
    int                 height;
    /* TFT_eSPI state tracking */
    uint16_t            textcolor;
    uint16_t            textbgcolor;
    uint8_t             textsize;
    uint8_t             rotation;
};

/* ===== Calling convention helpers ===== */

static uint32_t ds_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    /* Max register args: min(6, 16 - first_arg_reg).
     * For CALL8 (ci=2): first_arg = a10, max = 6.
     * For CALL4 (ci=1): first_arg = a6,  max = 6 (really 10 but ABI caps at 6).
     * For CALL0 (ci=0): a2-a7 = 6. */
    int first_arg = ci * 4 + 2;
    int max_reg = 16 - first_arg;
    if (max_reg > 6) max_reg = 6;
    if (n < max_reg)
        return ar_read(cpu, first_arg + n);
    /* Extra args beyond register capacity are on the caller's stack.
     * Hook fires before ENTRY, so SP (a1) is still the caller's SP.
     * Stack args start at [SP + 0] for the first extra arg. */
    uint32_t sp = ar_read(cpu, 1);
    return mem_read32(cpu->mem, sp + (uint32_t)((n - max_reg) * 4));
}

static void ds_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

static void ds_return(xtensa_cpu_t *cpu, uint32_t val) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, val);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, val);
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== Display stub implementations ===== */

static void stub_display_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    ds_return_void(cpu);
}

static void stub_display_clear(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    uint16_t color = (uint16_t)ds_arg(cpu, 0);
    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        for (int i = 0; i < ds->width * ds->height; i++)
            ds->framebuf[i] = color;
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

static void stub_display_fill_rect(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 0);
    int y = (int32_t)ds_arg(cpu, 1);
    int w = (int32_t)ds_arg(cpu, 2);
    int h = (int32_t)ds_arg(cpu, 3);
    uint16_t color = (uint16_t)ds_arg(cpu, 4);

    /* Clip */
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > ds->width)  w = ds->width - x;
    if (y + h > ds->height) h = ds->height - y;
    if (w <= 0 || h <= 0) { ds_return_void(cpu); return; }

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        for (int row = y; row < y + h; row++) {
            uint16_t *dst = &ds->framebuf[row * ds->width + x];
            for (int i = 0; i < w; i++)
                dst[i] = color;
        }
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

static void stub_display_char(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 0);
    int y = (int32_t)ds_arg(cpu, 1);
    char c = (char)(ds_arg(cpu, 2) & 0xFF);
    uint16_t fg = (uint16_t)ds_arg(cpu, 3);
    uint16_t bg = (uint16_t)ds_arg(cpu, 4);

    if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
    const uint8_t *glyph = font_8x16[c - FONT_FIRST];

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        for (int row = 0; row < FONT_H; row++) {
            int dy = y + row;
            if (dy < 0 || dy >= ds->height) continue;
            if (x < 0 || x + FONT_W > ds->width) continue;
            uint8_t bits = glyph[row];
            uint16_t *dst = &ds->framebuf[dy * ds->width + x];
            for (int col = 0; col < FONT_W; col++)
                dst[col] = (bits & (0x80 >> col)) ? fg : bg;
        }
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

static void stub_display_string(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 0);
    int y = (int32_t)ds_arg(cpu, 1);
    uint32_t str_addr = ds_arg(cpu, 2);
    uint16_t fg = (uint16_t)ds_arg(cpu, 3);
    uint16_t bg = (uint16_t)ds_arg(cpu, 4);

    if (!ds->framebuf || !ds->framebuf_mtx) { ds_return_void(cpu); return; }

    pthread_mutex_lock(ds->framebuf_mtx);
    int cx = x, cy = y;
    for (;;) {
        uint8_t ch = mem_read8(cpu->mem, str_addr++);
        if (ch == 0) break;
        if (ch == '\n') {
            cx = x;
            cy += FONT_H;
            continue;
        }
        if (cx + FONT_W > ds->width) {
            cx = 0;
            cy += FONT_H;
        }
        if (cy + FONT_H > ds->height) break;

        char c = (char)ch;
        if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
        const uint8_t *glyph = font_8x16[c - FONT_FIRST];
        for (int row = 0; row < FONT_H; row++) {
            int dy = cy + row;
            if (dy < 0 || dy >= ds->height) continue;
            if (cx < 0 || cx + FONT_W > ds->width) continue;
            uint8_t bits = glyph[row];
            uint16_t *dst = &ds->framebuf[dy * ds->width + cx];
            for (int col = 0; col < FONT_W; col++)
                dst[col] = (bits & (0x80 >> col)) ? fg : bg;
        }
        cx += FONT_W;
    }
    pthread_mutex_unlock(ds->framebuf_mtx);
    ds_return_void(cpu);
}

static void stub_display_draw_bitmap1bpp(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 0);
    int y = (int32_t)ds_arg(cpu, 1);
    int w = (int32_t)ds_arg(cpu, 2);
    int h = (int32_t)ds_arg(cpu, 3);
    uint32_t bmp_addr = ds_arg(cpu, 4);
    uint16_t fg = (uint16_t)ds_arg(cpu, 5);
    uint16_t bg = (uint16_t)ds_arg(cpu, 6);

    int row_bytes = (w + 7) / 8;

    if (!ds->framebuf || !ds->framebuf_mtx) { ds_return_void(cpu); return; }

    pthread_mutex_lock(ds->framebuf_mtx);
    for (int row = 0; row < h; row++) {
        int dy = y + row;
        if (dy < 0 || dy >= ds->height) continue;
        for (int col = 0; col < w; col++) {
            int dx = x + col;
            if (dx < 0 || dx >= ds->width) continue;
            uint8_t byte = mem_read8(cpu->mem, bmp_addr + (uint32_t)(row * row_bytes + col / 8));
            int bit = byte & (0x80 >> (col & 7));
            ds->framebuf[dy * ds->width + dx] = bit ? fg : bg;
        }
    }
    pthread_mutex_unlock(ds->framebuf_mtx);
    ds_return_void(cpu);
}

static void stub_display_draw_rgb565_line(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 0);
    int y = (int32_t)ds_arg(cpu, 1);
    int w = (int32_t)ds_arg(cpu, 2);
    uint32_t pix_addr = ds_arg(cpu, 3);

    if (y < 0 || y >= ds->height || w <= 0) { ds_return_void(cpu); return; }
    int skip = 0;
    if (x < 0) { skip = -x; w += x; x = 0; }
    if (x + w > ds->width) w = ds->width - x;
    if (w <= 0) { ds_return_void(cpu); return; }

    if (!ds->framebuf || !ds->framebuf_mtx) { ds_return_void(cpu); return; }

    pthread_mutex_lock(ds->framebuf_mtx);
    uint16_t *dst = &ds->framebuf[y * ds->width + x];
    uint32_t src = pix_addr + (uint32_t)(skip * 2);
    for (int i = 0; i < w; i++) {
        /* Read 16-bit LE pixel from emulator memory */
        uint8_t lo = mem_read8(cpu->mem, src);
        uint8_t hi = mem_read8(cpu->mem, src + 1);
        dst[i] = (uint16_t)((hi << 8) | lo);
        src += 2;
    }
    pthread_mutex_unlock(ds->framebuf_mtx);
    ds_return_void(cpu);
}

/* ===== TFT_eSPI C++ method stubs =====
 *
 * For C++ member functions, arg0 = this pointer. Actual parameters start at
 * arg1.  We ignore 'this' and work directly with the emulator framebuffer. */

/* TFT_eSPI::fillScreen(unsigned int color) */
static void stub_tft_fillScreen(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    uint16_t color = (uint16_t)ds_arg(cpu, 1);  /* arg0=this, arg1=color */
    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        for (int i = 0; i < ds->width * ds->height; i++)
            ds->framebuf[i] = color;
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::fillRect(int x, int y, int w, int h, unsigned int color) */
static void stub_tft_fillRect(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    int w = (int32_t)ds_arg(cpu, 3);
    int h = (int32_t)ds_arg(cpu, 4);
    uint16_t color = (uint16_t)ds_arg(cpu, 5);

    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > ds->width)  w = ds->width - x;
    if (y + h > ds->height) h = ds->height - y;
    if (w <= 0 || h <= 0) { ds_return_void(cpu); return; }

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        for (int row = y; row < y + h; row++) {
            uint16_t *dst = &ds->framebuf[row * ds->width + x];
            for (int i = 0; i < w; i++)
                dst[i] = color;
        }
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::drawPixel(int x, int y, unsigned int color) */
static void stub_tft_drawPixel(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    uint16_t color = (uint16_t)ds_arg(cpu, 3);

    if (x >= 0 && x < ds->width && y >= 0 && y < ds->height &&
        ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        ds->framebuf[y * ds->width + x] = color;
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::drawFastHLine(int x, int y, int w, unsigned int color) */
static void stub_tft_drawFastHLine(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    int w = (int32_t)ds_arg(cpu, 3);
    uint16_t color = (uint16_t)ds_arg(cpu, 4);

    if (y < 0 || y >= ds->height || w <= 0) { ds_return_void(cpu); return; }
    if (x < 0) { w += x; x = 0; }
    if (x + w > ds->width) w = ds->width - x;
    if (w <= 0) { ds_return_void(cpu); return; }

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        uint16_t *dst = &ds->framebuf[y * ds->width + x];
        for (int i = 0; i < w; i++)
            dst[i] = color;
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::drawFastVLine(int x, int y, int h, unsigned int color) */
static void stub_tft_drawFastVLine(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    int h = (int32_t)ds_arg(cpu, 3);
    uint16_t color = (uint16_t)ds_arg(cpu, 4);

    if (x < 0 || x >= ds->width || h <= 0) { ds_return_void(cpu); return; }
    if (y < 0) { h += y; y = 0; }
    if (y + h > ds->height) h = ds->height - y;
    if (h <= 0) { ds_return_void(cpu); return; }

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        for (int row = y; row < y + h; row++)
            ds->framebuf[row * ds->width + x] = color;
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::drawRect(int x, int y, int w, int h, unsigned int color) */
static void stub_tft_drawRect(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    int w = (int32_t)ds_arg(cpu, 3);
    int h = (int32_t)ds_arg(cpu, 4);
    uint16_t color = (uint16_t)ds_arg(cpu, 5);

    if (w <= 0 || h <= 0 || !ds->framebuf || !ds->framebuf_mtx) {
        ds_return_void(cpu); return;
    }

    pthread_mutex_lock(ds->framebuf_mtx);
    /* Top and bottom edges */
    for (int i = 0; i < w; i++) {
        int px = x + i;
        if (px >= 0 && px < ds->width) {
            if (y >= 0 && y < ds->height)
                ds->framebuf[y * ds->width + px] = color;
            int by = y + h - 1;
            if (by >= 0 && by < ds->height)
                ds->framebuf[by * ds->width + px] = color;
        }
    }
    /* Left and right edges */
    for (int i = 1; i < h - 1; i++) {
        int py = y + i;
        if (py >= 0 && py < ds->height) {
            if (x >= 0 && x < ds->width)
                ds->framebuf[py * ds->width + x] = color;
            int rx = x + w - 1;
            if (rx >= 0 && rx < ds->width)
                ds->framebuf[py * ds->width + rx] = color;
        }
    }
    pthread_mutex_unlock(ds->framebuf_mtx);
    ds_return_void(cpu);
}

/* TFT_eSPI::pushImage(int x, int y, int w, int h, const uint16_t *data) */
static void stub_tft_pushImage(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    int w = (int32_t)ds_arg(cpu, 3);
    int h = (int32_t)ds_arg(cpu, 4);
    uint32_t data_addr = ds_arg(cpu, 5);

    if (w <= 0 || h <= 0 || !ds->framebuf || !ds->framebuf_mtx) {
        ds_return_void(cpu); return;
    }

    pthread_mutex_lock(ds->framebuf_mtx);
    for (int row = 0; row < h; row++) {
        int dy = y + row;
        if (dy < 0 || dy >= ds->height) continue;
        for (int col = 0; col < w; col++) {
            int dx = x + col;
            if (dx < 0 || dx >= ds->width) continue;
            uint32_t src = data_addr + (uint32_t)((row * w + col) * 2);
            uint8_t lo = mem_read8(cpu->mem, src);
            uint8_t hi = mem_read8(cpu->mem, src + 1);
            ds->framebuf[dy * ds->width + dx] = (uint16_t)((hi << 8) | lo);
        }
    }
    pthread_mutex_unlock(ds->framebuf_mtx);
    ds_return_void(cpu);
}

/* Render one glyph at (cx, cy) with scaling, returns glyph pixel width */
static int tft_render_glyph(display_stubs_t *ds, int cx, int cy,
                             char c, uint16_t fg, uint16_t bg, int scale) {
    if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
    const uint8_t *glyph = font_8x16[c - FONT_FIRST];
    int gw = FONT_W * scale;

    for (int row = 0; row < FONT_H; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < FONT_W; col++) {
            uint16_t pix = (bits & (0x80 >> col)) ? fg : bg;
            /* Scale pixel */
            for (int sy = 0; sy < scale; sy++) {
                int dy = cy + row * scale + sy;
                if (dy < 0 || dy >= ds->height) continue;
                for (int sx = 0; sx < scale; sx++) {
                    int dx = cx + col * scale + sx;
                    if (dx < 0 || dx >= ds->width) continue;
                    ds->framebuf[dy * ds->width + dx] = pix;
                }
            }
        }
    }
    return gw;
}

/* TFT_eSPI::drawString(const char *str, int x, int y, unsigned char font)
 * Returns int16_t pixel width of rendered string. */
static void stub_tft_drawString(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    uint32_t str_addr = ds_arg(cpu, 1);
    int x = (int32_t)ds_arg(cpu, 2);
    int y = (int32_t)ds_arg(cpu, 3);
    /* arg4 = font number, ignored — we use our embedded font */

    uint16_t fg = ds->textcolor;
    uint16_t bg = ds->textbgcolor;
    int scale = ds->textsize > 0 ? ds->textsize : 1;
    int gw = FONT_W * scale;
    int total_w = 0;

    if (!ds->framebuf || !ds->framebuf_mtx) {
        ds_return(cpu, 0);
        return;
    }

    pthread_mutex_lock(ds->framebuf_mtx);
    int cx = x;
    for (;;) {
        uint8_t ch = mem_read8(cpu->mem, str_addr++);
        if (ch == 0) break;
        if (ch == '\n') {
            cx = x;
            y += FONT_H * scale;
            continue;
        }
        tft_render_glyph(ds, cx, y, (char)ch, fg, bg, scale);
        cx += gw;
        total_w += gw;
    }
    pthread_mutex_unlock(ds->framebuf_mtx);
    ds_return(cpu, (uint32_t)total_w);
}

/* TFT_eSPI::drawChar(unsigned short c, int x, int y)
 * Uses current textcolor/textbgcolor/textsize. */
static void stub_tft_drawChar_3(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    char c = (char)(ds_arg(cpu, 1) & 0xFF);
    int x = (int32_t)ds_arg(cpu, 2);
    int y = (int32_t)ds_arg(cpu, 3);

    int scale = ds->textsize > 0 ? ds->textsize : 1;

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        tft_render_glyph(ds, x, y, c, ds->textcolor, ds->textbgcolor, scale);
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::drawChar(int x, int y, unsigned short c, unsigned int fg,
 *                     unsigned int bg, unsigned char size) */
static void stub_tft_drawChar_6(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    int x = (int32_t)ds_arg(cpu, 1);
    int y = (int32_t)ds_arg(cpu, 2);
    char c = (char)(ds_arg(cpu, 3) & 0xFF);
    uint16_t fg = (uint16_t)ds_arg(cpu, 4);
    uint16_t bg = (uint16_t)ds_arg(cpu, 5);
    /* arg6 = size, on stack for CALL8 */
    int ci = XT_PS_CALLINC(cpu->ps);
    int first_arg = ci * 4 + 2;
    int max_reg = 16 - first_arg;
    if (max_reg > 6) max_reg = 6;
    uint32_t size_val;
    if (6 < max_reg)
        size_val = ar_read(cpu, first_arg + 6);
    else {
        uint32_t sp = ar_read(cpu, 1);
        size_val = mem_read32(cpu->mem, sp + (uint32_t)((6 - max_reg) * 4));
    }
    int scale = size_val > 0 ? (int)size_val : 1;

    if (ds->framebuf && ds->framebuf_mtx) {
        pthread_mutex_lock(ds->framebuf_mtx);
        tft_render_glyph(ds, x, y, c, fg, bg, scale);
        pthread_mutex_unlock(ds->framebuf_mtx);
    }
    ds_return_void(cpu);
}

/* TFT_eSPI::setTextColor(unsigned short fg) */
static void stub_tft_setTextColor_1(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    ds->textcolor = (uint16_t)ds_arg(cpu, 1);
    ds_return_void(cpu);
}

/* TFT_eSPI::setTextColor(unsigned short fg, unsigned short bg, bool fillbg) */
static void stub_tft_setTextColor_3(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    ds->textcolor = (uint16_t)ds_arg(cpu, 1);
    ds->textbgcolor = (uint16_t)ds_arg(cpu, 2);
    ds_return_void(cpu);
}

/* TFT_eSPI::setTextSize(unsigned char size) */
static void stub_tft_setTextSize(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    ds->textsize = (uint8_t)ds_arg(cpu, 1);
    ds_return_void(cpu);
}

/* TFT_eSPI::setRotation(unsigned char r) */
static void stub_tft_setRotation(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    ds->rotation = (uint8_t)ds_arg(cpu, 1);
    ds_return_void(cpu);
}

/* Generic void return for TFT_eSPI methods we don't need to emulate */
static void stub_tft_void(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    ds_return_void(cpu);
}

/* TFT_eSPI::width() / height() — return display dimensions */
static void stub_tft_width(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    ds_return(cpu, (uint32_t)ds->width);
}

static void stub_tft_height(xtensa_cpu_t *cpu, void *ctx) {
    display_stubs_t *ds = ctx;
    ds_return(cpu, (uint32_t)ds->height);
}

/* ===== Public API ===== */

display_stubs_t *display_stubs_create(xtensa_cpu_t *cpu) {
    display_stubs_t *ds = calloc(1, sizeof(*ds));
    if (!ds) return NULL;
    ds->cpu = cpu;
    ds->width = DISP_W;
    ds->height = DISP_H;
    ds->textcolor = 0xFFFF;   /* white */
    ds->textbgcolor = 0x0000; /* black */
    ds->textsize = 1;
    return ds;
}

void display_stubs_destroy(display_stubs_t *ds) {
    free(ds);
}

void display_stubs_set_framebuf(display_stubs_t *ds, uint16_t *fb,
                                 pthread_mutex_t *mtx, int w, int h) {
    if (!ds) return;
    ds->framebuf = fb;
    ds->framebuf_mtx = mtx;
    ds->width = w;
    ds->height = h;
}

int display_stubs_hook_symbols(display_stubs_t *ds, const elf_symbols_t *syms) {
    if (!ds || !syms) return 0;

    esp32_rom_stubs_t *rom = ds->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ds->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "display_init",              stub_display_init },
        { "display_clear",             stub_display_clear },
        { "display_fill_rect",         stub_display_fill_rect },
        { "display_char",              stub_display_char },
        { "display_string",            stub_display_string },
        { "display_draw_bitmap1bpp",   stub_display_draw_bitmap1bpp },
        { "display_draw_rgb565_line",  stub_display_draw_rgb565_line },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, ds);
            hooked++;
        }
    }

    return hooked;
}

int display_stubs_hook_tft_espi(display_stubs_t *ds, const elf_symbols_t *syms) {
    if (!ds || !syms) return 0;

    esp32_rom_stubs_t *rom = ds->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ds->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        /* Core rendering */
        { "_ZN8TFT_eSPI10fillScreenEj",    stub_tft_fillScreen },
        { "_ZN8TFT_eSPI8fillRectEiiiij",   stub_tft_fillRect },
        { "_ZN8TFT_eSPI9drawPixelEiij",    stub_tft_drawPixel },
        { "_ZN8TFT_eSPI13drawFastHLineEiiij", stub_tft_drawFastHLine },
        { "_ZN8TFT_eSPI13drawFastVLineEiiij", stub_tft_drawFastVLine },
        { "_ZN8TFT_eSPI8drawRectEiiiij",   stub_tft_drawRect },
        /* Image blitting — all overloads use same blit logic */
        { "_ZN8TFT_eSPI9pushImageEiiiiPKt", stub_tft_pushImage },
        { "_ZN8TFT_eSPI9pushImageEiiiiPt",  stub_tft_pushImage },
        /* Text */
        { "_ZN8TFT_eSPI10drawStringEPKciih", stub_tft_drawString },
        { "_ZN8TFT_eSPI8drawCharEtii",     stub_tft_drawChar_3 },
        { "_ZN8TFT_eSPI8drawCharEiitjjh",  stub_tft_drawChar_6 },
        /* State tracking */
        { "_ZN8TFT_eSPI12setTextColorEt",   stub_tft_setTextColor_1 },
        { "_ZN8TFT_eSPI12setTextColorEttb", stub_tft_setTextColor_3 },
        { "_ZN8TFT_eSPI11setTextSizeEh",    stub_tft_setTextSize },
        { "_ZN8TFT_eSPI11setRotationEh",    stub_tft_setRotation },
        /* Dimension queries */
        { "_ZN8TFT_eSPI4widthEv",           stub_tft_width },
        { "_ZN8TFT_eSPI5widthEv",           stub_tft_width },
        { "_ZN8TFT_eSPI4heightEv",          stub_tft_height },
        { "_ZN8TFT_eSPI6heightEv",          stub_tft_height },
        /* No-op config methods */
        { "_ZN8TFT_eSPI4initEh",            stub_tft_void },
        { "_ZN8TFT_eSPI7initBusEv",         stub_tft_void },
        { "_ZN8TFT_eSPI11setTextFontEh",    stub_tft_void },
        { "_ZN8TFT_eSPI12setTextDatumEh",   stub_tft_void },
        { "_ZN8TFT_eSPI11setFreeFontEPK7GFXfont", stub_tft_void },
        { "_ZN8TFT_eSPI12setSwapBytesEb",   stub_tft_void },
        { "_ZN8TFT_eSPI11setViewportEiiiib", stub_tft_void },
        { "_ZN8TFT_eSPI13resetViewportEv",  stub_tft_void },
        { "_ZN8TFT_eSPI10startWriteEv",     stub_tft_void },
        { "_ZN8TFT_eSPI8endWriteEv",        stub_tft_void },
        { "_ZN8TFT_eSPI13invertDisplayEb",  stub_tft_void },
        { "_ZN8TFT_eSPI10unloadFontEv",     stub_tft_void },
        { "_ZN8TFT_eSPI7dmaWaitEv",         stub_tft_void },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, ds);
            hooked++;
        }
    }

    return hooked;
}
