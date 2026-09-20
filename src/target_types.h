/* Stable target types shared by CPU and memory hot-path interfaces. */
#ifndef FLEXE_TARGET_TYPES_H
#define FLEXE_TARGET_TYPES_H

/* Fixed-size values used by public snapshots and attachment tables belong in
 * this stable boundary header. Descriptor-only limits remain in target.h. */
#define FLEXE_TARGET_GP_SPI_HOST_MAX 2u
#define FLEXE_TARGET_RTC_STORE_MAX 8u
#define FLEXE_TARGET_RTC_IO_PIN_MAX 22u

/* The complete, versioned descriptor lives in target.h. Most execution
 * headers only retain a pointer and should not be invalidated whenever a
 * peripheral descriptor grows. */
typedef struct flexe_target_desc flexe_target_desc_t;

typedef enum {
    FLEXE_TARGET_AUTO = 0,
    FLEXE_TARGET_ESP32,
    FLEXE_TARGET_ESP32S3,
} flexe_target_id_t;

typedef enum {
    FLEXE_SYSTEM_DEVICE_NONE = 0,
    FLEXE_SYSTEM_DEVICE_SYSTIMER,
    FLEXE_SYSTEM_DEVICE_TIMER_GROUP,
    FLEXE_SYSTEM_DEVICE_I2C,
    FLEXE_SYSTEM_DEVICE_GP_SPI,
    FLEXE_SYSTEM_DEVICE_SHA,
    FLEXE_SYSTEM_DEVICE_LEDC,
    FLEXE_SYSTEM_DEVICE_UART,
    FLEXE_SYSTEM_DEVICE_USB_SERIAL_JTAG,
    FLEXE_SYSTEM_DEVICE_EDMA,
    FLEXE_SYSTEM_DEVICE_I2S,
} flexe_system_device_t;

/* Optional board population; zero retains the target's standard board. The
 * AP profile represents a physical 64-Mbit APS6408L-3OBMx on S3 CS1. */
typedef enum {
    FLEXE_BOARD_PSRAM_DEFAULT = 0,
    FLEXE_BOARD_PSRAM_AP_8M_OPI,
} flexe_board_psram_t;

/* Host allocations used by the target memory map. This small enum is part of
 * memory's stable interface; target.h describes which guest regions use it. */
typedef enum {
    FLEXE_MEM_SRAM = 0,
    FLEXE_MEM_ROM,
    FLEXE_MEM_FLASH_DATA,
    FLEXE_MEM_FLASH_INSN,
    FLEXE_MEM_RTC_FAST,
    FLEXE_MEM_RTC_SLOW,
    FLEXE_MEM_PSRAM,
    FLEXE_MEM_BACKING_COUNT,
} flexe_mem_backing_t;

#endif /* FLEXE_TARGET_TYPES_H */
