/* The stock ESP-IDF 5.3 socket configuration has ten descriptors at 54..63.
 * Exercise the native VFS registration and real lwIP API ABI, not a firmware
 * profile or a Flexe-only symbol. */
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>

#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

void app_main(void)
{
    if (esp_netif_init() != ESP_OK) {
        puts("SOCKET_RANGE_FAIL netif_init");
        return;
    }

    int sockets[CONFIG_LWIP_MAX_SOCKETS];
    for (int i = 0; i < CONFIG_LWIP_MAX_SOCKETS; i++) {
        sockets[i] = lwip_socket(AF_INET, SOCK_DGRAM, 0);
        if (sockets[i] != LWIP_SOCKET_OFFSET + i) {
            printf("SOCKET_RANGE_FAIL slot=%d fd=%d expected=%d\n",
                   i, sockets[i], LWIP_SOCKET_OFFSET + i);
            return;
        }
    }

    int extra = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (extra != -1) {
        printf("SOCKET_RANGE_FAIL overflow=%d\n", extra);
        return;
    }

    fd_set writable;
    FD_ZERO(&writable);
    FD_SET(sockets[0], &writable);
    struct timeval timeout = {0, 0};
    int ready = lwip_select(sockets[0] + 1, NULL, &writable, NULL, &timeout);
    if (ready != 1 || !FD_ISSET(sockets[0], &writable)) {
        printf("SOCKET_RANGE_FAIL select=%d\n", ready);
        return;
    }
    printf("SOCKET_RANGE_OK base=%d last=%d count=%d select=%d\n",
           sockets[0], sockets[CONFIG_LWIP_MAX_SOCKETS - 1],
           CONFIG_LWIP_MAX_SOCKETS, ready);

    for (int i = 0; i < CONFIG_LWIP_MAX_SOCKETS; i++)
        lwip_close(sockets[i]);
    for (unsigned i = 1; i <= 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("SOCKET_RANGE_ALIVE %u\n", i);
    }
}
