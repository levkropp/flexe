#ifndef FLEXE_HOST_NET_SLIRP_H
#define FLEXE_HOST_NET_SLIRP_H

#include "wifi_stubs.h"

typedef struct flexe_host_net flexe_host_net_t;

/* An opt-in user-mode Ethernet network for native S3 Wi-Fi netifs. The spec
 * is "ap:HOST_PORT:GUEST_IP:GUEST_PORT" or "sta:..."; only loopback is
 * exposed on the host. Returns NULL with a diagnostic on invalid settings or
 * when libslirp was unavailable at build time. */
flexe_host_net_t *flexe_host_net_create(const char *spec, wifi_stubs_t *wifi);
/* Expose one more loopback listener on the same user-mode network. The
 * forward must target the network's own interface and /24. Returns 0 on
 * success, -1 with a diagnostic otherwise. */
int flexe_host_net_add_forward(flexe_host_net_t *net, const char *spec);
void flexe_host_net_attach(flexe_host_net_t *net, wifi_stubs_t *wifi);
void flexe_host_net_pump(flexe_host_net_t *net);
void flexe_host_net_destroy(flexe_host_net_t *net);

#endif
