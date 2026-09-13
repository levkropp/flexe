#include "host_net_slirp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef FLEXE_HAS_SLIRP

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <poll.h>
#include <libslirp.h>
#include <time.h>

typedef struct host_timer {
    struct host_timer *next;
    SlirpTimerId id;
    void *opaque;
    int64_t deadline_ms;
} host_timer_t;

struct flexe_host_net {
    Slirp *slirp;
    SlirpConfig config;
    SlirpCb callbacks;
    wifi_stubs_t *wifi;
    host_timer_t *timers;
    struct pollfd *fds;
    size_t fd_count;
    size_t fd_capacity;
    uint32_t iface;
    uint64_t host_frames;
    uint64_t guest_frames;
    uint64_t dropped_frames;
    bool trace;
};

static void trace_frame(const char *direction, uint64_t sequence,
                        const uint8_t *frame, size_t len)
{
    fprintf(stderr, "[net] %s frame %llu len=%zu:", direction,
            (unsigned long long)sequence, len);
    size_t shown = len < 64u ? len : 64u;
    for (size_t i = 0; i < shown; i++) fprintf(stderr, " %02X", frame[i]);
    if (shown < len) fprintf(stderr, " ...");
    fputc('\n', stderr);
}

static int64_t host_clock_ns(void *opaque)
{
    (void)opaque;
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static slirp_ssize_t host_send_packet(const void *frame, size_t len,
                                      void *opaque)
{
    flexe_host_net_t *net = opaque;
    if (!net->wifi ||
        wifi_stubs_queue_ethernet_frame(net->wifi, net->iface, frame, len) != 0) {
        net->dropped_frames++;
        return -1;
    }
    net->host_frames++;
    if (net->trace) trace_frame("host -> guest", net->host_frames,
                                frame, len);
    return (slirp_ssize_t)len;
}

static void host_guest_error(const char *message, void *opaque)
{
    (void)opaque;
    fprintf(stderr, "[net] guest packet: %s\n", message);
}

/* The run loop refreshes libslirp's fd set every batch; registration only
 * tells it the set changed, so there is no persistent host-side table. */
static void host_register_socket(slirp_os_socket socket, void *opaque)
{ (void)socket; (void)opaque; }

static void host_unregister_socket(slirp_os_socket socket, void *opaque)
{ (void)socket; (void)opaque; }

static void host_notify(void *opaque) { (void)opaque; }

static void *host_timer_new(SlirpTimerId id, void *timer_opaque,
                            void *opaque)
{
    flexe_host_net_t *net = opaque;
    host_timer_t *timer = calloc(1u, sizeof(*timer));
    if (!timer) return NULL;
    timer->id = id;
    timer->opaque = timer_opaque;
    timer->deadline_ms = INT64_MAX;
    timer->next = net->timers;
    net->timers = timer;
    return timer;
}

static void host_timer_free(void *handle, void *opaque)
{
    flexe_host_net_t *net = opaque;
    host_timer_t **link = &net->timers;
    while (*link) {
        if (*link == handle) {
            host_timer_t *timer = *link;
            *link = timer->next;
            free(timer);
            return;
        }
        link = &(*link)->next;
    }
}

static void host_timer_mod(void *handle, int64_t deadline_ms, void *opaque)
{
    (void)opaque;
    ((host_timer_t *)handle)->deadline_ms = deadline_ms;
}

static int host_add_poll(slirp_os_socket socket, int events, void *opaque)
{
    flexe_host_net_t *net = opaque;
    if (net->fd_count == net->fd_capacity) {
        size_t next = net->fd_capacity ? net->fd_capacity * 2u : 16u;
        struct pollfd *fds = realloc(net->fds, next * sizeof(*fds));
        if (!fds) return -1;
        net->fds = fds;
        net->fd_capacity = next;
    }
    short poll_events = 0;
    if (events & SLIRP_POLL_IN) poll_events |= POLLIN;
    if (events & SLIRP_POLL_OUT) poll_events |= POLLOUT;
    if (events & SLIRP_POLL_PRI) poll_events |= POLLPRI;
    net->fds[net->fd_count] = (struct pollfd){
        .fd = socket, .events = poll_events, .revents = 0,
    };
    return (int)net->fd_count++;
}

static int host_poll_revents(int index, void *opaque)
{
    flexe_host_net_t *net = opaque;
    if (index < 0 || (size_t)index >= net->fd_count) return 0;
    short events = net->fds[index].revents;
    int result = 0;
    if (events & POLLIN) result |= SLIRP_POLL_IN;
    if (events & POLLOUT) result |= SLIRP_POLL_OUT;
    if (events & POLLPRI) result |= SLIRP_POLL_PRI;
    if (events & POLLERR) result |= SLIRP_POLL_ERR;
    if (events & POLLHUP) result |= SLIRP_POLL_HUP;
    return result;
}

static int host_tx(void *opaque, uint32_t iface, const uint8_t *frame,
                   size_t len)
{
    flexe_host_net_t *net = opaque;
    if (iface != net->iface) return 1;
    if (!net->slirp || len > INT_MAX) return -1;
    net->guest_frames++;
    if (net->trace) trace_frame("guest -> host", net->guest_frames,
                                frame, len);
    slirp_input(net->slirp, frame, (int)len);
    return 0;
}

static int parse_port(const char *text, uint16_t *out)
{
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    if (errno || end == text || *end || value == 0u || value > 65535u)
        return -1;
    *out = (uint16_t)value;
    return 0;
}

static int parse_spec(const char *spec, uint32_t *iface,
                      uint16_t *host_port, struct in_addr *guest_ip,
                      uint16_t *guest_port)
{
    if (!spec || strlen(spec) >= 96u) return -1;
    char parts[96];
    memcpy(parts, spec, strlen(spec) + 1u);
    char *host = strchr(parts, ':');
    if (!host) return -1;
    *host++ = '\0';
    char *ip = strchr(host, ':');
    if (!ip) return -1;
    *ip++ = '\0';
    char *port = strchr(ip, ':');
    if (!port) return -1;
    *port++ = '\0';
    if (strcmp(parts, "ap") == 0) *iface = 1u;
    else if (strcmp(parts, "sta") == 0) *iface = 0u;
    else return -1;
    return parse_port(host, host_port) == 0 &&
           inet_pton(AF_INET, ip, guest_ip) == 1 &&
           parse_port(port, guest_port) == 0 ? 0 : -1;
}

flexe_host_net_t *flexe_host_net_create(const char *spec, wifi_stubs_t *wifi)
{
    if (!wifi_stubs_has_ethernet_boundary(wifi)) {
        fprintf(stderr,
                "[net] S3 Ethernet symbols unavailable; supply the matching application ELF with -s\n");
        return NULL;
    }
    uint32_t iface = 0;
    uint16_t host_port = 0, guest_port = 0;
    struct in_addr guest_ip = {0};
    if (parse_spec(spec, &iface, &host_port, &guest_ip, &guest_port) != 0) {
        fprintf(stderr, "[net] expected ap|sta:HOST_PORT:GUEST_IP:GUEST_PORT\n");
        return NULL;
    }
    uint32_t ip = ntohl(guest_ip.s_addr);
    uint32_t subnet = ip & 0xFFFFFF00u;
    if ((ip & 0xFFu) == 0u || (ip & 0xFFu) == 255u) {
        fprintf(stderr, "[net] guest IP must be a host address in its /24\n");
        return NULL;
    }
    flexe_host_net_t *net = calloc(1u, sizeof(*net));
    if (!net) return NULL;
    net->iface = iface;
    net->wifi = wifi;
    net->trace = getenv("FLEXE_NET_TRACE") != NULL;

    SlirpConfig *cfg = &net->config;
    cfg->version = 6;
    /* This CLI switch exposes only the requested inbound loopback service.
     * Firmware must not silently gain an outbound route to the host/LAN. */
    cfg->restricted = 1;
    cfg->disable_host_loopback = true;
    cfg->in_enabled = true;
    cfg->vnetwork.s_addr = htonl(subnet);
    cfg->vnetmask.s_addr = htonl(0xFFFFFF00u);
    cfg->vhost.s_addr = htonl(subnet | ((ip & 0xFFu) == 2u ? 253u : 2u));
    cfg->vnameserver.s_addr = htonl(subnet | ((ip & 0xFFu) == 3u ? 254u : 3u));
    cfg->vdhcp_start.s_addr = htonl(subnet | ((ip & 0xFFu) == 100u ? 101u : 100u));
    cfg->disable_dhcp = iface == 1u; /* AP firmware already serves its clients */
    SlirpCb *callbacks = &net->callbacks;
    callbacks->send_packet = host_send_packet;
    callbacks->guest_error = host_guest_error;
    callbacks->clock_get_ns = host_clock_ns;
    callbacks->timer_new_opaque = host_timer_new;
    callbacks->timer_free = host_timer_free;
    callbacks->timer_mod = host_timer_mod;
    callbacks->register_poll_socket = host_register_socket;
    callbacks->unregister_poll_socket = host_unregister_socket;
    callbacks->notify = host_notify;
    net->slirp = slirp_new(cfg, callbacks, net);
    if (!net->slirp) {
        flexe_host_net_destroy(net);
        return NULL;
    }
    struct in_addr loopback = {.s_addr = htonl(INADDR_LOOPBACK)};
    if (slirp_add_hostfwd(net->slirp, 0, loopback, host_port,
                          guest_ip, guest_port) != 0) {
        fprintf(stderr, "[net] cannot listen on 127.0.0.1:%u\n", host_port);
        flexe_host_net_destroy(net);
        return NULL;
    }
    wifi_stubs_set_ethernet_tx_callback(wifi, host_tx, net);
    fprintf(stderr, "[net] 127.0.0.1:%u -> %s %s:%u (Ethernet user mode)\n",
            host_port, iface ? "AP" : "STA", inet_ntoa(guest_ip),
            guest_port);
    return net;
}

void flexe_host_net_attach(flexe_host_net_t *net, wifi_stubs_t *wifi)
{
    if (!net) return;
    net->wifi = wifi;
    wifi_stubs_set_ethernet_tx_callback(wifi, host_tx, net);
}

void flexe_host_net_pump(flexe_host_net_t *net)
{
    if (!net || !net->slirp) return;
    for (unsigned count = 0; count < 16u; count++) {
        int64_t now_ms = host_clock_ns(net) / 1000000LL;
        host_timer_t *expired = NULL;
        for (host_timer_t *timer = net->timers; timer; timer = timer->next) {
            if (timer->deadline_ms <= now_ms) {
                expired = timer;
                break;
            }
        }
        if (!expired) break;
        expired->deadline_ms = INT64_MAX;
        slirp_handle_timer(net->slirp, expired->id, expired->opaque);
    }
    net->fd_count = 0;
    uint32_t timeout_ms = 0;
    slirp_pollfds_fill_socket(net->slirp, &timeout_ms, host_add_poll, net);
    int result = poll(net->fds, (nfds_t)net->fd_count, 0);
    slirp_pollfds_poll(net->slirp, result < 0, host_poll_revents, net);
}

void flexe_host_net_destroy(flexe_host_net_t *net)
{
    if (!net) return;
    fprintf(stderr, "[net] frames host->guest=%llu guest->host=%llu dropped=%llu\n",
            (unsigned long long)net->host_frames,
            (unsigned long long)net->guest_frames,
            (unsigned long long)net->dropped_frames);
    if (net->wifi) wifi_stubs_set_ethernet_tx_callback(net->wifi, NULL, NULL);
    if (net->slirp) slirp_cleanup(net->slirp);
    while (net->timers) {
        host_timer_t *next = net->timers->next;
        free(net->timers);
        net->timers = next;
    }
    free(net->fds);
    free(net);
}

#else

struct flexe_host_net { int unavailable; };

flexe_host_net_t *flexe_host_net_create(const char *spec, wifi_stubs_t *wifi)
{
    (void)spec;
    (void)wifi;
    fprintf(stderr, "[net] this build lacks libslirp; install it and rebuild\n");
    return NULL;
}

void flexe_host_net_attach(flexe_host_net_t *net, wifi_stubs_t *wifi)
{ (void)net; (void)wifi; }
void flexe_host_net_pump(flexe_host_net_t *net) { (void)net; }
void flexe_host_net_destroy(flexe_host_net_t *net) { (void)net; }

#endif
