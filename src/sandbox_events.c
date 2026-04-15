#include "sandbox_events.h"
#include <stddef.h>

static sbx_event_fn g_sink_fn  = NULL;
static void        *g_sink_ctx = NULL;

void sbx_events_set_sink(sbx_event_fn fn, void *ctx) {
    g_sink_fn  = fn;
    g_sink_ctx = ctx;
}

void sbx_events_emit(const sbx_event_t *ev) {
    if (g_sink_fn) g_sink_fn(ev, g_sink_ctx);
}
