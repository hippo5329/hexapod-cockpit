// heap_caps_{malloc,calloc,realloc}_prefer, wrapped at link time (wifi_mem.h). The Wi-Fi adapter calls
// them from IRAM, so the wrappers are IRAM too and call only the heap's IRAM functions.
#include "wifi_mem.h"

#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <stdarg.h>

namespace wifi_mem {
DRAM_ATTR volatile bool internal_first = true;
DRAM_ATTR volatile uint32_t to_internal = 0, to_psram = 0;
}  // namespace wifi_mem

namespace {

constexpr size_t kMaxCaps = 4;

// The caller's list, PSRAM entries moved behind the others when internal_first is set.
IRAM_ATTR size_t order(va_list ap, size_t num, uint32_t* caps, bool* prefers_psram) {
    num = num > kMaxCaps ? kMaxCaps : num;
    uint32_t in[kMaxCaps];
    for (size_t i = 0; i < num; i++) in[i] = va_arg(ap, uint32_t);
    *prefers_psram = num && (in[0] & MALLOC_CAP_SPIRAM);
    size_t n = 0;
    const bool reorder = *prefers_psram && wifi_mem::internal_first;
    for (int pass = 0; pass < (reorder ? 2 : 1); pass++)
        for (size_t i = 0; i < num; i++)
            if (!reorder || ((in[i] & MALLOC_CAP_SPIRAM) != 0) == (pass == 1)) caps[n++] = in[i];
    return n;
}

IRAM_ATTR void count(bool prefers_psram, uint32_t caps) {
    if (!prefers_psram) return;
    if (caps & MALLOC_CAP_SPIRAM) wifi_mem::to_psram = wifi_mem::to_psram + 1;
    else wifi_mem::to_internal = wifi_mem::to_internal + 1;
}

}  // namespace

extern "C" {

IRAM_ATTR void* __wrap_heap_caps_malloc_prefer(size_t size, size_t num, ...) {
    uint32_t caps[kMaxCaps];
    bool pp;
    va_list ap;
    va_start(ap, num);
    const size_t n = order(ap, num, caps, &pp);
    va_end(ap);
    for (size_t i = 0; i < n; i++)
        if (void* p = heap_caps_malloc(size, caps[i])) {
            count(pp, caps[i]);
            return p;
        }
    return nullptr;
}

IRAM_ATTR void* __wrap_heap_caps_calloc_prefer(size_t n_memb, size_t size, size_t num, ...) {
    uint32_t caps[kMaxCaps];
    bool pp;
    va_list ap;
    va_start(ap, num);
    const size_t n = order(ap, num, caps, &pp);
    va_end(ap);
    for (size_t i = 0; i < n; i++)
        if (void* p = heap_caps_calloc(n_memb, size, caps[i])) {
            count(pp, caps[i]);
            return p;
        }
    return nullptr;
}

IRAM_ATTR void* __wrap_heap_caps_realloc_prefer(void* ptr, size_t size, size_t num, ...) {
    uint32_t caps[kMaxCaps];
    bool pp;
    va_list ap;
    va_start(ap, num);
    const size_t n = order(ap, num, caps, &pp);
    va_end(ap);
    for (size_t i = 0; i < n; i++) {
        void* p = heap_caps_realloc(ptr, size, caps[i]);
        if (p) count(pp, caps[i]);
        if (p || !size) return p;  // size 0 freed ptr: never try it again (the IDF's own loop stops too)
    }
    return nullptr;
}

}  // extern "C"
