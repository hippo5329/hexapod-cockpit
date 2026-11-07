#include "irq_watch.h"

#include <esp_attr.h>
#include <esp_cpu.h>
#include <esp_timer.h>
#include <nvs.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr int kRing = 32;
struct FlashOp {
    uint32_t t_ms, held_us;
};
struct NvsOp {
    uint32_t t_ms;
    void* caller;
    char what[8];
    char key[16];
};
DRAM_ATTR FlashOp flash_ring[kRing];
DRAM_ATTR volatile int flash_n = 0;
DRAM_ATTR volatile uint32_t flash_all = 0, flash_max = 0, flash_dropped = 0;
DRAM_ATTR volatile uint32_t t0_cycles = 0;
NvsOp nvs_ring[kRing];
volatile int nvs_n = 0;
volatile uint32_t nvs_dropped = 0;
uint32_t ovf_t_ms[kRing];
volatile int ovf_n = 0;

void nvs_note(const char* what, const char* key, void* caller) {
    const int i = nvs_n;
    if (i >= kRing) {
        nvs_dropped = nvs_dropped + 1;
        return;
    }
    nvs_ring[i].t_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    nvs_ring[i].caller = caller;
    strncpy(nvs_ring[i].what, what, sizeof nvs_ring[i].what - 1);
    nvs_ring[i].what[sizeof nvs_ring[i].what - 1] = 0;
    strncpy(nvs_ring[i].key, key ? key : "", sizeof nvs_ring[i].key - 1);
    nvs_ring[i].key[sizeof nvs_ring[i].key - 1] = 0;
    nvs_n = i + 1;
}

}  // namespace

extern "C" {

void __real_spi_flash_disable_interrupts_caches_and_other_cpu(void);
void __real_spi_flash_enable_interrupts_caches_and_other_cpu(void);

IRAM_ATTR void __wrap_spi_flash_disable_interrupts_caches_and_other_cpu(void) {
    __real_spi_flash_disable_interrupts_caches_and_other_cpu();
    t0_cycles = esp_cpu_get_cycle_count();
}

IRAM_ATTR void __wrap_spi_flash_enable_interrupts_caches_and_other_cpu(void) {
    const uint32_t held = (esp_cpu_get_cycle_count() - t0_cycles) / 240;  // 240 MHz: cycles -> us
    __real_spi_flash_enable_interrupts_caches_and_other_cpu();
    flash_all = flash_all + 1;
    if (held > flash_max) flash_max = held;
    if (held < 100) return;
    const int i = flash_n;
    if (i < kRing) {
        flash_ring[i].t_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        flash_ring[i].held_us = held;
        flash_n = i + 1;
    } else {
        flash_dropped = flash_dropped + 1;
    }
}

// NVS writes, by their callers. Each wrapper notes the call and passes it on unchanged.
#define LINO_NVS_WRAP(name, type)                                                       \
    esp_err_t __real_##name(nvs_handle_t h, const char* key, type v);                   \
    esp_err_t __wrap_##name(nvs_handle_t h, const char* key, type v) {                  \
        nvs_note(#name + 8, key, __builtin_return_address(0));                          \
        return __real_##name(h, key, v);                                                \
    }
LINO_NVS_WRAP(nvs_set_u8, uint8_t)
LINO_NVS_WRAP(nvs_set_i8, int8_t)
LINO_NVS_WRAP(nvs_set_u16, uint16_t)
LINO_NVS_WRAP(nvs_set_i16, int16_t)
LINO_NVS_WRAP(nvs_set_u32, uint32_t)
LINO_NVS_WRAP(nvs_set_i32, int32_t)
LINO_NVS_WRAP(nvs_set_u64, uint64_t)
LINO_NVS_WRAP(nvs_set_str, const char*)
#undef LINO_NVS_WRAP

esp_err_t __real_nvs_set_blob(nvs_handle_t h, const char* key, const void* v, size_t n);
esp_err_t __wrap_nvs_set_blob(nvs_handle_t h, const char* key, const void* v, size_t n) {
    nvs_note("blob", key, __builtin_return_address(0));
    return __real_nvs_set_blob(h, key, v, n);
}
esp_err_t __real_nvs_erase_key(nvs_handle_t h, const char* key);
esp_err_t __wrap_nvs_erase_key(nvs_handle_t h, const char* key) {
    nvs_note("erase", key, __builtin_return_address(0));
    return __real_nvs_erase_key(h, key);
}
esp_err_t __real_nvs_erase_all(nvs_handle_t h);
esp_err_t __wrap_nvs_erase_all(nvs_handle_t h) {
    nvs_note("eraseall", "", __builtin_return_address(0));
    return __real_nvs_erase_all(h);
}
esp_err_t __real_nvs_commit(nvs_handle_t h);
esp_err_t __wrap_nvs_commit(nvs_handle_t h) {
    nvs_note("commit", "", __builtin_return_address(0));
    return __real_nvs_commit(h);
}

}  // extern "C"

namespace irq_watch {

void mark_overflow() {
    const int i = ovf_n;
    if (i < kRing) {
        ovf_t_ms[i] = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        ovf_n = i + 1;
    }
}

uint32_t flash_ops() { return flash_all; }
uint32_t flash_max_us() { return flash_max; }

void report(void (*line)(const char* text)) {
    char b[160];
    const int fn = flash_n, nn = nvs_n, on = ovf_n;
    for (int i = 0; i < fn; ++i) {
        snprintf(b, sizeof b, "irq: flash op at %u ms held the interrupts %u us", static_cast<unsigned>(flash_ring[i].t_ms),
                 static_cast<unsigned>(flash_ring[i].held_us));
        line(b);
    }
    for (int i = 0; i < nn; ++i) {
        snprintf(b, sizeof b, "irq: nvs %s '%s' at %u ms from %p", nvs_ring[i].what, nvs_ring[i].key,
                 static_cast<unsigned>(nvs_ring[i].t_ms), nvs_ring[i].caller);
        line(b);
    }
    for (int i = 0; i < on; ++i) {
        snprintf(b, sizeof b, "irq: uart fifo overflow at %u ms", static_cast<unsigned>(ovf_t_ms[i]));
        line(b);
    }
    if (flash_dropped || nvs_dropped) {
        snprintf(b, sizeof b, "irq: %u flash ops and %u nvs writes not kept (ring full)", static_cast<unsigned>(flash_dropped),
                 static_cast<unsigned>(nvs_dropped));
        line(b);
    }
    // Entries taken off; new ones may land meanwhile -- they are kept for the next report.
    flash_n = flash_n - fn;
    if (flash_n) memmove(flash_ring, flash_ring + fn, flash_n * sizeof flash_ring[0]);
    nvs_n = nvs_n - nn;
    if (nvs_n) memmove(nvs_ring, nvs_ring + nn, nvs_n * sizeof nvs_ring[0]);
    ovf_n = ovf_n - on;
    if (ovf_n) memmove(ovf_t_ms, ovf_t_ms + on, ovf_n * sizeof ovf_t_ms[0]);
    flash_dropped = nvs_dropped = 0;
}

}  // namespace irq_watch
