// Where the Wi-Fi driver and lwIP put their buffers. The Arduino core's prebuilt IDF has
// CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP: both ask heap_caps_*_prefer(PSRAM first, internal second), and
// their frames then share the PSRAM bus with the camera's DMA. The link wraps those three calls
// (platformio.ini: -Wl,--wrap=...): with wifi_mem::internal_first a request that prefers PSRAM tries
// internal RAM first and falls back to PSRAM only when internal RAM is out.
#pragma once
#include <stdint.h>

namespace wifi_mem {
// Set before WiFi.mode() (the driver's buffers are taken at init); later requests follow it too.
extern volatile bool internal_first;
// Requests that preferred PSRAM: served from internal RAM / from PSRAM.
extern volatile uint32_t to_internal, to_psram;
}  // namespace wifi_mem
