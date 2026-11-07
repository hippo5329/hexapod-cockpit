// hexacam: the miniHexa's camera. An ESP32-S3-CAM board (N16R8, OV3660) joins the robot's Wi-Fi and
// serves its JPEG frames; the host's ROS bridge (hexapod_bringup camera_bridge) turns them into
// camera/image_raw/compressed. Nothing is decoded or detected here: the board only captures and sends.
//
//   http://<ip>:81/stream    MJPEG (multipart/x-mixed-replace), paced to the `fps` setting; one client
//   http://<ip>/capture      one JPEG
//   http://<ip>/status       JSON: settings, measured rate and size, Wi-Fi (rssi, power save, tx power), memory
//                            (no chip temperature: temperatureRead() masks interrupts for ~500 us, and the robot's
//                            UART loses bytes -- IDF 5.5.5 esp_hw_support/sar_periph_ctrl_common.c
//                            temp_sensor_get_raw_value(): esp_rom_delay_us(200 + 300) inside portENTER_CRITICAL)
//   http://<ip>/control      ?vflip=0|1&hmirror=0|1&fps=1-30&xclk=5-24&frame=<size>: kept in NVS (a new size
//                            reboots, a new xclk restarts the camera); capture=0|1 stops/starts it (not kept)
//   POST http://<ip>/wifi    ssid=..&psk=..[&syslog_ip=..]: another network, on trial (60 s, else back)
//   /control?tx=2-20         Wi-Fi transmit power in dBm, kept (cheap boards lose frames at 20)
//   /control?wbuf=1|0        Wi-Fi and lwIP buffers in internal RAM (1, default) or PSRAM (0), kept, reboots
//   /control?ps=0|1|2|kick   Wi-Fi power save now (not kept): 0 none (the boot's), 1 min modem, 2 max modem;
//                            kick: 1 then 0, so the AP hears the station is awake (see ps_kick); /status reset:
//                            why the board last restarted (esp_reset_reason)
//   /control?lidar_rx=<gpio|-1>&lidar_ip=<a.b.c.d>&lidar_port=<p>&lidar_baud=<b>: the LiDAR forwarder
//                            (lidar_fwd.h), any of them, kept, reboots
//
// Settings in NVS namespace `hexacam` (tools/make_cam_nvs.py): ssid, psk (the robot's network), ota_pw
// (OTA on 3232, none: no OTA), name (hostname, default hexacam), frame (qqvga, qvga, vga, svga, xga, hd;
// default qvga), quality (JPEG 10-40, lower is better; default 12), fps (1-30; default 10), vflip,
// hmirror (1: flip the image for the mount), xclk (the sensor clock, MHz, 5-24; default 20), tx_dbm,
// wbuf (1: Wi-Fi buffers in internal RAM, wifi_mem.h; default 1), syslog_ip,
// syslog_port (the robot firmware's syslog), uart (the link to the robot, below; default echo),
// lidar_rx, lidar_ip, lidar_port, lidar_baud (the LiDAR forwarder, lidar_fwd.h; off without lidar_rx and lidar_ip).
//
// The robot's UART: UART1, RX GPIO1 <- the robot's GPIO32, TX GPIO2 -> its GPIO33, 1.5 Mbaud 8N1
// (wired 2026-11-07). NVS `uart`: echo (every byte sent back: the robot's console `uart` test checks
// the link) or off. Counted in /status (uart_bytes) and the health line.
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <driver/gpio.h>

#include "syslog.h"
#include "irq_watch.h"
#include "lidar_fwd.h"
#include "wifi_mem.h"

namespace {

Preferences prefs;
String ssid, psk, ota_pw, name = "hexacam";
framesize_t frame = FRAMESIZE_QVGA;
int quality = 12, fps = 10;
bool vflip = false, hmirror = false;
const char* sensor_name = "none";
bool cam_ok = false, net_started = false;
int xclk_mhz = 20;
// Wi-Fi transmit power, dBm (2-20). Cheap S3 boards cannot hold the default 20 dBm bursts: frames and
// acks are lost, the AP retries and falls to its slowest rate (pings of seconds, 2026-11-06).
float tx_dbm = 20.0f;
// /control?capture=0 stops the camera (its clock and DMA too) for a Wi-Fi test; the stream then idles.
volatile bool paused = false, in_grab = false;
// A Wi-Fi change from POST /wifi is on trial until the board joins the new network (see trial_loop).
const char* const kTrialKeys[] = {"ssid", "psk", "syslog_ip"};
constexpr uint32_t kTrialMs = 60000;
bool wifi_trial = false;

// What the stream has delivered, for /status and the health line.
volatile uint32_t frames_sent = 0, bytes_sent = 0, stream_clients = 0, capture_fails = 0;

// The robot's UART (see the top).
constexpr int kUartRx = 1, kUartTx = 2;
constexpr uint32_t kUartBaud = 1500000;
String uart_mode = "echo";

// The LiDAR forwarder (lidar_fwd.h).
int lidar_rx = -1;
IPAddress lidar_ip;
uint16_t lidar_port = 8889;
uint32_t lidar_baud = 230400;
volatile uint32_t uart_bytes = 0;
// The driver's receive errors (onReceiveError): where the robot's test loses bytes, if here.
volatile uint32_t uart_err_ovf = 0, uart_err_full = 0, uart_err_frame = 0;

// Echo: what arrives goes straight back. Its own task, so a stream or a Wi-Fi stall never holds it.
void uart_echo_task(void*) {
    uint8_t b[512];
    for (;;) {
        const int a = Serial1.available();
        if (a > 0) {
            const int n = Serial1.read(b, a < static_cast<int>(sizeof b) ? a : sizeof b);
            if (n > 0) {
                Serial1.write(b, n);
                uart_bytes = uart_bytes + n;
            }
        } else {
            vTaskDelay(1);
        }
    }
}

void uart_begin() {
    if (uart_mode != "echo") return;
    // Large driver buffers (the hardware FIFO is a fixed 128 bytes): 32 KB each way, 210 ms at 1.5 Mbaud.
    Serial1.setRxBufferSize(32768);
    Serial1.setTxBufferSize(32768);
    Serial1.begin(kUartBaud, SERIAL_8N1, kUartRx, kUartTx);
    // Empty the 128-byte FIFO at 32 bytes, not Arduino's 120: 640 us of slack at 1.5 Mbaud, not 53.
    Serial1.setRxFIFOFull(32);
    Serial1.onReceiveError([](hardwareSerial_error_t e) {
        if (e == UART_FIFO_OVF_ERROR) {
            uart_err_ovf = uart_err_ovf + 1;
            irq_watch::mark_overflow();
        }
        else if (e == UART_BUFFER_FULL_ERROR) uart_err_full = uart_err_full + 1;
        else if (e == UART_FRAME_ERROR) uart_err_frame = uart_err_frame + 1;
    });
    xTaskCreatePinnedToCore(uart_echo_task, "uart_echo", 4096, nullptr, 5, nullptr, 1);
}

struct Frame {
    const char* name;
    framesize_t size;
};
const Frame kFrames[] = {{"qqvga", FRAMESIZE_QQVGA}, {"qvga", FRAMESIZE_QVGA}, {"vga", FRAMESIZE_VGA},
                         {"svga", FRAMESIZE_SVGA},   {"xga", FRAMESIZE_XGA},   {"hd", FRAMESIZE_HD}};

const char* frame_name(framesize_t f) {
    for (const Frame& k : kFrames)
        if (k.size == f) return k.name;
    return "?";
}

void load_settings() {
    prefs.begin("hexacam", true);
    ssid = prefs.getString("ssid", "");
    psk = prefs.getString("psk", "");
    ota_pw = prefs.getString("ota_pw", "");
    name = prefs.getString("name", "hexacam");
    const String f = prefs.getString("frame", "qvga");
    for (const Frame& k : kFrames)
        if (f == k.name) frame = k.size;
    quality = constrain(prefs.getString("quality", "12").toInt(), 10, 40);
    fps = constrain(prefs.getString("fps", "10").toInt(), 1, 30);
    vflip = prefs.getString("vflip", "0") == "1";
    hmirror = prefs.getString("hmirror", "0") == "1";
    xclk_mhz = constrain(prefs.getString("xclk", "20").toInt(), 5, 24);
    wifi_trial = prefs.getBool("wifi_trial", false);
    tx_dbm = constrain(prefs.getString("tx_dbm", "20").toFloat(), 2.0f, 20.0f);
    wifi_mem::internal_first = prefs.getString("wbuf", "1") != "0";
    uart_mode = prefs.getString("uart", "echo");
    if (uart_mode != "echo" && uart_mode != "off") uart_mode = "echo";
    lidar_rx = prefs.getString("lidar_rx", "-1").toInt();
    if (!lidar_ip.fromString(prefs.getString("lidar_ip", ""))) lidar_ip = IPAddress(0, 0, 0, 0);
    lidar_port = constrain(prefs.getString("lidar_port", "8889").toInt(), 1, 65535);
    lidar_baud = constrain(prefs.getString("lidar_baud", "230400").toInt(), 9600, 2000000);
}

void apply_tx_power() { esp_wifi_set_max_tx_power(static_cast<int8_t>(tx_dbm * 4)); }

// Power save on for a moment, then off (/control?ps=kick): leaving power save sends the AP a null frame with
// the power-management bit clear. A test aid: on two different APs this board's uplink leaves only at beacons
// (pings a ~103 ms sawtooth, LiDAR datagrams in the same bursts) while esp_wifi_get_ps() says NONE, and
// neither a kick nor ps=1/0 nor tx power 2-20 dBm changed it (2026-11-07).
void ps_kick() {
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    delay(200);  // two beacons
    esp_wifi_set_ps(WIFI_PS_NONE);
}

const char* ps_name(int ps) { return ps == 0 ? "none" : ps == 1 ? "min modem" : "max modem"; }

const char* reset_name(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "power on";
        case ESP_RST_EXT: return "external pin";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        case ESP_RST_USB: return "usb";
        case ESP_RST_JTAG: return "jtag";
        default: return "unknown";
    }
}

// The common ESP32-S3-CAM pin map (ESP32-S3-EYE / Freenove layout), checked on the board: OV3660.
bool camera_begin() {
    camera_config_t c = {};
    c.pin_pwdn = -1;
    c.pin_reset = -1;
    c.pin_xclk = 15;
    c.pin_sccb_sda = 4;
    c.pin_sccb_scl = 5;
    c.pin_d7 = 16;
    c.pin_d6 = 17;
    c.pin_d5 = 18;
    c.pin_d4 = 12;
    c.pin_d3 = 10;
    c.pin_d2 = 8;
    c.pin_d1 = 9;
    c.pin_d0 = 11;
    c.pin_vsync = 6;
    c.pin_href = 7;
    c.pin_pclk = 13;
    c.xclk_freq_hz = xclk_mhz * 1000000;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = frame;
    c.jpeg_quality = quality;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;  // the newest frame, never a stale queue
    if (esp_camera_init(&c) != ESP_OK) return false;
    sensor_t* s = esp_camera_sensor_get();
    sensor_name = s->id.PID == OV3660_PID ? "OV3660" : s->id.PID == OV2640_PID ? "OV2640"
                : s->id.PID == OV5640_PID ? "OV5640" : "other";
    s->set_vflip(s, vflip);
    s->set_hmirror(s, hmirror);
    return true;
}

// ---------------------------------------------------------------- HTTP
constexpr const char* kBoundary = "hexacamframe";

esp_err_t stream_handler(httpd_req_t* req) {
    if (!cam_ok) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no camera");
    // httpd keeps the pointer, not a copy, and sends the headers with the first chunk: a constant (the
    // part-header buffer below was once reused here, and the response's headers were a part's).
    static const char kType[] = "multipart/x-mixed-replace;boundary=hexacamframe";
    httpd_resp_set_type(req, kType);
    char head[160];
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ++stream_clients;
    hexa_log::log(hexa_log::kInfo, "stream: client up (%u)", static_cast<unsigned>(stream_clients));
    uint32_t next = millis();
    esp_err_t r = ESP_OK;
    while (r == ESP_OK) {
        const uint32_t period_ms = 1000 / fps;  // each frame: /control may change it mid-stream
        const int32_t wait = static_cast<int32_t>(next - millis());
        if (wait > 0) delay(wait);
        next += period_ms;
        if (static_cast<int32_t>(millis() - next) > static_cast<int32_t>(period_ms)) next = millis();  // no burst
        if (paused || !cam_ok) {  // the camera is stopped or being restarted: keep the client, send nothing
            delay(50);
            continue;
        }
        in_grab = true;
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            in_grab = false;
            ++capture_fails;
            continue;
        }
        // The capture time on the board's clock (us since boot): the bridge measures intervals with it.
        const int n = snprintf(head, sizeof head,
                               "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %lld\r\n\r\n",
                               kBoundary, static_cast<unsigned>(fb->len),
                               static_cast<long long>(fb->timestamp.tv_sec) * 1000000LL + fb->timestamp.tv_usec);
        r = httpd_resp_send_chunk(req, head, n);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);
        if (r == ESP_OK) {
            ++frames_sent;
            bytes_sent += fb->len;
        }
        esp_camera_fb_return(fb);
        in_grab = false;
    }
    --stream_clients;
    hexa_log::log(hexa_log::kInfo, "stream: client gone");
    return r;
}

esp_err_t capture_handler(httpd_req_t* req) {
    camera_fb_t* fb = cam_ok ? esp_camera_fb_get() : nullptr;
    if (!fb) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no frame");
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t r = httpd_resp_send(req, reinterpret_cast<const char*>(fb->buf), fb->len);
    esp_camera_fb_return(fb);
    return r;
}

// The rate the stream really delivers: frames over the last health window.
float measured_fps = 0, measured_kbps = 0;

esp_err_t status_handler(httpd_req_t* req) {
    char j[1280];
    sensor_t* s = cam_ok ? esp_camera_sensor_get() : nullptr;
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);
    int8_t txp = 0;  // 0.25 dBm units
    esp_wifi_get_max_tx_power(&txp);
    snprintf(j, sizeof j,
             "{\"name\":\"%s\",\"sensor\":\"%s\",\"frame\":\"%s\",\"quality\":%d,\"fps_cap\":%d,\"fps\":%.1f,"
             "\"kbps\":%.0f,\"clients\":%u,\"frames\":%u,\"capture_fails\":%u,\"vflip\":%d,\"hmirror\":%d,"
             "\"rssi\":%d,\"uptime_s\":%u,\"heap\":%u,\"psram_free\":%u,\"wifi_ps\":%d,"
             "\"tx_dbm\":%.2f,\"channel\":%d,\"xclk_mhz\":%d,\"capture\":%d,\"ssid\":\"%s\",\"wifi_trial\":%d,"
             "\"wbuf\":\"%s\",\"wbuf_internal\":%u,\"wbuf_psram\":%u,\"heap_min\":%u,\"uart\":\"%s\",\"uart_bytes\":%u,\"uart_err\":[%u,%u,%u],"
             "\"lidar_rx\":%d,\"lidar_ip\":\"%s\",\"lidar_port\":%u,\"lidar_bytes\":%u,\"lidar_packets\":%u,\"lidar_err\":%u,\"reset\":\"%s\"}",
             name.c_str(), sensor_name, frame_name(s ? static_cast<framesize_t>(s->status.framesize) : frame), quality,
             fps, measured_fps, measured_kbps, static_cast<unsigned>(stream_clients),
             static_cast<unsigned>(frames_sent), static_cast<unsigned>(capture_fails), vflip, hmirror,
             static_cast<int>(WiFi.RSSI()), static_cast<unsigned>(millis() / 1000),
             static_cast<unsigned>(ESP.getFreeHeap()), static_cast<unsigned>(ESP.getFreePsram()),
             static_cast<int>(ps), txp / 4.0, static_cast<int>(WiFi.channel()), xclk_mhz, cam_ok && !paused,
             ssid.c_str(), wifi_trial, wifi_mem::internal_first ? "internal" : "psram",
             static_cast<unsigned>(wifi_mem::to_internal), static_cast<unsigned>(wifi_mem::to_psram),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)), uart_mode.c_str(),
             static_cast<unsigned>(uart_bytes), static_cast<unsigned>(uart_err_ovf), static_cast<unsigned>(uart_err_full),
             static_cast<unsigned>(uart_err_frame), lidar_rx, lidar_ip.toString().c_str(), static_cast<unsigned>(lidar_port),
             static_cast<unsigned>(lidar_fwd::bytes()), static_cast<unsigned>(lidar_fwd::packets()),
             static_cast<unsigned>(lidar_fwd::errors()), reset_name(esp_reset_reason()));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, j, HTTPD_RESP_USE_STRLEN);
}

// /control?vflip=0|1&hmirror=0|1&fps=1-30&frame=<size>: flip the image for the mount and pace the stream
// (both at once) and set the frame size, all kept in NVS (a mount change needs no USB). The frame
// buffers are sized at camera init, so a new frame size reboots the board after the reply. Every value
// is checked before any is applied; the reply is /status.
uint32_t restart_at = 0;

// Stop the camera driver (clock, DMA, buffers) once the stream holds no frame; start it again.
void camera_stop() {
    paused = true;
    delay(20);  // a grab that passed the check just before sets in_grab by now
    for (int i = 0; i < 100 && in_grab; i++) delay(10);
    if (cam_ok) esp_camera_deinit();
    cam_ok = false;
}
void camera_start() {
    cam_ok = camera_begin();
    paused = false;
}

esp_err_t control_handler(httpd_req_t* req) {
    const char* const keys[] = {"vflip", "hmirror"};
    bool* const flags[] = {&vflip, &hmirror};
    char q[192], v[2][2] = {}, fv[8] = {};
    bool given[2] = {}, frame_given = false;
    int new_fps = 0, new_xclk = 0, capture = -1, new_wbuf = -1, new_ps = -1;  // new_ps 3: kick
    float new_tx = 0;
    const Frame* new_frame = nullptr;
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK) {
        // The LiDAR forwarder's settings: on their own, kept, and a reboot applies them.
        const char* const lkeys[] = {"lidar_rx", "lidar_ip", "lidar_port", "lidar_baud"};
        char lv[4][20] = {};
        bool lany = false;
        for (int i = 0; i < 4; i++) lany |= httpd_query_key_value(q, lkeys[i], lv[i], sizeof lv[i]) == ESP_OK;
        if (lany) {
            IPAddress ip;
            const bool ok = (!lv[0][0] || (atoi(lv[0]) >= -1 && atoi(lv[0]) <= 48)) && (!lv[1][0] || ip.fromString(lv[1])) &&
                            (!lv[2][0] || (atoi(lv[2]) >= 1 && atoi(lv[2]) <= 65535)) &&
                            (!lv[3][0] || (atoi(lv[3]) >= 9600 && atoi(lv[3]) <= 2000000));
            if (!ok) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                    "lidar_rx -1..48, lidar_ip a.b.c.d, lidar_port 1-65535, lidar_baud 9600-2000000");
                return ESP_FAIL;
            }
            Preferences w;
            w.begin("hexacam", false);
            for (int i = 0; i < 4; i++)
                if (lv[i][0]) w.putString(lkeys[i], lv[i]);
            w.end();
            hexa_log::log(hexa_log::kNotice, "control: lidar rx %s ip %s port %s baud %s (kept), rebooting",
                          lv[0][0] ? lv[0] : "-", lv[1][0] ? lv[1] : "-", lv[2][0] ? lv[2] : "-", lv[3][0] ? lv[3] : "-");
            const esp_err_t r = status_handler(req);
            restart_at = millis() + 500;
            return r;
        }
        for (int i = 0; i < 2; i++) {
            char val[4];
            if (httpd_query_key_value(q, keys[i], val, sizeof val) != ESP_OK) continue;
            if (strcmp(val, "0") && strcmp(val, "1")) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "vflip and hmirror take 0 or 1");
                return ESP_FAIL;
            }
            given[i] = true;
            v[i][0] = val[0];
        }
        char f[8];
        if (httpd_query_key_value(q, "fps", f, sizeof f) == ESP_OK) {
            char* end = nullptr;
            const long n = strtol(f, &end, 10);
            if (end == f || *end || n < 1 || n > 30) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "fps takes 1 to 30");
                return ESP_FAIL;
            }
            new_fps = static_cast<int>(n);
        }
        if (httpd_query_key_value(q, "xclk", f, sizeof f) == ESP_OK) {
            char* end = nullptr;
            const long n = strtol(f, &end, 10);
            if (end == f || *end || n < 5 || n > 24) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "xclk takes 5 to 24 (MHz)");
                return ESP_FAIL;
            }
            new_xclk = static_cast<int>(n);
        }
        if (httpd_query_key_value(q, "tx", f, sizeof f) == ESP_OK) {
            char* end = nullptr;
            const float d = strtof(f, &end);
            if (end == f || *end || d < 2 || d > 20) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "tx takes 2 to 20 (dBm)");
                return ESP_FAIL;
            }
            new_tx = d;
        }
        if (httpd_query_key_value(q, "capture", f, sizeof f) == ESP_OK) {
            if (strcmp(f, "0") && strcmp(f, "1")) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "capture takes 0 or 1");
                return ESP_FAIL;
            }
            capture = f[0] == '1';
        }
        if (httpd_query_key_value(q, "ps", f, sizeof f) == ESP_OK) {
            if (!strcmp(f, "kick")) new_ps = 3;
            else if (!strcmp(f, "0") || !strcmp(f, "1") || !strcmp(f, "2")) new_ps = f[0] - '0';
            else {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ps takes 0 (none), 1 (min modem), 2 (max modem) or kick");
                return ESP_FAIL;
            }
        }
        if (httpd_query_key_value(q, "wbuf", f, sizeof f) == ESP_OK) {
            if (strcmp(f, "0") && strcmp(f, "1")) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "wbuf takes 1 (internal RAM) or 0 (PSRAM)");
                return ESP_FAIL;
            }
            new_wbuf = f[0] == '1';
        }
        if (httpd_query_key_value(q, "frame", fv, sizeof fv) == ESP_OK) {
            frame_given = true;
            for (const Frame& k : kFrames)
                if (!strcmp(fv, k.name)) new_frame = &k;
            if (!new_frame) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "frame takes qqvga, qvga, vga, svga, xga or hd");
                return ESP_FAIL;
            }
        }
    }
    if (!given[0] && !given[1] && !frame_given && !new_fps && !new_xclk && capture < 0 && !new_tx && new_wbuf < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "usage: /control?vflip=0|1&hmirror=0|1&fps=1-30&tx=2-20&xclk=5-24&capture=0|1&wbuf=0|1&frame=<size>"
                            " or lidar_rx=..&lidar_ip=..&lidar_port=..&lidar_baud=..");
        return ESP_FAIL;
    }
    Preferences w;
    w.begin("hexacam", false);
    sensor_t* s = cam_ok ? esp_camera_sensor_get() : nullptr;
    for (int i = 0; i < 2; i++) {
        if (!given[i]) continue;
        *flags[i] = v[i][0] == '1';
        w.putString(keys[i], v[i]);
    }
    if (new_frame) w.putString("frame", new_frame->name);
    if (new_fps) {
        fps = new_fps;
        w.putString("fps", String(new_fps));
    }
    if (new_xclk) w.putString("xclk", String(new_xclk));
    if (new_wbuf >= 0) w.putString("wbuf", new_wbuf ? "1" : "0");
    if (new_tx) {
        tx_dbm = new_tx;
        w.putString("tx_dbm", String(new_tx, 2));
        apply_tx_power();
    }
    w.end();
    if (s) {
        s->set_vflip(s, vflip);
        s->set_hmirror(s, hmirror);
    }
    // a new clock restarts the camera (unless it is stopped: it starts with the new clock)
    const bool reclock = new_xclk && new_xclk != xclk_mhz;
    if (new_xclk) xclk_mhz = new_xclk;
    if (new_ps == 3) ps_kick();
    else if (new_ps >= 0) esp_wifi_set_ps(new_ps == 0 ? WIFI_PS_NONE : new_ps == 1 ? WIFI_PS_MIN_MODEM : WIFI_PS_MAX_MODEM);
    if (new_ps >= 0) hexa_log::log(hexa_log::kNotice, "control: power save %s (not kept)", new_ps == 3 ? "kick" : ps_name(new_ps));
    if (capture == 0) camera_stop();
    else if (capture == 1 && !cam_ok) camera_start();
    else if (reclock && cam_ok) {
        camera_stop();
        camera_start();
    }
    // the Wi-Fi buffers are taken at driver init: a new place needs a boot
    const bool rewbuf = new_wbuf >= 0 && (new_wbuf == 1) != wifi_mem::internal_first;
    const bool reboot = (new_frame && new_frame->size != frame) || rewbuf;
    hexa_log::log(hexa_log::kNotice, "control: vflip %d hmirror %d fps %d tx %.1f dBm xclk %d MHz capture %s frame %s wbuf %s (kept)%s",
                  vflip, hmirror, fps, tx_dbm, xclk_mhz, cam_ok ? "on" : "off",
                  new_frame ? new_frame->name : frame_name(frame),
                  (new_wbuf >= 0 ? new_wbuf == 1 : wifi_mem::internal_first) ? "internal" : "psram",
                  reboot ? ", rebooting" : "");
    const esp_err_t r = status_handler(req);
    if (reboot) restart_at = millis() + 500;
    return r;
}

// %XX and '+' in a form value, in place.
void url_decode(char* v) {
    char* o = v;
    for (const char* i = v; *i; ++i) {
        if (*i == '+') *o++ = ' ';
        else if (*i == '%' && isxdigit(static_cast<unsigned char>(i[1])) && isxdigit(static_cast<unsigned char>(i[2]))) {
            const char h[3] = {i[1], i[2], 0};
            *o++ = static_cast<char>(strtol(h, nullptr, 16));
            i += 2;
        } else *o++ = *i;
    }
    *o = 0;
}

// POST /wifi, form body ssid=..&psk=..[&syslog_ip=..]: join another network, no USB. The values are kept
// and the board reboots on trial: if it has not joined the new network 60 s after boot, the old ssid,
// psk and syslog_ip are put back and it reboots again. Nothing of the body is logged but the ssid.
esp_err_t wifi_handler(httpd_req_t* req) {
    char body[256];
    const int len = req->content_len;
    if (len <= 0 || len >= static_cast<int>(sizeof body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body: ssid=..&psk=..[&syslog_ip=..]");
        return ESP_FAIL;
    }
    for (int got = 0; got < len;) {
        const int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0) return ESP_FAIL;
        got += r;
    }
    body[len] = 0;
    char ss[100] = {}, pk[200] = {}, ip[48] = {};
    const bool has_ip = httpd_query_key_value(body, "syslog_ip", ip, sizeof ip) == ESP_OK;
    if (httpd_query_key_value(body, "ssid", ss, sizeof ss) != ESP_OK ||
        httpd_query_key_value(body, "psk", pk, sizeof pk) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid and psk are needed (psk may be empty: an open network)");
        return ESP_FAIL;
    }
    url_decode(ss);
    url_decode(pk);
    url_decode(ip);
    const size_t sl = strlen(ss), pl = strlen(pk);
    IPAddress addr;
    if (sl < 1 || sl > 32 || (pl && (pl < 8 || pl > 63)) || (has_ip && !addr.fromString(ip))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid 1-32 chars, psk empty or 8-63, syslog_ip an IPv4 address");
        return ESP_FAIL;
    }
    Preferences w;
    w.begin("hexacam", false);
    // A second change before the first one joined keeps the first one's old values: those worked.
    if (!w.getBool("wifi_trial", false)) {
        for (const char* k : kTrialKeys) {
            const String prev = String("prev_") + k;
            if (w.isKey(k)) w.putString(prev.c_str(), w.getString(k, ""));
            else w.remove(prev.c_str());
        }
        w.putBool("wifi_trial", true);
    }
    w.putString("ssid", ss);
    w.putString("psk", pk);
    if (has_ip) w.putString("syslog_ip", ip);
    w.end();
    hexa_log::log(hexa_log::kNotice, "wifi: '%s' on trial after the reboot (60 s, else the old network)", ss);
    httpd_resp_set_type(req, "text/plain");
    const esp_err_t r = httpd_resp_sendstr(req, "ok: rebooting to join the new network on trial\n");
    restart_at = millis() + 500;
    return r;
}

// The trial of a Wi-Fi change: joined = kept; 60 s without = the old values back and a reboot.
void trial_loop(bool up) {
    if (!wifi_trial) return;
    Preferences w;
    if (up) {
        w.begin("hexacam", false);
        for (const char* k : kTrialKeys) w.remove((String("prev_") + k).c_str());
        w.remove("wifi_trial");
        w.end();
        wifi_trial = false;
        hexa_log::log(hexa_log::kNotice, "wifi: the new network joined; kept");
    } else if (millis() > kTrialMs) {
        w.begin("hexacam", false);
        for (const char* k : kTrialKeys) {
            const String prev = String("prev_") + k;
            if (w.isKey(prev.c_str())) w.putString(k, w.getString(prev.c_str(), ""));
            else w.remove(k);
            w.remove(prev.c_str());
        }
        w.remove("wifi_trial");
        w.end();
        Serial.println("wifi: no network in 60 s with the new settings; the old ones put back, rebooting");
        delay(100);
        ESP.restart();
    }
}

// GET /pins: which header pin carries a signal (the LiDAR's TX: thousands of edges a second). Samples
// each pin not taken by the camera, the UARTs to the robot and USB, flash/PSRAM or the console, 50 ms
// each, with only its input enabled (the LiDAR's own pin is read through its UART's input). Debug aid.
esp_err_t pins_handler(httpd_req_t* req) {
    // ?all=1: every GPIO but flash and octal PSRAM (26-37), the camera's too (stop it first, capture=0)
    static const int few[] = {14, 21, 47, 38, 39, 40, 41, 42, 3, 0, 45, 46};
    static const int every[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                                38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48};
    char q[16] = {}, a[4] = {};
    const bool all = httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK &&
                     httpd_query_key_value(q, "all", a, sizeof a) == ESP_OK && a[0] == '1';
    const int* const pins = all ? every : few;
    const int npins = all ? sizeof every / sizeof *every : sizeof few / sizeof *few;
    char j[1024];
    int o = snprintf(j, sizeof j, "{");
    for (int k = 0; k < npins; k++) {
        const int p = pins[k];
        if (p != lidar_rx) gpio_input_enable(static_cast<gpio_num_t>(p));
        int edges = 0, lows = 0, last = gpio_get_level(static_cast<gpio_num_t>(p)), n = 0;
        const uint32_t t0 = micros();
        while (micros() - t0 < 50000) {
            const int v = gpio_get_level(static_cast<gpio_num_t>(p));
            edges += v != last;
            lows += v == 0;
            last = v;
            ++n;
        }
        o += snprintf(j + o, sizeof j - o, "%s\"%d\":[%d,%d]", o > 1 ? "," : "", p, edges, n ? 100 * lows / n : 0);
    }
    snprintf(j + o, sizeof j - o, "}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, j, HTTPD_RESP_USE_STRLEN);
}

void http_begin() {
    // Two servers: a stream holds its handler for as long as the client reads, so /status and
    // /capture have their own (the esp32-camera examples do the same, stream on port 81).
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.server_port = 80;
    c.ctrl_port = 32768;
    httpd_handle_t ctl = nullptr;
    if (httpd_start(&ctl, &c) == ESP_OK) {
        httpd_uri_t st = {"/status", HTTP_GET, status_handler, nullptr};
        httpd_uri_t cap = {"/capture", HTTP_GET, capture_handler, nullptr};
        httpd_register_uri_handler(ctl, &st);
        httpd_register_uri_handler(ctl, &cap);
        httpd_uri_t ctrl = {"/control", HTTP_GET, control_handler, nullptr};
        httpd_register_uri_handler(ctl, &ctrl);
        httpd_uri_t wifi = {"/wifi", HTTP_POST, wifi_handler, nullptr};
        httpd_register_uri_handler(ctl, &wifi);
        httpd_uri_t pins = {"/pins", HTTP_GET, pins_handler, nullptr};
        httpd_register_uri_handler(ctl, &pins);
    }
    httpd_config_t sc = HTTPD_DEFAULT_CONFIG();
    sc.server_port = 81;
    sc.ctrl_port = 32769;
    sc.max_open_sockets = 2;
    httpd_handle_t str = nullptr;
    if (httpd_start(&str, &sc) == ESP_OK) {
        httpd_uri_t s = {"/stream", HTTP_GET, stream_handler, nullptr};
        httpd_register_uri_handler(str, &s);
    }
}

}  // namespace

void setup() {
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    load_settings();
    hexa_log::begin(prefs, !ssid.isEmpty(), true, name.c_str(), "camera");
    cam_ok = camera_begin();
    uart_begin();  // before Wi-Fi: the robot's link does not wait for a network
    if (ssid.isEmpty()) {
        hexa_log::log(hexa_log::kErr, "no ssid in NVS (hexacam): nothing to join (tools/make_cam_nvs.py)");
        return;
    }
    WiFi.mode(WIFI_STA);
    // Why a join fails, in the log: the reason code of every disconnect (WIFI_REASON_*: 2 auth expired,
    // 15 4-way handshake timeout (wrong key), 201 no AP found, 205 connection failed).
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
        hexa_log::log(hexa_log::kWarning, "wifi: disconnected, reason %d", info.wifi_sta_disconnected.reason);
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    // Every event's time, beside irq_watch's flash operations.
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t) {
        hexa_log::log(hexa_log::kInfo, "irq: wifi event %d at %u ms", static_cast<int>(e), static_cast<unsigned>(millis()));
    });
    // What it can hear, once (names, channels, levels; the wanted one marked).
    const int n = WiFi.scanNetworks();
    for (int i = 0; i < n && i < 12; ++i)
        hexa_log::log(hexa_log::kInfo, "wifi: sees '%s' ch %d %d dBm%s", WiFi.SSID(i).c_str(), WiFi.channel(i),
                      static_cast<int>(WiFi.RSSI(i)), WiFi.SSID(i) == ssid ? "  <- wanted" : "");
    hexa_log::log(hexa_log::kInfo, "wifi: %d network(s) heard", n);
    WiFi.scanDelete();
    WiFi.setHostname(name.c_str());
    WiFi.setSleep(false);  // modem sleep adds 100+ ms to every frame
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid.c_str(), psk.c_str());
    apply_tx_power();
    lidar_fwd::begin(lidar_rx, lidar_ip, lidar_port, lidar_baud);  // settings only; started once the network is up
    hexa_log::log(hexa_log::kNotice, "boot: hexacam %s %s (reset: %s), sensor %s, frame %s, quality %d, fps %d%s%s, wifi buffers in %s", __DATE__,
                  __TIME__, reset_name(esp_reset_reason()), cam_ok ? sensor_name : "FAILED", frame_name(frame), quality, fps, vflip ? ", vflip" : "",
                  hmirror ? ", hmirror" : "", wifi_mem::internal_first ? "internal RAM" : "PSRAM");
}

void loop() {
    if (restart_at && static_cast<int32_t>(millis() - restart_at) >= 0) ESP.restart();
    const bool up = WiFi.status() == WL_CONNECTED;
    trial_loop(up);
    static bool was_up = false;
    if (up && !was_up) {  // every join: the driver may restore its defaults (the robot's link.cpp does the same)
        esp_wifi_set_ps(WIFI_PS_NONE);
        apply_tx_power();
    }
    was_up = up;
    hexa_log::loop(up);
    if (up && !net_started) {
        net_started = true;
        http_begin();
        if (!ota_pw.isEmpty()) {
            ArduinoOTA.setPort(3232);
            ArduinoOTA.setPassword(ota_pw.c_str());
            ArduinoOTA.setMdnsEnabled(false);
            ArduinoOTA.setHostname(name.c_str());
            ArduinoOTA.onStart([]() {
                lidar_fwd::pause();  // nothing on the radio but the update
                hexa_log::log(hexa_log::kNotice, "ota: update started");
            });
            ArduinoOTA.onEnd([]() {
                hexa_log::log(hexa_log::kNotice, "ota: update written, rebooting");
                delay(100);
            });
            ArduinoOTA.onError([](ota_error_t e) { hexa_log::log(hexa_log::kErr, "ota: failed (error %d)", (int)e); });
            ArduinoOTA.begin();
        }
        hexa_log::log(hexa_log::kNotice, "wifi: up, %s, rssi %d dBm; stream http://%s:81/stream",
                      WiFi.localIP().toString().c_str(), static_cast<int>(WiFi.RSSI()),
                      WiFi.localIP().toString().c_str());
    }
    if (net_started) {
        lidar_fwd::start();
        lidar_fwd::report();
    }
    if (net_started && !ota_pw.isEmpty()) ArduinoOTA.handle();
    // Once the network is up: the boot's entries wait in irq_watch's ring (32), not in syslog's 12-line backlog.
    if (up) irq_watch::report([](const char* text) { hexa_log::log(hexa_log::kInfo, "%s", text); });
    static uint32_t last = 0, f0 = 0, b0 = 0;
    if (millis() - last > 30000) {
        const float dt = (millis() - last) / 1000.0f;
        last = millis();
        measured_fps = (frames_sent - f0) / dt;
        measured_kbps = (bytes_sent - b0) * 8 / 1000.0f / dt;
        f0 = frames_sent;
        b0 = bytes_sent;
        hexa_log::log(hexa_log::kInfo, "health: %s rssi %d dBm  stream %u client(s) %.1f fps %.0f kbit/s  fails %u  heap %u (min %u) psram %u  wbuf %u/%u  uart %s %u B err %u/%u/%u  lidar %u B %u pkt err %u  flash ops %u max %u us",
                      up ? "up" : "DOWN", static_cast<int>(WiFi.RSSI()), static_cast<unsigned>(stream_clients),
                      measured_fps, measured_kbps, static_cast<unsigned>(capture_fails),
                      static_cast<unsigned>(ESP.getFreeHeap()),
                      static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
                      static_cast<unsigned>(ESP.getFreePsram()), static_cast<unsigned>(wifi_mem::to_internal),
                      static_cast<unsigned>(wifi_mem::to_psram), uart_mode.c_str(), static_cast<unsigned>(uart_bytes),
                      static_cast<unsigned>(uart_err_ovf), static_cast<unsigned>(uart_err_full),
                      static_cast<unsigned>(uart_err_frame), static_cast<unsigned>(lidar_fwd::bytes()),
                      static_cast<unsigned>(lidar_fwd::packets()), static_cast<unsigned>(lidar_fwd::errors()),
                      static_cast<unsigned>(irq_watch::flash_ops()),
                      static_cast<unsigned>(irq_watch::flash_max_us()));
    }
    delay(5);
}
