// hexacam: the miniHexa's camera. An ESP32-S3-CAM board (N16R8, OV3660) joins the robot's Wi-Fi and
// serves its JPEG frames; the host's ROS bridge (hexapod_bringup camera_bridge) turns them into
// camera/image_raw/compressed. Nothing is decoded or detected here: the board only captures and sends.
//
//   http://<ip>:81/stream    MJPEG (multipart/x-mixed-replace), paced to the `fps` setting; one client
//   http://<ip>/capture      one JPEG
//   http://<ip>/status       JSON: settings, measured rate and size, Wi-Fi, memory
//
// Settings in NVS namespace `hexacam` (tools/make_cam_nvs.py): ssid, psk (the robot's network), ota_pw
// (OTA on 3232, none: no OTA), name (hostname, default hexacam), frame (qqvga, qvga, vga, svga, xga, hd;
// default qvga), quality (JPEG 10-40, lower is better; default 12), fps (1-30; default 10), vflip,
// hmirror (1: flip the image for the mount), syslog_ip, syslog_port (the robot firmware's syslog).
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>

#include "syslog.h"

namespace {

Preferences prefs;
String ssid, psk, ota_pw, name = "hexacam";
framesize_t frame = FRAMESIZE_QVGA;
int quality = 12, fps = 10;
bool vflip = false, hmirror = false;
const char* sensor_name = "none";
bool cam_ok = false, net_started = false;

// What the stream has delivered, for /status and the health line.
volatile uint32_t frames_sent = 0, bytes_sent = 0, stream_clients = 0, capture_fails = 0;

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
    c.xclk_freq_hz = 20000000;
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
    char head[160];
    snprintf(head, sizeof head, "multipart/x-mixed-replace;boundary=%s", kBoundary);
    httpd_resp_set_type(req, head);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ++stream_clients;
    hexa_log::log(hexa_log::kInfo, "stream: client up (%u)", static_cast<unsigned>(stream_clients));
    const uint32_t period_ms = 1000 / fps;
    uint32_t next = millis();
    esp_err_t r = ESP_OK;
    while (r == ESP_OK) {
        const int32_t wait = static_cast<int32_t>(next - millis());
        if (wait > 0) delay(wait);
        next += period_ms;
        if (static_cast<int32_t>(millis() - next) > static_cast<int32_t>(period_ms)) next = millis();  // no burst
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
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
    char j[512];
    sensor_t* s = cam_ok ? esp_camera_sensor_get() : nullptr;
    snprintf(j, sizeof j,
             "{\"name\":\"%s\",\"sensor\":\"%s\",\"frame\":\"%s\",\"quality\":%d,\"fps_cap\":%d,\"fps\":%.1f,"
             "\"kbps\":%.0f,\"clients\":%u,\"frames\":%u,\"capture_fails\":%u,\"vflip\":%d,\"hmirror\":%d,"
             "\"rssi\":%d,\"uptime_s\":%u,\"heap\":%u,\"psram_free\":%u}",
             name.c_str(), sensor_name, frame_name(s ? static_cast<framesize_t>(s->status.framesize) : frame), quality,
             fps, measured_fps, measured_kbps, static_cast<unsigned>(stream_clients),
             static_cast<unsigned>(frames_sent), static_cast<unsigned>(capture_fails), vflip, hmirror,
             static_cast<int>(WiFi.RSSI()), static_cast<unsigned>(millis() / 1000),
             static_cast<unsigned>(ESP.getFreeHeap()), static_cast<unsigned>(ESP.getFreePsram()));
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
    if (ssid.isEmpty()) {
        hexa_log::log(hexa_log::kErr, "no ssid in NVS (hexacam): nothing to join (tools/make_cam_nvs.py)");
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(name.c_str());
    WiFi.setSleep(false);  // modem sleep adds 100+ ms to every frame
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid.c_str(), psk.c_str());
    hexa_log::log(hexa_log::kNotice, "boot: hexacam %s %s, sensor %s, frame %s, quality %d, fps %d%s%s", __DATE__,
                  __TIME__, cam_ok ? sensor_name : "FAILED", frame_name(frame), quality, fps, vflip ? ", vflip" : "",
                  hmirror ? ", hmirror" : "");
}

void loop() {
    const bool up = WiFi.status() == WL_CONNECTED;
    hexa_log::loop(up);
    if (up && !net_started) {
        net_started = true;
        http_begin();
        if (!ota_pw.isEmpty()) {
            ArduinoOTA.setPort(3232);
            ArduinoOTA.setPassword(ota_pw.c_str());
            ArduinoOTA.setMdnsEnabled(false);
            ArduinoOTA.setHostname(name.c_str());
            ArduinoOTA.onStart([]() { hexa_log::log(hexa_log::kNotice, "ota: update started"); });
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
    if (net_started && !ota_pw.isEmpty()) ArduinoOTA.handle();
    static uint32_t last = 0, f0 = 0, b0 = 0;
    if (millis() - last > 30000) {
        const float dt = (millis() - last) / 1000.0f;
        last = millis();
        measured_fps = (frames_sent - f0) / dt;
        measured_kbps = (bytes_sent - b0) * 8 / 1000.0f / dt;
        f0 = frames_sent;
        b0 = bytes_sent;
        hexa_log::log(hexa_log::kInfo, "health: %s rssi %d dBm  stream %u client(s) %.1f fps %.0f kbit/s  fails %u  heap %u psram %u",
                      up ? "up" : "DOWN", static_cast<int>(WiFi.RSSI()), static_cast<unsigned>(stream_clients),
                      measured_fps, measured_kbps, static_cast<unsigned>(capture_fails),
                      static_cast<unsigned>(ESP.getFreeHeap()), static_cast<unsigned>(ESP.getFreePsram()));
    }
    delay(5);
}
