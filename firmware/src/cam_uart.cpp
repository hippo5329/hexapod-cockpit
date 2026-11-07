#include "cam_uart.h"

#include "board_minihexa.h"

namespace cam_uart {
namespace {

uint32_t current_baud = 0;
// The driver's receive errors (onReceiveError), since begin().
volatile uint32_t err_fifo_ovf = 0, err_buf_full = 0, err_frame = 0, err_parity = 0, err_break = 0;

// The stream's blocks: kBlock bytes, a 4-byte header (0xA5 0x5A, block number LE) and a body the
// block number seeds (xorshift32), so a block is checked alone and a lost byte costs one block.
constexpr int kBlock = 256;

void fill_block(uint16_t n, uint8_t* b) {
    b[0] = 0xA5;
    b[1] = 0x5A;
    b[2] = static_cast<uint8_t>(n);
    b[3] = static_cast<uint8_t>(n >> 8);
    uint32_t s = 0x9E3779B9u ^ (n * 2654435761u);
    if (!s) s = 1;
    for (int i = 4; i < kBlock; ++i) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        b[i] = static_cast<uint8_t>(s);
    }
}

void drain(uint32_t quiet_ms) {
    uint8_t junk[64];
    uint32_t t = millis();
    while (millis() - t < quiet_ms) {
        if (Serial2.available() && Serial2.read(junk, sizeof junk) > 0) t = millis();
    }
}

}  // namespace

void begin(uint32_t baud) {
    if (!baud) baud = HEXA_CAM_UART_BAUD;
    if (current_baud) Serial2.end();
    // Large driver buffers (the hardware FIFO is a fixed 128 bytes): a loop held up by Wi-Fi or a
    // console print loses nothing until 16 KB (110 ms at 1.5 Mbaud) wait unread.
    Serial2.setRxBufferSize(16384);
    Serial2.setTxBufferSize(8192);
    Serial2.begin(baud, SERIAL_8N1, HEXA_CAM_UART_RX, HEXA_CAM_UART_TX);
    // The 128-byte FIFO is emptied into the ring buffer when this many bytes wait: Arduino's 120 leaves
    // 8 bytes (53 us at 1.5 Mbaud) for an interrupt held off by Wi-Fi. 32 leaves 96 (640 us).
    Serial2.setRxFIFOFull(32);
    Serial2.onReceiveError([](hardwareSerial_error_t e) {
        switch (e) {
            case UART_FIFO_OVF_ERROR: ++err_fifo_ovf; break;
            case UART_BUFFER_FULL_ERROR: ++err_buf_full; break;
            case UART_FRAME_ERROR: ++err_frame; break;
            case UART_PARITY_ERROR: ++err_parity; break;
            case UART_BREAK_ERROR: ++err_break; break;
            default: break;
        }
    });
    current_baud = baud;
}

void test(Print& out, float seconds, uint32_t baud) {
    if (!baud) baud = HEXA_CAM_UART_BAUD;
    if (baud != current_baud) begin(baud);
    drain(50);
    err_fifo_ovf = err_buf_full = err_frame = err_parity = err_break = 0;
    out.printf("uart: TX GPIO%d RX GPIO%d %u baud, echo test %.1f s\n", HEXA_CAM_UART_TX, HEXA_CAM_UART_RX,
               static_cast<unsigned>(baud), seconds);
    // Round trips: one byte out, the same byte back.
    uint32_t rtt_min = UINT32_MAX, rtt_max = 0, rtt_sum = 0;
    int got = 0;
    for (int i = 0; i < 20; ++i) {
        const uint8_t b = static_cast<uint8_t>(0x30 + i);
        const uint32_t t0 = micros();
        Serial2.write(b);
        int r = -1;
        while (micros() - t0 < 20000 && (r = Serial2.read()) < 0) {
        }
        const uint32_t dt = micros() - t0;
        if (r == b) {
            ++got;
            rtt_min = min(rtt_min, dt);
            rtt_max = max(rtt_max, dt);
            rtt_sum += dt;
        }
        drain(2);
    }
    if (!got) {
        out.println("uart: FAIL -- no byte came back in 20 ms (20 tries): wiring, the camera's echo, or the baud");
        return;
    }
    out.printf("uart: round trip %d/20, %u-%u us (mean %u)\n", got, static_cast<unsigned>(rtt_min),
               static_cast<unsigned>(rtt_max), static_cast<unsigned>(rtt_sum / got));
    // The stream: numbered blocks as fast as the TX buffer takes them, at most kInFlight bytes
    // unanswered; each block coming back is found by its header and checked whole.
    constexpr uint32_t kInFlight = 6144;
    static uint8_t out_blk[kBlock], want[kBlock], in[kBlock], rbuf[512];
    uint16_t next_out = 0;
    int out_pos = kBlock;  // nothing pending
    uint32_t sent = 0, recv = 0, good = 0, bad = 0, skipped = 0, rx_peak = 0;
    int in_pos = 0;
    uint16_t expect = 0;
    const uint32_t t0 = millis(), dur = static_cast<uint32_t>(seconds * 1000);
    uint32_t last_feed = t0;
    auto take = [&](const uint8_t* p, int n) {
        for (int i = 0; i < n; ++i) {
            const uint8_t c = p[i];
            ++recv;
            if (in_pos == 0 && c != 0xA5) { ++skipped; continue; }
            if (in_pos == 1 && c != 0x5A) { in_pos = c == 0xA5 ? 1 : 0; ++skipped; continue; }
            in[in_pos++] = c;
            if (in_pos < kBlock) continue;
            in_pos = 0;
            const uint16_t num = in[2] | (in[3] << 8);
            fill_block(num, want);
            if (!memcmp(in, want, kBlock) && num == expect) ++good;
            else ++bad;
            expect = num + 1;
        }
    };
    bool sending = true;
    uint32_t quiet = 0;
    while (true) {
        const uint32_t now = millis();
        if (sending && now - t0 >= dur) {
            sending = false;
            quiet = now;
        }
        if (sending && sent - recv < kInFlight) {
            if (out_pos == kBlock) {
                fill_block(next_out++, out_blk);
                out_pos = 0;
            }
            const int room = Serial2.availableForWrite();
            if (room > 0) {
                const int n = min(room, kBlock - out_pos);
                const int w = Serial2.write(out_blk + out_pos, n);
                out_pos += w;
                sent += w;
            }
        }
        const int a = Serial2.available();
        rx_peak = max(rx_peak, static_cast<uint32_t>(a));
        if (a > 0) take(rbuf, Serial2.read(rbuf, min(a, static_cast<int>(sizeof rbuf))));
        if (!sending) {
            if (a > 0) quiet = now;
            else if (now - quiet > 100) break;
        }
        if (now - last_feed > 500) {
            last_feed = now;
            feedLoopWDT();
        }
    }
    const float s = (millis() - t0) / 1000.0f;
    const float wire = baud / 10.0f;  // 8N1: 10 bits a byte
    const uint32_t blocks = sent / kBlock;
    out.printf("uart: %u blocks of %d B out, back good %u bad %u missing %d; bytes out %u back %u (lost %d, %u off-block)\n",
               static_cast<unsigned>(blocks), kBlock, static_cast<unsigned>(good), static_cast<unsigned>(bad),
               static_cast<int>(blocks - good - bad), static_cast<unsigned>(sent), static_cast<unsigned>(recv),
               static_cast<int>(sent - recv), static_cast<unsigned>(skipped));
    out.printf("uart: %.0f B/s each way (%.0f%% of the wire's %.0f)  rx buffer peak %u  robot rx errors: fifo overflow %u, buffer full %u, frame %u, parity %u, break %u\n",
               recv / s, 100 * recv / s / wire, wire, static_cast<unsigned>(rx_peak), static_cast<unsigned>(err_fifo_ovf),
               static_cast<unsigned>(err_buf_full), static_cast<unsigned>(err_frame), static_cast<unsigned>(err_parity),
               static_cast<unsigned>(err_break));
    out.println(bad == 0 && good == blocks && sent == recv ? "uart: PASS" : "uart: FAIL");
}

}  // namespace cam_uart
