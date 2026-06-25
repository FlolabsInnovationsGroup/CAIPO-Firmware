#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ======================================================================
//  Push-to-talk voice device firmware (ESP32-S3 class chip)
//
//  Hardware:
//    - I2S microphone (e.g. INMP441)      -> I2S_NUM_0  (RX / capture)
//    - I2S speaker amp  (e.g. MAX98357A)  -> I2S_NUM_1  (TX / playback)
//    - A touch / push button              -> GPIO 2
//
//  It talks to a Python host over USB serial using a small framed
//  protocol. Record on first touch, stop on second touch, then the
//  host streams synthesized audio back to be played on the speaker.
// ======================================================================

// ---- Microphone (I2S input) pins ----
#define MIC_SCK    7   // bit clock
#define MIC_WS     8   // word select / left-right clock
#define MIC_DOUT   9   // data out of the mic, into the ESP

// ---- Push / touch button ----
#define TOUCH_PIN  2

// ---- Speaker amplifier (I2S output) pins ----
#define SPK_BCLK   4   // bit clock
#define SPK_WS     5   // word select / left-right clock
#define SPK_DIN    6   // data into the amp, out of the ESP

#define SAMPLE_RATE 16000  // 16 kHz, both capture and playback
#define CHUNK       256    // samples processed per block

// ---- Packet types: ESP -> Python ----
#define OUT_STATE      0x01   // a text status string ("IDLE", "RECORDING"...)
#define OUT_AUDIO      0x02   // a block of captured PCM audio
#define OUT_PLAY_ACK   0x03   // "I handled your playback packet"

// ---- Packet types: Python -> ESP ----
#define IN_PLAY_START  0x10   // start of a playback stream
#define IN_PLAY_AUDIO  0x11   // one block of audio to play
#define IN_PLAY_END    0x12   // end of the playback stream

// =====================
// Playback tuning
// =====================
// If you hear clipping / popping, lower PLAY_GAIN first:
// 0.38 -> 0.32 -> 0.28
const float PLAY_GAIN   = 0.22f;

// Reduces plosive / low-frequency pops (DC-blocker pole).
// 0.95 = more aggressive, 0.98 = softer
const float HP_A        = 0.97f;

// Soft limiter threshold
const float LIMIT_THR   = 9000.0f;

// How much signal above the threshold is allowed to grow.
// 0.25 = softer, 0.15 = harder limiter
const float LIMIT_RATIO = 0.20f;

// The four states the device can be in.
enum AppState {
  IDLE,        // doing nothing, waiting for a touch
  RECORDING,   // capturing mic audio and streaming it out
  WAIT_PLAY,   // recording stopped, waiting for the host to send audio
  PLAYING      // playing audio received from the host
};

volatile AppState state = IDLE;

// Touch button state, shared with the interrupt handler.
volatile bool touched = false;
volatile uint32_t last_touch = 0;

// Mic capture buffers.
int32_t raw[CHUNK];   // raw 32-bit samples straight from the I2S mic
int16_t pcm[CHUNK];   // downshifted 16-bit PCM that we actually send

// Playback buffers.
uint8_t rx_payload[CHUNK * 2];   // raw bytes of an incoming packet payload
int16_t play_mono[CHUNK];        // incoming mono audio
int16_t play_stereo[CHUNK * 2];  // duplicated into L/R for the stereo amp

// DC-blocker filter memory (previous input and output samples).
float hp_x_prev = 0.0f;
float hp_y_prev = 0.0f;

// State machine for parsing the incoming serial byte stream.
enum RxParseState {
  RX_WAIT_FF,   // looking for the first sync byte 0xFF
  RX_WAIT_FE,   // looking for the second sync byte 0xFE
  RX_TYPE,      // next byte is the packet type
  RX_LEN_H,     // high byte of the length
  RX_LEN_L,     // low byte of the length
  RX_PAYLOAD    // reading len bytes of payload
};

RxParseState rx_state = RX_WAIT_FF;
uint8_t  rx_type = 0;
uint16_t rx_len  = 0;
uint16_t rx_pos  = 0;

// Button interrupt. Runs on the rising edge with a 300 ms debounce so
// one physical press only registers once.
void IRAM_ATTR isr_touch() {
  uint32_t now = millis();
  if (now - last_touch > 300) {
    touched = true;
    last_touch = now;
  }
}

// Clear the filter history so each playback starts clean.
void resetPlaybackFilter() {
  hp_x_prev = 0.0f;
  hp_y_prev = 0.0f;
}

// Per-sample playback conditioning: high-pass, gain, soft limit, clamp.
int16_t processPlaybackSample(int16_t in) {
  float x = (float)in;

  // DC blocker / high-pass: removes the constant offset and very low
  // rumble that causes speaker pops.
  float y = x - hp_x_prev + HP_A * hp_y_prev;
  hp_x_prev = x;
  hp_y_prev = y;

  // Overall volume reduction.
  y *= PLAY_GAIN;

  // Soft limiter: above the threshold, extra signal grows slowly
  // instead of slamming into the ceiling.
  if (y > LIMIT_THR) {
    y = LIMIT_THR + (y - LIMIT_THR) * LIMIT_RATIO;
  } else if (y < -LIMIT_THR) {
    y = -LIMIT_THR + (y + LIMIT_THR) * LIMIT_RATIO;
  }

  // Hard clamp to the valid 16-bit range as a final safety net.
  if (y > 32767.0f) y = 32767.0f;
  if (y < -32768.0f) y = -32768.0f;

  return (int16_t)y;
}

// Build and send one framed packet: 0xFF 0xFE [type] [len_hi] [len_lo] [payload].
void pkt_write(uint8_t type, const void* data, uint16_t len) {
  uint8_t hdr[5] = {0xFF, 0xFE, type, uint8_t(len >> 8), uint8_t(len)};
  Serial.write(hdr, 5);
  if (len > 0) {
    Serial.write((const uint8_t*)data, len);
  }
}

// Send a text status string to the host.
void send_state(const char* s) {
  pkt_write(OUT_STATE, s, strlen(s));
}

// Send an empty acknowledgement packet.
void send_ack() {
  pkt_write(OUT_PLAY_ACK, nullptr, 0);
}

// Configure the I2S peripheral for the microphone (receive only).
void mic_init() {
  i2s_config_t c = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,   // mic delivers 32-bit frames
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,    // single mic channel
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = CHUNK,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t p = {
    .bck_io_num = MIC_SCK,
    .ws_io_num = MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_DOUT
  };

  i2s_driver_install(I2S_NUM_0, &c, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &p);
  i2s_start(I2S_NUM_0);
}

// Configure the I2S peripheral for the speaker amp (transmit only).
void spk_init() {
  i2s_config_t c = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // stereo out
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = CHUNK,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t p = {
    .bck_io_num = SPK_BCLK,
    .ws_io_num = SPK_WS,
    .data_out_num = SPK_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_1, &c, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &p);
  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_start(I2S_NUM_1);
}

// React to a fully parsed packet coming from the host.
void handle_play_packet(uint8_t type, uint8_t* data, uint16_t len) {
  // Host signals the start of a playback stream.
  if (type == IN_PLAY_START) {
    if (state == WAIT_PLAY || state == IDLE) {
      state = PLAYING;
      resetPlaybackFilter();
      i2s_zero_dma_buffer(I2S_NUM_1);   // flush any stale audio
      send_state("PLAYING");
      send_ack();
    }
    return;
  }

  // Host signals the end of the stream.
  if (type == IN_PLAY_END) {
    if (state == PLAYING) {
      i2s_zero_dma_buffer(I2S_NUM_1);
      resetPlaybackFilter();
      state = IDLE;
      send_state("PLAY_DONE");
      send_state("IDLE");
      send_ack();
    }
    return;
  }

  // One block of audio to play.
  if (type == IN_PLAY_AUDIO) {
    if (state != PLAYING) return;          // ignore audio if not playing
    if (len == 0) return;
    if (len > sizeof(play_mono)) return;   // reject oversized blocks
    if (len & 0x01) return;                // payload must be whole 16-bit samples

    memcpy(play_mono, data, len);

    // Condition each sample and duplicate it into both stereo channels.
    int samples = len / 2;
    for (int i = 0; i < samples; i++) {
      int16_t s = processPlaybackSample(play_mono[i]);
      play_stereo[2 * i]     = s;   // left
      play_stereo[2 * i + 1] = s;   // right
    }

    // Push it to the amp. Blocks until the DMA buffer has room.
    size_t bytes_written = 0;
    i2s_write(I2S_NUM_1, play_stereo, samples * 4, &bytes_written, portMAX_DELAY);
    send_ack();
  }
}

// Drain the serial buffer one byte at a time and feed the parser.
void serial_rx_task() {
  while (Serial.available()) {
    int r = Serial.read();
    if (r < 0) break;
    uint8_t b = (uint8_t)r;

    switch (rx_state) {
      case RX_WAIT_FF:
        if (b == 0xFF) rx_state = RX_WAIT_FE;
        break;

      case RX_WAIT_FE:
        if (b == 0xFE) rx_state = RX_TYPE;
        else rx_state = RX_WAIT_FF;        // false start, resync
        break;

      case RX_TYPE:
        rx_type = b;
        rx_state = RX_LEN_H;
        break;

      case RX_LEN_H:
        rx_len = ((uint16_t)b << 8);
        rx_state = RX_LEN_L;
        break;

      case RX_LEN_L:
        rx_len |= b;
        rx_pos = 0;

        if (rx_len == 0) {
          // Zero-length packet (START / END): handle immediately.
          handle_play_packet(rx_type, rx_payload, 0);
          rx_state = RX_WAIT_FF;
        } else if (rx_len > sizeof(rx_payload)) {
          rx_state = RX_WAIT_FF;            // too big, drop and resync
        } else {
          rx_state = RX_PAYLOAD;
        }
        break;

      case RX_PAYLOAD:
        rx_payload[rx_pos++] = b;
        if (rx_pos >= rx_len) {             // full payload received
          handle_play_packet(rx_type, rx_payload, rx_len);
          rx_state = RX_WAIT_FF;
        }
        break;
    }
  }
}

void setup() {
  Serial.setRxBufferSize(2048);   // bigger buffer so audio bursts don't overflow
  Serial.begin(460800);

  pinMode(TOUCH_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), isr_touch, RISING);

  mic_init();
  spk_init();

  send_state("IDLE");
}

void loop() {
  // Always service incoming serial first.
  serial_rx_task();

  // While playing, ignore button presses and let handle_play_packet drive things.
  if (state == PLAYING) {
    if (touched) touched = false;   // swallow any press
    return;
  }

  // First touch while idle: start recording.
  if (state == IDLE && touched) {
    touched = false;
    state = RECORDING;
    send_state("RECORDING");

    // Throw away the first couple of mic blocks (settling / click).
    for (int k = 0; k < 2; k++) {
      size_t bytes_read = 0;
      i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes_read, portMAX_DELAY);
    }
  }

  // While recording, stream mic audio until the next touch.
  if (state == RECORDING) {
    // Second touch: stop and wait for the host's reply audio.
    if (touched) {
      touched = false;
      state = WAIT_PLAY;
      send_state("STOP");
      delay(20);
      return;
    }

    // Read one block from the mic.
    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes_read, portMAX_DELAY);

    int n = bytes_read / 4;     // 4 bytes per 32-bit sample
    if (n <= 0) return;
    if (n > CHUNK) n = CHUNK;

    // Convert 32-bit mic samples to 16-bit PCM (drop the low 8 bits).
    for (int i = 0; i < n; i++) {
      pcm[i] = raw[i] >> 8;
    }

    // Send the captured block to the host.
    pkt_write(OUT_AUDIO, pcm, n * 2);
  }

  // Waiting for playback packets. Nothing to do but yield briefly.
  if (state == WAIT_PLAY) {
    delay(1);
  }
}