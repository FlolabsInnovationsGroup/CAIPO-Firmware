#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

#define MIC_SCK    7
#define MIC_WS     8
#define MIC_DOUT   9

#define TOUCH_PIN  2

#define SPK_BCLK   4
#define SPK_WS     5
#define SPK_DIN    6

#define SAMPLE_RATE 16000
#define CHUNK       256

// ESP -> Python
#define OUT_STATE      0x01
#define OUT_AUDIO      0x02
#define OUT_PLAY_ACK   0x03

// Python -> ESP
#define IN_PLAY_START  0x10
#define IN_PLAY_AUDIO  0x11
#define IN_PLAY_END    0x12

// =====================
// Playback tuning
// =====================
// Patlama varsa önce PLAY_GAIN'i düşür:
// 0.38 -> 0.32 -> 0.28
const float PLAY_GAIN   = 0.22f;

// Plosive / low-frequency patlamaları azaltır
// 0.95 daha agresif, 0.98 daha yumuşak
const float HP_A        = 0.97f;

// Soft limiter eşiği
const float LIMIT_THR   = 9000.0f;

// Eşik üstündeki artış oranı
// 0.25 daha yumuşak, 0.15 daha sert limiter
const float LIMIT_RATIO = 0.20f;

enum AppState {
  IDLE,
  RECORDING,
  WAIT_PLAY,
  PLAYING
};

volatile AppState state = IDLE;

volatile bool touched = false;
volatile uint32_t last_touch = 0;

int32_t raw[CHUNK];
int16_t pcm[CHUNK];

uint8_t rx_payload[CHUNK * 2];
int16_t play_mono[CHUNK];
int16_t play_stereo[CHUNK * 2];

float hp_x_prev = 0.0f;
float hp_y_prev = 0.0f;

enum RxParseState {
  RX_WAIT_FF,
  RX_WAIT_FE,
  RX_TYPE,
  RX_LEN_H,
  RX_LEN_L,
  RX_PAYLOAD
};

RxParseState rx_state = RX_WAIT_FF;
uint8_t  rx_type = 0;
uint16_t rx_len  = 0;
uint16_t rx_pos  = 0;

void IRAM_ATTR isr_touch() {
  uint32_t now = millis();
  if (now - last_touch > 300) {
    touched = true;
    last_touch = now;
  }
}

void resetPlaybackFilter() {
  hp_x_prev = 0.0f;
  hp_y_prev = 0.0f;
}

int16_t processPlaybackSample(int16_t in) {
  float x = (float)in;

  // DC blocker / high-pass
  float y = x - hp_x_prev + HP_A * hp_y_prev;
  hp_x_prev = x;
  hp_y_prev = y;

  // overall gain reduction
  y *= PLAY_GAIN;

  // soft limiter
  if (y > LIMIT_THR) {
    y = LIMIT_THR + (y - LIMIT_THR) * LIMIT_RATIO;
  } else if (y < -LIMIT_THR) {
    y = -LIMIT_THR + (y + LIMIT_THR) * LIMIT_RATIO;
  }

  // hard clamp
  if (y > 32767.0f) y = 32767.0f;
  if (y < -32768.0f) y = -32768.0f;

  return (int16_t)y;
}

void pkt_write(uint8_t type, const void* data, uint16_t len) {
  uint8_t hdr[5] = {0xFF, 0xFE, type, uint8_t(len >> 8), uint8_t(len)};
  Serial.write(hdr, 5);
  if (len > 0) {
    Serial.write((const uint8_t*)data, len);
  }
}

void send_state(const char* s) {
  pkt_write(OUT_STATE, s, strlen(s));
}

void send_ack() {
  pkt_write(OUT_PLAY_ACK, nullptr, 0);
}

void mic_init() {
  i2s_config_t c = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
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

void spk_init() {
  i2s_config_t c = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
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

void handle_play_packet(uint8_t type, uint8_t* data, uint16_t len) {
  if (type == IN_PLAY_START) {
    if (state == WAIT_PLAY || state == IDLE) {
      state = PLAYING;
      resetPlaybackFilter();
      i2s_zero_dma_buffer(I2S_NUM_1);
      send_state("PLAYING");
      send_ack();
    }
    return;
  }

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

  if (type == IN_PLAY_AUDIO) {
    if (state != PLAYING) return;
    if (len == 0) return;
    if (len > sizeof(play_mono)) return;
    if (len & 0x01) return;

    memcpy(play_mono, data, len);

    int samples = len / 2;
    for (int i = 0; i < samples; i++) {
      int16_t s = processPlaybackSample(play_mono[i]);
      play_stereo[2 * i]     = s;
      play_stereo[2 * i + 1] = s;
    }

    size_t bytes_written = 0;
    i2s_write(I2S_NUM_1, play_stereo, samples * 4, &bytes_written, portMAX_DELAY);
    send_ack();
  }
}

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
        else rx_state = RX_WAIT_FF;
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
          handle_play_packet(rx_type, rx_payload, 0);
          rx_state = RX_WAIT_FF;
        } else if (rx_len > sizeof(rx_payload)) {
          rx_state = RX_WAIT_FF;
        } else {
          rx_state = RX_PAYLOAD;
        }
        break;

      case RX_PAYLOAD:
        rx_payload[rx_pos++] = b;
        if (rx_pos >= rx_len) {
          handle_play_packet(rx_type, rx_payload, rx_len);
          rx_state = RX_WAIT_FF;
        }
        break;
    }
  }
}

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(460800);

  pinMode(TOUCH_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), isr_touch, RISING);

  mic_init();
  spk_init();

  send_state("IDLE");
}

void loop() {
  serial_rx_task();

  if (state == PLAYING) {
    if (touched) touched = false;
    return;
  }

  if (state == IDLE && touched) {
    touched = false;
    state = RECORDING;
    send_state("RECORDING");

    for (int k = 0; k < 2; k++) {
      size_t bytes_read = 0;
      i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes_read, portMAX_DELAY);
    }
  }

  if (state == RECORDING) {
    if (touched) {
      touched = false;
      state = WAIT_PLAY;
      send_state("STOP");
      delay(20);
      return;
    }

    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes_read, portMAX_DELAY);

    int n = bytes_read / 4;
    if (n <= 0) return;
    if (n > CHUNK) n = CHUNK;

    for (int i = 0; i < n; i++) {
      pcm[i] = raw[i] >> 8;
    }

    pkt_write(OUT_AUDIO, pcm, n * 2);
  }

  if (state == WAIT_PLAY) {
    delay(1);
  }
}