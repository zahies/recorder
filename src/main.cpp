#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Pin definitions ──────────────────────────────────────────────
#define I2S_BCK_PIN   5   // SCK / BCLK
#define I2S_WS_PIN    6   // WS  / LRCK
#define I2S_DATA_PIN  7   // SD  / Data Out
#define BUTTON_PIN    0   // PRG button on Heltec V3
#define LED_PIN       35  // White LED on Heltec V3

// ── Audio settings ───────────────────────────────────────────────
#define SAMPLE_RATE     16000
#define I2S_PORT        I2S_NUM_0
#define DMA_BUF_COUNT   8
#define DMA_BUF_LEN     256

// ── WiFi AP settings ────────────────────────────────────────────
static const char *AP_SSID = "ESP32-Recorder";
static const char *AP_PASS = "12345678";

// ── Audio buffer ────────────────────────────────────────────────
// 16 kHz × 2 bytes × 1 ch = 32 KB/s
// PSRAM: up to ~1.8 MB on 2 MB chip.  Internal RAM fallback: ~5 s.
#define PSRAM_AUDIO_BYTES (1800 * 1024)
#define FALLBACK_AUDIO_BYTES (160 * 1024)
static uint8_t *audioBuffer = nullptr;
static size_t maxAudioBytes = 0;
static volatile size_t audioBytes = 0;

// ── State ────────────────────────────────────────────────────────
static volatile bool recording  = false;
static volatile bool fileReady  = false;

// ── Button debounce ──────────────────────────────────────────────
static volatile unsigned long lastButtonPress = 0;
#define DEBOUNCE_MS 200

// ── Web server ───────────────────────────────────────────────────
WebServer server(80);

// ── WAV header helper ────────────────────────────────────────────
static void writeWavHeader(uint8_t *buf, uint32_t dataSize) {
    uint32_t fileSize = 36 + dataSize;
    uint16_t numChannels = 1;
    uint32_t sampleRate = SAMPLE_RATE;
    uint16_t bitsPerSample = 16;
    uint32_t byteRate = sampleRate * numChannels * bitsPerSample / 8;
    uint16_t blockAlign = numChannels * bitsPerSample / 8;

    memcpy(buf, "RIFF", 4);
    memcpy(buf + 4, &fileSize, 4);
    memcpy(buf + 8, "WAVE", 4);

    memcpy(buf + 12, "fmt ", 4);
    uint32_t fmtSize = 16;
    memcpy(buf + 16, &fmtSize, 4);
    uint16_t audioFmt = 1;
    memcpy(buf + 20, &audioFmt, 2);
    memcpy(buf + 22, &numChannels, 2);
    memcpy(buf + 24, &sampleRate, 4);
    memcpy(buf + 28, &byteRate, 4);
    memcpy(buf + 32, &blockAlign, 2);
    memcpy(buf + 34, &bitsPerSample, 2);

    memcpy(buf + 36, "data", 4);
    memcpy(buf + 40, &dataSize, 4);
}

// ── I2S setup ────────────────────────────────────────────────────
static void i2sInit() {
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = SAMPLE_RATE;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = DMA_BUF_COUNT;
    i2s_config.dma_buf_len = DMA_BUF_LEN;
    i2s_config.use_apll = false;
    i2s_config.fixed_mclk = 0;

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {};
    pin_config.bck_io_num = I2S_BCK_PIN;
    pin_config.ws_io_num = I2S_WS_PIN;
    pin_config.data_in_num = I2S_DATA_PIN;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;

    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
}

// ── Recording task (runs on core 1) ─────────────────────────────
static void recordingTask(void *param) {
    const size_t readBufSamples = 256;
    int32_t readBuf[readBufSamples];
    size_t bytesRead;

    for (;;) {
        if (recording) {
            esp_err_t err = i2s_read(I2S_PORT, readBuf, sizeof(readBuf), &bytesRead, portMAX_DELAY);
            if (err == ESP_OK && bytesRead > 0) {
                size_t samplesRead = bytesRead / sizeof(int32_t);
                for (size_t i = 0; i < samplesRead; i++) {
                    if (audioBytes + 2 > maxAudioBytes) {
                        recording = false;
                        fileReady = true;
                        break;
                    }
                    int16_t sample = (int16_t)(readBuf[i] >> 16);
                    memcpy(audioBuffer + audioBytes, &sample, 2);
                    audioBytes += 2;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ── Button ISR ───────────────────────────────────────────────────
static void IRAM_ATTR buttonISR() {
    unsigned long now = millis();
    if (now - lastButtonPress < DEBOUNCE_MS) return;
    lastButtonPress = now;

    if (!recording && !fileReady) {
        audioBytes = 0;
        recording = true;
    } else if (recording) {
        recording = false;
        fileReady = true;
    } else if (fileReady) {
        audioBytes = 0;
        fileReady = false;
        recording = true;
    }
}

// ── HTML page ────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Recorder</title>
<style>
  body { font-family: sans-serif; text-align: center; padding: 2em;
         background: #1a1a2e; color: #eee; }
  h1 { color: #e94560; }
  #status { font-size: 1.4em; margin: 1.5em 0; }
  .waiting    { color: #aaa; }
  .recording  { color: #e94560; }
  .ready      { color: #0f3460; }
  a.dl { display: inline-block; margin-top: 1em; padding: 0.8em 2em;
         background: #e94560; color: #fff; text-decoration: none;
         border-radius: 8px; font-size: 1.1em; }
  a.dl:hover { background: #c73550; }
  .hidden { display: none; }
</style>
</head>
<body>
<h1>ESP32 Sound Recorder</h1>
<div id="status" class="waiting">Waiting — press PRG button to record</div>
<a id="download" class="dl hidden" href="/download">Download WAV</a>
<script>
  function poll() {
    fetch('/status').then(r => r.json()).then(d => {
      const el = document.getElementById('status');
      const dl = document.getElementById('download');
      if (d.recording) {
        el.textContent = 'Recording\u2026';
        el.className = 'recording';
        dl.classList.add('hidden');
      } else if (d.file_ready) {
        el.textContent = 'Done — download your recording';
        el.className = 'ready';
        dl.classList.remove('hidden');
      } else {
        el.textContent = 'Waiting — press PRG button to record';
        el.className = 'waiting';
        dl.classList.add('hidden');
      }
    }).catch(() => {});
  }
  setInterval(poll, 500);
  poll();
</script>
</body>
</html>
)rawliteral";

// ── Web server handlers ──────────────────────────────────────────
static void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

static void handleStatus() {
    String json = "{\"recording\":";
    json += recording ? "true" : "false";
    json += ",\"file_ready\":";
    json += fileReady ? "true" : "false";
    json += "}";
    server.send(200, "application/json", json);
}

static void handleDownload() {
    if (!fileReady || audioBytes == 0) {
        server.send(404, "text/plain", "No recording available");
        return;
    }

    const size_t wavHeaderSize = 44;
    size_t totalSize = wavHeaderSize + audioBytes;

    uint8_t wavHeader[44];
    writeWavHeader(wavHeader, audioBytes);

    WiFiClient client = server.client();
    client.printf("HTTP/1.1 200 OK\r\n");
    client.printf("Content-Type: audio/wav\r\n");
    client.printf("Content-Disposition: attachment; filename=\"recording.wav\"\r\n");
    client.printf("Content-Length: %u\r\n", totalSize);
    client.printf("Connection: close\r\n\r\n");

    client.write(wavHeader, wavHeaderSize);

    const size_t chunkSize = 4096;
    size_t sent = 0;
    while (sent < audioBytes) {
        size_t toSend = audioBytes - sent;
        if (toSend > chunkSize) toSend = chunkSize;
        client.write(audioBuffer + sent, toSend);
        sent += toSend;
    }
}

// ── Setup ────────────────────────────────────────────────────────
void setup() {
    // LED indicator — turn on immediately to show firmware is alive
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.begin(115200);
    // Long delay so USB-Serial/JTAG has time to reconnect on host
    delay(3000);

    Serial.println("\n=== ESP32 Sound Recorder ===");
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM size: %u bytes\n", ESP.getPsramSize());

    // Allocate audio buffer — prefer PSRAM, fall back to internal heap
    if (ESP.getPsramSize() > 0) {
        maxAudioBytes = PSRAM_AUDIO_BYTES;
        if (maxAudioBytes > ESP.getFreePsram()) {
            maxAudioBytes = ESP.getFreePsram() - 4096;
        }
        audioBuffer = (uint8_t *)ps_malloc(maxAudioBytes);
        Serial.printf("PSRAM buffer: %u bytes (~%u s)\n",
                       maxAudioBytes, maxAudioBytes / 32000);
    }
    if (!audioBuffer) {
        maxAudioBytes = FALLBACK_AUDIO_BYTES;
        audioBuffer = (uint8_t *)malloc(maxAudioBytes);
        Serial.printf("Internal RAM buffer: %u bytes (~%u s)\n",
                       maxAudioBytes, maxAudioBytes / 32000);
    }
    if (!audioBuffer) {
        Serial.println("ERROR: buffer allocation failed!");
        // Blink LED rapidly to indicate error
        for (;;) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    // Button setup
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

    // I2S setup
    i2sInit();
    Serial.println("I2S initialized");

    // WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(100);
    Serial.printf("WiFi AP: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    // Web server routes
    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.on("/download", handleDownload);
    server.begin();
    Serial.println("Web server started on port 80");

    // Recording task on core 1
    xTaskCreatePinnedToCore(recordingTask, "rec", 4096, NULL, 1, NULL, 1);
    Serial.println("Ready — press PRG button to start recording");

    // Turn LED off to indicate setup is complete
    digitalWrite(LED_PIN, LOW);
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    // Blink LED while recording
    static unsigned long lastBlink = 0;
    if (recording && millis() - lastBlink > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    } else if (!recording) {
        digitalWrite(LED_PIN, LOW);
    }
}
