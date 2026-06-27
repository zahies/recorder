#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Pin definitions ──────────────────────────────────────────────
#define I2S_BCK_PIN   4   // SCK / BCLK
#define I2S_WS_PIN    5   // LRCLK
#define I2S_DATA_PIN  6   // SD  / Data Out
#define BUTTON_PIN    0   // PRG button on Heltec V3
#define LED_PIN       35  // White LED on Heltec V3

// ── Audio settings ───────────────────────────────────────────────
#define SAMPLE_RATE     16000
#define I2S_PORT        I2S_NUM_0
#define DMA_BUF_COUNT   8
#define DMA_BUF_LEN     256

// ── WiFi AP settings ────────────────────────────────────────────
static const char *AP_SSID = "ESP32-Recorder";
static const char AP_PASS[] __attribute__((aligned(4))) = "%%WIFI_PASS_PLACEHOLDER%%\0\0\0\0\0\0";

// ── Audio buffer ────────────────────────────────────────────────
// 16 kHz × 2 bytes × 1 ch = 32 KB/s
// PSRAM: up to ~1.8 MB on 2 MB chip.  Internal RAM fallback: ~5 s.
#define PSRAM_AUDIO_BYTES (1800 * 1024)
#define FALLBACK_AUDIO_BYTES (160 * 1024)
static uint8_t *audioBuffer = nullptr;
static size_t maxAudioBytes = 0;
static volatile size_t audioBytes = 0;

// ── State machine ───────────────────────────────────────────────
enum State : uint8_t { STATE_IDLE, STATE_STREAMING, STATE_RECORDING, STATE_FILE_READY };
static volatile State currentState = STATE_IDLE;

// ── Streaming queue ─────────────────────────────────────────────
#define STREAM_CHUNK_SAMPLES 512
#define STREAM_CHUNK_BYTES   (STREAM_CHUNK_SAMPLES * 2)  // 512 samples x 2 bytes = 1024
#define STREAM_QUEUE_SLOTS   4
static QueueHandle_t streamQueue = NULL;

// ── Button debounce ──────────────────────────────────────────────
static volatile unsigned long lastButtonPress = 0;
#define DEBOUNCE_MS 200

// ── Web server ──────────────────────────────────────────────────
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
    uint32_t debugCounter = 0;

    // Streaming chunk accumulator
    int16_t streamChunk[STREAM_CHUNK_SAMPLES];
    size_t streamChunkPos = 0;

    for (;;) {
        State st = currentState;

        if (st == STATE_RECORDING) {
            esp_err_t err = i2s_read(I2S_PORT, readBuf, sizeof(readBuf), &bytesRead, portMAX_DELAY);
            if (err == ESP_OK && bytesRead > 0) {
                size_t samplesRead = bytesRead / sizeof(int32_t);

                debugCounter++;
                if (debugCounter % 32 == 1) {
                    Serial.printf("I2S: %u samples, raw[0]=0x%08X raw[1]=0x%08X raw[2]=0x%08X raw[3]=0x%08X\n",
                                  samplesRead, readBuf[0], readBuf[1], readBuf[2], readBuf[3]);
                }

                for (size_t i = 0; i < samplesRead; i++) {
                    if (audioBytes + 2 > maxAudioBytes) {
                        currentState = STATE_FILE_READY;
                        break;
                    }
                    int16_t sample = (int16_t)(readBuf[i] >> 14);
                    memcpy(audioBuffer + audioBytes, &sample, 2);
                    audioBytes += 2;
                }
            }
        } else if (st == STATE_STREAMING) {
            esp_err_t err = i2s_read(I2S_PORT, readBuf, sizeof(readBuf), &bytesRead, portMAX_DELAY);
            if (err == ESP_OK && bytesRead > 0) {
                size_t samplesRead = bytesRead / sizeof(int32_t);
                for (size_t i = 0; i < samplesRead; i++) {
                    int16_t sample = (int16_t)(readBuf[i] >> 14);
                    streamChunk[streamChunkPos++] = sample;
                    if (streamChunkPos >= STREAM_CHUNK_SAMPLES) {
                        xQueueSend(streamQueue, streamChunk, 0);
                        streamChunkPos = 0;
                    }
                }
            }
        } else {
            streamChunkPos = 0;
            debugCounter = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ── Button ISR ───────────────────────────────────────────────────
static void IRAM_ATTR buttonISR() {
    unsigned long now = millis();
    if (now - lastButtonPress < DEBOUNCE_MS) return;
    lastButtonPress = now;

    State st = currentState;
    if (st == STATE_IDLE || st == STATE_STREAMING) {
        audioBytes = 0;
        currentState = STATE_RECORDING;
    } else if (st == STATE_RECORDING) {
        currentState = STATE_FILE_READY;
    } else if (st == STATE_FILE_READY) {
        audioBytes = 0;
        currentState = STATE_RECORDING;
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
  .streaming  { color: #16c79a; }
  .ready      { color: #0f3460; }
  .btn { display: inline-block; margin: 0.5em; padding: 0.8em 2em;
         color: #fff; text-decoration: none; border: none;
         border-radius: 8px; font-size: 1.1em; cursor: pointer; }
  .btn-rec  { background: #e94560; }
  .btn-stop { background: #c73550; }
  .btn-dl   { background: #0f3460; }
  .btn-spec { background: #16c79a; }
  .btn-spec-stop { background: #0e8c6b; }
  .hidden { display: none; }
  #spectrumCanvas {
    display: block; margin: 1em auto; border-radius: 8px;
    background: #0f0f23; width: 100%; max-width: 600px; height: auto;
  }
</style>
</head>
<body>
<h1>ESP32 Sound Recorder</h1>
<div id="status" class="waiting">Waiting</div>
<div id="controls">
  <button id="recBtn" class="btn btn-rec" onclick="doToggle()">Record</button>
  <button id="stopBtn" class="btn btn-stop hidden" onclick="doToggle()">Stop</button>
  <a id="dlBtn" class="btn btn-dl hidden" href="/download">Download WAV</a>
  <button id="specBtn" class="btn btn-spec" onclick="startSpectrum()">Live Spectrum</button>
  <button id="specStopBtn" class="btn btn-spec-stop hidden" onclick="stopSpectrum()">Stop Spectrum</button>
</div>
<canvas id="spectrumCanvas" width="600" height="350"></canvas>
<script>
// ── Recording controls ──
function doToggle() {
  fetch('/toggle', {method:'POST'}).then(function() { poll(); });
}

function poll() {
  fetch('/status').then(function(r) { return r.json(); }).then(function(d) {
    var st  = document.getElementById('status');
    var rec = document.getElementById('recBtn');
    var stp = document.getElementById('stopBtn');
    var dl  = document.getElementById('dlBtn');
    var specB = document.getElementById('specBtn');
    var specS = document.getElementById('specStopBtn');
    var state = d.state || 'idle';

    if (state === 'recording') {
      st.textContent = 'Recording\u2026';
      st.className = 'recording';
      rec.classList.add('hidden');
      stp.classList.remove('hidden');
      dl.classList.add('hidden');
      specB.classList.add('hidden');
      specS.classList.add('hidden');
    } else if (state === 'file_ready') {
      st.textContent = 'Done \u2014 ' + (d.bytes / 1024).toFixed(0) + ' KB recorded';
      st.className = 'ready';
      rec.classList.remove('hidden');
      rec.textContent = 'New Recording';
      stp.classList.add('hidden');
      dl.classList.remove('hidden');
      specB.classList.remove('hidden');
      specS.classList.add('hidden');
    } else if (state === 'streaming') {
      st.textContent = 'Live Spectrum';
      st.className = 'streaming';
      rec.classList.remove('hidden');
      rec.textContent = 'Record';
      stp.classList.add('hidden');
      dl.classList.add('hidden');
      specB.classList.add('hidden');
      specS.classList.remove('hidden');
    } else {
      st.textContent = 'Waiting';
      st.className = 'waiting';
      rec.classList.remove('hidden');
      rec.textContent = 'Record';
      stp.classList.add('hidden');
      dl.classList.add('hidden');
      specB.classList.remove('hidden');
      specS.classList.add('hidden');
    }
  }).catch(function(){});
}
setInterval(poll, 500);
poll();

// ── Spectrum streaming via HTTP polling ──
var streaming = false;

function startSpectrum() {
  fetch('/stream-start', {method:'POST'}).then(function() {
    streaming = true;
    poll();
    fetchChunk();
  });
}

function stopSpectrum() {
  streaming = false;
  fetch('/stream-stop', {method:'POST'}).then(function() { poll(); });
}

function fetchChunk() {
  if (!streaming) return;
  fetch('/stream-chunk').then(function(r) {
    if (r.status === 200) {
      return r.arrayBuffer().then(function(buf) {
        processPCM(buf);
        setTimeout(fetchChunk, 15);
      });
    } else {
      setTimeout(fetchChunk, 30);
    }
  }).catch(function() {
    setTimeout(fetchChunk, 100);
  });
}

// ── FFT + Spectrum Drawing ──
var canvas = document.getElementById('spectrumCanvas');
var ctx = canvas.getContext('2d');
var N = 512;
var smoothBars = new Float32Array(64);
for (var sb = 0; sb < 64; sb++) smoothBars[sb] = -100;

// Hann window
var hann = new Float32Array(N);
for (var i = 0; i < N; i++) hann[i] = 0.5 * (1 - Math.cos(2 * Math.PI * i / (N - 1)));

// Bit-reversal permutation
function bitrev(n, bits) {
  var r = 0;
  for (var i = 0; i < bits; i++) { r = (r << 1) | (n & 1); n >>= 1; }
  return r;
}

// Radix-2 FFT (in-place, complex arrays re/im)
function fft(re, im) {
  var n = re.length, bits = Math.round(Math.log2(n));
  for (var i = 0; i < n; i++) {
    var j = bitrev(i, bits);
    if (j > i) {
      var t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (var size = 2; size <= n; size *= 2) {
    var half = size / 2;
    var ang = -2 * Math.PI / size;
    for (var i = 0; i < n; i += size) {
      for (var k = 0; k < half; k++) {
        var wr = Math.cos(ang * k), wi = Math.sin(ang * k);
        var idx = i + k, idx2 = idx + half;
        var tre = wr * re[idx2] - wi * im[idx2];
        var tim = wr * im[idx2] + wi * re[idx2];
        re[idx2] = re[idx] - tre; im[idx2] = im[idx] - tim;
        re[idx] += tre; im[idx] += tim;
      }
    }
  }
}

function processPCM(buf) {
  var samples = new Int16Array(buf);
  if (samples.length < N) return;
  var re = new Float32Array(N), im = new Float32Array(N);
  for (var i = 0; i < N; i++) re[i] = (samples[i] / 32768.0) * hann[i];

  fft(re, im);

  // Compute magnitude in dB for 64 bars (0 to N/2 = 256 bins = 0 to 8kHz)
  var binsPerBar = 4;
  var bars = new Float32Array(64);
  for (var b = 0; b < 64; b++) {
    var sum = 0;
    for (var k = 0; k < binsPerBar; k++) {
      var idx = b * binsPerBar + k;
      var mag = Math.sqrt(re[idx] * re[idx] + im[idx] * im[idx]);
      sum += mag;
    }
    var avg = sum / binsPerBar;
    bars[b] = avg > 1e-10 ? 20 * Math.log10(avg) : -100;
  }

  // Exponential moving average
  for (var b = 0; b < 64; b++) {
    smoothBars[b] = smoothBars[b] * 0.6 + bars[b] * 0.4;
  }

  drawSpectrum();
}

function drawSpectrum() {
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  var dbMin = -70, dbMax = 0;
  var barW = W / 64;
  var labelH = 20;
  var plotH = H - labelH;

  for (var b = 0; b < 64; b++) {
    var db = Math.max(dbMin, Math.min(dbMax, smoothBars[b]));
    var h = ((db - dbMin) / (dbMax - dbMin)) * plotH;
    if (h < 1) continue;
    var x = b * barW;
    var ratio = h / plotH;
    var r = ratio > 0.5 ? 255 : Math.round(ratio * 2 * 255);
    var g = ratio < 0.5 ? 255 : Math.round((1 - ratio) * 2 * 255);
    ctx.fillStyle = 'rgb(' + r + ',' + g + ',50)';
    ctx.fillRect(x + 1, plotH - h, barW - 2, h);
  }

  // Frequency labels
  ctx.fillStyle = '#888';
  ctx.font = '10px sans-serif';
  ctx.textAlign = 'center';
  var labels = [{hz:'0',bar:0},{hz:'500',bar:4},{hz:'1k',bar:8},{hz:'2k',bar:16},{hz:'4k',bar:32},{hz:'8k',bar:64}];
  for (var i = 0; i < labels.length; i++) {
    var lx = labels[i].bar * barW;
    ctx.fillText(labels[i].hz, lx, H - 3);
  }

  // dB labels
  ctx.textAlign = 'right';
  ctx.fillText('0dB', W - 2, 12);
  ctx.fillText('-70dB', W - 2, plotH);
}

// Initial draw (empty spectrum)
drawSpectrum();
</script>
</body>
</html>
)rawliteral";

// ── Web server handlers ──────────────────────────────────────────
static void handleRoot() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "0");
    server.send(200, "text/html", INDEX_HTML);
}

static void handleStatus() {
    State st = currentState;
    const char *stateStr;
    switch (st) {
        case STATE_IDLE:      stateStr = "idle"; break;
        case STATE_STREAMING: stateStr = "streaming"; break;
        case STATE_RECORDING: stateStr = "recording"; break;
        case STATE_FILE_READY: stateStr = "file_ready"; break;
        default:              stateStr = "idle"; break;
    }

    String json = "{\"state\":\"";
    json += stateStr;
    json += "\",\"recording\":";
    json += (st == STATE_RECORDING) ? "true" : "false";
    json += ",\"file_ready\":";
    json += (st == STATE_FILE_READY) ? "true" : "false";
    json += ",\"streaming\":";
    json += (st == STATE_STREAMING) ? "true" : "false";
    json += ",\"bytes\":";
    json += String((unsigned long)audioBytes);
    json += "}";
    server.send(200, "application/json", json);
}

static void handleToggle() {
    State st = currentState;
    if (st == STATE_IDLE || st == STATE_STREAMING) {
        audioBytes = 0;
        currentState = STATE_RECORDING;
        Serial.println(">> Recording started (web)");
    } else if (st == STATE_RECORDING) {
        currentState = STATE_FILE_READY;
        Serial.printf(">> Recording stopped (web) — %u bytes\n", (unsigned)audioBytes);
    } else if (st == STATE_FILE_READY) {
        audioBytes = 0;
        currentState = STATE_RECORDING;
        Serial.println(">> New recording started (web)");
    }
    server.send(200, "text/plain", "ok");
}

static void handleDownload() {
    if (currentState != STATE_FILE_READY || audioBytes == 0) {
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

// ── Streaming HTTP handlers ──────────────────────────────────────
static void handleStreamStart() {
    if (currentState == STATE_IDLE || currentState == STATE_FILE_READY) {
        currentState = STATE_STREAMING;
        Serial.println(">> Streaming started");
    }
    server.send(200, "text/plain", "ok");
}

static void handleStreamStop() {
    if (currentState == STATE_STREAMING) {
        currentState = STATE_IDLE;
        Serial.println(">> Streaming stopped");
    }
    server.send(200, "text/plain", "ok");
}

static void handleStreamChunk() {
    if (currentState != STATE_STREAMING || !streamQueue) {
        server.send(204, "text/plain", "");
        return;
    }
    uint8_t chunk[STREAM_CHUNK_BYTES];
    if (xQueueReceive(streamQueue, chunk, 0) == pdTRUE) {
        WiFiClient client = server.client();
        client.printf("HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: %d\r\nConnection: keep-alive\r\n\r\n", STREAM_CHUNK_BYTES);
        client.write(chunk, STREAM_CHUNK_BYTES);
    } else {
        server.send(204, "text/plain", "");
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
        for (;;) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    // Streaming queue: 4 slots × 1024 bytes each
    streamQueue = xQueueCreate(STREAM_QUEUE_SLOTS, STREAM_CHUNK_BYTES);
    if (!streamQueue) {
        Serial.println("ERROR: stream queue allocation failed!");
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
    server.on("/toggle", HTTP_POST, handleToggle);
    server.on("/download", handleDownload);
    server.on("/stream-start", HTTP_POST, handleStreamStart);
    server.on("/stream-stop", HTTP_POST, handleStreamStop);
    server.on("/stream-chunk", HTTP_GET, handleStreamChunk);
    server.begin();
    Serial.println("Web server started on port 80");

    // Recording task on core 1 (increased stack for streaming chunk)
    xTaskCreatePinnedToCore(recordingTask, "rec", 6144, NULL, 1, NULL, 1);
    Serial.println("Ready — press PRG button to start recording");

    // Turn LED off to indicate setup is complete
    digitalWrite(LED_PIN, LOW);
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    // Flush queue when not streaming to prevent stale data
    if (currentState != STATE_STREAMING && streamQueue) {
        uint8_t discard[STREAM_CHUNK_BYTES];
        while (xQueueReceive(streamQueue, discard, 0) == pdTRUE) {}
    }

    // Blink LED while recording
    static unsigned long lastBlink = 0;
    if (currentState == STATE_RECORDING && millis() - lastBlink > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    } else if (currentState != STATE_RECORDING) {
        digitalWrite(LED_PIN, LOW);
    }
}
