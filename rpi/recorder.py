#!/usr/bin/env python3
"""4-Channel Raspberry Pi 5 Recorder

Captures 4-channel 16 kHz 16-bit audio from an ICS-43434 tetrahedral
microphone array via ALSA device 'fourmics' and serves a web UI for
recording, downloading WAV files, and live spectrum analysis.
"""

import argparse
import os
import queue
import signal
import struct
import sys
import threading
import wave
from datetime import datetime

import numpy as np
import sounddevice as sd
from flask import Flask, Response, send_file

# -- Configuration -----------------------------------------------------------
SAMPLE_RATE = 16000
CHANNELS = 4
DTYPE = "int16"
BLOCK_SIZE = 256  # frames per callback (matches ESP32 DMA buffer)
STREAM_CHUNK_SAMPLES = 512
STREAM_CHUNK_BYTES = STREAM_CHUNK_SAMPLES * 2  # 512 int16 = 1024 bytes
STREAM_QUEUE_SLOTS = 4
DEFAULT_DEVICE = "hw:fourmics,0"

# -- State machine -----------------------------------------------------------
STATE_IDLE = "idle"
STATE_STREAMING = "streaming"
STATE_RECORDING = "recording"
STATE_FILE_READY = "file_ready"

state_lock = threading.Lock()
current_state = STATE_IDLE

# -- Recording state ---------------------------------------------------------
wav_file = None
wav_lock = threading.Lock()
recorded_bytes = 0
last_filename = None

# -- Streaming queue ---------------------------------------------------------
stream_queue = queue.Queue(maxsize=STREAM_QUEUE_SLOTS)
stream_accum = []
stream_accum_len = 0

# -- Audio stream reference --------------------------------------------------
audio_stream = None

# -- Embedded HTML/JS --------------------------------------------------------
INDEX_HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>4-Channel Recorder</title>
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
<h1>4-Channel Sound Recorder</h1>
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
// -- Recording controls --
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
      var secs = (d.bytes / (16000 * 4 * 2)).toFixed(1);
      st.textContent = 'Recording\u2026 ' + secs + ' s';
      st.className = 'recording';
      rec.classList.add('hidden');
      stp.classList.remove('hidden');
      dl.classList.add('hidden');
      specB.classList.add('hidden');
      specS.classList.add('hidden');
    } else if (state === 'file_ready') {
      var secs = (d.bytes / (16000 * 4 * 2)).toFixed(1);
      st.textContent = 'Done \u2014 ' + secs + ' s (' + (d.bytes / 1024).toFixed(0) + ' KB)';
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

// -- Spectrum streaming via HTTP polling --
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

// -- FFT + Spectrum Drawing --
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
</html>"""


# -- Flask app ---------------------------------------------------------------
app = Flask(__name__)


@app.route("/")
def index():
    resp = Response(INDEX_HTML, mimetype="text/html")
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


@app.route("/status")
def status():
    with state_lock:
        st = current_state
    with wav_lock:
        nbytes = recorded_bytes
    return {
        "state": st,
        "recording": st == STATE_RECORDING,
        "file_ready": st == STATE_FILE_READY,
        "streaming": st == STATE_STREAMING,
        "bytes": nbytes,
    }


@app.route("/toggle", methods=["POST"])
def toggle():
    global current_state, wav_file, recorded_bytes, last_filename
    with state_lock:
        st = current_state
        if st in (STATE_IDLE, STATE_STREAMING):
            _open_wav()
            current_state = STATE_RECORDING
            print(">> Recording started (web)")
        elif st == STATE_RECORDING:
            _close_wav()
            current_state = STATE_FILE_READY
            with wav_lock:
                print(f">> Recording stopped (web) -- {recorded_bytes} bytes")
        elif st == STATE_FILE_READY:
            _open_wav()
            current_state = STATE_RECORDING
            print(">> New recording started (web)")
    return "ok"


@app.route("/download")
def download():
    with state_lock:
        st = current_state
    if st != STATE_FILE_READY or last_filename is None:
        return "No recording available", 404
    return send_file(
        last_filename,
        mimetype="audio/wav",
        as_attachment=True,
        download_name=os.path.basename(last_filename),
    )


@app.route("/stream-start", methods=["POST"])
def stream_start():
    global current_state
    with state_lock:
        if current_state in (STATE_IDLE, STATE_FILE_READY):
            current_state = STATE_STREAMING
            # Drain any stale chunks
            while not stream_queue.empty():
                try:
                    stream_queue.get_nowait()
                except queue.Empty:
                    break
            print(">> Streaming started")
    return "ok"


@app.route("/stream-stop", methods=["POST"])
def stream_stop():
    global current_state
    with state_lock:
        if current_state == STATE_STREAMING:
            current_state = STATE_IDLE
            print(">> Streaming stopped")
    return "ok"


@app.route("/stream-chunk")
def stream_chunk():
    with state_lock:
        st = current_state
    if st != STATE_STREAMING:
        return "", 204
    try:
        chunk = stream_queue.get_nowait()
        return Response(chunk, mimetype="application/octet-stream")
    except queue.Empty:
        return "", 204


# -- WAV file helpers --------------------------------------------------------
def _open_wav():
    """Open a new WAV file for recording. Must be called with state_lock held."""
    global wav_file, recorded_bytes, last_filename
    with wav_lock:
        if wav_file is not None:
            wav_file.close()
        recorded_bytes = 0
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        last_filename = os.path.join(output_dir, f"recording_{ts}.wav")
        wf = wave.open(last_filename, "wb")
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(SAMPLE_RATE)
        wav_file = wf


def _close_wav():
    """Close the current WAV file. Must be called with state_lock held."""
    global wav_file
    with wav_lock:
        if wav_file is not None:
            wav_file.close()
            wav_file = None


# -- Audio callback ----------------------------------------------------------
def audio_callback(indata, frames, time_info, status):
    """Called by sounddevice for each block of audio data."""
    global recorded_bytes, stream_accum, stream_accum_len

    if status:
        print(f"Audio status: {status}", file=sys.stderr)

    with state_lock:
        st = current_state

    if st == STATE_RECORDING:
        raw = indata.tobytes()
        with wav_lock:
            if wav_file is not None:
                wav_file.writeframes(raw)
                recorded_bytes += len(raw)

    elif st == STATE_STREAMING:
        # Extract channel 0 for spectrum analysis
        mono = indata[:, 0].copy()
        stream_accum.append(mono)
        stream_accum_len += len(mono)

        while stream_accum_len >= STREAM_CHUNK_SAMPLES:
            # Concatenate and split at chunk boundary
            combined = np.concatenate(stream_accum)
            chunk = combined[:STREAM_CHUNK_SAMPLES]
            remainder = combined[STREAM_CHUNK_SAMPLES:]
            stream_accum = [remainder] if len(remainder) > 0 else []
            stream_accum_len = len(remainder)

            raw = chunk.astype(np.int16).tobytes()
            try:
                stream_queue.put_nowait(raw)
            except queue.Full:
                # Drop oldest, enqueue new
                try:
                    stream_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    stream_queue.put_nowait(raw)
                except queue.Full:
                    pass

    else:
        # IDLE or FILE_READY -- reset accumulator
        stream_accum = []
        stream_accum_len = 0


# -- Device detection --------------------------------------------------------
def find_device(preferred):
    """Return device index or string for sounddevice.

    Try the preferred ALSA device first, then fall back to searching
    by card name 'fourmics' among available devices.
    """
    # First try to use the explicit ALSA hw string
    try:
        info = sd.query_devices(preferred)
        if info["max_input_channels"] >= CHANNELS:
            print(f"Using device: {preferred}")
            return preferred
    except Exception:
        pass

    # Fall back: search by name
    for i, dev in enumerate(sd.query_devices()):
        name = dev["name"].lower()
        if "fourmics" in name or "four-mics" in name:
            if dev["max_input_channels"] >= CHANNELS:
                print(f"Using device #{i}: {dev['name']}")
                return i

    print("WARNING: 'fourmics' device not found, using system default")
    return None


# -- Graceful shutdown -------------------------------------------------------
def shutdown(signum, frame):
    global current_state
    print(f"\nReceived signal {signum}, shutting down...")
    with state_lock:
        if current_state == STATE_RECORDING:
            _close_wav()
        current_state = STATE_IDLE
    if audio_stream is not None:
        audio_stream.stop()
        audio_stream.close()
    sys.exit(0)


# -- Main --------------------------------------------------------------------
output_dir = "./recordings"


def main():
    global output_dir, audio_stream

    parser = argparse.ArgumentParser(description="4-Channel RPi5 Recorder")
    parser.add_argument("--port", type=int, default=80, help="Web server port")
    parser.add_argument("--host", default="0.0.0.0", help="Web server bind address")
    parser.add_argument("--output-dir", default="./recordings", help="WAV output directory")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="ALSA capture device")
    args = parser.parse_args()

    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    device = find_device(args.device)

    audio_stream = sd.InputStream(
        device=device,
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype=DTYPE,
        blocksize=BLOCK_SIZE,
        callback=audio_callback,
    )
    audio_stream.start()
    print(f"Audio stream started: {SAMPLE_RATE} Hz, {CHANNELS} ch, {DTYPE}")

    print(f"Starting web server on {args.host}:{args.port}")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
