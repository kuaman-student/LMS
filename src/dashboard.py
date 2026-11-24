import asyncio
import json
import os
import socket
import webbrowser
from aiohttp import web

ESP_IP = "192.168.4.1"
ESP_CMD_PORT = 8080
LOCAL_UDP_PORT = 9000
WEB_PORT = 8050

clients = set()

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ProjectX — Autonomous Telemetry & PID Cockpit</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&family=Space+Grotesk:wght@500;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #07090F; --card: #0E1320; --border: #1C253B;
    --cyan: #00E5FF; --violet: #9B7BFF; --danger: #FF3B5C; --ok: #00E676;
    --text: #E6EDF7; --muted: #7B8AB0;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: 'Space Grotesk', sans-serif; padding: 16px; }
  header { display: flex; justify-content: space-between; align-items: center; padding: 12px 18px; background: var(--card); border: 1px solid var(--border); border-radius: 12px; margin-bottom: 14px; }
  h1 { font-size: 1.2rem; letter-spacing: 0.5px; }
  h1 span { color: var(--cyan); }
  .badge { font-family: 'JetBrains Mono', monospace; font-size: 0.8rem; padding: 5px 10px; border-radius: 999px; background: #131B2E; border: 1px solid var(--border); }
  .grid { display: grid; grid-template-columns: 360px 1fr; gap: 14px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }
  .card h2 { font-size: 0.85rem; text-transform: uppercase; letter-spacing: 1px; color: var(--muted); margin-bottom: 12px; }
  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 16px; }
  button { font-family: 'Space Grotesk', sans-serif; font-weight: 700; font-size: 0.85rem; padding: 10px; border-radius: 8px; border: 1px solid var(--border); cursor: pointer; background: #131A2B; color: var(--text); transition: 0.15s; }
  button:hover { border-color: var(--cyan); transform: translateY(-1px); }
  .btn-start { background: rgba(0, 230, 118, 0.14); border-color: var(--ok); color: var(--ok); }
  .btn-stop { background: rgba(255, 59, 92, 0.18); border-color: var(--danger); color: var(--danger); grid-column: span 2; font-size: 0.95rem; }
  .btn-accent { border-color: var(--violet); color: var(--violet); }
  .slider-row { margin-bottom: 14px; }
  .slider-head { display: flex; justify-content: space-between; font-family: 'JetBrains Mono', monospace; font-size: 0.82rem; margin-bottom: 5px; }
  .slider-head span:last-child { color: var(--cyan); font-weight: 700; }
  .slider-inputs { display: grid; grid-template-columns: 1fr 78px; gap: 8px; align-items: center; }
  input[type=range] { width: 100%; accent-color: var(--cyan); cursor: pointer; }
  input[type=number], input[type=text] { background: #090D16; border: 1px solid var(--border); color: var(--text); font-family: 'JetBrains Mono', monospace; font-size: 0.8rem; padding: 5px 7px; border-radius: 6px; width: 100%; }
  .charts { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 12px; }
  canvas { width: 100%; height: 150px; background: #080B14; border: 1px solid var(--border); border-radius: 8px; display: block; }
  .ir-bars { display: grid; grid-template-columns: repeat(16, 1fr); gap: 4px; height: 70px; align-items: end; background: #080B14; padding: 8px; border-radius: 8px; border: 1px solid var(--border); margin-bottom: 12px; }
  .ir-col { background: linear-gradient(180deg, var(--cyan), var(--violet)); width: 100%; height: 4%; border-radius: 3px; transition: height 0.05s; }
  .console { height: 140px; overflow-y: auto; background: #06080E; border: 1px solid var(--border); border-radius: 8px; padding: 10px; font-family: 'JetBrains Mono', monospace; font-size: 0.78rem; color: #A9B7D0; margin-bottom: 8px; }
  .cmd-bar { display: grid; grid-template-columns: 1fr 80px; gap: 8px; }
</style>
</head>
<body>
<header>
  <h1>PROJECT<span>X</span> // PID & TELEMETRY COCKPIT</h1>
  <div class="badge" id="connStatus">WS: CONNECTING...</div>
</header>

<div class="grid">
  <!-- LEFT CONTROLS -->
  <div class="card">
    <h2>Motion & Calibration</h2>
    <div class="btn-grid">
      <button class="btn-stop" onclick="sendCmd('stop')">EMERGENCY STOP</button>
      <button class="btn-start" onclick="sendCmd('start')">START / ARM</button>
      <button class="btn-accent" onclick="sendCmd('cal')">CAL (4s)</button>
      <button onclick="sendCmd('invert')">INVERT STEER</button>
      <button onclick="sendCmd('status')">STATUS</button>
    </div>

    <h2>Live PID & Speed Sliders</h2>
    <div class="slider-row">
      <div class="slider-head"><span>Kp (Proportional)</span><span id="lbl_kp">0.0250</span></div>
      <div class="slider-inputs">
        <input type="range" id="rng_kp" min="0" max="0.25" step="0.001" value="0.025" oninput="syncSlider('kp', this.value)">
        <input type="number" id="num_kp" step="0.001" value="0.025" onchange="syncSlider('kp', this.value)">
      </div>
    </div>

    <div class="slider-row">
      <div class="slider-head"><span>Ki (Integral)</span><span id="lbl_ki">0.0000</span></div>
      <div class="slider-inputs">
        <input type="range" id="rng_ki" min="0" max="0.05" step="0.0005" value="0" oninput="syncSlider('ki', this.value)">
        <input type="number" id="num_ki" step="0.0005" value="0" onchange="syncSlider('ki', this.value)">
      </div>
    </div>

    <div class="slider-row">
      <div class="slider-head"><span>Kd (Derivative)</span><span id="lbl_kd">0.1800</span></div>
      <div class="slider-inputs">
        <input type="range" id="rng_kd" min="0" max="1.5" step="0.005" value="0.18" oninput="syncSlider('kd', this.value)">
        <input type="number" id="num_kd" step="0.005" value="0.18" onchange="syncSlider('kd', this.value)">
      </div>
    </div>

    <div class="slider-row">
      <div class="slider-head"><span>Base PWM (Speed)</span><span id="lbl_base">100</span></div>
      <div class="slider-inputs">
        <input type="range" id="rng_base" min="0" max="255" step="1" value="100" oninput="syncSlider('base', this.value)">
        <input type="number" id="num_base" min="0" max="255" step="1" value="100" onchange="syncSlider('base', this.value)">
      </div>
    </div>
  </div>

  <!-- RIGHT GRAPHS & CONSOLE -->
  <div class="card">
    <h2>ARC-16 Live Sensor Array (ch0 – ch15)</h2>
    <div class="ir-bars" id="irBars"></div>

    <div class="charts">
      <div>
        <div class="slider-head"><span>Line Error (err)</span><span id="val_err">0</span></div>
        <canvas id="chartErr" width="420" height="150"></canvas>
      </div>
      <div>
        <div class="slider-head"><span>PID Steering Output (steer)</span><span id="val_steer">0</span></div>
        <canvas id="chartSteer" width="420" height="150"></canvas>
      </div>
      <div>
        <div class="slider-head"><span>Left Motor PWM (pwmL)</span><span id="val_pwmL">0</span></div>
        <canvas id="chartPwmL" width="420" height="150"></canvas>
      </div>
      <div>
        <div class="slider-head"><span>Right Motor PWM (pwmR)</span><span id="val_pwmR">0</span></div>
        <canvas id="chartPwmR" width="420" height="150"></canvas>
      </div>
    </div>

    <h2>ESP32 Telemetry & CLI Console</h2>
    <div class="console" id="consoleBox"></div>
    <div class="cmd-bar">
      <input type="text" id="rawCmd" placeholder="Type custom command (e.g. set kp 0.04, start, stop)..." onkeydown="if(event.key==='Enter') sendRaw()">
      <button onclick="sendRaw()">SEND</button>
    </div>
  </div>
</div>

<script>
  const irContainer = document.getElementById('irBars');
  const irCols = [];
  for (let i = 0; i < 16; i++) {
    const d = document.createElement('div');
    d.className = 'ir-col';
    irContainer.appendChild(d);
    irCols.push(d);
  }

  const buffers = { err: [], steer: [], pwmL: [], pwmR: [] };
  const MAX_PTS = 120;

  function pushData(key, val) {
    if (!buffers[key]) return;
    buffers[key].push(val);
    if (buffers[key].length > MAX_PTS) buffers[key].shift();
    const el = document.getElementById('val_' + key);
    if (el) el.textContent = val.toFixed(1);
  }

  function drawChart(canvasId, data, minY, maxY, color) {
    const c = document.getElementById(canvasId);
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, c.width, c.height);

    // Zero reference line
    const zeroY = c.height - ((0 - minY) / (maxY - minY)) * c.height;
    ctx.strokeStyle = '#1C253B';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, zeroY); ctx.lineTo(c.width, zeroY); ctx.stroke();

    if (data.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    data.forEach((v, idx) => {
      const x = (idx / (MAX_PTS - 1)) * c.width;
      const clamped = Math.max(minY, Math.min(maxY, v));
      const y = c.height - ((clamped - minY) / (maxY - minY)) * c.height;
      if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  setInterval(() => {
    drawChart('chartErr', buffers.err, -7500, 7500, '#00E5FF');
    drawChart('chartSteer', buffers.steer, -255, 255, '#9B7BFF');
    drawChart('chartPwmL', buffers.pwmL, -255, 255, '#00E676');
    drawChart('chartPwmR', buffers.pwmR, -255, 255, '#FFAB00');
  }, 50);

  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => {
    const b = document.getElementById('connStatus');
    b.textContent = 'UDP BRIDGE: LIVE (192.168.4.1)';
    b.style.color = '#00E676';
  };
  ws.onclose = () => {
    const b = document.getElementById('connStatus');
    b.textContent = 'DISCONNECTED';
    b.style.color = '#FF3B5C';
  };

  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.vars) {
      for (const [k, v] of Object.entries(msg.vars)) {
        if (k.startsWith('ch')) {
          const idx = parseInt(k.slice(2));
          if (irCols[idx]) irCols[idx].style.height = Math.max(4, Math.min(100, (v / 1023) * 100)) + '%';
        } else {
          pushData(k, v);
        }
      }
    }
    if (msg.logs) {
      const box = document.getElementById('consoleBox');
      msg.logs.forEach(line => {
        const div = document.createElement('div');
        div.textContent = line;
        box.appendChild(div);
      });
      box.scrollTop = box.scrollHeight;
    }
  };

  function sendCmd(cmd) {
    if (ws.readyState === WebSocket.OPEN) ws.send(cmd);
  }

  let debounceTimers = {};
  function syncSlider(param, val) {
    document.getElementById('rng_' + param).value = val;
    document.getElementById('num_' + param).value = val;
    document.getElementById('lbl_' + param).textContent = (param === 'base') ? parseInt(val) : parseFloat(val).toFixed(4);
    clearTimeout(debounceTimers[param]);
    debounceTimers[param] = setTimeout(() => {
      sendCmd(`set ${param} ${val}`);
    }, 40);
  }

  function sendRaw() {
    const inp = document.getElementById('rawCmd');
    if (inp.value.trim()) {
      sendCmd(inp.value.trim());
      inp.value = '';
    }
  }
</script>
</body>
</html>
"""

class UdpBridgeProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        if addr[0] != ESP_IP:
            return
        text = data.decode(errors="ignore")
        vars_out = {}
        logs_out = []
        for line in text.splitlines():
            if line.startswith(">") and ":" in line:
                k, v = line[1:].split(":", 1)
                try:
                    vars_out[k.strip()] = float(v.strip())
                except ValueError:
                    pass
            elif line.strip():
                logs_out.append(line.strip())

        if (vars_out or logs_out) and clients:
            payload = json.dumps({"vars": vars_out, "logs": logs_out})
            for ws in list(clients):
                asyncio.create_task(ws.send_str(payload))

async def index(request):
    return web.Response(text=HTML_PAGE, content_type="text/html")

async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    clients.add(ws)
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT and udp_protocol.transport:
                udp_protocol.transport.sendto(msg.data.encode(), (ESP_IP, ESP_CMD_PORT))
    finally:
        clients.discard(ws)
    return ws

async def main():
    global udp_protocol
    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", LOCAL_UDP_PORT))
    _, udp_protocol = await loop.create_datagram_endpoint(lambda: UdpBridgeProtocol(), sock=sock)

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", WEB_PORT)
    await site.start()

    url = f"http://127.0.0.1:{WEB_PORT}"
    print(f"ProjectX Cockpit Live at {url}")
    webbrowser.open(url)
    await asyncio.Event().wait()

if __name__ == "__main__":
    asyncio.run(main())