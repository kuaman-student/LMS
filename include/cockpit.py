import asyncio
import json
import socket
import webbrowser
from aiohttp import web

ESP_IP = "192.168.4.1"
TELNET_PORT = 23
UDP_PORT = 9000
WEB_PORT = 8050

clients = set()
telnet_writer = None

HTML_UI = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ProjectX — Telnet + UDP Hybrid Cockpit</title>
<style>
  :root {
    --bg: #07090F; --card: #0E1320; --border: #1C253B;
    --cyan: #00E5FF; --violet: #9B7BFF; --danger: #FF3B5C; --ok: #00E676; --amber: #FFAB00;
    --text: #E6EDF7; --muted: #7B8AB0;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: system-ui, sans-serif; padding: 14px; }
  header { display: flex; justify-content: space-between; align-items: center; padding: 12px 16px; background: var(--card); border: 1px solid var(--border); border-radius: 12px; margin-bottom: 12px; }
  h1 { font-size: 1.15rem; } h1 span { color: var(--cyan); }
  .badge { font-family: monospace; font-size: 0.78rem; padding: 5px 10px; border-radius: 999px; background: #131B2E; border: 1px solid var(--border); }
  .grid { display: grid; grid-template-columns: 350px 1fr; gap: 12px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 14px; margin-bottom: 12px; }
  .card h2 { font-size: 0.8rem; text-transform: uppercase; letter-spacing: 1px; color: var(--muted); margin-bottom: 10px; display: flex; justify-content: space-between; align-items: center; }
  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 14px; }
  button { font-weight: 700; font-size: 0.82rem; padding: 9px; border-radius: 8px; border: 1px solid var(--border); cursor: pointer; background: #131A2B; color: var(--text); }
  button:hover { border-color: var(--cyan); }
  .btn-start { background: rgba(0,230,118,0.14); border-color: var(--ok); color: var(--ok); }
  .btn-stop { background: rgba(255,59,92,0.18); border-color: var(--danger); color: var(--danger); grid-column: span 2; }
  .btn-accent { border-color: var(--violet); color: var(--violet); }
  .btn-mini { padding: 4px 8px; font-size: 0.72rem; }
  .slider-row { margin-bottom: 12px; }
  .slider-head { display: flex; justify-content: space-between; font-family: monospace; font-size: 0.8rem; margin-bottom: 4px; }
  .slider-head span:last-child { color: var(--cyan); font-weight: 700; }
  .slider-inputs { display: grid; grid-template-columns: 1fr 74px; gap: 8px; }
  input[type=range] { width: 100%; accent-color: var(--cyan); }
  input[type=number], input[type=text] { background: #090D16; border: 1px solid var(--border); color: var(--text); font-family: monospace; font-size: 0.8rem; padding: 5px 7px; border-radius: 6px; width: 100%; }
  .imu-box { display: grid; grid-template-columns: 92px 1fr; gap: 12px; align-items: center; background: #080B14; border: 1px solid var(--border); border-radius: 10px; padding: 10px; }
  .dial { width: 86px; height: 86px; border-radius: 50%; border: 2px solid var(--cyan); position: relative; display: flex; align-items: center; justify-content: center; }
  .needle { position: absolute; width: 3px; height: 35px; background: var(--danger); top: 8px; left: 50%; transform-origin: 50% 35px; }
  .dial-val { font-family: monospace; font-size: 0.8rem; font-weight: 700; color: var(--cyan); z-index: 2; background: #07090F; padding: 2px 4px; border-radius: 4px; }
  .imu-metrics { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; font-family: monospace; font-size: 0.78rem; }
  .metric { background: #0D1322; padding: 6px 8px; border-radius: 6px; border: 1px solid #162038; }
  .metric small { display: block; color: var(--muted); font-size: 0.68rem; }
  .charts { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px; }
  canvas { width: 100%; height: 135px; background: #080B14; border: 1px solid var(--border); border-radius: 8px; display: block; }
  .ir-bars { display: grid; grid-template-columns: repeat(16, 1fr); gap: 4px; height: 64px; align-items: end; background: #080B14; padding: 8px; border-radius: 8px; border: 1px solid var(--border); margin-bottom: 10px; }
  .ir-col { background: linear-gradient(180deg, var(--cyan), var(--violet)); width: 100%; height: 4%; border-radius: 3px; }
  .path-strip { font-family: monospace; font-size: 0.85rem; background: #080B14; border: 1px solid var(--border); padding: 8px 10px; border-radius: 8px; margin-bottom: 10px; color: var(--amber); }
  .console { height: 120px; overflow-y: auto; background: #06080E; border: 1px solid var(--border); border-radius: 8px; padding: 8px; font-family: monospace; font-size: 0.76rem; color: #A9B7D0; margin-bottom: 8px; }
  .cmd-bar { display: grid; grid-template-columns: 1fr 75px; gap: 8px; }
</style>
</head>
<body>
<header>
  <h1>PROJECT<span>X</span> // TELNET + UDP COCKPIT</h1>
  <div class="badge" id="statusBadge">CONNECTING...</div>
</header>
<div class="grid">
  <div>
    <div class="card">
      <h2>Controls (Over Telnet TCP:23)</h2>
      <div class="btn-grid">
        <button class="btn-stop" onclick="sendCmd('stop')">EMERGENCY STOP</button>
        <button class="btn-start" onclick="sendCmd('start')">START / ARM</button>
        <button class="btn-accent" onclick="sendCmd('cal')">CAL IR (4s)</button>
        <button onclick="sendCmd('invert')">INVERT STEER</button>
        <button onclick="sendCmd('status')">STATUS</button>
      </div>
      <h2>PID & Speed Sliders</h2>
      <div class="slider-row">
        <div class="slider-head"><span>Kp</span><span id="lbl_kp">0.0250</span></div>
        <div class="slider-inputs"><input type="range" id="rng_kp" min="0" max="0.25" step="0.001" value="0.025" oninput="sync('kp',this.value)"><input type="number" id="num_kp" step="0.001" value="0.025" onchange="sync('kp',this.value)"></div>
      </div>
      <div class="slider-row">
        <div class="slider-head"><span>Ki</span><span id="lbl_ki">0.0000</span></div>
        <div class="slider-inputs"><input type="range" id="rng_ki" min="0" max="0.05" step="0.0005" value="0" oninput="sync('ki',this.value)"><input type="number" id="num_ki" step="0.0005" value="0" onchange="sync('ki',this.value)"></div>
      </div>
      <div class="slider-row">
        <div class="slider-head"><span>Kd</span><span id="lbl_kd">0.1800</span></div>
        <div class="slider-inputs"><input type="range" id="rng_kd" min="0" max="1.5" step="0.005" value="0.18" oninput="sync('kd',this.value)"><input type="number" id="num_kd" step="0.005" value="0.18" onchange="sync('kd',this.value)"></div>
      </div>
      <div class="slider-row">
        <div class="slider-head"><span>Base PWM</span><span id="lbl_base">120</span></div>
        <div class="slider-inputs"><input type="range" id="rng_base" min="0" max="255" step="1" value="120" oninput="sync('base',this.value)"><input type="number" id="num_base" min="0" max="255" step="1" value="120" onchange="sync('base',this.value)"></div>
      </div>
    </div>
    <div class="card">
      <h2><span>LSM6DSO IMU (UDP:9000)</span><button class="btn-mini btn-accent" onclick="sendCmd('zeroyaw')">ZERO YAW</button></h2>
      <div class="imu-box">
        <div class="dial"><div class="needle" id="yawNeedle"></div><div class="dial-val" id="imuYawDial">0°</div></div>
        <div class="imu-metrics">
          <div class="metric"><small>YAW ANGLE</small><strong id="imuYaw">0.00°</strong></div>
          <div class="metric"><small>RATE Z</small><strong id="imuRateZ">0.0 dps</strong></div>
          <div class="metric"><small>ACCEL X / Y</small><strong id="imuAccXY">0.00 / 0.00g</strong></div>
          <div class="metric"><small>ACCEL Z</small><strong id="imuAccZ">1.00g</strong></div>
        </div>
      </div>
    </div>
  </div>
  <div class="card">
    <h2>ARC-16 Array (ch0–ch15)</h2>
    <div class="ir-bars" id="irBars"></div>
    <div class="path-strip">LSRB PATH: <strong id="lsrbPath">—</strong></div>
    <div class="charts">
      <div><div class="slider-head"><span>Error</span><span id="val_err">0</span></div><canvas id="cErr" width="400" height="135"></canvas></div>
      <div><div class="slider-head"><span>Steer</span><span id="val_steer">0</span></div><canvas id="cSteer" width="400" height="135"></canvas></div>
      <div><div class="slider-head"><span>PWM L</span><span id="val_pwmL">0</span></div><canvas id="cPwmL" width="400" height="135"></canvas></div>
      <div><div class="slider-head"><span>PWM R</span><span id="val_pwmR">0</span></div><canvas id="cPwmR" width="400" height="135"></canvas></div>
    </div>
    <h2>Telnet CLI Console</h2>
    <div class="console" id="logBox"></div>
    <div class="cmd-bar"><input type="text" id="cmdIn" placeholder="Telnet command..." onkeydown="if(event.key==='Enter') sendRaw()"><button onclick="sendRaw()">SEND</button></div>
  </div>
</div>
<script>
  const irCols = [], irWrap = document.getElementById('irBars');
  for (let i = 0; i < 16; i++) { const d = document.createElement('div'); d.className = 'ir-col'; irWrap.appendChild(d); irCols.push(d); }
  const buf = { err: [], steer: [], pwmL: [], pwmR: [] }, MAX_N = 100;
  function addPt(k, v) { buf[k].push(v); if (buf[k].length > MAX_N) buf[k].shift(); document.getElementById('val_' + k).textContent = v.toFixed(1); }
  function plot(id, arr, minY, maxY, col) {
    const c = document.getElementById(id), ctx = c.getContext('2d'); ctx.clearRect(0,0,c.width,c.height);
    const zY = c.height - ((0 - minY) / (maxY - minY)) * c.height;
    ctx.strokeStyle = '#1C253B'; ctx.beginPath(); ctx.moveTo(0, zY); ctx.lineTo(c.width, zY); ctx.stroke();
    if (arr.length < 2) return;
    ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath();
    arr.forEach((v, i) => {
      const x = (i / (MAX_N - 1)) * c.width;
      const y = c.height - ((Math.max(minY, Math.min(maxY, v)) - minY) / (maxY - minY)) * c.height;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  setInterval(() => { plot('cErr',buf.err,-7500,7500,'#00E5FF'); plot('cSteer',buf.steer,-255,255,'#9B7BFF'); plot('cPwmL',buf.pwmL,-255,255,'#00E676'); plot('cPwmR',buf.pwmR,-255,255,'#FFAB00'); }, 50);

  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    if (d.status) { const b = document.getElementById('statusBadge'); b.textContent = d.status; b.style.color = d.ok ? '#00E676' : '#FFAB00'; }
    if (d.tel) {
      const t = d.tel;
      addPt('err', t.err); addPt('steer', t.str); addPt('pwmL', t.pL); addPt('pwmR', t.pR);
      document.getElementById('imuYaw').textContent = t.yaw.toFixed(2) + '°';
      document.getElementById('imuYawDial').textContent = t.yaw.toFixed(0) + '°';
      document.getElementById('yawNeedle').style.transform = `translateX(-50%) rotate(${-t.yaw}deg)`;
      document.getElementById('imuRateZ').textContent = t.rz.toFixed(1) + ' dps';
      document.getElementById('imuAccXY').textContent = `${t.ax.toFixed(2)} / ${t.ay.toFixed(2)}g`;
      document.getElementById('imuAccZ').textContent = t.az.toFixed(2) + 'g';
      document.getElementById('lsrbPath').textContent = t.path;
      t.ir.forEach((v, i) => { if (irCols[i]) irCols[i].style.height = Math.max(4, Math.min(100, v / 10)) + '%'; });
    }
    if (d.log) { const box = document.getElementById('logBox'), div = document.createElement('div'); div.textContent = d.log; box.appendChild(div); box.scrollTop = box.scrollHeight; }
  };
  function sendCmd(c) { if (ws.readyState === 1) ws.send(c); }
  let tmr = {};
  function sync(p, v) {
    document.getElementById('rng_'+p).value = v; document.getElementById('num_'+p).value = v;
    document.getElementById('lbl_'+p).textContent = (p==='base') ? parseInt(v) : parseFloat(v).toFixed(4);
    clearTimeout(tmr[p]); tmr[p] = setTimeout(() => sendCmd(`set ${p} ${v}`), 35);
  }
  function sendRaw() { const i = document.getElementById('cmdIn'); if (i.value.trim()) { sendCmd(i.value.trim()); i.value = ''; } }
</script>
</body>
</html>
"""

def broadcast_ws(obj):
    if not clients:
        return
    msg = json.dumps(obj)
    for ws in list(clients):
        asyncio.create_task(ws.send_str(msg))

class UdpTelemetryProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        if addr[0] != ESP_IP:
            return
        for line in data.decode(errors="ignore").splitlines():
            if line.startswith("TEL|"):
                parts = line.split("|")
                if len(parts) >= 12:
                    ir_vals = [int(x) for x in parts[11].split(",") if x.isdigit()]
                    broadcast_ws({"tel": {
                        "err": float(parts[1]), "str": float(parts[2]),
                        "pL": int(parts[3]), "pR": int(parts[4]),
                        "yaw": float(parts[5]), "rz": float(parts[6]),
                        "ax": float(parts[7]), "ay": float(parts[8]), "az": float(parts[9]),
                        "path": parts[10], "ir": ir_vals
                    }})
            elif line.strip():
                broadcast_ws({"log": line.strip()})

async def telnet_keeper():
    global telnet_writer
    while True:
        try:
            reader, writer = await asyncio.open_connection(ESP_IP, TELNET_PORT)
            telnet_writer = writer
            broadcast_ws({"status": "TELNET:23 + UDP:9000 LIVE", "ok": True})
            while True:
                line = await reader.readline()
                if not line:
                    break
                broadcast_ws({"log": line.decode(errors="ignore").strip()})
        except Exception:
            telnet_writer = None
            broadcast_ws({"status": "WAITING FOR ESP32 (192.168.4.1)...", "ok": False})
            await asyncio.sleep(1.5)

async def index(req):
    return web.Response(text=HTML_UI, content_type="text/html")

async def ws_handler(req):
    ws = web.WebSocketResponse()
    await ws.prepare(req)
    clients.add(ws)
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT and telnet_writer:
                telnet_writer.write((msg.data.strip() + "\n").encode())
                await telnet_writer.drain()
    finally:
        clients.discard(ws)
    return ws

async def main():
    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    await loop.create_datagram_endpoint(lambda: UdpTelemetryProtocol(), sock=sock)

    asyncio.create_task(telnet_keeper())

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", WEB_PORT).start()

    url = f"http://127.0.0.1:{WEB_PORT}"
    print(f"Hybrid Telnet + UDP Cockpit running at {url}")
    webbrowser.open(url)
    await asyncio.Event().wait()

if __name__ == "__main__":
    asyncio.run(main())