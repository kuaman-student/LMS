import ctypes
import os
import socket
import threading

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Disable Windows WinError 10054 (ICMP Port Unreachable on UDP)
if os.name == "nt":
  SIO_UDP_CONNRESET = 0x9800000C
  bytes_returned = ctypes.c_ulong()
  enable = ctypes.c_bool(False)
  ctypes.windll.ws2_32.WSAIoctl(
      sock.fileno(),
      SIO_UDP_CONNRESET,
      ctypes.byref(enable),
      ctypes.sizeof(enable),
      None,
      0,
      ctypes.byref(bytes_returned),
      None,
      None,
  )

sock.bind(("0.0.0.0", 9000))


def forward_to_teleplot():
  while True:
    try:
      data, addr = sock.recvfrom(2048)
      if addr[0] == "192.168.4.1":
        teleplot_lines = []
        for line in data.decode(errors="ignore").splitlines():
          if line.startswith(">"):
            teleplot_lines.append(line[1:])
          elif line.strip():
            teleplot_lines.append(line.replace(":", "="))
            print(f"\r{line}\n> ", end="", flush=True)  # <-- Isko elif ke andar indent kar do!
        if teleplot_lines:
          payload = ("\n".join(teleplot_lines) + "\n").encode()
          sock.sendto(payload, ("127.0.0.1", 47269))
    except Exception:
      pass

threading.Thread(target=forward_to_teleplot, daemon=True).start()
print(
    "ProjectX Bridge Active! Commands: start | stop | status | set"
    " kp/ki/kd/base <val>"
)

while True:
  cmd = input("> ").strip()
  if cmd:
    sock.sendto(cmd.encode(), ("192.168.4.1", 8080))
