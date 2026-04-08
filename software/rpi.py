from __future__ import annotations

import RPi.GPIO as GPIO
import math
import time
import json
import signal
import socket
import threading
import time
import uuid
from typing import Callable, Optional, Tuple

from zeroconf import ServiceInfo, ServiceListener, Zeroconf

SERVICE_TYPE = "_p2psignal._tcp.local."
DEFAULT_PORT = 50000
HANDSHAKE_FREE = True  # kept for readability; protocol is line-based JSON.

running = threading.Event()
running.set()

zc = Zeroconf()
node_id = uuid.uuid4().hex[:12]
display_name = socket.gethostname()
server_socket: Optional[socket.socket] = None


motor1_pins = [24, 25, 8, 7]
motor2_pins = [12, 16, 20, 21]

dremel_pin = 26
pump_pin = 0

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(pump_pin, GPIO.OUT)
GPIO.output(pump_pin, GPIO.LOW)
GPIO.setup(dremel_pin, GPIO.OUT)
GPIO.output(dremel_pin, GPIO.LOW)

for pin in motor2_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

for pin in motor1_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
GPIO.setup(18, GPIO.IN, GPIO.PUD_UP)

sequence = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]

STEPS_PER_REV = 4096  # punkt startowy dla 28BYJ-48
STEPS_PER_MM = 86

def local_ipv4() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 1))
        return s.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        s.close()


def handle_message(peer_addr: Tuple[str, int], message: str, conn: socket.socket) -> None:
    """Example reaction. Replace this with your own logic."""
    print(f"[app] from {peer_addr}: {message}")

    if message == "START":
        print("[app] received START -> do something here")
    elif message == "PUMP_ON":
        GPIO.output(pump_pin, GPIO.HIGH)
    elif message == "PUMP_OFF":
        GPIO.output(pump_pin, GPIO.LOW)
    elif message == "DREMEL_ON":
        GPIO.output(dremel_pin, GPIO.HIGH)
    elif message == "DREMEL_OFF":
        GPIO.output(dremel_pin, GPIO.LOW)
    elif message.startswith("ALARM:"):
        level = message.split(":", 1)[1]
        print(f"[app] alarm level = {level}")
    elif message.startswith("SPIN:"):
        angle = float(message.split(":", 1)[1])
        print(f"[app] rotate motor2 {angle}")
        if angle > 0:
            direction=-1
        else:
            direction = 1
            angle = angle * -1
        angle = angle * 2.2
        rotate_degrees(angle, motor_pins=motor2_pins, direction=direction, delay=0.0005)
    elif message.startswith("GO:"):
        distance = message.split(":", 1)[1]
        distance = int(float(distance))
        if distance > 0:
            direction=-1
        else:
            direction = 1
            distance = distance * -1
        steps = distance * STEPS_PER_MM
        steps = int(math.ceil(steps))
        rotate(steps=steps, direction=direction)
    elif message == "STOP":
        print("[app] received STOP -> stop your action here")
    elif message == "HOME":
        result = home()
        send_lock = threading.Lock()
        send_json(conn, send_lock, {
            "type": "done",
            "command": "HOME",
            "ok": True,
            "data": {"home_distance": result}
        })
    else:
        print("[app] generic command processed")


def advertise() -> None:
    ip = local_ipv4()
    info = ServiceInfo(
        SERVICE_TYPE,
        f"{display_name}-{node_id}.{SERVICE_TYPE}",
        addresses=[socket.inet_aton(ip)],
        port=server_socket.getsockname()[1] if server_socket else DEFAULT_PORT,
        properties={
            b"id": node_id.encode("utf-8"),
            b"name": display_name.encode("utf-8"),
        },
        server=f"{socket.gethostname()}.local.",
    )
    zc.register_service(info)
    return info


def start_server(port: int = DEFAULT_PORT) -> None:
    global server_socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.listen(5)
    sock.settimeout(1.0)
    server_socket = sock


def stop() -> None:
    running.clear()
    try:
        if server_socket is not None:
            server_socket.close()
    except OSError:
        pass
    try:
        zc.close()
    except Exception:
        pass


def client_loop(conn: socket.socket, addr: Tuple[str, int]) -> None:
    send_lock = threading.Lock()
    try:
        file = conn.makefile("r", encoding="utf-8", newline="\n")
        while running.is_set():
            line = file.readline()
            if not line:
                break

            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue

            msg_type = msg.get("type")

            if msg_type == "ping":
                send_json(conn, send_lock, {"type": "pong"})
                continue

            if msg_type == "msg":
                payload = str(msg.get("payload", ""))

                send_json(conn, send_lock, {"type": "accepted", "command": payload})

                threading.Thread(
                    target=process_command,
                    args=(addr, payload, conn, send_lock),
                    daemon=True
                ).start()
                continue
    except Exception as exc:
        print(f"[rx] client error from {addr}: {exc}")
    finally:
        try:
            conn.close()
        except OSError:
            pass
        print(f"[rx] disconnected {addr}")


def server_loop() -> None:
    assert server_socket is not None
    while running.is_set():
        try:
            conn, addr = server_socket.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        print(f"[rx] connected {addr}")
        threading.Thread(target=client_loop, args=(conn, addr), daemon=True).start()

def rotate_degrees(degrees, delay=0.002, direction=1, motor_pins=None):
    if motor_pins is None:
        motor_pins = motor1_pins

    steps = int(math.ceil(STEPS_PER_REV * degrees / 360.0))
    seq = sequence if direction == 1 else list(reversed(sequence))

    for i in range(steps):
        step = seq[i % len(seq)]
        for pin, value in zip(motor_pins, step):
            GPIO.output(pin, value)
        time.sleep(delay)
    for pin, value in zip(motor_pins, step):
        GPIO.output(pin, GPIO.LOW)
        
    
def rotate(steps, delay=0.002, direction=1, motor_pins=None):
    if motor_pins is None:
        motor_pins = motor1_pins

    seq = sequence if direction == 1 else list(reversed(sequence))

    for i in range(steps):
        step = seq[i % len(seq)]
        for pin, value in zip(motor_pins, step):
            GPIO.output(pin, value)
        time.sleep(delay)
    for pin, value in zip(motor_pins, step):
        GPIO.output(pin, GPIO.LOW)
            
def home():
    home_distance = 0
    home_state = GPIO.input(18)
    if home_state == GPIO.HIGH:
        rotate(228)
        home_distance = home_distance - 228
    while True:
            home_state = GPIO.input(18)
            if home_state == GPIO.LOW:
                rotate(5, direction=-1)
                home_distance = home_distance + 5
            else:
                rotate(228)
                home_distance = home_distance - 228
                while True:
                    home_state = GPIO.input(18)
                    if home_state == GPIO.LOW:
                        rotate(5, direction=-1)
                        home_distance = home_distance + 5
                    else:
                        home_distance = float(home_distance)
                        home_distance = home_distance / STEPS_PER_MM
                        return home_distance
                break

action_lock = threading.Lock()

def send_json(conn: socket.socket, send_lock: threading.Lock, payload: dict) -> None:
    data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
    with send_lock:
        conn.sendall(data)
        
def process_command(peer_addr: Tuple[str, int], message: str, conn: socket.socket, send_lock: threading.Lock) -> None:
    try:
        with action_lock:
            handle_message(peer_addr, message, conn)
        send_json(conn, send_lock, {"type": "done", "command": message, "ok": True})
    except Exception as exc:
        send_json(conn, send_lock, {"type": "error", "command": message, "ok": False, "error": str(exc)})

def main() -> int:
    start_server(DEFAULT_PORT)
    info = advertise()
    print(f"[rx] node_id={node_id}")
    print(f"[rx] advertised as {display_name} on {local_ipv4()}:{server_socket.getsockname()[1]}")
    print("[rx] waiting for sender messages...")

    def _shutdown(*_args: object) -> None:
        stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        server_loop()
    finally:
        stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
