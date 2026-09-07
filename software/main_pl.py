from __future__ import annotations
import os
import readchar
import configparser
import subprocess
import glob
import time
import tkinter as tk
from tkinter import filedialog
import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
import serial
from threading import Event
import serial.tools.list_ports
import sys
import re
import argparse
import csv
from scipy.interpolate import griddata
import math
import io
from collections import deque
import socket
import threading
import uuid
from typing import Dict, Optional, Tuple
from zeroconf import ServiceBrowser, ServiceListener, Zeroconf
import trimesh

xa = 0
ya = 0
x = 0
y = 0
polaczenie = 0
Z_RR=0.0

root = tk.Tk()
root.withdraw()

CONFIG_FILE = "config.ini"
FOOTPRINT_DB_FILE = "footprint_sizes.json"
CALIBRATION_FILE = 'homography.npy'
CALIBRATION_FILE_UP = 'homography_up.npy'
COLOR_CALIBRATION_FILE = 'color_params.json'

# Bazowe polecenie
cmd = ["pcb2gcode"]
vias = ["pcb2gcode"]

# Lista plików do wyboru
pliki = ["Górna warstwa", "Dolna warstwa", "outline", "vias", "through holes"]
sciezki = {}  # Słownik do przechowywania ścieżek

home = [
    "$H",
    "G92 X-16.4 Y-18.2 Z0"
]

home_z = [
    "$HZ"
]

gcode_zdj = [
    "G90 G0 Y192 X72.6"
]

unlock = [
    "$X"
]

cfg = configparser.ConfigParser()
if os.path.exists(CONFIG_FILE):
    cfg.read(CONFIG_FILE)
    if "Connection" not in cfg:
        raise RuntimeError("Brak sekcji [Connection] w config.ini mimo tego że plik istnieje. Najprawdopodobnie jest on uszkodzony.")
    conn = cfg["Connection"]
    try:
        pnp_cam_x = float(conn.get("camera_up_x_mm"))
        pnp_cam_y = float(conn.get("camera_up_y_mm"))
    except Exception:
        raise RuntimeError("Brak camera_up_x_mm / camera_up_y_mm w sekcji [Connection] mimo tego że plik config.ini istnieje. Najprawdopodobnie jest on uszkodzony.")

    pnp_cam_x = pnp_cam_x + float(conn.get("pnp_offset_x_mm"))
    pnp_cam_y = pnp_cam_y + float(conn.get("pnp_offset_y_mm"))
    gcode_zdj_up = [f"G90 G0 X{pnp_cam_x} Y{pnp_cam_y}"]
else:
    cfg["Settings"] = {
        "tool_diam": "0.1643",
        "multi_depth": "0.5",
        "passes": "1",
        "cut_z": "0.12",
        "pass_overlap": "10",
        "spindle": "0",
        "end_move": "2",
        "xyfeedrate": "120",
        "zfeedrate": "20",
        "travel_z": "2",
        "zdrill": "1.6",
        "milldrill_diameter": "1.5",
        "zcut_outline": "1.4",
        "outline_xyfeedrate": "70",
        "bridges": "4",
        "bridgesnum": "0",
        "zbridges": "1",
        "delta": "5",
        "zlevelmax": "0.15",
        "zlevelmin": "-3",
        "levelfeed": "10"
    }
    cfg["Connection"] = {
        "camera_up_x_mm": "123.2",
        "camera_up_y_mm": "176.1",
        "camera_rotation_deg": "0",
        "camera_up_rotation_deg": "0",
        "camera_ip": "",
        "camera_up_source": "",
        "serial_port": "",
        "pnp_offset_x_mm": "-9",
        "pnp_offset_y_mm": "40",
        "camera_up_corners": "127.6,179.4;127.6,176.4;122,176.4;122,179.4"
    }
    with open(CONFIG_FILE, "w") as config:
        cfg.write(config)
    print("NIE ZNALEZIONO PLIKU KONFIGURACYJNEGO config.ini (został on usunięty lub uszkodzony)! Utworzono nowy z wartościami domyślnymi. PROSZĘ NATYCHMIAST WYKONAĆ WSTĘPNĄ KONFIGURACJĘ POPRZEZ WYBRANIE OPCJI 2 NA NASTĘPNYM EKRANIE A NASTĘPNIE PONOWNIE URUCHOMIĆ PROGRAM!")
    input()



BAUD_RATE = 115200


SERVICE_TYPE = "_p2psignal._tcp.local."
DEFAULT_DISCOVERY_PORT = 50000
CONNECT_TIMEOUT_SECONDS = 3.0
HEARTBEAT_INTERVAL_SECONDS = 2.0
STALE_PONG_SECONDS = 6.0
DISCOVERY_RETRY_SECONDS = 2.0

_lock = threading.Lock()
_running = False
_zc: Optional[Zeroconf] = None
_browser: Optional[ServiceBrowser] = None
_discovery_thread: Optional[threading.Thread] = None
_recv_thread: Optional[threading.Thread] = None
_heartbeat_thread: Optional[threading.Thread] = None

_node_id = uuid.uuid4().hex[:12]
_display_name = socket.gethostname()
_peer_info: Optional[Tuple[str, int, str, str]] = None  # (peer_id, host, port, peer_name)
_sock: Optional[socket.socket] = None
_sock_lock = threading.Lock()
_last_pong = 0.0
_last_error: Optional[str] = None

_response_event = threading.Event()
_response_lock = threading.Lock()
_last_response = None
_pending_command = None

def _local_ipv4() -> str:
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


def _drop_socket(sock: Optional[socket.socket]) -> None:
    if sock is None:
        return
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        sock.close()
    except OSError:
        pass


def _set_disconnected(reason: str = "") -> None:
    global _sock, _last_error
    with _sock_lock:
        old = _sock
        _sock = None
        _last_error = reason or _last_error
    _drop_socket(old)


class _ReceiverDiscovery(ServiceListener):
    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        if not _running:
            return
        try:
            info = zc.get_service_info(type_, name, timeout=2000)
        except Exception:
            return
        if info is None:
            return

        props = info.properties or {}
        peer_id = props.get(b"id", b"").decode("utf-8", errors="ignore")
        peer_name = props.get(b"name", b"receiver").decode("utf-8", errors="ignore")
        if not peer_id or peer_id == _node_id:
            return

        host = next((addr for addr in info.parsed_addresses() if "." in addr), None)
        if host is None:
            return

        global _peer_info
        with _lock:
            _peer_info = (peer_id, host, int(info.port), peer_name)

    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        # If the receiver disappears, the TCP connection will be dropped by the heartbeat.
        pass

    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        self.add_service(zc, type_, name)


def start_sender(display_name: Optional[str] = None, service_type: str = SERVICE_TYPE) -> None:
    """Start discovery and auto-connect to the first discovered receiver."""
    global _running, _zc, _browser, _discovery_thread, _recv_thread, _heartbeat_thread, _display_name, _last_error

    if _running:
        return

    _display_name = display_name or socket.gethostname()
    _running = True
    _last_error = None

    # Zeroconf browser that discovers the receiver.
    _zc = Zeroconf()
    _browser = ServiceBrowser(_zc, service_type, _ReceiverDiscovery())

    _discovery_thread = threading.Thread(target=_discovery_loop, daemon=True)
    _discovery_thread.start()

    _heartbeat_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
    _heartbeat_thread.start()

    #print(f"[sender] node_id={_node_id}")
    #print(f"[sender] name={_display_name}")
    #print("[sender] waiting for receiver discovery...")


def stop_sender() -> None:
    """Stop background threads and close the TCP connection."""
    global _running, _zc
    _running = False
    _set_disconnected("stopped")

    try:
        if _zc is not None:
            _zc.close()
    except Exception:
        pass
    _zc = None


def _discovery_loop() -> None:
    while _running:
        if _sock is None:
            with _lock:
                peer = _peer_info
            if peer is not None:
                peer_id, host, port, peer_name = peer
                if _connect_to_peer(peer_id, host, port, peer_name):
                    # connection established
                    pass
        time.sleep(DISCOVERY_RETRY_SECONDS)


def _connect_to_peer(peer_id: str, host: str, port: int, peer_name: str) -> bool:
    global _sock, _last_pong, _last_error, _recv_thread
    with _sock_lock:
        if _sock is not None:
            return True

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(CONNECT_TIMEOUT_SECONDS)
        try:
            sock.connect((host, port))
            sock.settimeout(None)  # keep idle connections alive
            _sock = sock
            _last_pong = time.monotonic()
            _last_error = None
        except Exception as exc:
            _last_error = str(exc)
            try:
                sock.close()
            except OSError:
                pass
            return False

    _recv_thread = threading.Thread(
        target=_recv_loop, args=(sock, peer_id, host, port, peer_name), daemon=True
    )
    _recv_thread.start()
    #print(f"[sender] connected to {peer_name} ({peer_id}) at {host}:{port}")
    return True


def _recv_loop(sock: socket.socket, peer_id: str, host: str, port: int, peer_name: str) -> None:
    global _last_pong, _last_error, _last_response, _pending_command
    try:
        file = sock.makefile("r", encoding="utf-8", newline="\n")
        while _running:
            line = file.readline()
            if not line:
                break

            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                # Ignore malformed or legacy lines.
                continue

            msg_type = msg.get("type")
            if msg_type in ("pong"):
                _last_pong = time.monotonic()
            elif msg_type == "ack":
                _last_pong = time.monotonic()
            elif msg_type in ("accepted", "progress"):
                None
            elif msg_type in ("done", "error"):
                with _response_lock:
                    _last_response = msg
                _response_event.set()
            else:
                # Receiver may optionally send application messages back.
                print(f"[sender] rx from {peer_name}: {msg}")
    except Exception as exc:
        _last_error = f"recv error: {exc}"
    finally:
        print(f"[sender] disconnected from {peer_name} ({peer_id})")
        _set_disconnected("recv loop ended")


def _heartbeat_loop() -> None:
    while _running:
        time.sleep(HEARTBEAT_INTERVAL_SECONDS)

        if not is_connected():
            continue

        try:
            _send_json({"type": "ping", "ts": time.time()})
        except Exception:
            _set_disconnected("heartbeat failed")


def _send_json(payload: dict) -> None:
    sock = _sock
    if sock is None:
        raise ConnectionError("not connected")

    data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
    with _sock_lock:
        sock.sendall(data)


def send_message(text: str, timeout: float = 30.0):
    """Send a command and block until receiver replies with done/error.

    Returns the final response dict on success.
    Raises RuntimeError on receiver error.
    Raises TimeoutError if no final response arrives in time.
    """
    if not is_connected():
        raise ConnectionError("not connected")

    global _pending_command, _last_response
    with _response_lock:
        _pending_command = text
        _last_response = None
        _response_event.clear()

    try:
        _send_json({"type": "msg", "payload": text})
    except Exception:
        _set_disconnected("send failed")
        raise

    if not _response_event.wait(timeout):
        raise TimeoutError(f"no done/error response for command: {text}")

    with _response_lock:
        resp = _last_response

    if not resp:
        raise RuntimeError("response event set, but no response stored")

    if resp.get("type") == "error":
        raise RuntimeError(resp.get("error", "receiver returned error"))

    return resp

def is_connected() -> bool:
    """Best-effort connection state.

    Returns True only when:
      - a TCP socket exists,
      - the socket still looks open,
      - and the last PONG arrived recently.
    """
    sock = _sock
    if sock is None:
        return False

    try:
        sock.getpeername()
    except OSError:
        return False

    if (time.monotonic() - _last_pong) > STALE_PONG_SECONDS:
        return False
    return True


def peers_snapshot() -> Dict[str, Tuple[str, int, str]]:
    """Return the last discovered receiver info for debugging."""
    with _lock:
        peer = _peer_info
    if peer is None:
        return {}
    peer_id, host, port, peer_name = peer
    return {peer_id: (host, port, peer_name)}


def last_error() -> Optional[str]:
    return _last_error

def format_value(value):
    """Formatuje wartość: bez '.0' dla liczb całkowitych, z kropką dla ułamkowych."""
    return str(int(value)) if value == int(value) else str(value)

def save_millproject(config):
    millproject_content = f"""
# Pcb2GCode settings
metric=true
metricoutput=true
zero-start=true
zsafe={format_value(config['travel_z'])}
zchange=5
software=custom
mirror-axis=1

# Milling - Trace engraving
mill-vertfeed=20
zwork=-{format_value(config['cut_z'])}mm
mill-feed={format_value(config['xyfeedrate'])}
mill-speed={format_value(config['spindle'])}
mill-diameters={format_value(config['tool_diam'])}mm
isolation-width=0.55mm
milling-overlap={format_value(config['pass_overlap'])}%

# Drilling
zdrill=-{format_value(config['zdrill'])}
zmilldrill=-{format_value(config['zdrill'])}
drill-side=back
drill-feed={format_value(config['zfeedrate'])}
drill-speed={format_value(config['spindle'])}
drills-available=0.3mm,0.4mm,0.5mm,0.6mm,0.7mm,0.8mm,0.9mm
milldrill-diameter={format_value(config['milldrill_diameter'])}mm
min-milldrill-hole-diameter={format_value(config['milldrill_diameter'])}mm

# Outline
zcut=-{format_value(config['zcut_outline'])}
cut-side=back
cut-feed={format_value(config['outline_xyfeedrate'])}
cut-vertfeed={format_value(config['zfeedrate'])}
cut-speed={format_value(config['spindle'])}
cut-infeed={format_value(config['multi_depth'])}
cutter-diameter={format_value(config['milldrill_diameter'])}mm
bridges={format_value(config['bridges'])}
bridgesnum={format_value(config['bridgesnum'])}
zbridges=-{format_value(config['zbridges'])}

# GRBL shenanigans
nog64=true
nog81=true
nog91-1=true
"""
    return millproject_content

def cls():
    os.system('cls' if os.name == 'nt' else 'clear')

def manual_calibration(img_bgr, real_coords=None):
    """
    Ręczna kalibracja homografii: klik 4 punkty kalibracyjne.
    img_bgr: obraz w przestrzeni BGR (numpy.ndarray)
    real_coords: lista 4 krotek (X,Y) w docelowej przestrzeni.
    Zapisuje macierz H do pliku.
    """
    if real_coords is None:
        real_coords = [(0, 0), (0, 194), (194, 194), (194, 0)]

    if img_bgr is None:
        raise ValueError("Przekazany obraz jest pusty (None)")

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(img_rgb)
    ax.set_title('Kliknij kolejno: (0, 0), (0, 194), (194, 194), (194, 0)')
    pts = []

    def onclick(event):
        if event.xdata is None or event.ydata is None:
            return
        pts.append([event.xdata, event.ydata])
        ax.plot(event.xdata, event.ydata, 'rx', markersize=5)
        fig.canvas.draw()
        if len(pts) == 4:
            plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    if len(pts) != 4:
        print(f"Kalibracja przerwana, znaleziono tylko {len(pts)} punktów.")
        return

    src = np.array(pts, dtype=np.float32)
    dst = np.array(real_coords, dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    np.save(CALIBRATION_FILE, H)
    print(f"Zapisano macierz homografii w {CALIBRATION_FILE}")

def manual_color_calibration(img_bgr):
    """
    Ręczna kalibracja koloru: klik punkty na laminacie.
    img_bgr: obraz w przestrzeni BGR (numpy.ndarray)
    Zapisuje parametry koloru do JSON.
    """
    if img_bgr is None:
        raise ValueError("Przekazany obraz jest pusty (None)")

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(img_rgb)
    ax.set_title('Kliknij miejsca na laminacie, zamknij okno po skończeniu')
    samples = []

    def onclick(event):
        if event.xdata is None or event.ydata is None:
            return
        x, y = int(event.xdata), int(event.ydata)
        samples.append(img_hsv[y, x])
        ax.plot(x, y, 'bo', markersize=5)
        fig.canvas.draw()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    if not samples:
        print("Brak próbek koloru. Kalibracja przerwana.")
        return

    arr = np.array(samples, dtype=np.float32)
    H0, S0, V0 = arr.mean(axis=0)
    dH, dS, dV = 15, 50, 50
    lower = [int(max(0, H0 - dH)), int(max(0, S0 - dS)), int(max(0, V0 - dV))]
    upper = [int(min(179, H0 + dH)), int(min(255, S0 + dS)), int(min(255, V0 + dV))]

    with open(COLOR_CALIBRATION_FILE, 'w') as f:
        json.dump({'lower': lower, 'upper': upper}, f, indent=2)
    print(f"Zapisano parametry koloru w {COLOR_CALIBRATION_FILE}")

def load_homography(camera='top'):
    """
    Zwraca macierz H (image -> machine mm) dla 'top' lub 'up'.
    'top' używa istniejącego CALIBRATION_FILE, 'up' używa CALIBRATION_FILE_UP.
    Rzuca FileNotFoundError jeśli brak pliku.
    """
    if camera == 'top':
        if not os.path.isfile(CALIBRATION_FILE):
            raise FileNotFoundError("Brak homografii. Wykonaj kalibrację homografii.")
        return np.load(CALIBRATION_FILE)
    elif camera == 'up':
        if not os.path.isfile(CALIBRATION_FILE_UP):
            raise FileNotFoundError("Brak homografii dla kamery up. Wykonaj kalibrację (manual_calibration_up).")
        return np.load(CALIBRATION_FILE_UP)
    else:
        raise ValueError("camera must be 'top' or 'up'")
    
def pixel_to_machine_using_homography(px, py, camera='up'):
    """
    px,py - pixel coords (x=col,y=row) w obrazie danej kamery.
    Dla 'top' używa load_homography(), dla 'up' load_homography_for('up').
    Zwraca (X_mm, Y_mm).
    """
    H = load_homography(camera)
    p = np.array([[[float(px), float(py)]]], dtype=np.float32)
    mm = cv2.perspectiveTransform(p, H)[0][0]
    return float(mm[0]), float(mm[1])

def load_color_params():
    if not os.path.isfile(COLOR_CALIBRATION_FILE):
        raise FileNotFoundError("Brak kalibracji koloru. Wykonaj kalibrację koloru.")
    data = json.load(open(COLOR_CALIBRATION_FILE))
    return np.array(data['lower'], dtype=np.uint8), np.array(data['upper'], dtype=np.uint8)

def detect_copper_corner(img_bgr):
    """
    Wykrywa róg laminatu bez używania ścieżki pliku.
    img_bgr: obraz w przestrzeni BGR (numpy.ndarray)
    Zwraca:
      X, Y – współrzędne rogu po transformacji projekcyjnej
    """
    # Załaduj parametry tylko raz, jeżeli nie zostały przekazane
    H = load_homography()
    lower, upper = load_color_params()

    if img_bgr is None:
        raise ValueError("Przekazany obraz jest pusty (None)")

    # Konwersja na RGB i HSV
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # 1) Oryginał
    plt.figure(figsize=(6,6))
    plt.imshow(img_rgb)
    plt.title('1) Oryginalne zdjęcie')
    plt.axis('off')
    plt.show()

    # 2) Maska koloru
    mask = cv2.inRange(img_hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    plt.figure(figsize=(6,6))
    plt.imshow(mask, cmap='gray')
    plt.title('2) Maska koloru laminatu')
    plt.axis('off')
    plt.show()

def detect_copper_origin(img_bgr, width_mm, height_mm):
    """
    Znajduje optymalny punkt początkowy (X0, Y0) na laminacie,
    który minimalizuje odpad z lewej i dolnej krawędzi,
    uwzględniając wymiary PCB.

    img_bgr: obraz BGR
    width_mm, height_mm: wymiary PCB w milimetrach
    Zwraca: (X0, Y0) w mm
    """
    # 1) Wczytaj kalibracje
    H = load_homography()
    lower, upper = load_color_params()

    # Podgląd etapów detekcji przez matplotlib, nie cv2.imshow — OpenCV-HighGUI (Qt)
    # ma trwały konflikt z backendem strumienia sieciowego górnej kamery w tym
    # środowisku. Matplotlib (jak w kalibracji kolorów/rogów) działa niezawodnie.
    def _show_stage(img, title, cmap=None):
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(img, cmap=cmap)
        ax.set_title(title + " — zamknij okno aby przejść dalej")
        ax.axis('off')
        plt.show()

    # 2) Przygotuj maskę laminatu
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    _show_stage(mask, "Maska koloru", cmap='gray')

    # 3) Wyznacz kontur laminatu
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = mask.shape
    area = h * w
    candidates = [c for c in contours if cv2.contourArea(c) > 0.001 * area]
    if not candidates:
        input("Brak koonturu laminatu")
        return False
    board_contour = max(candidates, key=lambda c: cv2.contourArea(c)).reshape(-1, 2)
    vis_cont = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    cv2.polylines(vis_cont, [board_contour.astype(int)], True, (255, 0, 0), 2)
    _show_stage(vis_cont, "Kontur laminatu")

    # 4) Przelicz w mm->px dla czterech narożników PCB i oblicz bounding box
    H_inv = np.linalg.inv(H)
    
    # punkty narożników w mm
    pts_mm = np.array([[[0, 0],
                       [width_mm, 0],
                       [width_mm, height_mm],
                       [0, height_mm]]], dtype=np.float32)
    pts_px = cv2.perspectiveTransform(pts_mm, H_inv)[0]
    xs = pts_px[:, 0]
    ys = pts_px[:, 1]
    min_x, max_x = xs.min(), xs.max()
    min_y, max_y = ys.min(), ys.max()
    kx = int(np.ceil(max_x - min_x))
    ky = int(np.ceil(max_y - min_y))
    print(f'[DEBUG] Kernel size: kx={kx}, ky={ky}')
    if kx < 1 or ky < 1:
        raise ValueError('Niepoprawne wymiary PCB lub homografia niewłaściwa.')

    # 5) Znajdź wszystkie piksele maski
    mask_pts = np.column_stack(np.where(mask > 0))  # (row, col)
    h, w = mask.shape
    valid = []
    # dla każdego punktu sprawdź, czy prostokąt (kx x ky) mieści się całkowicie w masce
    for row, col in mask_pts:
        if col + kx <= w and row + ky <= h:
            # wycinek maski
            window = mask[row:row+ky, col:col+kx]
            if window.shape == (ky, kx) and np.all(window > 0):
                valid.append((row, col))
    if not valid:
        raise RuntimeError(f'Brak miejsca na PCB o wymiarach {width_mm:.1f}x{height_mm:.1f} mm.')

    # 6) Finalny wybór z valid optymalizujący pozycję
    # Możemy minimalizować:
    # a) sumę odpadów: col*ky + row*kx
    # b) największy odpad: max(col*ky, row*kx)
    # c) maksymalną odległość od krawędzi: max(col, row)
    # Tutaj użyjemy minimalizacji maksymalnego odpadu (wariant b):
    costs = [max(col * ky, row * kx) for row, col in valid]
    idx = int(np.argmin(costs))
    row, col = valid[idx]

    # 7) Przelicz piksel -> mm piksel -> mm
    p_pix = np.array([[[float(col), float(row)]]], dtype=np.float32)
    mm = cv2.perspectiveTransform(p_pix, H)[0][0]
    X0, Y0 = float(mm[0]), float(mm[1])

    # 9) Wizualizacja finalna: punkt + obrys PCB
    vis = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    cv2.circle(vis, (int(col), int(row)), 5, (0, 255, 0), -1)
    cv2.rectangle(vis,
                  (int(col), int(row)),
                  (int(col + kx), int(row + ky)),
                  (0, 255, 0), 2)
    _show_stage(vis, "Punkt 0 i obrys PCB")

    return X0, Y0

    
def select_file_dialog(title="Wybierz plik"): 
    """Otwiera okno dialogowe do wyboru pliku przed wywołaniem akcji."""
    file_path = filedialog.askopenfilename(title=title,
                                           filetypes=[("Obrazy", "*.png *.jpg *.jpeg *.bmp"), ("Wszystkie pliki", "*.*")])
    return file_path if file_path else None

def capture_write(filename="image.jpeg", port=0, ramp_frames=30, x=1440, y=1080):
    camera = cv2.VideoCapture(f"http://{ip}")
    print("Użyto starej funkcji capture write!")
    # Set Resolution
    camera.set(3, x)
    camera.set(4, y)

    # Adjust camera lightin g
    for i in range(ramp_frames):
        temp = camera.read()
    retval, im = camera.read()
    cv2.imwrite(filename,im)
    del(camera)
    return True

def open_video_capture_from_source(source):
    """
    Otwiera i zwraca cv2.VideoCapture dla zadanego źródła.
    source może być:
      - URL (http://..., rtsp://...) -> IP camera
      - device path (Linux) '/dev/video2'
      - indeks OpenCV '0', '1' (string lub int)
    Zwraca otwarty obiekt VideoCapture lub None jeśli nie udało się otworzyć.
    """
    src = str(source).strip()
    # URL (IP camera)
    if src.startswith("http://") or src.startswith("https://") or src.startswith("rtsp://"):
        cap = cv2.VideoCapture(src)
        if cap.isOpened():
            return cap
        return None

    # Linux device path
    if sys.platform.startswith("linux") and src.startswith("/dev/"):
        cap = cv2.VideoCapture(src)
        if cap.isOpened():
            return cap
        # fallback: spróbuj indeksu extractowanego z nazwy (/dev/video2 -> 2)
        m = re.search(r"(\d+)$", src)
        if m:
            idx = int(m.group(1))
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                return cap
        return None

    # numeric index (Windows/Linux)
    if src.isdigit():
        idx = int(src)
        # Na Windows spróbuj DirectShow lub MSMF (czasem stabilniejsze niż domyślny backend)
        if sys.platform.startswith("win"):
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                if cap.isOpened():
                    return cap
            except:
                pass
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
                if cap.isOpened():
                    return cap
            except:
                pass
        # fallback: domyślne otwarcie
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            return cap
        return None

    # fallback attempt (spróbuj otworzyć jako ścieżkę)
    try:
        cap = cv2.VideoCapture(src)
        if cap.isOpened():
            return cap
    except:
        pass
    return None

def list_possible_local_cameras(max_index=8):
    """
    Szybkie narzędzie: sprawdza indeksy 0..max_index oraz /dev/video* (Linux).
    Zwraca listę znalezionych źródeł jako stringi (np. '0', '/dev/video2').
    """
    found = []
    for i in range(max_index + 1):
        cap = None
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                found.append(str(i))
                cap.release()
        except:
            if cap:
                cap.release()
    if sys.platform.startswith("linux"):
        devs = glob.glob("/dev/video*")
        for d in devs:
            if d not in found:
                found.append(d)
    return found

def read_camera_rotation_from_config(camera):
    if camera == 'top':
        """Czyta camera_rotation_deg z sekcji Connection w CONFIG_FILE (domyślnie 0)."""
        cfg = configparser.ConfigParser()
        cfg.read(CONFIG_FILE)
        rot = 0
        if "Connection" in cfg:
            try:
                rot = int(float(cfg["Connection"].get("camera_rotation_deg", "0")))
            except Exception:
                rot = 0
        return rot
    else:
        cfg = configparser.ConfigParser()
        cfg.read(CONFIG_FILE)
        rot = 0
        if "Connection" in cfg:
            try:
                rot = int(float(cfg["Connection"].get("camera_up_rotation_deg", "0")))
            except Exception:
                rot = 0
        return rot       
    
def capture_write_unified(filename="image.jpeg", camera='top', camera_source=None, ramp_frames=30, width=None, height=None):
    """
    Uniwersalna wersja capture_write.
    camera_source:
      - jeśli None: spróbuje użyć zmiennej globalnej 'ip' jeśli istnieje (backward compat),
        lub (lepiej) odczyta wartość z CONFIG_FILE sekcja Connection: camera_source / camera_ip.
      - może być: URL, '/dev/videoX', lub '0' (index).
    Zapisuje ramkę do filename i zwraca True/raises exception.
    """
    # prefer: jeśli caller poda camera_source, użyj go; jeżeli nie, spróbuj użyć globalnej zmiennej ip
    src = camera_source
    if src is None:
        # zgodnie z Twoim oryginalnym kodem był 'ip' w globalu - zachowaj kompatybilność
        try:
            src = ip  # jeśli zdefiniowane globalnie w mainie
        except NameError:
            src = None

    # optional: jeżeli masz config.ini z sekcją Connection i camera_source, użyj jej
    if src is None:
        # zachowujemy Twoją obecną logikę load_config() — nie tworzymy nowego ensure
        try:
            cfg = configparser.ConfigParser()
            cfg.read(CONFIG_FILE)
            connection = cfg["Connection"] if "Connection" in cfg else {}
            if camera == 'top':
                src = connection.get("camera_ip", "") or connection.get("camera_source", "")
            else:
                # dla 'up' używamy klucza camera_up_source (jeśli nie ma, zostaw puste)
                src = connection.get("camera_up_source", "")
            src = src.strip() if src else None
        except Exception:
            src = None

    if src is None:
        raise RuntimeError("Brak zdefiniowanego źródła kamery (camera_source / camera_ip / global ip).")

    cap = open_video_capture_from_source(src)
    if cap is None:
        raise RuntimeError(f"Nie udało się otworzyć źródła kamery: {src}")

    # Latarka pomocnicza jest fizycznie przy głowicy PnP/kamerze w blacie —
    # nie ma potrzeby (ani sensu) włączać jej przy zdjęciach z górnej kamery.
    if camera == 'up':
        send_message("LIGHT_ON")

    # ustaw rozdzielczość jeśli podano
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # rampa (podobnie jak miałeś)
    for i in range(ramp_frames):
        retval, frame = cap.read()
        if not retval:
            time.sleep(0.02)
            retval, frame = cap.read()
        # nie zapisujemy, tylko wyrównujemy ekspozycję itp.

    retval, im = cap.read()
    if not retval or im is None:
        cap.release()
        raise RuntimeError("Kamera nie zwróciła klatki.")
    rot = read_camera_rotation_from_config(camera)
    if rot != 0:
        im = rotate_image(im, rot)
    cv2.imwrite(filename, im)
    cap.release()
    if camera == 'up':
        send_message("LIGHT_OFF")
    return True


def load_config():
    config = configparser.ConfigParser()
    if os.path.exists(CONFIG_FILE):
        config.read(CONFIG_FILE)
    else:
        config["Settings"] = {
            "tool_diam": "0.1643",
            "multi_depth": "0.5",
            "passes": "1",
            "cut_z": "0.12",
            "pass_overlap": "10",
            "spindle": "0",
            "end_move": "2",
            "xyfeedrate": "120",
            "zfeedrate": "20",
            "travel_z": "2",
            "zdrill": "1.6",
            "milldrill_diameter": "1.5",
            "zcut_outline": "1.4",
            "outline_xyfeedrate": "70",
            "bridges": "4",
            "bridgesnum": "0",
            "zbridges": "1",
            "delta": "5",
            "zlevelmax": "0.15",
            "zlevelmin": "-3",
            "levelfeed": "10",
            "holder_pocket_scale": "1.0"
        }
        config["Connection"] = {
            "camera_up_x_mm": "123.2",
            "camera_up_y_mm": "176.1",
            "camera_rotation_deg": "0",
            "camera_up_rotation_deg": "0",
            "camera_ip": "",
            "camera_up_source": "",
            "serial_port": "",
            "pnp_offset_x_mm": "-9",
            "pnp_offset_y_mm": "40",
            "camera_up_corners": "127.6,179.4;127.6,176.4;122,176.4;122,179.4"
        }
        with open(CONFIG_FILE, "w") as configfile:
            config.write(configfile)
        print("NIE ZNALEZIONO PLIKU KONFIGURACYJNEGO config.ini (został on usunięty lub uszkodzony)! Utworzono nowy z wartościami domyślnymi. PROSZĘ NATYCHMIAST WYKONAĆ WSTĘPNĄ KONFIGURACJĘ POPRZEZ WYBRANIE OPCJI 2 NA NASTĘPNYM EKRANIE!")
        input()
    # migracja: dopisz nowe ustawienia, jeśli plik config.ini pochodzi ze starszej wersji programu
    if "holder_pocket_scale" not in config["Settings"]:
        config["Settings"]["holder_pocket_scale"] = "1.0"
        with open(CONFIG_FILE, "w") as configfile:
            config.write(configfile)
    settings = {k: float(v) for k, v in config["Settings"].items()}
    # Parsujemy połączenia (mogą być puste)
    conn = config["Connection"]
    serial_port = conn.get("serial_port", "").strip()
    camera_ip  = conn.get("camera_ip", "").strip()
    camera_up_ip = conn.get("camera_up_source", "").strip()
    return settings, serial_port, camera_ip, camera_up_ip

def save_config(config_dict):
    config = configparser.ConfigParser()
    if os.path.exists(CONFIG_FILE):
        config.read(CONFIG_FILE)
    if "Settings" not in config:
        config["Settings"] = {}
    for key, value in config_dict.items():
        if isinstance(value, float) and value.is_integer():
            config["Settings"][key] = str(int(value))
        else:
            config["Settings"][key] = str(value)
    with open(CONFIG_FILE, "w") as configfile:
        config.write(configfile)
    
def remove_comment(string):
    # 1) usuń komentarze od średnika
    line = string.split(';', 1)[0]
    # 2) usuń wszystkie nawiasowe komentarze
    #    (.*? ) – non-greedy, usunie każde '(' ... ')'
    line = re.sub(r'\(.*?\)', '', line)
    # 3) obetnij białe znaki na brzegach
    return line.strip()

def remove_eol_chars(string):
    # removed \n or trailing spaces
    return string.strip()


def send_wake_up(ser):
    # Wake up
    # Hit enter a few times to wake the Printrbot
    ser.write(str.encode("\r\n\r\n"))
    time.sleep(2)   # Wait for Printrbot to initialize
    ser.flushInput()  # Flush startup text in serial input


def wait_for_movement_completion(ser, cleaned_line):

    Event().wait(1)

    if cleaned_line not in ('$X', '$$'):

        idle_counter = 0

        while True:

            Event().wait(0.05)
            ser.write(b'?')
            grbl_out = ser.readline()
            grbl_response = grbl_out.strip().decode('utf-8', errors='ignore')

            if not grbl_response:
                continue

            if grbl_response.lower().startswith('alarm') or 'Alarm' in grbl_response:
                raise RuntimeError(
                    f"GRBL zgłosił ALARM podczas oczekiwania na ruch '{cleaned_line}': {grbl_response}. "
                    f"Maszyna prawdopodobnie uderzyła w krańcówkę lub napotkała błąd. "
                    f"Sprawdź maszynę, a następnie odblokuj alarm ($X) przed kontynuowaniem."
                )

            if grbl_response != 'ok':
                if 'Idle' in grbl_response:
                    idle_counter += 1
                else:
                    idle_counter = 0

            if idle_counter > 10:
                break
    return

def stream_gcode(ser, gcode_path):
    """Wyślij G‑code z pliku, używając już otwartego ser."""
    with open(gcode_path, "r") as file:
        for line in file:
            cleaned = remove_eol_chars(remove_comment(line))
            if cleaned:
                print("Sending:", cleaned)
                ser.write((line + '\n').encode())
                wait_for_movement_completion(ser, cleaned)
                print(" ->", ser.readline().strip().decode())
        print("End of file")

def stream_gcode_list(ser, gcode_lines):
    """Wyślij G-code z listy linia po linii."""
    for line in gcode_lines:
        cleaned = remove_eol_chars(remove_comment(line))
        if not cleaned:
            continue

        print("Sending:", cleaned)
        ser.write((cleaned + '\n').encode('ascii'))
        ser.flush()

        if cleaned == '$H' or cleaned.startswith('G38.2') or cleaned.startswith('G0') or cleaned.startswith('G1') or cleaned.startswith('G2') or cleaned.startswith('G3') or cleaned.startswith('G90') or cleaned.startswith('G91'):
            wait_for_movement_completion(ser, cleaned)
            continue

        # tylko czekaj na "ok"
        while True:
            resp = ser.readline().strip().decode('utf-8', errors='ignore')
            if not resp:
                continue
            print(" ->", resp)
            if resp.lower() == 'ok':
                break
            if resp.lower().startswith('error') or resp.lower().startswith('alarm'):
                raise RuntimeError(f"GRBL błąd po '{cleaned}': {resp}")

    print("End of list")

def choose_serial_port():
    """Wyświetla listę dostępnych portów i pozwala wybrać jeden z nich.
       Na Linuxie domyślnie filtruje do /dev/ttyUSB* i /dev/ttyACM*."""
    ports = list(serial.tools.list_ports.comports())
    # na Linuxie najpierw pokaż tylko USB i ACM
    if sys.platform.startswith('linux'):
        usb_ports = [p for p in ports if 'ttyUSB' in p.device or 'ttyACM' in p.device]
        if usb_ports:
            ports = usb_ports

    if not ports:
        print("Nie wykryto żadnych portów szeregowych!")
        exit(1)

    print("Dostępne porty szeregowe:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")

    try:
        idx = int(input(f"Wybierz port (0–{len(ports)-1}): "))
        return ports[idx].device
    except (ValueError, IndexError):
        print("Nieprawidłowy wybór.")
        exit(1)

def choose_gcode_file():
    """Otwiera okienko dialogowe, aby użytkownik wybrał plik .nc/.gcode."""
    root = tk.Tk()
    root.withdraw()  # nie pokazuj głównego okna Tk
    file_path = filedialog.askopenfilename(
        title="Wybierz plik G-code",
        filetypes=[("G-code files", "*.nc *.gcode"), ("Wszystkie pliki", "*.*")]
    )
    if not file_path:
        print("Nie wybrano pliku.")
        exit(1)
    return file_path

def save_connections(port, ip, ip_up):
    config = configparser.ConfigParser()
    config.read(CONFIG_FILE)
    if "Connection" not in config:
        config["Connection"] = {}
    config["Connection"]["serial_port"] = port
    config["Connection"]["camera_ip"]  = ip
    config["Connection"]["camera_up_source"]  = ip_up
    with open(CONFIG_FILE, "w") as cfg:
        config.write(cfg)


def parse_dimensions(infile):
    """
    Czyta plik Gerber (.gbr/.gtl/...) lub Excellon (.drl) i zwraca (width_mm, height_mm).
    Dla Gerbera liczy bbox geometrii z uwzględnieniem aktywnej apertury (Dnn),
    więc wynik jest dużo bliższy pcb2gcode niż bbox samych współrzędnych.
    """
    import re
    import sys
    import math

    def detect_units_and_format(lines):
        is_drill = False
        unit = None
        dec_digits = None

        mo_re = re.compile(r"^%MO([A-Z]+)\*%")
        fs_re = re.compile(r"%FS.*X(\d)(\d)Y(\d)(\d)\*%")
        inch_re = re.compile(r"^INCH", re.IGNORECASE)
        metric_re = re.compile(r"^METRIC", re.IGNORECASE)
        file_fmt_re = re.compile(r"^;FILE_FORMAT=(\d+):(\d+)")

        for line in lines:
            s = line.strip()
            if s.startswith("M48"):
                is_drill = True
                continue

            if is_drill:
                if unit is None:
                    if inch_re.match(s):
                        unit = "IN"
                    elif metric_re.match(s):
                        unit = "MM"
                if dec_digits is None:
                    m = file_fmt_re.match(s)
                    if m:
                        dec_digits = int(m.group(2))
            else:
                if unit is None:
                    m = mo_re.match(s)
                    if m:
                        unit = "IN" if m.group(1) == "IN" else "MM"
                if dec_digits is None:
                    m = fs_re.search(s)
                    if m:
                        dec_digits = int(m.group(2))

            if unit and dec_digits is not None:
                break

        if unit is None:
            print(f"[{infile}] Ostrzeżenie: nie wykryto jednostek w nagłówku, przyjmuję MM.", file=sys.stderr)
            unit = "MM"
        if dec_digits is None:
            print(f"[{infile}] Ostrzeżenie: nie wykryto formatu, przyjmuję 5 miejsc po przecinku.", file=sys.stderr)
            dec_digits = 5

        return is_drill, unit, dec_digits

    def parse_num_list(s):
        return [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", s)]

    def rot_bbox_extents(w, h, deg):
        th = math.radians(deg % 180.0)
        c = abs(math.cos(th))
        s = abs(math.sin(th))
        ex = (w * 0.5) * c + (h * 0.5) * s
        ey = (w * 0.5) * s + (h * 0.5) * c
        return ex, ey

    try:
        with open(infile, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()

        lines = text.splitlines()
        is_drill, unit, dec = detect_units_and_format(lines)
        scale = 10 ** dec
        unit_mm = 25.4 if unit == "IN" else 1.0

        if is_drill:
            # Zachowanie podobne do Twojej starej wersji dla Excellon:
            xmin = ymin = float("inf")
            xmax = ymax = float("-inf")
            coord_re = re.compile(r"^X(-?\d+)Y(-?\d+)")
            for lineno, raw in enumerate(lines, start=1):
                line = raw.strip()
                m = coord_re.match(line)
                if not m:
                    continue
                try:
                    x_raw = int(m.group(1))
                    y_raw = int(m.group(2))
                    x = x_raw / scale * unit_mm
                    y = y_raw / scale * unit_mm
                    xmin = min(xmin, x)
                    xmax = max(xmax, x)
                    ymin = min(ymin, y)
                    ymax = max(ymax, y)
                except Exception as e:
                    print(f"[{infile}] Błąd parsowania linii {lineno}: '{line}' → {e}", file=sys.stderr)

            if xmin == float("inf"):
                print(f"[{infile}] Nie znaleziono żadnych współrzędnych X/Y.", file=sys.stderr)
                return 0.0, 0.0

            return xmax - xmin, ymax - ymin

        # --- Gerber path ---
        # parsowanie macro definicji i aperture table
        macro_defs = {}
        for m in re.finditer(r"%AM([A-Za-z0-9_]+)\*(.*?)\*%", text, re.S):
            macro_defs[m.group(1).upper()] = m.group(2)

        apertures = {}
        add_re = re.compile(r"%ADD(\d+)([A-Za-z0-9_]+),([^*]+)\*%")
        for line in lines:
            s = line.strip()
            m = add_re.match(s)
            if not m:
                continue
            dcode = int(m.group(1))
            kind = m.group(2).upper()
            params_raw = m.group(3).strip()
            params = parse_num_list(params_raw)

            ap = {"kind": kind, "params": params, "raw": params_raw}

            # Prostokąt / obround w prostym macro
            macro_body = macro_defs.get(kind, "")
            macro_body_norm = macro_body.replace(" ", "")
            if kind.startswith("C"):
                ap["shape"] = "CIRCLE"
            elif kind.startswith("R"):
                ap["shape"] = "RECT"
            elif kind in macro_defs or macro_body_norm:
                # Najczęstszy przypadek u Ciebie: %AM...*21,1,$1,$2,0,0,$3*%
                if "21,1,$1,$2,0,0,$3" in macro_body_norm:
                    ap["shape"] = "RECT_MACRO"
                else:
                    ap["shape"] = "MACRO"
            else:
                ap["shape"] = "UNKNOWN"

            apertures[dcode] = ap

        def aperture_extents(ap):
            if ap is None:
                return 0.0, 0.0

            kind = ap.get("shape", "UNKNOWN")
            p = ap.get("params", [])

            if kind == "CIRCLE":
                if not p:
                    return 0.0, 0.0
                r = abs(p[0]) * 0.5
                return r, r

            if kind == "RECT":
                if len(p) >= 2:
                    w_ap = abs(p[0])
                    h_ap = abs(p[1])
                    rot = p[2] if len(p) >= 3 else 0.0
                    return rot_bbox_extents(w_ap, h_ap, rot)
                if len(p) == 1:
                    r = abs(p[0]) * 0.5
                    return r, r
                return 0.0, 0.0

            if kind == "RECT_MACRO":
                # format typu 21,1,$1,$2,0,0,$3 -> pierwszy i drugi parametr to w/h,
                # trzeci może być rotacją
                if len(p) >= 2:
                    w_ap = abs(p[0])
                    h_ap = abs(p[1])
                    rot = p[2] if len(p) >= 3 else 0.0
                    return rot_bbox_extents(w_ap, h_ap, rot)
                return 0.0, 0.0

            if kind == "MACRO":
                # bezpieczny fallback: bierz pierwsze 2 parametry jako w/h,
                # jeśli są 3 — traktuj 3-ci jako rotację
                if len(p) >= 2:
                    w_ap = abs(p[0])
                    h_ap = abs(p[1])
                    rot = p[2] if len(p) >= 3 else 0.0
                    return rot_bbox_extents(w_ap, h_ap, rot)
                if len(p) == 1:
                    r = abs(p[0]) * 0.5
                    return r, r
                return 0.0, 0.0

            if len(p) >= 2:
                return abs(p[0]) * 0.5, abs(p[1]) * 0.5
            if len(p) == 1:
                r = abs(p[0]) * 0.5
                return r, r
            return 0.0, 0.0

        def update_bbox_point(x, y, ap, bbox):
            xmin, xmax, ymin, ymax = bbox
            ex, ey = aperture_extents(ap)
            xmin = min(xmin, x - ex)
            xmax = max(xmax, x + ex)
            ymin = min(ymin, y - ey)
            ymax = max(ymax, y + ey)
            return xmin, xmax, ymin, ymax

        def update_bbox_segment(x1, y1, x2, y2, ap, bbox):
            xmin, xmax, ymin, ymax = bbox
            ex, ey = aperture_extents(ap)
            xmin = min(xmin, min(x1, x2) - ex)
            xmax = max(xmax, max(x1, x2) + ex)
            ymin = min(ymin, min(y1, y2) - ey)
            ymax = max(ymax, max(y1, y2) + ey)
            return xmin, xmax, ymin, ymax

        xmin = ymin = float("inf")
        xmax = ymax = float("-inf")

        current_ap = None
        current_op = None
        last_pt = None

        coord_re = re.compile(r"X(-?\d+)Y(-?\d+)")
        dcode_re = re.compile(r"D0([123])\*?$")
        select_ap_re = re.compile(r"^D(\d+)\*$")

        for lineno, raw in enumerate(lines, start=1):
            s = raw.strip()
            if not s or s.startswith("G04"):
                continue

            msel = select_ap_re.match(s)
            if msel:
                dnum = int(msel.group(1))
                current_ap = apertures.get(dnum, current_ap)
                continue

            m = coord_re.search(s)
            if not m:
                continue

            try:
                x_raw = int(m.group(1))
                y_raw = int(m.group(2))
                x = x_raw / scale * unit_mm
                y = y_raw / scale * unit_mm
            except Exception as e:
                print(f"[{infile}] Błąd parsowania linii {lineno}: '{s}' → {e}", file=sys.stderr)
                continue

            mop = dcode_re.search(s)
            if mop:
                current_op = int(mop.group(1))

            op = current_op
            if op is None:
                continue

            if op == 2:
                last_pt = (x, y)
                continue

            if op == 3:
                xmin, xmax, ymin, ymax = update_bbox_point(x, y, current_ap, (xmin, xmax, ymin, ymax))
                last_pt = (x, y)
                continue

            if op == 1:
                if last_pt is None:
                    last_pt = (x, y)
                    continue
                x1, y1 = last_pt
                xmin, xmax, ymin, ymax = update_bbox_segment(x1, y1, x, y, current_ap, (xmin, xmax, ymin, ymax))
                last_pt = (x, y)
                continue

        if xmin == float("inf"):
            print(f"[{infile}] Nie znaleziono żadnych współrzędnych X/Y.", file=sys.stderr)
            return 0.0, 0.0

        width = xmax - xmin
        height = ymax - ymin
        return width, height

    except FileNotFoundError:
        print(f"[{infile}] Błąd: plik nie istnieje.", file=sys.stderr)
    except PermissionError:
        print(f"[{infile}] Błąd: brak praw dostępu.", file=sys.stderr)
    except Exception as e:
        print(f"[{infile}] Nieoczekiwany błąd: {e}", file=sys.stderr)

    return 0.0, 0.0

def parse_probe_response(response):
    """
    Parsuje odpowiedź w formacie [PRB:X,Y,Z:...]
    i zwraca wartość Z jako float.
    """
    # Wzorzec: PRB: <liczba>,<liczba>,<liczba> (ignorujemy potem dwukropek i resztę)
    pattern = r"PRB:\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)"
    match = re.search(pattern, response)
    if match:
        # match.group(1) → X, match.group(2) → Y, match.group(3) → Z
        return float(match.group(3))
    # Gdy brak dopasowania, można rzucić wyjątek lub zwrócić None
    return None

# Probe a single XY point and return Z
def probe_point(ser, x, y, zlevelmax, zlevelmin, levelfeed):
    # Move XY
    stream_gcode_list(ser, [f"G90 G0 X{x} Y{y}"])
    # Send probe command and read PRB response
    cmd = f"G38.2 Z{zlevelmin} F{levelfeed}"
    print("Sending:", cmd)
    ser.write((cmd + "\n").encode())
    z = None
    start_time = time.time()
    # read until PRB: or timeout
    while time.time() - start_time < 10:
        line = ser.readline().strip().decode('utf-8', errors='ignore')
        if not line:
            continue
        print(" ->", line)
        if 'PRB:' in line:
            z = parse_probe_response(line)
            break
    if z is None:
        raise RuntimeError(f"Probe failed, no PRB response received")
    # Retract to safe Z
    stream_gcode_list(ser, [f"G90 G0 Z{zlevelmax}"])
    return z

# Main auto-level function
def auto_level(ser, w_points, h_points, delta_w, delta_h, zlevelmax, zlevelmin, levelfeed, Z_RR, output_file="probe_data.csv"):
    data = []
    # Podnieś wrzeciono do wysokości startowej i ustaw zero
    startt = [f"G90 G0 Z{zlevelmax}"]
    stream_gcode_list(ser, startt)

    for i in range(int(h_points)):
        y = i * delta_h
        for j in range(int(w_points)):
            x = j * delta_w
            # zmierz oryginalne Z
            z_measured = probe_point(ser, x, y, zlevelmax, zlevelmin, levelfeed)
            # odjęcie korekty
            z_corrected = -Z_RR+z_measured-float(cut_z)
            # zapisz skorygowany wynik
            data.append((x, y, z_corrected))

    # Zapisz do CSV
    with open(output_file, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["x", "y", "z"])
        writer.writerows(data)

    print(f"Probe data saved to {output_file}")
    return data

def determine_z_zero(ser, feed1_0, retract_0, feed2_0):
    # First probe
    cmd1 = f"G38.2 Z-50 F{feed1_0}"
    print("Sending first probe:", cmd1)
    ser.write((cmd1 + "\n").encode())
    z1 = None
    start = time.time()
    while True:
        raw = ser.readline().strip().decode('utf-8', errors='ignore')
        if not raw:
            continue
        print(" ->", raw)
        if 'PRB:' in raw:
            z1 = parse_probe_response(raw)
            break
    if z1 is None:
        raise RuntimeError("First probe failed: no PRB response received")

    # Retract Z axis
    stream_gcode_list(ser, [f"G91 G0 Z{retract_0}"])

    # Second probe (allow override of feed/z_stop)
    cmd2 = f"G38.2 Z-3 F{feed2_0}"
    print("Sending second probe:", cmd2)
    ser.write((cmd2 + "\n").encode())
    z2 = None
    start = time.time()
    while True:
        raw = ser.readline().strip().decode('utf-8', errors='ignore')
        if not raw:
            continue
        print(" ->", raw)
        if 'PRB:' in raw:
            z2 = parse_probe_response(raw)
            break
    if z2 is None:
        raise RuntimeError("Second probe failed: no PRB response received")
    
    time.sleep(2)
    stream_gcode_list(ser, [f"G92 Z0"]) # Set the current position to Z=0
    stream_gcode_list(ser, [f"G91 G0 Z{retract_0}"])    
    

    # Return the machine's measured Z (to be used as Z_RR offset)
    return z2

def load_probe_csv(csv_path="probe_data.csv"):
    xs, ys, zs = [], [], []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            zs.append(float(row['z']))
    return np.array(xs), np.array(ys), np.array(zs)

def make_interpolator(xs, ys, zs, method='cubic'):
    pts = np.vstack((xs, ys)).T
    def interp(xq, yq):
        xq_arr = np.atleast_1d(xq)
        yq_arr = np.atleast_1d(yq)
        zq = griddata(pts, zs, (xq_arr, yq_arr), method=method)
        # fallback nearest dla punktów poza konweksem
        mask = np.isnan(zq)
        if mask.any():
            zq[mask] = griddata(pts, zs,
                                (xq_arr[mask], yq_arr[mask]),
                                method='nearest')
        return zq
    return interp

MOTION_RE = re.compile(
    r'^(?P<prefix>G0?1\s+[^XY\n]*?)'    # G1 or G01
    r'X(?P<X>-?\d+\.?\d*)\s+'
    r'Y(?P<Y>-?\d+\.?\d*)'
    r'(?:\s+(?P<suffix>.*))?$'
)

def apply_height_to_gcode(in_path, out_path, interp_fn, z_format="{:.4f}"):
    with open(in_path, 'r') as src, open(out_path, 'w') as dst:
        for raw in src:
            line = raw.rstrip('\n')
            up = line.lstrip().upper()

            # 1) If it's a rapid move (G0 or G00), just copy it
            if up.startswith('G0 ') or up.startswith('G00 '):
                dst.write(raw)
                continue

            # 2) Try matching a G1/G01 move
            m = MOTION_RE.match(line)
            if not m:
                dst.write(raw)
                continue

            # 3) Inject interpolated Z
            x = float(m.group('X'))
            y = float(m.group('Y'))
            z = interp_fn([x], [y])[0]
            rebuilt = (
                f"{m.group('prefix')}"
                f"X{m.group('X')} Y{m.group('Y')} "
                f"Z{z_format.format(z)}"
            )
            if m.group('suffix'):
                rebuilt += f" {m.group('suffix')}"
            dst.write(rebuilt + "\n")

def batch_apply_levels(gcode_paths):
    xs, ys, zs = load_probe_csv()
    interp = make_interpolator(xs, ys, zs, method='cubic')
    for path in gcode_paths:
        base, ext = os.path.splitext(path)
        out = f"{base}_leveled{ext}"
        print(f"Leveling {path} → {out}")
        apply_height_to_gcode(path, out, interp)
    print("Gotowe: wszystkie pliki _leveled")
    

def stream_gcode_file_pipelined(ser, gcode_path, parser_buffer=127, timeout_s=60.0):
    """
    Wysyła G-code z pliku, pilnując kolejki wysłanych linii.
    Jeśli pojedyncza linia jest dłuższa niż parser_buffer, wysyła ją osobno
    i czeka na 'ok' zanim przejdzie dalej.
    """
    pending_lengths = deque()
    pending_bytes = 0
    warned_once = {"flag": False}  # ostrzeż tylko raz na cały plik, nie przy każdym oczekiwaniu

    def read_one_response():
        start = time.time()
        while True:
            elapsed = time.time() - start
            if elapsed > timeout_s:
                return None
            if elapsed > 5.0 and not warned_once["flag"]:
                # Zniwelowany (leveled) G-code ma dużo więcej, krótszych segmentów niż
                # zwykły plik — GRBL na Arduino Nano potrafi legalnie "zapchać się"
                # na dłużej niż kilka sekund przy gęstych ścieżkach/wolnym posuwie.
                # To ostrzeżenie informuje, że program wciąż czeka, a nie wisi.
                print(f"[info] Czekam na odpowiedź GRBL już {elapsed:.0f}s — maszyna prawdopodobnie po prostu wykonuje gęste segmenty, program nie zawiesił się. (ten komunikat pojawi się tylko raz)")
                warned_once["flag"] = True
            raw = ser.readline()
            if not raw:
                continue
            resp = raw.decode("ascii", errors="ignore").strip()
            if not resp:
                continue
            low = resp.lower()
            if low == "ok":
                return "ok"
            if low.startswith("error") or low.startswith("alarm"):
                raise RuntimeError(f"GRBL zwrócił błąd: {resp}")
            # inne wiadomości ignorujemy

    with open(gcode_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = remove_eol_chars(remove_comment(raw))
            if not line:
                continue

            payload = (line + "\n").encode("ascii", errors="ignore")
            length = len(payload)

            # jeśli jedna linia jest większa niż bufor, nie próbuj "pipeliningu"
            # dla niej — wyślij ją osobno i poczekaj na ok
            if length > parser_buffer:
                print(f"[WARN] Linia ma {length} bajtów, więcej niż parser_buffer={parser_buffer}: {line}")
                ser.write(payload)
                ser.flush()

                resp = read_one_response()
                if resp != "ok":
                    raise TimeoutError("Timeout lub brak 'ok' po wysłaniu długiej linii G-code.")
                continue

            # czekaj tylko wtedy, gdy faktycznie mamy już coś w locie
            while pending_lengths and pending_bytes + length > parser_buffer:
                resp = read_one_response()
                if resp is None:
                    raise TimeoutError("Timeout czekania na odpowiedź 'ok' od GRBL.")
                if resp == "ok":
                    pending_bytes -= pending_lengths.popleft()
                    if pending_bytes < 0:
                        pending_bytes = 0

            ser.write(payload)
            ser.flush()
            pending_lengths.append(length)
            pending_bytes += length

    # opróżnij resztę
    while pending_lengths:
        resp = read_one_response()
        if resp is None:
            raise TimeoutError("Timeout podczas opróżniania bufora GRBL po wysłaniu pliku.")
        if resp == "ok":
            pending_bytes -= pending_lengths.popleft()
            if pending_bytes < 0:
                pending_bytes = 0

    print("End of file (pipelined)")

def _resize_for_display(img, max_dim=900):
    """Skaluje obraz do podglądu tak, by żaden bok nie przekraczał max_dim px
    (zachowując proporcje). Bardzo duże obrazy (np. z kamery górnej) potrafią
    powodować, że okno OpenCV nie wyrenderuje się poprawnie na niektórych
    konfiguracjach (zwłaszcza Wayland) i waitKey wraca natychmiast bez realnego
    naciśnięcia klawisza — stąd wrażenie, że zdjęcia 'przelatują' błyskawicznie."""
    h, w = img.shape[:2]
    scale = max_dim / max(h, w)
    if scale >= 1.0:
        return img
    return cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)

def _show_rotation_grid_and_choose(variants, prompt_prefix="Wpisz wybraną rotację (0/90/180/270): "):
    """
    Pokazuje 4 warianty obrotu (0/90/180/270) jako jeden złożony obraz — przez
    matplotlib, a nie cv2.imshow. Okazało się, że to konkretnie OpenCV-HighGUI (Qt)
    ma trwały konflikt z backendem strumienia sieciowego górnej kamery w tym
    środowisku (waitKey zaczynał zwracać ten sam kod za każdym razem, niezależnie
    od liczby okien). Matplotlib używa zupełnie innego mechanizmu okien — tego
    samego, którego już niezawodnie używają "Kalibracja kolorów" i "kalibracja rogów".
    """
    disp = [_resize_for_display(v, max_dim=500) for _, v in variants]
    h = max(im.shape[0] for im in disp)
    w = max(im.shape[1] for im in disp)
    padded = []
    for (deg, _), im in zip(variants, disp):
        canvas = np.zeros((h + 30, w, 3), dtype=np.uint8)
        canvas[30:30 + im.shape[0], 0:im.shape[1]] = im
        cv2.putText(canvas, f"{deg} deg", (5, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        padded.append(canvas)
    top_row = np.hstack([padded[0], padded[1]])
    bottom_row = np.hstack([padded[2], padded[3]])
    grid_bgr = np.vstack([top_row, bottom_row])
    grid_rgb = cv2.cvtColor(grid_bgr, cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots(figsize=(9, 6.5))
    ax.imshow(grid_rgb)
    ax.set_title("Wybierz orientację (podpisana na obrazie) i zamknij okno")
    ax.axis('off')
    plt.show()

    choice = input(prompt_prefix).strip()
    return choice

def rotate_image(img, deg):
    """Szybka rotacja o 0/90/180/270 stopni; dla innych kątów używa warpAffine."""
    deg = deg % 360
    if deg == 0:
        return img
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # ogólny przypadek (rzadko używany)
    (h, w) = img.shape[:2]
    center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, deg, 1.0)
    cos = abs(M[0, 0]); sin = abs(M[0, 1])
    # new bounds
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    # adjust
    M[0, 2] += (nW / 2) - center[0]
    M[1, 2] += (nH / 2) - center[1]
    return cv2.warpAffine(img, M, (nW, nH))

def detect_nozzle_pixel(img_bgr, debug_show=False, expected_radius_px=None):
    if img_bgr is None:
        return None

    h, w = img_bgr.shape[:2]
    center = (w/2.0, h/2.0)
    diag = math.hypot(w, h)

    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    try:
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        gray_clahe = clahe.apply(gray)
    except:
        gray_clahe = gray.copy()
    blurred = cv2.medianBlur(gray_clahe, 5)

    # Hough
    if expected_radius_px:
        minR = max(4, int(expected_radius_px * 0.6))
        maxR = max(8, int(expected_radius_px * 1.6))
    else:
        minR = max(4, int(min(w,h) * 0.03))
        maxR = max(10, int(min(w,h) * 0.25))

    try:
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2,
                                   minDist=max(8, minR*2),
                                   param1=100, param2=100,
                                   minRadius=minR, maxRadius=maxR)
    except:
        circles = None

    # edges for strength measurement and contour fallback
    edges = cv2.Canny(blurred, 50, 150)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)
    contours, _ = cv2.findContours(edges_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []  # dicts: {type, cx,cy, r, circ, area, edge_strength, score}

    # helper: edge strength along circle perimeter (mask ring)
    def edge_strength_at_circle(cx, cy, r, thickness=3):
        mask = np.zeros_like(edges, dtype=np.uint8)
        cv2.circle(mask, (int(cx), int(cy)), int(r), 255, thickness)
        # mean edge value on ring
        vals = edges[mask==255]
        if vals.size == 0:
            return 0.0
        return float(vals.mean()) / 255.0  # normalized 0..1

    # add Hough candidates
    if circles is not None:
        for (cx, cy, r) in np.round(circles[0]).astype(int):
            d = math.hypot(cx - center[0], cy - center[1])
            # estimate circularity via edges on circle (approx)
            es = edge_strength_at_circle(cx, cy, r, thickness=3)
            # approximate circularity unknown -> set medium (0.5) but factor edge strength
            circ = es * 0.9 + 0.1
            candidates.append({
                "type":"hough", "cx":int(cx), "cy":int(cy), "r":int(r),
                "circ":circ, "area":math.pi*r*r, "edge":es, "d":d
            })

    # add contour candidates (with real circularity)
    for c in contours:
        area = cv2.contourArea(c)
        if area < 30:
            continue
        perim = cv2.arcLength(c, True)
        if perim <= 0:
            continue
        circularity = 4.0 * math.pi * (area / (perim*perim))
        (cx_f, cy_f), radius = cv2.minEnclosingCircle(c)
        if radius < 1:
            continue
        # skip extremely small/large
        if radius < minR*0.5 or radius > maxR*2.0:
            continue
        d = math.hypot(cx_f - center[0], cy_f - center[1])
        es = edge_strength_at_circle(cx_f, cy_f, radius, thickness=3)
        candidates.append({
            "type":"contour", "cx":int(round(cx_f)), "cy":int(round(cy_f)),
            "r":int(round(radius)), "circ":float(circularity), "area":area, "edge":es, "d":d
        })

    if not candidates:
        if debug_show:
            print("No candidates found.")
        return None

    # scoring weights (tuneable)
    w_dist = 0.6      # prefer small distance from center
    w_circ = 1.0      # prefer high circularity
    w_edge = -0.8     # prefer high edge strength (negative because higher edge reduces score)
    w_rad = 0.4       # prefer radius close to expected

    # compute normalized score for each candidate (lower = better)
    for c in candidates:
        dist_norm = c["d"] / diag  # 0..~1
        circ_score = 1.0 - max(0.0, min(1.0, c.get("circ", 0.0)))  # 0 if circ=1, 1 if circ=0
        edge_score = 1.0 - c.get("edge", 0.0)  # lower is better
        if expected_radius_px:
            rad_diff = abs(c["r"] - expected_radius_px) / max(1.0, expected_radius_px)
        else:
            # if no expected radius, prefer medium radii less strongly
            rad_diff = 0.0
        score = w_dist * dist_norm + w_circ * circ_score + w_rad * rad_diff + w_edge * edge_score
        c["score"] = float(score)

    # Prefer any contour with very high circularity outright
    high_circ_contours = [c for c in candidates if c["type"]=="contour" and c["circ"] >= 0.70]
    if high_circ_contours:
        best = min(high_circ_contours, key=lambda x: x["score"])
    else:
        best = min(candidates, key=lambda x: x["score"])

    cx, cy = int(best["cx"]), int(best["cy"])

    if debug_show:
        vis = img_bgr.copy()
        # draw all houghs blue, contours red, annotate scores
        for c in candidates:
            color = (255,0,0) if c["type"]=="hough" else (0,0,255)
            cv2.circle(vis, (int(c["cx"]), int(c["cy"])), int(max(3, c["r"])), color, 1)
            cv2.putText(vis, f"{c['score']:.2f}", (int(c["cx"])+6, int(c["cy"])+6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        # draw best in green
        cv2.circle(vis, (cx,cy), 8, (0,255,0), 2)
        cv2.putText(vis, f"BEST ({cx},{cy})", (cx+10, cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)
        win = "detect_nozzle_improved_scored"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        try:
            cv2.resizeWindow(win, min(1200, w), min(900, h))
        except:
            pass
        cv2.imshow(win, vis)
        print("Wybrano punkt:", (cx,cy), "score:", best["score"])
        cv2.waitKey(0)
        time.sleep(3)
        cv2.destroyWindow(win)

    return (cx, cy)

# --- kalibracja offsetu PnP (iteracyjna) ---
def pnpp_calibrate_tool_offset(ser, tol_mm=0.1, max_iter=12, show_debug=False):
    """
    Iteracyjna kalibracja: dążymy aby wykryta pozycja końcówki (kamera 'up')
    pokryła się z zadaną pozycją maszyny (connection.camera_up_x_mm, camera_up_y_mm).
    Po zakończeniu zapisuje pnp_offset_x_mm,pnp_offset_y_mm do sekcji [Connection] w CONFIG_FILE.
    ser: otwarty serial.Serial (połączenie z maszyną)
    """
    # odczytaj pozycję docelową kamery up (absolutną w mm)
    cfg = configparser.ConfigParser()
    cfg.read(CONFIG_FILE)
    if "Connection" not in cfg:
        raise RuntimeError("Brak sekcji [Connection] w config.ini — ustaw camera_up_x_mm i camera_up_y_mm")
    conn = cfg["Connection"]
    try:
        tgt_x = float(conn.get("camera_up_x_mm"))
        tgt_y = float(conn.get("camera_up_y_mm"))
    except Exception:
        raise RuntimeError("Brak camera_up_x_mm / camera_up_y_mm w sekcji [Connection]. Ustaw je (pozycja kamery w mm).")

    cumulative_dx = 0.0
    cumulative_dy = 0.0

    for it in range(1, max_iter+1):
        print(f"[PnP calib] iteracja {it}/{max_iter} — robie zdjecie kamery up...")
        capture_write_unified("pnp_calib.jpeg", camera='up')
        img = cv2.imread("pnp_calib.jpeg", cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError("Nie udało się wczytać pnp_calib.jpeg")

        # wykryj nozle na obrazie (pixel coords)
        detected = detect_nozzle_pixel(img, debug_show=show_debug)
        if detected is None:
            print("Nie wykryto końcówki na zdjęciu. Popraw pozycję i spróbuj ponownie.")
            return False

        px_x, px_y = detected  # (col, row)

        # przelicz pixel -> absolutne X,Y (mm) maszynowe przy użyciu homografii 'up'
        det_x_mm, det_y_mm = pixel_to_machine_using_homography(px_x, px_y, camera='up')

        # ile trzeba przesunąć, by nozzle znalazło się w target (tgt_x,tgt_y)
        ddx = tgt_x - det_x_mm
        ddy = tgt_y - det_y_mm
        dist = math.hypot(ddx, ddy)
        print(f"[PnP calib] detected at ({det_x_mm:.4f},{det_y_mm:.4f}) — delta -> ({ddx:.4f},{ddy:.4f}) mm, dist={dist:.4f} mm")

        # debug: pokaż obraz z naniesionymi punktami
        if show_debug:
            vis = img.copy()
            cv2.circle(vis, (int(px_x), int(px_y)), 6, (0,255,0), 2)
            # projektujemy też target px dla informacji (opcjonalnie)
            # Tutaj nie mamy target w px — mamy w mm; można obliczyć odwrotną transformację jeśli chcesz.
            cv2.imshow("pnp calib detect", vis)
            cv2.waitKey(500)
            cv2.destroyWindow("pnp calib detect")

        if dist <= tol_mm:
            print(f"[PnP calib] osiągnięto tolerancję {tol_mm} mm. Koniec.")
            break

        # wykonaj korekcyjny ruch względny (G91)
        cmd = f"G91 G01 X{ddx:.4f} Y{ddy:.4f} F10"
        print("[PnP calib] Wysyłam korekcyjny ruch:", cmd)
        stream_gcode_list(ser, [cmd])
        # powróć do trybu bezwzględnego
        stream_gcode_list(ser, ["G90"])
        time.sleep(0.2)

        cumulative_dx += ddx
        cumulative_dy += ddy

    # zapis offsetu do config (sekcja Connection)
    # jeśli już były jakieś wartości, dodajemy kumulatywnie
    prev_x = float(conn.get("pnp_offset_x_mm", "0.0"))
    prev_y = float(conn.get("pnp_offset_y_mm", "0.0"))
    conn["pnp_offset_x_mm"] = str(prev_x + cumulative_dx)
    conn["pnp_offset_y_mm"] = str(prev_y + cumulative_dy)
    with open(CONFIG_FILE, "w") as f:
        cfg.write(f)
    print(f"[PnP calib] zapisano pnp_offset_x_mm={conn['pnp_offset_x_mm']} pnp_offset_y_mm={conn['pnp_offset_y_mm']} do {CONFIG_FILE}")
    return True

def read_up_corners_from_config():
    """
    Odczytuje klucz camera_up_corners z sekcji [Connection].
    Format: "x1,y1;x2,y2;x3,y3;x4,y4"
    Zwraca listę 4 tuples [(x1,y1),...], albo None jeśli brak.
    """
    cfg = configparser.ConfigParser()
    cfg.read(CONFIG_FILE)
    if "Connection" not in cfg:
        return None
    s = cfg["Connection"].get("camera_up_corners", "").strip()
    if not s:
        return None
    try:
        parts = [p.strip() for p in s.split(';') if p.strip()]
        corners = []
        for p in parts:
            x_str, y_str = p.split(',')
            corners.append((float(x_str), float(y_str)))
        if len(corners) != 4:
            return None
        return corners
    except Exception:
        return None

def save_up_corners_to_config(corners):
    """
    corners: lista 4 (x,y)
    Zapisuje do config.ini w formacie "x1,y1;x2,y2;..."
    """
    if not corners or len(corners) != 4:
        raise ValueError("Corners must be list of 4 (x,y) tuples")
    s = ";".join(f"{x:.4f},{y:.4f}" for x, y in corners)
    cfg = configparser.ConfigParser()
    cfg.read(CONFIG_FILE)
    if "Connection" not in cfg:
        cfg["Connection"] = {}
    cfg["Connection"]["camera_up_corners"] = s
    with open(CONFIG_FILE, "w") as f:
        cfg.write(f)
    print(f"Zapisano 4 rogi kamery up do {CONFIG_FILE} (camera_up_corners).")

def calibrate_up_via_tool(ser,
                          corner_machine_coords=None,
                          z_focus=-24,
                          camera='up',
                          ramp_frames=30,
                          auto_detect=True,
                          show_debug=False,
                          allow_manual_click=True):
    """
    Rozszerzona kalibracja homografii dla kamery 'up'.
    Jeśli corner_machine_coords == None -> próbuje wczytać z config.ini.
    Podczas zbierania punktów: po autodetekcji pokazuje użytkownikowi zaznaczenie
    i pyta czy zaakceptować (t/n). Jeśli 'n' -> pozwala na ręczne kliknięcie.
    """
    # 1) jeżeli nie dostarczono cornerów, spróbuj wczytać z configu
    if corner_machine_coords is None:
        corner_machine_coords = read_up_corners_from_config()
        if corner_machine_coords is None:
            print("Brak cornerów w configu. Podaj 4 punkty ręcznie (x,y) oddzielone przecinkiem, lub przerwij.")
            corner_machine_coords = []
            for i in range(4):
                val = input(f"Podaj wspolrzedne punktu {i+1} jako 'X,Y' (Enter by przerwać): ").strip()
                if not val:
                    print("Przerwano wprowadzanie cornerów.")
                    return False
                try:
                    x_s, y_s = val.split(',')
                    corner_machine_coords.append((float(x_s), float(y_s)))
                except Exception as e:
                    print("Niepoprawny format. Spróbuj ponownie.")
                    return False
            # zapisz do configu, bo to przydatne
            save_up_corners_to_config(corner_machine_coords)

    if len(corner_machine_coords) != 4:
        raise ValueError("corner_machine_coords musi mieć 4 punkty (x,y)")

    src_pixels = []
    dst_mm = []

    print("Upewnij się, że przed kalibracją ustawiłeś camera_up_rotation_deg poprawnie.")
    input("Naciśnij Enter aby kontynuować...")

    for idx, (mx, my) in enumerate(corner_machine_coords, start=1):
        print(f"\n--- Punkt kalibracji {idx}/4: X{mx} Y{my} Z{z_focus} ---")
        # przejedź do punktu i ustaw Z (jeśli chcesz, możesz tu pominąć, bo robisz to wcześniej)
        stream_gcode_list(ser, [f"G90 G01 X{mx} Y{my} F1000"])
        stream_gcode_list(ser, [f"G90 G0 Z{z_focus}"])
        time.sleep(0.12)

        fname = f"cal_up_point_{idx}.jpg"
        capture_write_unified(filename=fname, camera=camera, ramp_frames=ramp_frames)
        img = cv2.imread(fname, cv2.IMREAD_COLOR)
        if img is None:
            print("Nie wczytano obrazu:", fname)
            return False

        chosen_px = None
        # autoprobe
        if auto_detect:
            det = detect_nozzle_pixel(img, debug_show=show_debug)
            if det:
                px, py = det
                # POKAŻ użytkownikowi ten punkt i zapytaj czy akceptuje
                vis = img.copy()
                cv2.circle(vis, (int(px), int(py)), 6, (0,255,0), 2)
                cv2.putText(vis, f"Auto: ({px},{py})", (10,20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                winname = "Auto-detekcja: zaakceptuj (t/n)"
                # stwórz okno, ustaw rozmiar adekwatny do obrazu, żeby nie było miniaturowe
                cv2.namedWindow(winname, cv2.WINDOW_NORMAL)
                h_img, w_img = vis.shape[:2]
                try:
                    cv2.resizeWindow(winname, w_img, h_img)
                except Exception:
                    pass
                cv2.imshow(winname, vis)

                print("W oknie obrazu naciśnij: 't' aby zaakceptować, 'n' aby odrzucić (ESC też = odrzuć).")
                # czekaj na klawisz w oknie (blokujące waitKey)
                while True:
                    k = cv2.waitKey(0) & 0xFF
                    if k in (ord('t'), ord('T')):
                        key = 't'
                        break
                    if k in (ord('n'), ord('N'), 27):  # 27 = ESC
                        key = 'n'
                        break

                cv2.destroyWindow(winname)
                if key == 't':
                    chosen_px = (int(px), int(py))
                else:
                    print("Odrzucono autodetekcję — przejdź do ręcznego wyboru.")
            else:
                print("Autodetekcja nie powiodła się.")
        # ręczne kliknięcie jeśli autodetekcja nie zaakceptowana
        if chosen_px is None and allow_manual_click:
            print("Kliknij na obrazie dokładne miejsce czubka frezu (zamknij okno po wyborze).")
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            fig, ax = plt.subplots(figsize=(6,6))
            ax.imshow(img_rgb)
            pts = []
            def onclick(event):
                if event.xdata is None or event.ydata is None:
                    return
                pts.append((event.xdata, event.ydata))
                ax.plot(event.xdata, event.ydata, 'rx'); fig.canvas.draw()
                if len(pts) >= 1:
                    plt.close(fig)
            fig.canvas.mpl_connect('button_press_event', onclick)
            plt.show()
            if not pts:
                print("Nie wybrano punktu. Przerywam kalibrację.")
                return False
            px, py = pts[0]
            chosen_px = (int(round(px)), int(round(py)))
            print(f"Wybrano pixel = {chosen_px}")

        if chosen_px is None:
            print("Brak punktu do zapisania. Przerywam.")
            return False

        # zapisz parę pixel -> machine
        src_pixels.append([float(chosen_px[0]), float(chosen_px[1])])
        dst_mm.append([float(mx), float(my)])
        time.sleep(0.12)

    # oblicz homografię: src_pixels -> dst_mm
    src_np = np.array(src_pixels, dtype=np.float32)
    dst_np = np.array(dst_mm, dtype=np.float32)
    H = cv2.getPerspectiveTransform(src_np, dst_np)
    np.save(CALIBRATION_FILE_UP, H)
    print(f"Zapisano homografię kamery 'up' do {CALIBRATION_FILE_UP}")
    input("Naciśnij enter aby kontynuować...")
    cls()
    return True

# Wczytanie konfiguracji przy starcie
config, saved_port, saved_ip, saved_up_ip = load_config()

if saved_port or saved_ip:
    print("Znaleziono zapisane połączenia:")
    print(f"  Port szeregowy: {saved_port or '<brak>'}")
    print(f"  Kamera górna:      {saved_ip or '<brak>'}")
    print(f"  Kamera w blacie:      {saved_up_ip or '<brak>'}")
    odp = input("Czy chcesz spróbować przywrócić te połączenia? (t/n): ").lower()
    if odp == "t":
        start_sender()
        # próba połączenia z maszyną
        if saved_port:
            try:
                ser = serial.Serial(saved_port, BAUD_RATE, timeout=1)
                send_wake_up(ser)
                print(f"✅ Połączono z maszyną na porcie {saved_port}")
                polaczenie += 1
            except Exception as e:
                print(f"❌ Nie udało się otworzyć portu {saved_port}: {e}")
        # próba połączenia z kamerą
        if saved_ip:
            try:
                # saved_ip może być dotychczasowym "http://..." albo (po zmianie) może
                # zawierać dowolne źródło kamery: URL RTSP/HTTP, '/dev/video2' lub indeks '0'
                camera_src = saved_ip  # nazwa tutaj zachowana dla kompatybilności

                # OTWIERANIE KAMERY — uniwersalny wrapper
                def _open_capture(src):
                    src = str(src).strip()
                    # URL (IP camera)
                    if src.startswith("http://") or src.startswith("https://") or src.startswith("rtsp://"):
                        cap = cv2.VideoCapture(src)
                        if cap.isOpened():
                            return cap
                        return None
                    # Linux device path
                    if sys.platform.startswith("linux") and src.startswith("/dev/"):
                        cap = cv2.VideoCapture(src)
                        if cap.isOpened():
                            return cap
                        # fallback: spróbuj indeksu wyciągniętego z nazwy (/dev/video2 -> 2)
                        m = re.search(r"(\d+)$", src)
                        if m:
                            idx = int(m.group(1))
                            cap = cv2.VideoCapture(idx)
                            if cap.isOpened():
                                return cap
                        return None
                    # numeric index (Windows/Linux)
                    if src.isdigit():
                        idx = int(src)
                        # na Windowsie spróbuj DirectShow/MSMF dla stabilności
                        if sys.platform.startswith("win"):
                            try:
                                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                                if cap.isOpened():
                                    return cap
                            except:
                                pass
                            try:
                                cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
                                if cap.isOpened():
                                    return cap
                            except:
                                pass
                        cap = cv2.VideoCapture(idx)
                        if cap.isOpened():
                            return cap
                        return None
                    # fallback
                    try:
                        cap = cv2.VideoCapture(src)
                        if cap.isOpened():
                            return cap
                    except:
                        pass
                    return None

                cap = _open_capture(camera_src)
                if cap is None:
                    print(f"❌ Nie udało się otworzyć kamery pod: {camera_src}")
                else:
                    # spróbuj odczytać jedną klatkę
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        print(f"✅ Połączono z kamerą pod adresem {camera_src}")
                        polaczenie += 1
                        # zachowaj stare zachowanie: przypisz ip tylko jeśli źródło to URL
                        if str(camera_src).startswith("http"):
                            ip = camera_src
                        else:
                            # możesz zapisać w zmiennej camera_source, aby później używać jej
                            camera_source = camera_src
                    else:
                        print(f"❌ Kamera nie odpowiedziała na {camera_src}")
            except Exception as e:
                print(f"❌ Błąd przy łączeniu z kamerą: {e}")
        if saved_up_ip:
            try:
                # saved_ip może być dotychczasowym "http://..." albo (po zmianie) może
                # zawierać dowolne źródło kamery: URL RTSP/HTTP, '/dev/video2' lub indeks '0'
                camera_src = saved_up_ip  # nazwa tutaj zachowana dla kompatybilności

                # OTWIERANIE KAMERY — uniwersalny wrapper
                def _open_capture(src):
                    src = str(src).strip()
                    # URL (IP camera)
                    if src.startswith("http://") or src.startswith("https://") or src.startswith("rtsp://"):
                        cap = cv2.VideoCapture(src)
                        if cap.isOpened():
                            return cap
                        return None
                    # Linux device path
                    if sys.platform.startswith("linux") and src.startswith("/dev/"):
                        cap = cv2.VideoCapture(src)
                        if cap.isOpened():
                            return cap
                        # fallback: spróbuj indeksu wyciągniętego z nazwy (/dev/video2 -> 2)
                        m = re.search(r"(\d+)$", src)
                        if m:
                            idx = int(m.group(1))
                            cap = cv2.VideoCapture(idx)
                            if cap.isOpened():
                                return cap
                        return None
                    # numeric index (Windows/Linux)
                    if src.isdigit():
                        idx = int(src)
                        # na Windowsie spróbuj DirectShow/MSMF dla stabilności
                        if sys.platform.startswith("win"):
                            try:
                                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                                if cap.isOpened():
                                    return cap
                            except:
                                pass
                            try:
                                cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
                                if cap.isOpened():
                                    return cap
                            except:
                                pass
                        cap = cv2.VideoCapture(idx)
                        if cap.isOpened():
                            return cap
                        return None
                    # fallback
                    try:
                        cap = cv2.VideoCapture(src)
                        if cap.isOpened():
                            return cap
                    except:
                        pass
                    return None

                cap = _open_capture(camera_src)
                if cap is None:
                    print(f"❌ Nie udało się otworzyć kamery pod: {camera_src}")
                else:
                    # spróbuj odczytać jedną klatkę
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        print(f"✅ Połączono z kamerą pod adresem {camera_src}")
                        polaczenie += 1
                        # zachowaj stare zachowanie: przypisz ip tylko jeśli źródło to URL
                        if str(camera_src).startswith("http"):
                            ip = camera_src
                        else:
                            # możesz zapisać w zmiennej camera_source, aby później używać jej
                            camera_source = camera_src
                    else:
                        print(f"❌ Kamera nie odpowiedziała na {camera_src}")
            except Exception as e:
                print(f"❌ Błąd przy łączeniu z kamerą w blacie: {e}")
        if not is_connected():
            time.sleep(5)
            if not is_connected():
                print("Błąd połączenia bezprzewodowego z rpi")
            else:
                print(f"✅ Połączono bezprzewodowo z malinką")
        else:
            print(f"✅ Połączono bezprzewodowo z malinką")
        input("Naciśnij Enter, aby kontynuować...")


def _normalize_angle_deg(a):
    return ((a + 180) % 360) - 180

def detect_component_orientation_up(img_bgr,
                                    debug_show=False,
                                    min_area_px=40,
                                    morph_kernel_close=(8,8),
                                    adaptive_blocksize=15,
                                    adaptive_C=2):
    """
    Ulepszona detekcja orientacji elementu (kamera 'up').
    Zwraca dict z kluczami:
      'angle_image'  - wybrany kąt w stopniach (obraz)
      'angle_machine'- kąt poprawiony o obrót kamery (z config)
      'centroid'     - (cx,cy) w px
      'area_px'      - area konturu
      'bbox'         - (x,y,w,h) bounding box
      'mask'         - binarna maska (uint8)
      'vis'          - RGB wizualizacja (BGR if you want raw)
    Jeśli nic nie znaleziono -> zwraca None.
    """
    if img_bgr is None:
        return None

    # 1) Konwersja, kontrast, rozmycie
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
    except:
        pass
    gray = cv2.GaussianBlur(gray, (5,5), 0)

    # 2) progowanie adaptacyjne + morfologie (cel: wydobyć element w środku)
    th = cv2.adaptiveThreshold(gray, 255,
                               cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY_INV,
                               adaptive_blocksize, adaptive_C)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_close)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)), iterations=1)

    # 3) usuń drobne punkty
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(th, connectivity=8)
    mask_clean = np.zeros_like(th)
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area_px:
            mask_clean[labels == i] = 255

    # 4) kontury
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        if debug_show:
            cv2.imwrite("test_orient_mask.jpg", mask_clean)
            cv2.imshow("mask", mask_clean); cv2.waitKey(0); cv2.destroyAllWindows()
        return None

    # 5) wybierz najlepszy kontur: preferuj area i bliskość środka
    h, w = mask_clean.shape
    cx_center, cy_center = w/2.0, h/2.0
    best = None
    best_score = 1e9
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        M = cv2.moments(c)
        if M.get("m00", 0) == 0:
            continue
        cx = M["m10"]/M["m00"]; cy = M["m01"]/M["m00"]
        dist = math.hypot(cx - cx_center, cy - cy_center)
        # score: chcemy minimalizować odległość, preferować większą area (wagowo)
        score = dist - 0.005 * area
        if score < best_score:
            best_score = score
            best = (c, cx, cy, area)

    if best is None:
        return None

    c, cx, cy, area = best
    x,y,bw,bh = cv2.boundingRect(c)
    rect = cv2.minAreaRect(c)  # ((cx,cy),(w,h),angle)
    box = cv2.boxPoints(rect).astype(int)

    # 6) PCA (główna oś) - stabilna dla prostokątnych kształtów
    pts = c.reshape(-1,2).astype(np.float32)
    try:
        mean, eigenvectors = cv2.PCACompute(pts, mean=None, maxComponents=1)
        vx, vy = eigenvectors[0]
        angle_pca = math.degrees(math.atan2(vy, vx))
        angle_pca = _normalize_angle_deg(angle_pca)
    except Exception:
        angle_pca = None

    # 7) angle z minAreaRect — zamieniamy żeby 0 = poziomo wzdłuż długiej osi
    ((rx,ry),(rw,rh),ang_rect) = rect
    if rw < rh:
        ang_rect = ang_rect + 90.0
    angle_rect = _normalize_angle_deg(ang_rect)

    # 8) fitEllipse jeśli wystarczająco punktów
    angle_ellipse = None
    if len(c) >= 5:
        try:
            ellipse = cv2.fitEllipse(c)
            angle_ellipse = _normalize_angle_deg(ellipse[2])
        except:
            angle_ellipse = None

    # 9) wybór finalnej metody
    # jeśli prostokąt ma wyraźny aspect ratio -> preferujemy angle_rect, inaczej PCA
    aspect = max(rw, rh) / (min(rw, rh) + 1e-6)
    if aspect >= 1.25 and angle_rect is not None:
        chosen_angle = angle_rect
        chosen_method = "rect"
    elif angle_pca is not None:
        chosen_angle = angle_pca
        chosen_method = "pca"
    elif angle_ellipse is not None:
        chosen_angle = angle_ellipse
        chosen_method = "ellipse"
    else:
        chosen_angle = angle_rect if angle_rect is not None else (angle_pca or 0.0)
        chosen_method = "fallback"

    chosen_angle = _normalize_angle_deg(chosen_angle)

    # 10) zamień na kąt w układzie maszyny (odjąć rotację kamery z config)
    rot_cfg = 0
    try:
        cfg = configparser.ConfigParser(); cfg.read(CONFIG_FILE)
        if "Connection" in cfg:
            rot_cfg = int(float(cfg["Connection"].get("camera_up_rotation_deg", "0")))
    except:
        rot_cfg = 0
    angle_machine = _normalize_angle_deg(chosen_angle - rot_cfg)

    # 11) wizualizacja
    vis = img_bgr.copy()
    cv2.drawContours(vis, [c], -1, (0,255,0), 1)           # kontur (zielony)
    hull = cv2.convexHull(c)
    cv2.drawContours(vis, [hull], -1, (255,0,0), 1)        # hull (niebieski)
    cv2.drawContours(vis, [box], -1, (0,0,255), 2)         # rect box (czerwony)
    cv2.circle(vis, (int(cx), int(cy)), 4, (0,0,255), -1)  # centroid
    # PCA axis line
    if angle_pca is not None:
        L = max(w,h)//2
        vx = math.cos(math.radians(angle_pca))
        vy = math.sin(math.radians(angle_pca))
        x1 = int(cx - vx * L); y1 = int(cy - vy * L)
        x2 = int(cx + vx * L); y2 = int(cy + vy * L)
        cv2.line(vis, (x1,y1),(x2,y2),(255,255,0),2)

    # put text
    cv2.putText(vis, f"img:{chosen_angle:.1f} deg ({chosen_method})", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)
    cv2.putText(vis, f"mach:{angle_machine:.1f} deg rotcfg={rot_cfg}", (10,45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)
    cv2.putText(vis, f"area:{area:.1f}", (10,70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)

    # zapisz maskę i wiz
    cv2.imwrite("test_orient_mask.jpg", mask_clean)
    cv2.imwrite("test_orient_result.jpg", vis)

    # dodatkowo zapisz "obracany próbny" obraz do wizualnej weryfikacji
    rotated = rotate_image(img_bgr, +chosen_angle)
    cv2.imwrite("test_orient_rotated_by_angle.jpg", rotated)

    if debug_show:
        cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
        cv2.imshow("mask", mask_clean)
        cv2.namedWindow("vis", cv2.WINDOW_NORMAL)
        cv2.imshow("vis", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    out = {
        'angle_image': float(chosen_angle),
        'angle_machine': float(angle_machine),
        'centroid': (float(cx), float(cy)),
        'area_px': float(area),
        'bbox': (int(x), int(y), int(bw), int(bh)),
        'mask': mask_clean,
        'vis': vis
    }
    return out

def read_file_text_with_encoding(path, sample_bytes=4096):
    """Odczytaj plik binarnie, wykryj encoding (BOM/utf-16 heurystyka) i zwróć zdekodowany tekst."""
    with open(path, 'rb') as f:
        raw = f.read()
    # BOM checks
    if raw.startswith(b'\xff\xfe') or raw.startswith(b'\xfe\xff'):
        try:
            return raw.decode('utf-16')
        except:
            pass
    # jeśli w pierwszych kilkuset bajtach jest dużo null bytes -> prawdopodobnie utf-16
    head = raw[:sample_bytes]
    nulls = head.count(b'\x00')
    if nulls * 2 > len(head):  # dużo nulls → utf-16
        try:
            return raw.decode('utf-16')
        except:
            try:
                return raw.decode('utf-16-le')
            except:
                pass
    # fallback: spróbuj utf-8, latin1
    try:
        return raw.decode('utf-8')
    except:
        try:
            return raw.decode('latin1')
        except:
            return raw.decode('utf-8', errors='ignore')

def detect_delimiter_from_text(text, max_lines=10):
    """Heurystyka: policz wystąpienia ',' ';' '\t' w pierwszych kilku liniach i wybierz najczęstszy."""
    lines = text.splitlines()[:max_lines]
    counts = {',':0, ';':0, '\t':0}
    for ln in lines:
        counts[','] += ln.count(',')
        counts[';'] += ln.count(';')
        counts['\t'] += ln.count('\t')
    best = max(counts, key=lambda k: counts[k])
    if counts[best] == 0:
        return ','
    return best

def csv_rows_from_text(text, delimiter=None):
    """Zwraca iterator list (rows) z wyczyszczonymi wartościami (usuwamy null bytes i otaczające cudzysłowy)."""
    if delimiter is None:
        delimiter = detect_delimiter_from_text(text)
    s = io.StringIO(text)
    reader = csv.reader(s, delimiter=delimiter)
    for row in reader:
        # clean each cell: usuń null bytes i otaczające cudzysłowy oraz trim
        cleaned = [ (cell.replace('\x00','').strip().strip('"').strip()) for cell in row ]
        yield cleaned

def robust_dict_rows(path):
    """
    Czyta plik i zwraca (header_list, generator_of_rowdicts).
    Nagłówki i wartości są oczyszczone z null chars i zewnętrznych cudzysłowów.
    """
    text = read_file_text_with_encoding(path)
    # usuń ewentualne BOM z początku
    if text.startswith('\ufeff'):
        text = text.lstrip('\ufeff')

    delim = detect_delimiter_from_text(text)
    rows_gen = csv_rows_from_text(text, delimiter=delim)

    try:
        header = next(rows_gen)
    except StopIteration:
        return [], iter([])

    # jeśli header jest pojedynczą kolumną zawierającą przecinki (np. "Rząd 1 Ref,Val,Package,..."),
    # rozbij ją i ustaw proper header + adjust rows accordingly
    if len(header) == 1 and (',' in header[0] or ';' in header[0]):
        sep_in_header = ',' if ',' in header[0] else ';'
        header = [h.strip().strip('"') for h in header[0].split(sep_in_header)]
        # teraz zbuduj nowy generator, który dla każdej dalszej lini zrobi split po sep_in_header
        def gen():
            for ln in text.splitlines()[text.splitlines().index(next(iter(text.splitlines())))+1:]:
                # (nie eleganckie, ale nasz csv_rows_from_text już działa poprawnie)
                pass
        # easier approach: create csv reader from substring starting at header line
        # build csv part starting from header occurrence:
        all_lines = text.splitlines()
        # find header line index by matching a line that contains all header tokens
        idx = None
        for i, ln in enumerate(all_lines):
            if all(token.lower() in ln.lower() for token in header):
                idx = i
                break
        if idx is None:
            idx = 0
        csv_part = "\n".join(all_lines[idx:])
        # now create standard reader
        f2 = io.StringIO(csv_part)
        reader2 = csv.reader(f2, delimiter=',' if ',' in csv_part.splitlines()[0] else detect_delimiter_from_text(csv_part))
        # first line is header; skip it
        _ = next(reader2, None)
        def rowdict_iter():
            for row in reader2:
                cleaned = [ (cell.replace('\x00','').strip().strip('"').strip()) for cell in row ]
                # map header -> cell (shorter rows padded with '')
                d = { header[i]: (cleaned[i] if i < len(cleaned) else '') for i in range(len(header)) }
                yield d
        return header, rowdict_iter()

    # otherwise header is list of column names
    header_clean = [h.replace('\x00','').strip().strip('"') for h in header]

    # create generator of dicts
    def gen_dicts():
        for row in rows_gen:
            # if row shorter than header, pad ''
            row = row + [''] * max(0, len(header_clean) - len(row))
            d = { header_clean[i]: row[i] for i in range(len(header_clean)) }
            yield d

    return header_clean, gen_dicts()

# Example small helper to debug first rows (użyj przed parsowaniem)
def debug_show_first_rows(path, n=5):
    text = read_file_text_with_encoding(path)
    delim = detect_delimiter_from_text(text)
    print("DEBUG: chosen delimiter:", repr(delim))
    rows = csv_rows_from_text(text, delimiter=delim)
    for i, r in enumerate(rows):
        print("ROW", i, r)
        if i+1 >= n:
            break

# ---- ZAMIENIĆ/POPRAWIĆ parse_easyeda_csv by użyć robust_dict_rows ----
def parse_easyeda_csv(path):
    rows = []
    header, rowdicts = robust_dict_rows(path)
    # rowdicts yields dicts keyed by cleaned header names
    for r in rowdicts:
        # map possible header names to canonical fields
        ref = (r.get('Designator') or r.get('designator') or r.get('Ref') or r.get('ref') or '').strip()
        fp = r.get('Footprint') or r.get('footprint') or ''
        # try several possible column keys
        x_raw = r.get('Mid X') or r.get('MidX') or r.get('Mid X(mm)') or r.get('Ref X') or r.get('Mid_X') or r.get('mid x') or r.get('Pad X')
        y_raw = r.get('Mid Y') or r.get('MidY') or r.get('Ref Y') or r.get('Mid_Y') or r.get('mid y') or r.get('Pad Y')

        x_mm = parse_coord_to_mm(x_raw, file_unit_hint=None)
        y_mm = parse_coord_to_mm(y_raw, file_unit_hint=None)
        if x_mm is None or y_mm is None:
            continue

        side = r.get('Layer') or r.get('layer') or r.get('Side') or ''
        rot_raw = r.get('Rotation') or r.get('rotation') or '0'
        comment = r.get('Comment') or r.get('comment') or ''

        try:
            rotf = float(str(rot_raw).replace(',','.'))
        except:
            rotf = 0.0

        rows.append({
            'ref': ref,
            'x': float(x_mm),
            'y': float(y_mm),
            'rotation': float(rotf),
            'side': ('bottom' if str(side).strip().lower().startswith('b') else 'top'),
            'footprint': fp,
            'value': comment
        })
    return rows

def parse_coord_to_mm(value_str, file_unit_hint=None):
    if value_str is None:
        return None
    s = str(value_str).strip()
    if s == '':
        return None
    s0 = s.lower()
    # explicit units
    if 'mil' in s0:
        m = re.search(r'(-?[\d\.]+)', s0)
        if not m:
            return None
        val = float(m.group(1))
        return val * MIL_TO_MM
    if 'in' in s0 or '"' in s0 or 'inch' in s0:
        m = re.search(r'(-?[\d\.]+)', s0)
        if not m:
            return None
        val = float(m.group(1))
        return val * MM_PER_INCH
    # clean
    s_clean = re.sub(r'[^\d\.\-]', '', s)
    if s_clean == '':
        return None
    # hint
    try:
        if file_unit_hint is not None:
            hint = str(file_unit_hint).lower()
            val = float(s_clean)
            if hint in ('mm', 'millimeter', 'millimetre'):
                return val
            if hint in ('in', 'inch', 'inches'):
                return val * MM_PER_INCH
            if hint in ('mil', 'mils'):
                return val * MIL_TO_MM
    except:
        pass
    # decimals present -> treat as float mm
    if '.' in s_clean:
        return float(s_clean)
    # integer-like heuristic: if big -> treat as mils
    try:
        val_int = int(s_clean)
    except:
        try:
            return float(s_clean)
        except:
            return None
    if abs(val_int) >= 1000 and abs(val_int) <= 100000000:
        return val_int * MIL_TO_MM
    return float(val_int)


# ---------- helper: konwersja cal->mm ----------
def inches_to_mm(x):
    return float(x) * 25.4


# ---- poprawiona parse_kicad_csv ----
def parse_kicad_csv(path):
    with open(path, encoding='utf-8', errors='ignore') as f:
        txt = f.read()
    lines = txt.splitlines()
    header_line = None
    for ln in lines:
        if ('Ref' in ln or 'Ref,' in ln) and ('PosX' in ln or 'PosX' in ln or 'PosY' in ln):
            header_line = ln
            break
    if header_line is None:
        # try normal csv
        try:
            f2 = io.StringIO(txt)
            reader = csv.DictReader(f2)
            out = []
            for r in reader:
                x = parse_coord_to_mm(r.get('PosX') or r.get('PosX(mm)') or r.get('X') or r.get('x'))
                y = parse_coord_to_mm(r.get('PosY') or r.get('PosY(mm)') or r.get('Y') or r.get('y'))
                if x is None or y is None:
                    continue
                out.append({'ref': r.get('Ref') or r.get('ref',''), 'x':x, 'y':y, 'rotation': float(r.get('Rot') or 0.0), 'side': (r.get('Side') or 'top').lower(), 'footprint': r.get('Package','')})
            return out
        except Exception:
            raise RuntimeError("Nie rozpoznano formatu KiCad CSV")
    start = lines.index(header_line)
    csv_part = "\n".join(lines[start:])
    f2 = io.StringIO(csv_part)
    reader = csv.DictReader(f2, delimiter=',' if ',' in header_line else detect_delimiter(path))
    rows = []
    for r in reader:
        x = parse_coord_to_mm(r.get('PosX') or r.get('PosX(mm)') or r.get('Posx') or r.get('X'))
        y = parse_coord_to_mm(r.get('PosY') or r.get('PosY(mm)') or r.get('Posy') or r.get('Y'))
        if x is None or y is None:
            continue
        ref = r.get('Ref') or r.get('ref') or ''
        try:
            rot = float(r.get('Rot') or 0.0)
        except:
            rot = 0.0
        side = (r.get('Side') or 'top').lower()
        rows.append({'ref':ref, 'x':x, 'y':y, 'rotation':rot, 'side': 'bottom' if side.startswith('b') else 'top', 'footprint': r.get('Package','')})
    return rows


# ---------- KiCad ASCII .pos (format przestrzenny, jak w przykładzie) ----------
def parse_kicad_ascii_pos(path):
    rows = []
    unit = 'mm'
    with open(path, encoding='utf-8', errors='ignore') as f:
        for line in f:
            if line.startswith('## Unit'):
                if 'inch' in line.lower():
                    unit = 'inch'
            if line.strip().startswith('#') or line.strip()=='':
                continue
            # data lines look like: Ref Val Package PosX PosY Rot Side
            # split by whitespace but Val/Package may contain hyphens etc; assume columns by position
            parts = re.split(r'\s+', line.strip())
            if len(parts) >= 6:
                ref = parts[0]
                val = parts[1]
                package = parts[2]
                try:
                    px = float(parts[3])
                    py = float(parts[4])
                    rot = float(parts[5])
                except:
                    continue
                if unit == 'inch':
                    px = inches_to_mm(px); py = inches_to_mm(py)
                side = 'top'
                if len(parts) >= 7:
                    s = parts[6].lower()
                    side = 'bottom' if s.startswith('b') else 'top'
                rows.append({'ref':ref, 'x':px, 'y':py, 'rotation':rot, 'side':side, 'footprint':package, 'value':val})
    return rows

def parse_gerber_x3_positions(path):
    """
    Parsuje Gerber X3 i grupuje D03 flashe per %TO.C / %TO.P bloki.
    Zwraca listę komponentów: {'ref':..., 'flashes':[{'x':..., 'y':...}, ...], 'rotation':..., 'footprint':...}
    Pozycje w mm.
    """
    txt = open(path, encoding='utf-8', errors='ignore').read()
    unit_mm = '%MOMM*%' in txt.upper() or ('%MOIN*%' not in txt.upper())
    # detect decimals from FSLAX.. token (fallback decimals=4)
    m = re.search(r'FSLAX(\d)(\d)Y(\d)(\d)\*', txt)
    decimals = int(m.group(2)) if m else 4
    lines = txt.splitlines()

    components = []
    current_comp = None

    def finish_current():
        nonlocal current_comp
        if current_comp:
            # collapse flashes to centroid (optional) or keep list
            components.append(current_comp)
            current_comp = None

    for ln in lines:
        ln = ln.strip()
        if not ln:
            continue
        # start TO block: %TO.C,<ref>*% or %TO.C,<ref>,...*%
        if ln.startswith('%TO.C,') or ln.startswith('%TO.C'):
            finish_current()
            # extract ref between commas until *
            m = re.match(r'%TO\.C,([^,*%]+)', ln)
            ref = m.group(1) if m else 'C?'
            current_comp = {'ref': ref, 'flashes': [], 'rotation': 0.0, 'footprint': ''}
            continue
        # some TO metadata lines e.g. %TO.CVal,R*% or %TO.CFtp,footprint*%
        if ln.startswith('%TO.'):
            # try to capture rotation/footprint/value for current component
            m_val = re.match(r'%TO\.CVal,([^*%]+)\*%', ln)
            m_ftp = re.match(r'%TO\.CFtp,([^*%]+)\*%', ln)
            m_rot = re.match(r'%TO\.CRot,(-?\d+)', ln)
            if current_comp:
                if m_ftp:
                    current_comp['footprint'] = m_ftp.group(1)
                if m_rot:
                    try:
                        current_comp['rotation'] = float(m_rot.group(1))
                    except:
                        pass
            continue
        # D03 flash lines: X<digits>Y<digits>D03*
        m = re.search(r'[Xx](-?\d+)[Yy](-?\d+)D03\*', ln)
        if m:
            xs, ys = m.group(1), m.group(2)
            try:
                xv = int(xs) * (10 ** (-decimals))
                yv = int(ys) * (10 ** (-decimals))
                if not unit_mm:
                    xv = inches_to_mm(xv)
                    yv = inches_to_mm(yv)
            except:
                continue
            if current_comp is None:
                # no surrounding %TO - create anonymous component container
                current_comp = {'ref': '', 'flashes': [], 'rotation': 0.0, 'footprint': ''}
            current_comp['flashes'].append({'x': xv, 'y': yv})
            continue
        # end token of TO blocks sometimes: %TD*% -> skip or finish
        if ln.startswith('%TD') or ln.startswith('%TD*') or ln.startswith('%TD*%'):
            finish_current()
            continue

    # finalize
    finish_current()
    # optionally compute centroid per component for convenience
    out = []
    for c in components:
        if c['flashes']:
            cx = sum(p['x'] for p in c['flashes']) / len(c['flashes'])
            cy = sum(p['y'] for p in c['flashes']) / len(c['flashes'])
        else:
            cx = cy = 0.0
        out.append({'ref': c.get('ref',''), 'centroid_x': cx, 'centroid_y': cy, 'rotation': c.get('rotation',0.0), 'footprint': c.get('footprint',''), 'raw_flashes': c['flashes']})
    return out

def parse_pick_and_place(path):
    mapping_keys = {
        'ref': ['designator', 'ref', 'reference', 'reference designator', 'designator'],
        'x': ['mid x','x','x(mm)','pos_x','xpos','x_mm','x [mm]','x(mm)','posx','posx(mm)','ref x','pad x'],
        'y': ['mid y','y','y(mm)','pos_y','ypos','y_mm','y [mm]','y(mm)','posy','posy(mm)','ref y','pad y'],
        'rotation': ['rotation','rot','angle','orient','rotation(deg)','rotation(degrees)'],
        'side': ['side','layer','top/bottom','side?'],
        'value': ['value','dvalue','package','comment','val'],
        'footprint': ['footprint','package']
    }

    # pomocnik: bezpieczne czyszczenie klucza nagłówka
    def clean_key(k):
        return k.strip().lower()

    # funkcja wyszukująca wartość w dict-lub-row z priorytetami:
    def find_field_from_dict(d, candidates):
        # 1) exact match (after cleaning)
        key_map = {clean_key(k): k for k in d.keys()}
        for cand in candidates:
            ck = cand.lower()
            if ck in key_map:
                return d[key_map[ck]]
        # 2) key starts/ends or equals word-wise
        for cand in candidates:
            ck = cand.lower()
            for k in d.keys():
                lk = clean_key(k)
                # whole-word like "designator" or "ref" (split punctuation/spaces)
                tokens = re.split(r'[\s\-_:]+', lk)
                if ck in tokens:
                    return d[k]
                # startswith or endswith (e.g. "ref x" vs "ref")
                if lk.startswith(ck + ' ') or lk.endswith(' ' + ck) or lk == ck:
                    return d[k]
        # 3) fallback: substring match but avoid returning Ref X/Y when asking for 'ref' (we'll exclude keys containing ' x' or ' y' or 'pad')
        for cand in candidates:
            ck = cand.lower()
            for k in d.keys():
                lk = clean_key(k)
                if ck in lk:
                    # if requesting 'ref' ensure we don't return coordinates accidentally
                    if cand == 'designator' or cand == 'ref':
                        if re.search(r'\b(x|y|pos|pad|mid)\b', lk):
                            continue
                    return d[k]
        return None

    # robust reader first
    try:
        header, rowdicts = robust_dict_rows(path)
    except Exception:
        header, rowdicts = [], iter(())

    rows = []
    if header and rowdicts:
        for r in rowdicts:
            x_raw = find_field_from_dict(r, mapping_keys['x'])
            y_raw = find_field_from_dict(r, mapping_keys['y'])
            # dodatkowe aliasy
            x_raw = x_raw or r.get('Mid X') or r.get('Ref X') or r.get('Pad X')
            y_raw = y_raw or r.get('Mid Y') or r.get('Ref Y') or r.get('Pad Y')
            x = parse_coord_to_mm(x_raw)
            y = parse_coord_to_mm(y_raw)
            if x is None or y is None:
                continue

            ref_raw = find_field_from_dict(r, mapping_keys['ref'])
            # zabezpieczenie gdyby ref_raw zawierało wartość numeryczną z kolumny Ref X
            if ref_raw is None:
                ref_raw = r.get('Designator') or r.get('Ref') or r.get('designator') or ''
            try:
                rot_raw = find_field_from_dict(r, mapping_keys['rotation']) or r.get('Rotation') or '0'
                rot = float(str(rot_raw).replace(',','.'))
            except:
                rot = 0.0

            side_raw = find_field_from_dict(r, mapping_keys['side']) or r.get('Layer') or ''
            side = 'bottom' if str(side_raw).strip().lower().startswith('b') else 'top'

            value = find_field_from_dict(r, mapping_keys['value']) or ''
            footprint = find_field_from_dict(r, mapping_keys['footprint']) or ''

            rows.append({
                'ref': str(ref_raw).strip().strip('"'),
                'x': float(x),
                'y': float(y),
                'rotation': float(rot),
                'side': side,
                'value': str(value).strip().strip('"'),
                'footprint': str(footprint).strip().strip('"')
            })

        if rows:
            return rows

    # fallback CSV DictReader (heurystyczny delimiter)
    try:
        delim = detect_delimiter_from_text(read_file_text_with_encoding(path))
        with open(path, newline='', encoding='utf-8', errors='ignore') as f:
            rdr = csv.DictReader(f, delimiter=delim)
            for r in rdr:
                x_raw = find_field_from_dict(r, mapping_keys['x'])
                y_raw = find_field_from_dict(r, mapping_keys['y'])
                x_raw = x_raw or r.get('Mid X') or r.get('Ref X') or r.get('Pad X')
                y_raw = y_raw or r.get('Mid Y') or r.get('Ref Y') or r.get('Pad Y')
                x = parse_coord_to_mm(x_raw)
                y = parse_coord_to_mm(y_raw)
                if x is None or y is None:
                    continue
                ref_raw = find_field_from_dict(r, mapping_keys['ref']) or r.get('Designator') or ''
                try:
                    rot = float((find_field_from_dict(r, mapping_keys['rotation']) or 0) or 0)
                except:
                    rot = 0.0
                side_raw = find_field_from_dict(r, mapping_keys['side']) or r.get('Layer') or ''
                side = 'bottom' if str(side_raw).strip().lower().startswith('b') else 'top'
                rows.append({'ref':str(ref_raw).strip('"'), 'x':x, 'y':y, 'rotation':rot, 'side':side,
                             'value': find_field_from_dict(r, mapping_keys['value']) or '',
                             'footprint': find_field_from_dict(r, mapping_keys['footprint']) or ''})
        if rows:
            return rows
    except Exception:
        pass

    raise RuntimeError("Nie wykryto współrzędnych w pliku PnP.")

# ---- poprawiona parse_pick_and_place_auto (wybór parsera) ----
def parse_pick_and_place_auto(path):
    with open(path, encoding='utf-8', errors='ignore') as f:
        txt = f.read(8192)
    lower = txt.lower()
    if 'designator' in lower and ('mid x' in lower or 'midx' in lower):
        return parse_easyeda_csv(path)
    if 'footprint positions' in lower and 'printed by kicad' in lower:
        return parse_kicad_ascii_pos(path)
    if ('posx' in lower and 'posy' in lower and 'rot' in lower) or (',posx' in lower and ',posy' in lower):
        return parse_kicad_csv(path)
    if 'fslax' in lower or 'g04 gerber fmt' in lower or 'd03*' in lower:
        return parse_gerber_x3_positions(path)
    # fallback genericparse_pick_and_place
    return parse_pick_and_place(path)


# =====================================================================================
# ==================  GENERATOR PODSTAWKI STL POD ELEMENTY Z BOM  ===================
# =====================================================================================
#
# Funkcja generate_component_holder_stl() tworzy plik STL prostopadłościennej podstawki
# z wyfrezowanymi (wyciętymi) kieszeniami na komponenty wymienione w pliku BOM/pick&place
# (wykorzystuje istniejące parsery, np. parse_pick_and_place_auto()) albo w ręcznie podanej
# liście komponentów. Rozmiar fizyczny obudowy (footprintu) jest brany z bazy w pliku
# footprint_sizes.json (tworzonej automatycznie z sensownymi wartościami domyślnymi przy
# pierwszym uruchomieniu — podobnie jak config.ini). Nierozpoznane obudowy są szacowane
# na podstawie nazwy (np. TQFP44, SOIC8, DIP16...) albo dostają rozmiar domyślny z ostrzeżeniem.
#
# Wymaga bibliotek: trimesh + manifold3d (operacje boolowskie CSG).
#   pip install trimesh manifold3d
# =====================================================================================

MM_PER_INCH_HOLDER = 25.4  # lokalna stała (nienazywana MM_PER_INCH, żeby nie kolidować z resztą kodu)

def _default_footprint_db() -> dict:
    """
    Domyślna baza wymiarów obudów: {"NAZWA": {"w":szerokość_mm, "h":wysokość_mm, "depth":głębokość_kieszeni_mm, "pins":liczba_nóżek}}.
    Klucze są dopasowywane po znormalizowaniu (wielkie litery, bez separatorów) jako:
    1) dopasowanie dokładne, 2) najdłuższy pasujący podciąg (np. "SOT23-3"/"SOT-23-5" trafią w "SOT23").
    "w" to zawsze dłuższy/naturalny wymiar obudowy (oś X), "h" krótszy (oś Y) — dzięki temu
    orientacja wszystkich kieszeni w podstawce jest spójna (patrz allow_rotation=False).
    Wymiary są przybliżone (typowe wartości katalogowe) — w razie potrzeby edytuj plik
    footprint_sizes.json, który zostanie utworzony z tą zawartością przy pierwszym uruchomieniu.
    """
    return {
        # --- elementy bierne SMD (kod calowy), zawsze 2 nóżki ---
        "0201": {"w": 0.6, "h": 0.3, "depth": 0.3, "pins": 2},
        "0402": {"w": 1.0, "h": 0.5, "depth": 0.5, "pins": 2},
        "0603": {"w": 1.6, "h": 0.8, "depth": 0.8, "pins": 2},
        "0805": {"w": 2.0, "h": 1.25, "depth": 1.0, "pins": 2},
        "1206": {"w": 3.2, "h": 1.6, "depth": 1.1, "pins": 2},
        "1210": {"w": 3.2, "h": 2.5, "depth": 1.3, "pins": 2},
        "1812": {"w": 4.5, "h": 3.2, "depth": 1.5, "pins": 2},
        "2010": {"w": 5.0, "h": 2.5, "depth": 1.3, "pins": 2},
        "2220": {"w": 5.7, "h": 5.0, "depth": 1.6, "pins": 2},
        "2512": {"w": 6.4, "h": 3.2, "depth": 1.6, "pins": 2},
        # --- diody / tranzystory ---
        "SOT23": {"w": 3.0, "h": 1.75, "depth": 1.3, "pins": 3},
        "SOT233": {"w": 3.0, "h": 1.5, "depth": 1.3, "pins": 3},
        "SOT235": {"w": 3.0, "h": 1.75, "depth": 1.3, "pins": 5},
        "SOT236": {"w": 3.0, "h": 1.75, "depth": 1.3, "pins": 6},
        "SOT238": {"w": 3.0, "h": 1.75, "depth": 1.3, "pins": 8},
        "SOT223": {"w": 6.7, "h": 3.7, "depth": 1.8, "pins": 4},
        "SOT2233": {"w": 6.7, "h": 3.7, "depth": 1.8, "pins": 3},
        "SOT2234": {"w": 6.7, "h": 3.7, "depth": 1.8, "pins": 4},
        "SOT89": {"w": 4.5, "h": 4.0, "depth": 1.6, "pins": 3},
        "SOT323": {"w": 2.1, "h": 1.35, "depth": 1.0, "pins": 3},
        "SOD123": {"w": 2.7, "h": 1.7, "depth": 1.1, "pins": 2},
        "SOD323": {"w": 1.8, "h": 1.35, "depth": 1.05, "pins": 2},
        "SOD523": {"w": 1.25, "h": 0.85, "depth": 0.65, "pins": 2},
        "SMA": {"w": 4.3, "h": 2.7, "depth": 1.7, "pins": 2},
        "DO214AC": {"w": 4.3, "h": 2.7, "depth": 1.7, "pins": 2},
        "SMB": {"w": 5.4, "h": 3.5, "depth": 2.1, "pins": 2},
        "DO214AA": {"w": 5.4, "h": 3.5, "depth": 2.1, "pins": 2},
        "SMC": {"w": 7.9, "h": 6.5, "depth": 2.3, "pins": 2},
        "DO214AB": {"w": 7.9, "h": 6.5, "depth": 2.3, "pins": 2},
        "TO92": {"w": 5.2, "h": 4.5, "depth": 5.0, "pins": 3},
        "TO220": {"w": 10.5, "h": 4.5, "depth": 15.0, "pins": 3},
        "TO220V": {"w": 10.5, "h": 15.0, "depth": 4.5, "pins": 3},
        "TO252": {"w": 6.6, "h": 6.1, "depth": 2.3, "pins": 3},
        "TO263": {"w": 10.2, "h": 9.2, "depth": 2.3, "pins": 3},
        # --- rezonatory / cewki ---
        "HC49": {"w": 11.3, "h": 4.7, "depth": 13.0, "pins": 2},
        "3225": {"w": 3.2, "h": 2.5, "depth": 1.0, "pins": 2},
        "2016": {"w": 2.0, "h": 1.6, "depth": 0.7, "pins": 2},
        "5032": {"w": 5.0, "h": 3.2, "depth": 1.3, "pins": 2},
        "7050": {"w": 7.0, "h": 5.0, "depth": 1.5, "pins": 2},
        # --- ogólne domyślne ---
        "__DEFAULT__": {"w": 5.0, "h": 5.0, "depth": 3.0, "pins": 2},
    }


def load_footprint_db(path: str = FOOTPRINT_DB_FILE) -> dict:
    """
    Wczytuje bazę wymiarów obudów z pliku JSON. Jeśli plik nie istnieje — tworzy go
    z wartościami domyślnymi (analogicznie do obsługi config.ini w tym programie).
    Klucze w pliku są już znormalizowane (wielkie litery, bez spacji/myślników/podkreśleń).
    """
    if not os.path.exists(path):
        db_default = _default_footprint_db()
        with open(path, "w", encoding="utf-8") as f:
            json.dump(db_default, f, indent=2, ensure_ascii=False, sort_keys=True)
        print(f"Nie znaleziono pliku {path} — utworzono nowy z wymiarami domyślnymi obudów. "
              f"Możesz go edytować, aby dodać/poprawić rozmiary konkretnych footprintów.")
        return db_default
    try:
        with open(path, "r", encoding="utf-8") as f:
            db = json.load(f)
        if "__DEFAULT__" not in db:
            db["__DEFAULT__"] = _default_footprint_db()["__DEFAULT__"]
        # wsteczna kompatybilność: jeśli ktoś ma stary plik bez "pins", dopisz domyślnie 2
        for k, v in db.items():
            if "pins" not in v:
                v["pins"] = 2
        return db
    except Exception as e:
        print(f"Nie udało się wczytać {path} ({e}) — używam wbudowanych wartości domyślnych.")
        return _default_footprint_db()


def _norm_fp(name: str) -> str:
    """Normalizacja nazwy footprintu/pakietu do porównań: same wielkie litery i cyfry.
    Dzięki temu np. 'SOT-23-3', 'SOT23_3', 'sot 23-3' dają ten sam klucz i trafiają
    w tę samą (lub najdłuższą pasującą częściowo) pozycję w bazie."""
    return re.sub(r'[^A-Z0-9]', '', str(name or '').upper())


# rodziny obudów wieloodprowadzeniowych parsowane parametrycznie z nazwy, gdy nie ma ich w bazie
# pitch [mm], czy obudowa jest "kwadratowa 4-stronna" (QFP/QFN) czy "2-stronna" (SOIC/SSOP/TSSOP/DIP)
_PARAM_FAMILIES = {
    "TQFP": {"pitch": 0.5, "sides": 4, "lead_margin": 2.0, "depth": 1.4},
    "LQFP": {"pitch": 0.5, "sides": 4, "lead_margin": 2.0, "depth": 1.4},
    "QFP":  {"pitch": 0.8, "sides": 4, "lead_margin": 2.2, "depth": 1.6},
    "QFN":  {"pitch": 0.5, "sides": 4, "lead_margin": 0.4, "depth": 0.9},
    "DFN":  {"pitch": 0.5, "sides": 4, "lead_margin": 0.4, "depth": 0.8},
    "SOIC": {"pitch": 1.27, "sides": 2, "lead_margin": 2.0, "depth": 1.75},
    "SSOP": {"pitch": 0.65, "sides": 2, "lead_margin": 1.5, "depth": 1.75},
    "TSSOP": {"pitch": 0.65, "sides": 2, "lead_margin": 1.5, "depth": 1.1},
    "DIP":  {"pitch": 2.54, "sides": 2, "lead_margin": 3.0, "depth": 5.0},
    "SDIP": {"pitch": 2.54, "sides": 2, "lead_margin": 3.0, "depth": 5.0},
}
_PARAM_FAMILY_RE = re.compile(r'(TQFP|LQFP|QFP|QFN|DFN|SOIC|SSOP|TSSOP|SDIP|DIP)0*(\d+)')
_RADIAL_CAP_RE = re.compile(r'(?:^|[^0-9])D(\d+(?:\.\d+)?)')  # np. "CP_Radial_D5.0mm_P2.00mm" -> 5.0


def _estimate_parametric(norm_name: str):
    """Szacuje wymiary i liczbę nóżek dla obudów QFP/QFN/SOIC/TSSOP/SSOP/DIP na podstawie
    liczby nóżek zawartej w nazwie (np. 'TQFP44', 'SOIC8', 'DIP16')."""
    m = _PARAM_FAMILY_RE.search(norm_name)
    if not m:
        return None
    family, pins_s = m.group(1), m.group(2)
    pins = int(pins_s)
    if pins <= 0:
        return None
    fam = _PARAM_FAMILIES[family]
    pitch, lead_margin, depth = fam["pitch"], fam["lead_margin"], fam["depth"]

    if fam["sides"] == 4:
        leads_per_side = max(1, pins / 4)
        size = leads_per_side * pitch + 2 * lead_margin
        size = max(3.0, round(size * 2) / 2)  # zaokrąglenie do 0.5 mm
        return {"w": size, "h": size, "depth": depth, "pins": pins}
    else:
        leads_per_side = max(1, pins / 2)
        length = (leads_per_side - 1) * pitch + 2 * lead_margin
        length = max(3.0, round(length * 2) / 2)
        if family in ("SOIC",):
            width = 7.5 if pins > 16 else 3.9
        elif family in ("SSOP", "TSSOP"):
            width = 6.1 if pins > 20 else 4.4
        else:  # DIP / SDIP
            width = 15.24 if pins >= 24 else 7.62
        return {"w": length, "h": width, "depth": depth, "pins": pins}


def estimate_footprint_dimensions(footprint: str, value: str, db: dict):
    """
    Zwraca (w_mm, h_mm, depth_mm, pins, dopasowano:bool, opis_dopasowania:str) dla danego footprintu.
    Kolejność prób: 1) dokładne dopasowanie w bazie, 2) najdłuższy pasujący podciąg w bazie
    (dzięki temu warianty typu 'SOT23-3', 'SOT-23-5' itp. trafiają w bazową obudowę 'SOT23'
    nawet jeśli nie ma dla nich osobnego wpisu), 3) parametryczne oszacowanie
    (QFP/QFN/SOIC/TSSOP/SSOP/DIP wg liczby nóżek w nazwie), 4) kondensator radialny
    (po średnicy w nazwie footprintu), 5) wartość domyślna.
    """
    norm = _norm_fp(footprint) or _norm_fp(value)

    if norm in db:
        d = db[norm]
        return d["w"], d["h"], d["depth"], d.get("pins", 2), True, f"dokładne dopasowanie '{norm}'"

    candidates = [k for k in db.keys() if k != "__DEFAULT__" and k in norm]
    if candidates:
        best = max(candidates, key=len)
        d = db[best]
        return d["w"], d["h"], d["depth"], d.get("pins", 2), True, f"dopasowanie częściowe do '{best}'"

    param = _estimate_parametric(norm)
    if param:
        return param["w"], param["h"], param["depth"], param["pins"], True, "oszacowanie parametryczne (liczba nóżek)"

    m = _RADIAL_CAP_RE.search(norm)
    if m:
        dia = float(m.group(1))
        return dia, dia, max(dia * 1.1, dia), 2, True, f"oszacowanie kondensatora radialnego (Ø{dia}mm)"

    d = db.get("__DEFAULT__", {"w": 5.0, "h": 5.0, "depth": 3.0, "pins": 2})
    return d["w"], d["h"], d["depth"], d.get("pins", 2), False, "nierozpoznany footprint — użyto wartości domyślnej"


def _pack_shelves(items: list, max_width: float, spacing: float = 2.0, allow_rotation: bool = False):
    """
    Prosty, ale skuteczny algorytm pakowania 2D typu "shelf" (pakowanie w wiersze/półki).
    items: lista dict z kluczami co najmniej 'w','h' (mm) — pozostałe pola są zachowywane.
    Domyślnie allow_rotation=False, żeby wszystkie kieszenie zachowały tę samą, naturalną
    orientację z bazy footprintów (patrz _default_footprint_db) — bez tego pakowanie
    potrafiło obracać pojedyncze elementy o 90°, żeby "upchnąć" je ciaśniej, co dawało
    niespójną orientację takich samych elementów w podstawce.
    Zwraca (placements, plate_w, plate_h):
      placements: lista (item, x, y, w_użyte, h_użyte) — x,y to lewy-dolny róg kieszeni w mm.
      plate_w/plate_h: wymiary całej płyty potrzebne, żeby wszystko się zmieściło (z marginesami).
    """
    shelves = []  # {'y','height','x_cursor'}
    placements = []
    y_cursor = spacing

    for it in items:
        w0, h0 = float(it['w']), float(it['h'])
        options = [(w0, h0)]
        if allow_rotation and abs(w0 - h0) > 1e-6:
            options.append((h0, w0))

        best = None  # (marnowana_wysokosc, shelf, w, h)
        for shelf in shelves:
            avail_w = max_width - shelf['x_cursor']
            for (w, h) in options:
                if (w + spacing) <= avail_w and h <= shelf['height'] + 1e-9:
                    waste = shelf['height'] - h
                    if best is None or waste < best[0]:
                        best = (waste, shelf, w, h)

        if best is not None:
            _, shelf, w, h = best
            x, y = shelf['x_cursor'], shelf['y']
            placements.append((it, x, y, w, h))
            shelf['x_cursor'] += w + spacing
        else:
            valid = [(w, h) for (w, h) in options if (w + 2 * spacing) <= max_width]
            w, h = min(valid, key=lambda wh: wh[1]) if valid else min(options, key=lambda wh: wh[1])
            shelf = {'y': y_cursor, 'height': h, 'x_cursor': spacing}
            x, y = shelf['x_cursor'], shelf['y']
            placements.append((it, x, y, w, h))
            shelf['x_cursor'] += w + spacing
            y_cursor += h + spacing
            shelves.append(shelf)

    plate_h = y_cursor
    return placements, max_width, plate_h


def generate_component_holder_stl(
    bom_path: str = None,
    components: list = None,
    output_path: str = "component_holder.stl",
    base_thickness_mm: float = 1.5,
    spacing_mm: float = 2.0,
    clearance_mm: float = 0.3,
    pocket_scale: float = 1.0,
    max_width_mm: float = None,
    group_by: str = "footprint",
    add_finger_notches: bool = False,
    finger_notch_diameter_mm: float = 3.0,
    add_orientation_markers: bool = True,
    marker_diameter_mm: float = 1.2,
    marker_height_mm: float = 0.6,
    footprint_db_path: str = FOOTPRINT_DB_FILE,
    allow_rotation: bool = False,
    add_alignment_holes: bool = True,
    alignment_hole_diameter_mm: float = 3.5,
    alignment_left_offset_mm: float = 8.0,
    alignment_edge_offset_mm: float = 8.0,   # od górnej krawędzi do środka górnego otworu
    alignment_pitch_mm: float = 25.0,        # rozstaw pionowy środków otworów
    # Stała wysokość płyty (wymiar, w którym rozstawione są otwory kalibracyjne) — zawsze ta sama.
    # Jeśli None, dobierana z otworów tak, aby zmieściły się przy górnej krawędzi
    # (górny otwór y=edge_offset, dolny y=edge_offset+pitch) plus symetryczny dolny margines.
    plate_height_mm: float = None,
    # Docelowy minimalny stosunek szerokości do wysokości — 0 = brak wymuszenia (szerokość wynika
    # z liczby elementów; małe zestawy wychodzą smukłe). >0 wymusza, by płyta była przynajmniej
    # tak szeroka jak ten ułamek stałej wysokości.
    plate_width_ratio: float = 0.0,
) -> str:
    """
    Generuje plik STL prostopadłościennej podstawki z wyciętymi kieszeniami na komponenty
    z BOM/pliku pick&place (lub z ręcznie podanej listy komponentów) i zapisuje go pod output_path.

    Dane wejściowe (jedno z dwóch):
      - bom_path: ścieżka do pliku BOM/pick&place — zostanie wczytany przez
        parse_pick_and_place_auto() (obsługuje formaty EasyEDA/KiCad CSV, KiCad .pos, Gerber X3
        i format ogólny). Każdy wiersz musi mieć co najmniej pole 'footprint' (i opcjonalnie 'value').
      - components: ręcznie podana lista komponentów, np.:
            [{'footprint': '0805', 'value': '100nF', 'ref': 'C1'},
             {'footprint': '0805', 'value': '100nF', 'ref': 'C2'},
             {'footprint': 'TQFP-44', 'value': 'ATmega328', 'ref': 'U1'}, ...]
        (wystarczy klucz 'footprint'; 'value' i 'ref' są opcjonalne, używane tylko do grupowania/logu)

    Parametry:
      base_thickness_mm  - grubość LITEGO DNA (podłogi) pod wgłębieniami (mm), domyślnie 1.5.
                              Elementy SMD „leżą” w wgłębieniach (kieszeniach z dnem), które
                              sięgają od górnej powierzchni płyty w dół o głębokość obudowy —
                              NIGDY nie przebijają dna. Całkowita wysokość płyty =
                              base_thickness_mm + najgłębsze wgłębienie.
      spacing_mm          - odstęp/margines między kieszeniami i od krawędzi płyty (mm)
      clearance_mm         - dodatkowy luz doliczany do każdego wymiaru kieszeni przed przeskalowaniem (mm)
      pocket_scale          - mnożnik rozmiaru (szerokość/wysokość) każdej kieszeni,
                              np. 1.2 = kieszenie o 20% większe. Przydatne przy drukarkach, które
                              drukują otwory zaniżone. Domyślnie 1.0 (brak zmiany).
      max_width_mm          - maksymalna szerokość płyty (mm); jeśli None, dobierana automatycznie
                              tak, żeby płyta była zwarta (bez sztucznego minimum)
      group_by              - 'footprint' (grupuj tylko po obudowie) albo 'footprint+value'
                              (osobne kieszenie np. dla różnych wartości rezystorów o tym samym footprint)
      add_finger_notches    - czy dodać dodatkowe półokrągłe wycięcia ułatwiające wyjmowanie elementów
                              palcami (domyślnie wyłączone — przydatne tylko przy ręcznym montażu;
                              przy wyjmowaniu głowicą pick&place są zbędne)
      finger_notch_diameter_mm - średnica wycięcia na palec (mm), używane tylko gdy add_finger_notches=True
      add_orientation_markers - czy dodać małe wypustki oznaczające orientację elementu (pin 1 /
                              katodę/polaryzację). Wypustka leży w LEWYM DOLNYM rogu obrysu
                              (footprintu) KAŻDEGO elementu, stycznie NA ZEWNĄTRZ wgłębienia —
                              nie zachodzi na otwór. Kropka jest dodawana do każdej kieszeni bez
                              zgadywania, czy dana obudowa faktycznie ma polaryzację (takie
                              rozpoznawanie łatwo przeoczyłoby jakiś komponent) — dzięki stałej
                              konwencji zawsze wiesz, gdzie jest pin 1 / katoda.
      marker_diameter_mm       - średnica podstawy wypustki orientacyjnej (mm)
      marker_height_mm          - wysokość wypustki ponad górną powierzchnią płyty (mm)
      footprint_db_path       - ścieżka do pliku JSON z bazą wymiarów obudów (tworzony automatycznie)
      allow_rotation           - czy pakowanie może obracać kieszenie o 90 stopni dla lepszego
                              upakowania. Domyślnie False, żeby wszystkie elementy tego samego
                              (i różnych) typu miały spójną, przewidywalną orientację w podstawce.
      add_alignment_holes    - czy dodać dwa otwory kalibracyjne (do mocowania płyty i jako
                              jednoznaczny punkt odniesienia „lewej/górnej” strony). Przelotowe
                              przez całą grubość płyty.
      alignment_hole_diameter_mm - średnica otworów kalibracyjnych (mm), domyślnie 3.5
      alignment_left_offset_mm   - odległość środka otworu od LEWEJ krawędzi płyty (mm), domyślnie 8
      alignment_edge_offset_mm   - odległość środka GÓRNEGO otworu od GÓRNEJ krawędzi płyty (mm),
                              domyślnie 8 (środek dolnego otworu = +alignment_pitch_mm niżej)
      alignment_pitch_mm         - rozstaw pionowy (odległość środków) obu otworów (mm), domyślnie 25

    Zwraca ścieżkę do zapisanego pliku STL.
    """
    if not bom_path and not components:
        raise ValueError("Podaj bom_path (ścieżkę do pliku BOM/pick&place) albo components (listę komponentów).")

    if components is None:
        rows = parse_pick_and_place_auto(bom_path)
    else:
        rows = components

    if not rows:
        raise ValueError("Lista komponentów jest pusta — brak danych do wygenerowania podstawki.")

    db = load_footprint_db(footprint_db_path)

    # --- grupowanie komponentów wg footprintu (lub footprint+value) ---
    groups = {}  # key -> {'w','h','depth','pins','count','label','matched','note'}
    unmatched_labels = set()
    for r in rows:
        fp = r.get('footprint', '') or r.get('Footprint', '')
        val = r.get('value', '') or r.get('Value', '')
        w, h, depth, pins, matched, note = estimate_footprint_dimensions(fp, val, db)
        w_c = (w + 2 * clearance_mm) * pocket_scale
        h_c = (h + 2 * clearance_mm) * pocket_scale

        key = _norm_fp(fp) if group_by == "footprint" else (_norm_fp(fp) + "|" + _norm_fp(val))
        if key not in groups:
            label = fp.strip() if fp else (val.strip() or "?")
            if group_by == "footprint+value" and val:
                label = f"{fp.strip()} ({val.strip()})" if fp else val.strip()
            groups[key] = {"w": w_c, "h": h_c, "depth": depth, "pins": pins, "count": 0,
                           "label": label, "matched": matched, "note": note}
        groups[key]["count"] += 1
        if not matched:
            unmatched_labels.add(fp.strip() or val.strip() or "?")

    if unmatched_labels:
        print("UWAGA: następujące footprinty nie zostały rozpoznane i otrzymały rozmiar domyślny "
              f"({db.get('__DEFAULT__', {})}). Popraw je w {footprint_db_path}, aby zwiększyć dokładność:")
        for lbl in sorted(unmatched_labels):
            print(f"  - {lbl}")

    # --- rozbicie grup na pojedyncze kieszenie (utrzymując te same typy obok siebie) ---
    ordered_groups = sorted(groups.values(), key=lambda g: max(g["w"], g["h"]), reverse=True)
    items = []
    for g in ordered_groups:
        for _ in range(g["count"]):
            items.append({"w": g["w"], "h": g["h"], "depth": g["depth"], "pins": g["pins"],
                          "label": g["label"]})

    total_area = sum(it["w"] * it["h"] for it in items)
    largest_dim = max(max(it["w"], it["h"]) for it in items)

    # ------------------------------------------------------------------
    # Stała wysokość płyty H — zawsze taka sama (wynika z otworów kalibracyjnych,
    # które są rozstawione pionowo przy lewej krawędzi). Szerokość płyty rośnie
    # z liczbą komponentów (dorysowujemy kolejne kieszenie w poziomych "wierszach").
    # ------------------------------------------------------------------
    if add_alignment_holes:
        # Stała wysokość płyty to oś, w której otwory leżą na LEWYM brzegu (każdy środek 8 mm
        # od swojej krawędzi), a między nimi 25 mm:
        #   górny otwór:  y = edge(8)    od górnej krawędzi
        #   dolny otwór:  y = plate_h - edge(8)  →  8 mm od DOLNEJ krawędzi
        #   plate_h = 8 + 25 + 8 = 41
        min_holes_h = 2.0 * alignment_edge_offset_mm + alignment_pitch_mm
    else:
        min_holes_h = 0.0
    const_plate_h = min_holes_h if plate_height_mm is None else float(plate_height_mm)

    # Szerokość pakowania dobieramy tak, aby kieszenie wypełniły dostatecznie stałą wysokość
    # const_plate_h minimalną szerokością — bez gigantycznych, prawie pustych półek.
    # Szerokość pakowania dobieramy jako NAJMNIEJSZĄ, która mieści kieszenie (rośnie z liczbą
    # komponentów). Przy stałej wysokości (otwory) małe zestawy dadzą wąska, "smukłą" płytkę —
    # co jest oczekiwane. plate_width_ratio >=0 pozwala wymusić minimalny stosunek szer. do wys.
    def _pack_try(width):
        return _pack_shelves(items, width, spacing=spacing_mm, allow_rotation=allow_rotation)

    if max_width_mm is None:
        # dolna granica: najszersza kieszeń (lub ewentualny ratio minimum)
        floor_w = largest_dim + 2 * spacing_mm
        pwr = float(plate_width_ratio)
        if pwr > 0:
            floor_w = max(floor_w, const_plate_h * pwr)
        # górny, wykonalny punkt zaczynamy od "szerokiej" wartości (mieści się na pewno,
        # jeśli nie, podwajamy) — potem schodzimy w dół do minimum.
        width = max(floor_w, const_plate_h)
        _, _, hh = _pack_try(width)
        k = 0
        while hh > const_plate_h + 1e-9 and k < 400:
            width = width * 1.25
            _, _, hh = _pack_try(width)
            k += 1
        if hh > const_plate_h + 1e-9:
            raise ValueError(
                f"Za dużo kieszeni, by zmieścić się w stałej wysokości {const_plate_h:.0f} mm "
                f"nawet dla szerokości {width:.0f} mm. Zwiększ plate_height_mm lub podziel listę.")
        # binarne szukanie najmniejszej szerokości (lo=floor_w, hi=wykonalna)
        lo, hi = floor_w, width
        for _ in range(50):
            mid = (lo + hi) / 2.0
            _, _, hmid = _pack_try(mid)
            if hmid <= const_plate_h + 1e-9:
                hi = mid
            else:
                lo = mid
        width = hi
        placements, plate_w, _pack_h = _pack_try(width)
    else:
        # user podał max_width_mm -> używamy go wprost (także dla wygody eksperymentów)
        width = float(max_width_mm)
        placements, plate_w, _pack_h = _pack_try(width)
    # Wysokość płyty (wymiar, w którym stałe są otwory) zawsze = const_plate_h.
    plate_h = const_plate_h

    # Dopisz lewy pas pod otwory kalibracyjne (kieszenie po prawej).
    wall_extra = 1.0  # mm litej płyty między otworem a kieszenią
    hole_r = alignment_hole_diameter_mm / 2.0 if add_alignment_holes else 0.0
    first_left = min((x for (it, x, y, w, h) in placements), default=0.0)
    if add_alignment_holes:
        need_left = alignment_left_offset_mm + hole_r + wall_extra
        left_gutter = max(0.0, need_left - first_left)
    else:
        left_gutter = 0.0
    if left_gutter > 1e-9:
        placements = [(it, x + left_gutter, y, w, h) for (it, x, y, w, h) in placements]
        plate_w = plate_w + left_gutter

    # wyrównanie w pionie: środkujemy treść w wysokości const_plate_h (top area przy y=0?)
    # Uwaga: otwory przy górze (y=edge_offset), treść może wypełniać od wierzchu w dół.
    # Prosty wybór: zostawiamy y z pakowania (startuje od spacing poniżej y=0).
    # ------------------------------------------------------------------------------------

    # ---- otwory kalibracyjne (2 szt., przelot przez całą grubość) ----
    hole_centers = []  # (cx, cy) w płaszczyźnie XY
    alignment_r = 0.0
    if add_alignment_holes:
        alignment_r = alignment_hole_diameter_mm / 2.0
        if alignment_r <= 0.0:
            raise ValueError("alignment_hole_diameter_mm musi być > 0.")
        # otwory na LEWEJ krawędzi płyty, pionowo: górny 8 mm od górnej krawędzi,
        # dolny 8 mm od DOLNEJ krawędzi (i 25 mm od górnego → plate_h = 8+25+8)
        hole_centers.append((alignment_left_offset_mm, alignment_edge_offset_mm))
        hole_centers.append((alignment_left_offset_mm, plate_h - alignment_edge_offset_mm))
        for nm, (cx, cy) in (("górny", hole_centers[0]), ("dolny", hole_centers[1])):
            if not (alignment_r - 0.5 <= cx <= plate_w - alignment_r + 0.5
                    and alignment_r - 0.5 <= cy <= plate_h - alignment_r + 0.5):
                raise ValueError(
                    f"Otwór kalibracyjny {nm} (środek {cx:.1f},{cy:.1f}, r={alignment_r:.3f}) nie "
                    f"mieści się w płycie {plate_w:.1f} x {plate_h:.1f} mm — zwiększ szerokość "
                    f"(więcej elementów) lub zmień alignment_*_offset / plate_height_mm.")
    # ------------------------------------------------------------------------------------------

    # Kieszenie -> SĄ WCIĘCIAMI (wgłębieniami) z litym dnem, a nie otworami przelotowymi,
    # bo element SMD ma "leżeć" w kieszeni, a nie przez nią przewlekać. Grubość litego dna
    # (podłogi) pod elementami jest STAŁA i równa base_thickness_mm (domyślnie 1.5 mm).
    # Całkowita wysokość bryły to podłoga + najgłębsze wgłębienie, dzięki czemu każda kieszeń
    # sięga od górnej powierzchni w dół o swoją głębokość, ale nigdy głębiej niż do podłogi.
    max_depth = max(it["depth"] for it in items)
    total_h = base_thickness_mm + max_depth
    top_z = total_h
    overshoot = 1.0  # tylko na zewnątrz (powyżej) krawędzi wgłębienia, by uniknąć współpłaszczyznowych ścian

    print(f"Podstawka: {plate_w:.1f} x {plate_h:.1f} x {total_h:.1f} mm, kieszeni: {len(items)}, "
          f"typów komponentów: {len(ordered_groups)}, skala kieszeni: {pocket_scale:.2f}x")

    # --- bryła bazowa: podłoga + słup o wysokości najgłębszej kieszeni ---
    base = trimesh.creation.box(extents=[plate_w, plate_h, total_h])
    base.apply_translation([plate_w / 2.0, plate_h / 2.0, total_h / 2.0])

    # otwory kalibracyjne wykrawamy przez całą grubość płyty
    cutters = []
    markers = []
    for (cx, cy) in hole_centers:
        ch = trimesh.creation.cylinder(radius=alignment_r, height=total_h + 2.0 * overshoot, sections=32)
        ch.apply_translation([cx, cy, total_h / 2.0])
        cutters.append(ch)
    for (it, x, y, w, h) in placements:
        depth = it["depth"]
        cx, cy = x + w / 2.0, y + h / 2.0
        # wgłębienie: od górnej powierzchni (top_z) w dół o (depth+overshoot); spód wgłębienia
        # zatrzymuje się nad podłogą base_thickness_mm — dno nigdy nie jest przebite.
        cz = top_z - depth / 2.0 + overshoot / 2.0
        pocket = trimesh.creation.box(extents=[w, h, depth + overshoot])
        pocket.apply_translation([cx, cy, cz])
        cutters.append(pocket)

        if add_finger_notches:
            notch_r = min(finger_notch_diameter_mm / 2.0, spacing_mm * 0.9, w * 0.4, h * 0.4)
            if notch_r >= 0.5:
                notch = trimesh.creation.cylinder(radius=notch_r, height=depth + overshoot, sections=24)
                notch.apply_translation([cx, y, cz])
                cutters.append(notch)

        # Pin 1 / polaryzacja — KAŻDA kieszeń dostaje wypustkę w DOLNYM-LEWYM rogu obrysu
        # (footprintu) elementu, patrząc na płytę od góry w układzie:
        #   * otwory kalibracyjne po LEWEJ, pionowo (top hole u góry),
        #   * wiersze kieszeni biegną POZIOMO w prawo od kolumny otworów,
        #   * "dół" płyty = większe y (wiersze poniżej / dalsze od góry).
        # Dla kieszeni o narożniku (x,y) (x = odległość od lewej, y od wiersza/góry), lewy-dolny
        # róg obrysu to (x, y+h). Środek kropki jest odsunięty po przekątnej NA ZEWNĄTRZ tego
        # narożnika o r/√2, więc okrąg podstawy jest styczny do rogu otworu i w całości leży na
        # litej płycie (% nie nachodzi na wgłębienie). Kropka jest dodawana dla KAŻDEGO elementu
        # (konwencja zawsze taka sama - bez zgadywania polaryzacji).
        if add_orientation_markers:
            marker_r = min(marker_diameter_mm / 2.0, spacing_mm * 0.45, w * 0.35, h * 0.35)
            if marker_r >= 0.3:
                off = marker_r / (2.0 ** 0.5)  # odsunięcie od narożnika po przekątnej
                # lewy-dolny róg kieszeni (x, y+h): "lewo" (=mały x, strona otworów) i "dół"
                # (=duży y). Odsuwamy kropkę w lewo i w dół (na zewnątrz rogu).
                mx, my = x - off, (y + h) + off
                marker = trimesh.creation.cone(radius=marker_r, height=marker_height_mm, sections=16)
                # Stożek z trimesh.creation.cone ma PODSTAWĘ na z=0, wierzchołek na z=height.
                # Przesunięcie o top_z stawia podstawę na górnej powierzchni płyty.
                marker.apply_translation([mx, my, top_z])
                markers.append(marker)

    result = base.difference(cutters, engine="manifold") if cutters else base
    if markers:
        result = result.union(markers, engine="manifold")

    if not result.is_watertight:
        print("OSTRZEŻENIE: wygenerowana siatka nie jest w pełni szczelna (is_watertight=False). "
              "Plik STL i tak zostanie zapisany, ale warto zweryfikować go np. w PrusaSlicerze/Cura "
              "(funkcja 'napraw modelu').")

    out_dir = os.path.dirname(output_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    result.export(output_path)
    print(f"Zapisano podstawkę STL: {output_path}")
    return output_path


def menu_generate_component_holder_stl():
    """
    Prosty interfejs konsolowy (z użyciem okien dialogowych do wyboru plików) do wygenerowania
    podstawki STL na podstawie pliku BOM/pick&place. Do podpięcia w menu głównym programu.
    """
    print("Generator podstawki STL pod elementy z BOM/pick&place.")
    bom_path = filedialog.askopenfilename(
        title="Wybierz plik BOM / pick&place (CSV/POS/Gerber)",
    )
    if not bom_path:
        print("Nie wybrano pliku.")
        return
    out_path = filedialog.asksaveasfilename(
        title="Wybierz miejsce zapisu podstawki STL",
        defaultextension=".stl",
        filetypes=[("STL Files", "*.stl")],
    )
    if not out_path:
        print("Nie wybrano miejsca zapisu.")
        return

    pocket_scale = float(config.get("holder_pocket_scale", 1.0)) if isinstance(config, dict) else 1.0
    base_thickness_mm = 1.5

    try:
        generate_component_holder_stl(
            bom_path=bom_path,
            output_path=out_path,
            pocket_scale=pocket_scale,
            base_thickness_mm=base_thickness_mm,
        )
    except Exception as e:
        print(f"Błąd podczas generowania podstawki STL: {e}")
    input("Naciśnij Enter, aby kontynuować...")

# =====================================================================================
# ================  KONIEC GENERATORA PODSTAWKI STL POD ELEMENTY Z BOM  ==============
# =====================================================================================


# --- pomocnik: pix -> machine (używamy istniejącej homografii top) ---
def pix_to_machine(px, py, camera='top'):
    H = load_homography(camera=camera)   # pix -> machine (mm)
    pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)[0][0]
    return float(out[0]), float(out[1])

def detect_parts_on_workbench(img, roi=None,
                                 min_area_px=50,
                                 canny=(50,150),
                                 morph_kernel_close=(5,5),
                                 debug_dir="debug_pnp",
                                 save_debug=True):
    """
    Zwraca listę detekcji: [{'px':cx,'py':cy,'area':area,'bbox':(x,y,w,h),'contour':cnt, 'mask':mask_crop}, ...]
    roi = (x,y,w,h) lub None -> wtedy używa całego obrazu.
    """
    if not os.path.exists(debug_dir):
        os.makedirs(debug_dir)
    h0,w0 = img.shape[:2]
    if roi is None:
        x0,y0,w,h = 0,0,w0,h0
    else:
        x0,y0,w,h = roi

    crop = img[y0:y0+h, x0:x0+w].copy()
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    # 1) HSV-based coarse mask (separates metal/ceramic from background in many cases)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h_ch,s_ch,v_ch = cv2.split(hsv)
    # try: low-saturation OR low-value areas (metal may be bright & low sat)
    mask_hsv = cv2.inRange(hsv, (0,0,40), (179,150,255))  # szeroka maska, dopasuj 40/150
    # 2) adaptive threshold on gray for texture-based mask
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    try:
        th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 15, 4)
    except:
        th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    # 3) Canny edges + dilate to close edges
    edges = cv2.Canny(blur, canny[0], canny[1])
    kernel_e = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    edges = cv2.dilate(edges, kernel_e, iterations=1)

    # 4) combine masks
    combined = cv2.bitwise_or(mask_hsv, th)
    combined = cv2.bitwise_or(combined, edges)

    # 5) morphological closing to fill holes
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_close)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, ker, iterations=2)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, ker, iterations=1)

    if save_debug:
        cv2.imwrite(os.path.join(debug_dir, "crop.png"), crop)
        cv2.imwrite(os.path.join(debug_dir, "mask_hsv.png"), mask_hsv)
        cv2.imwrite(os.path.join(debug_dir, "th.png"), th)
        cv2.imwrite(os.path.join(debug_dir, "edges.png"), edges)
        cv2.imwrite(os.path.join(debug_dir, "combined.png"), combined)

    # 6) find contours
    cnts, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    vis = crop.copy()
    cv2.drawContours(vis, cnts, -1, (0,255,0), 1)
    if save_debug:
        cv2.imwrite(os.path.join(debug_dir, "combined_contours.png"), vis)

    for i, c in enumerate(cnts):
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        x,y,wc,hc = cv2.boundingRect(c)
        aspect = float(wc)/max(hc,1)
        # prosty filtr: odrzuć maxi-prostokąty zajmujące niemal całość obrazu
        if wc >= 0.95*w or hc >= 0.95*h:
            # prawdopodobnie belka/edge -> pomiń
            continue
        # dodatkowy filtr po współczynniku kształtu (dostosuj)
        # akceptuj ręcznie: szerokie lub wąskie, ale odrzuć ekstremy
        if aspect > 10 or aspect < 0.05:
            continue

        # Wycięcie masky danego konturu i odszukanie centrum via distanceTransform
        mask_c = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask_c, [c], -1, 255, -1)
        crop_mask = mask_c[y:y+hc, x:x+wc]
        if crop_mask.size == 0:
            continue

        # distance transform -> centrum "najdalsze od brzegu" (dobrze działa na zapełnione obiekty)
        dist = cv2.distanceTransform(crop_mask, cv2.DIST_L2, 5)
        _, maxval, _, maxloc = cv2.minMaxLoc(dist)
        # jeśli maxval == 0 -> użyj moments
        if maxval <= 0.1:
            M = cv2.moments(c)
            if M["m00"] == 0:
                cx = x + wc/2.0
                cy = y + hc/2.0
            else:
                cx = (M["m10"] / M["m00"])
                cy = (M["m01"] / M["m00"])
        else:
            # maxloc jest względne do crop region (x..x+wc,y..y+hc)
            cx = x + maxloc[0]
            cy = y + maxloc[1]

        # transform to image coords (add ROI offset)
        img_cx = x0 + int(round(cx))
        img_cy = y0 + int(round(cy))
        det = {'px': float(img_cx), 'py': float(img_cy), 'area': float(area),
               'bbox': (x0+x, y0+y, wc, hc), 'contour': c, 'mask': mask_c}
        detections.append(det)

        # save crop debug
        if save_debug:
            dbg = crop.copy()
            cv2.rectangle(dbg, (x,y), (x+wc, y+hc), (0,0,255), 2)
            cv2.circle(dbg, (int(cx), int(cy)), 4, (0,255,0), -1)
            cv2.imwrite(os.path.join(debug_dir, f"det_{i}_overlay.png"), dbg)
            cv2.imwrite(os.path.join(debug_dir, f"det_{i}_mask.png"), crop_mask*255)

    # sort by area desc (largest first)
    detections = sorted(detections, key=lambda d: -d['area'])
    return detections

def show_parts_and_ask_label(placements, detected_parts, img=None, window_name="Detected parts (press f to fullscreen, q/Space/Enter to continue)"):
    if img is None:
        img = cv2.imread("img_top.jpeg", cv2.IMREAD_COLOR)
    if img is None:
        img = np.zeros((800,1200,3), dtype=np.uint8)
    vis_base = img.copy()

    # normalize detected_parts: ensure px/py
    cleaned = []
    for i, d in enumerate(detected_parts):
        if not isinstance(d, dict):
            continue
        px = d.get('px', None); py = d.get('py', None)
        if px is None or py is None:
            continue
        cleaned.append({**d, 'px': float(px), 'py': float(py), 'det_idx': i})
    detected_parts = cleaned
    if not detected_parts:
        raise RuntimeError("Brak wykrytych części po normalizacji.")

    def make_visual(idx_highlight=None):
        vis = vis_base.copy()
        cv2.putText(vis, "f=fullscreen, q/Space/Enter=accept, p=view single idx, v=view all", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,0), 2)
        for i, d in enumerate(detected_parts):
            x = int(round(d['px'])); y = int(round(d['py']))
            col = (200,200,200)
            if idx_highlight is not None and idx_highlight == i:
                col = (0,255,0)
            cv2.circle(vis, (x,y), 6, col, -1)
            cv2.putText(vis, str(i), (x+8,y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            if d.get('bbox'):
                try:
                    bx,by,bw,bh = d['bbox']
                    cv2.rectangle(vis, (int(bx),int(by)), (int(bx+bw), int(by+bh)), col, 1)
                except:
                    pass
        return vis

    # prepare window
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    vis = make_visual()
    cv2.imshow(window_name, vis)
    # krótka pętla by przepchnąć eventy i narysować dobrze
    for _ in range(5):
        cv2.waitKey(50)

    # główna pętla oczekiwania — akceptujemy tylko konkretne klawisze
    fullscreen = False
    while True:
        key = cv2.waitKey(0)
        if key == -1:
            continue
        # standardowe kody: 27=ESC, 32=Space, 13=Enter, ord('q')=q, ord('f')=f
        if key in (27, 32, 13) or key in (ord('q'), ord('Q')):
            # zaakceptuj i wyjdź
            break
        if key in (ord('f'), ord('F')):
            # toggle fullscreen
            fullscreen = not fullscreen
            if fullscreen:
                try:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                except:
                    # niektóre buildy nie wspierają - spróbuj resize
                    cv2.resizeWindow(window_name, 1920, 1080)
            else:
                try:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                except:
                    cv2.resizeWindow(window_name, int(vis.shape[1] * 0.75), int(vis.shape[0] * 0.75))
            # odśwież rysunek
            cv2.imshow(window_name, make_visual())
            continue
        if key in (ord('v'), ord('V')):
            # pokaż pełny widok ponownie (znowu czekamy na klawisz)
            cv2.imshow(window_name, make_visual())
            continue
        if key in (ord('p'), ord('P')):
            idxs = input("Podaj indeks detekcji do podglądu: ").strip()
            try:
                ii = int(idxs)
                if 0 <= ii < len(detected_parts):
                    cv2.imshow(window_name, make_visual(idx_highlight=ii))
                else:
                    print("Indeks poza zakresem.")
            except:
                print("Nieprawidłowy indeks.")
            continue
        # inne klawisze ignorujemy i kontynuujemy pętlę

    try:
        cv2.destroyWindow(window_name)
    except:
        pass

    print("\n--- Interaktywne przypisanie detekcji do pozycji PnP ---")
    print("Instrukcje:")
    print(" - Dla każdego placementu wpisz numer wykrytej części (np. 0,1,2...).")
    print(" - Możesz wpisać 'v' aby ponownie obejrzeć pełny widok, 'p' by podejrzeć pojedynczą detekcję, 's' aby pominąć placement.")
    print(" - Po wpisaniu numeru naciśnij Enter.\n")

    results = []
    for pi, plc in enumerate(placements):
        ref = plc.get('ref') or plc.get('Ref') or f"#{pi}"
        fp = plc.get('footprint','')
        print(f"\nPlacement {pi}: ref='{ref}' footprint='{fp}' x={plc.get('x')} y={plc.get('y')}")
        while True:
            ans = input(f"  Wpisz indeks detekcji (0..{len(detected_parts)-1}), 'v' pokaz, 'p' podgląd idx, 's' skip: ").strip().lower()
            if ans == 'v':
                # pokaż pełny widok i czekaj na klawisz w oknie
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.imshow(window_name, make_visual())
                for _ in range(5):
                    cv2.waitKey(50)
                cv2.waitKey(0)
                try: cv2.destroyWindow(window_name)
                except: pass
                continue
            if ans == 'p':
                idxs = input("  Podaj indeks detekcji do podglądu: ").strip()
                try:
                    ii = int(idxs)
                    if 0 <= ii < len(detected_parts):
                        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                        cv2.imshow(window_name, make_visual(idx_highlight=ii))
                        for _ in range(5):
                            cv2.waitKey(50)
                        cv2.waitKey(0)
                        try: cv2.destroyWindow(window_name)
                        except: pass
                    else:
                        print("  Indeks poza zakresem.")
                except:
                    print("  Nieprawidłowy indeks.")
                continue
            if ans == 's' or ans == '':
                print("  Pomijasz ten placement.")
                mapped = None
                break
            # numeric selection
            try:
                idx = int(ans)
                if idx < 0 or idx >= len(detected_parts):
                    print("  Indeks poza zakresem.")
                    continue
                d = detected_parts[idx]
                mapped = {'ref': ref, 'footprint': fp, 'px': float(d['px']), 'py': float(d['py']), 'area': d.get('area'), 'det_idx': idx}
                print(f"  Zmapowano placement '{ref}' -> det #{idx} at ({mapped['px']:.1f},{mapped['py']:.1f})")
                break
            except ValueError:
                print("  Nieprawidłowy wpis. Podaj numer, 'p', 'v' albo 's'.")
        if mapped:
            results.append(mapped)

    try:
        cv2.destroyWindow(window_name)
    except:
        pass

    print("\nMapping finished. Mapped:", len(results), "placements.")
    return results


def pnp_session_loop(pick_coords, placements, ser, camera_x, camera_y, pnp_z_zero, camera_distance,
                     safe_z=0.0, pick_z=-29.05, inspect_z=0, place_z=0,
                     max_retries=3):
    """
    pick_coords: lista dictów zwróconych przez parts_pixels_to_machine,
                 z polami 'ref','machine_x','machine_y','px','py'...
    placements: list zwrócona przez parser PnP (ma pola 'ref','x','y','rotation' etc.)
    ser: serial lub interfejs do stream_gcode_list
    pump_on_cmd / pump_off_cmd: jeśli None -> użytkownik ręcznie steruje pompą
    safe_z/pick_z/inspect_z/place_z: wysokości Z [mm, machine coords]
    camera_up_pos: (x,y) docel dla kamery up (zazwyczaj camera_up_spindle_center + pnp_offset)
    """
    cfg = configparser.ConfigParser()
    cfg.read(CONFIG_FILE)
    conn = cfg["Connection"] if "Connection" in cfg else {}
    stream_gcode_list(ser, ["G90"])
    
    for i, pick in enumerate(pick_coords):
        ref = pick.get('ref') or f"#{i}"
        px = pick.get('px'); py = pick.get('py')
        mx = float(pick['machine_x']); my = float(pick['machine_y'])

        # 1) PODJAZD nad pick (XY) i safe Z
        stream_gcode_list(ser, [f"G0 X{mx:.4f} Y{my:.4f}"])
        send_message(f"GO:{-pnp_z_zero}")
        input("Sprwadź czy prawidłowo podniesiono element i naciśnij enter...")
        send_message("PUMP_ON")
        time.sleep(0.2)  # chwila by złapać element
        send_message(f"GO:{pnp_z_zero}")


        # 4) przejedź nad kamerę up żeby zrobić inspekcję orientacji (jeśli dostępna)
        if camera_x and camera_y:
            stream_gcode_list(ser, [f"G0 X{camera_x:.4f} Y{camera_y:.4f}"])
            send_message(f"GO:{-camera_distance}")
        else:
            raise ValueError("brak koordynatów kamery")
        # zrob capture i wykryj orientację
        capture_write_unified("pnp_up.jpg", camera='up')
        img_up = cv2.imread("pnp_up.jpg", cv2.IMREAD_COLOR)
        det = detect_component_orientation_up(img_up, debug_show=False)
        if det is None:
            print(f"[WARN] Nie wykryto orientacji dla {ref}")
            angle_machine = 0.0
        else:
            angle_image = float(det.get('angle_image', 0.0))
            angle_machine = float(det.get('angle_machine', angle_image))
            # opcjonalnie pokaż centroid/area
            centroid = det.get('centroid')
            area_px = det.get('area_px')
            send_message(f"SPIN:{angle_image:.2f}")
            print(f"[INFO] det_up: angle_img={angle_image:.2f}, angle_mach={angle_machine:.2f}, centroid={centroid}, area={area_px}")

        # 5) Oblicz docelowe miejsce odłożenia z PnP placements:
        #    znaleźć matching placement entry (np. po 'ref' lub footprint)
        place_entry = None
        for pl in placements:
            if pl.get('ref') and pick.get('ref') and pl['ref'].strip().lower() == pick['ref'].strip().lower():
                place_entry = pl
                break
        if place_entry is None:
            # fallback: bierz pierwszą nie przypisaną pozycję (lub zrób skip)
            print(f"[WARN] Nie znaleziono wpisu PnP dla {ref}. Pomijam.")
            send_message("PUMP_OFF"); continue

        # oblicz coords place (masz) = xa,ya + place_entry.x/y  (jeśli już masz xa,ya globalne)
        # tutaj zakładamy, że place_entry ma już machine coords lub będzie przeliczony poza pętlą
        place_mx = (float(X0) + float(place_entry['x'])) #place_entry.get('machine_x') or 
        place_my = (float(Y0) + float(place_entry['y'])) #place_entry.get('machine_y') or
        
        place_mx = place_mx + float(conn.get("pnp_offset_x_mm","0.0"))
        place_my = place_my + float(conn.get("pnp_offset_y_mm","0.0"))
        
        # jeśli mamy pickup_center_offset (kx,ky) w maszynowych mm -> skompensuj
        # (powinieneś zapisać pickup_center_offset w calibracji pick)
        pickup_cx = float(conn.get("pickup_center_offset_x_mm","0.0"))
        pickup_cy = float(conn.get("pickup_center_offset_y_mm","0.0"))
        # jeśli element ma rotację, obróć offset zgodnie z rotacją (rotation deg)
        theta = math.radians(place_entry.get('rotation',0.0))
        dx =  pickup_cx*math.cos(theta) - pickup_cy*math.sin(theta)
        dy =  pickup_cx*math.sin(theta) + pickup_cy*math.cos(theta)
        place_mx_adj = place_mx - dx
        place_my_adj = place_my - dy

        # 6) przejazd nad miejsce odkładania i opuszczenie
        stream_gcode_list(ser, [f"G0 X{place_mx_adj:.4f} Y{place_my_adj:.4f}"])
        send_message(f"GO:{camera_distance-pnp_z_zero}")
        send_message("PUMP_OFF")
        time.sleep(2)
        send_message(f"GO:{pnp_z_zero}")

        # 7) log i ewentualne ponowne wykrycie po odłożeniu
        print(f"[OK] Delivered {ref} to ({place_mx_adj:.3f},{place_my_adj:.3f})")
        # opcjonalnie: update 'placements' status, wizualizacja, zapis logu

def _extract_px_py(d):
    """Zwraca (px,py) z dict d próbując różnych kluczy; zwraca None jeśli brak."""
    for a,b in (('px','py'), ('x','y'), ('cx','cy'), ('col','row'), ('col_px','row_px')):
        if a in d and b in d and d[a] is not None and d[b] is not None:
            try:
                return float(d[a]), float(d[b])
            except:
                pass
    # też spróbuj 'pixel_x'/'pixel_y' lub 'col','row' jako stringi
    for k in ('pixel_x','pixel_y','pixel_x_mm','pixel_y_mm'):
        pass
    return None

def parts_pixels_to_machine(parts_px, camera='top', homography=None, add_offset_mm=(0.0,0.0)):
    """
    Konwertuje listę wykrytych części z pikseli (px,py) na współrzędne maszyny (mm).
    - parts_px: lista słowników; każdy słownik MUSI zawierać piksele pod kluczami:
         'px'/'py' lub 'x'/'y' lub 'cx'/'cy' (funkcja próbuje kilku wariantów).
         Możesz mieć też 'footprint','ref' etc. Funkcja zachowuje istniejące pola.
    - camera: 'top' lub 'up' (przekazywane do load_homography)
    - homography: opcjonalna macierz 3x3 (np. z np.load) — jeśli None, zostanie wywołane load_homography(camera)
    - add_offset_mm: krotka (dx,dy) dodawana do wynikowych machine coords (przydatne np. do korekcji pnp_offset)
    Zwraca nową listę słowników (kopie) z dopisanymi polami 'machine_x','machine_y'.
    Rzuca RuntimeError jeżeli nie uda się uzyskać macierzy homografii lub brakuje współrzędnych pikselowych.
    """
    if not isinstance(parts_px, (list,tuple)):
        raise ValueError("parts_px musi być listą słowników.")

    cfg = configparser.ConfigParser(); cfg.read(CONFIG_FILE)
    conn = cfg["Connection"] if "Connection" in cfg else {}
    pnp_offset_x = float(conn.get("pnp_offset_x_mm","0.0"))
    pnp_offset_y = float(conn.get("pnp_offset_y_mm","0.0"))
    # zbierz punkty px,py
    pts = []
    idx_map = []  # mapuje indeksy części do pozycji w pts
    for i, d in enumerate(parts_px):
        tup = _extract_px_py(d)
        if tup is None:
            # spróbuj pola 'px','py' stringowe w formacie '123.4, 456.7' lub tuple
            if 'pixel' in d and isinstance(d['pixel'], (list,tuple)) and len(d['pixel'])>=2:
                try:
                    tup = (float(d['pixel'][0]), float(d['pixel'][1]))
                except:
                    tup = None
        if tup is None:
            # brak współrzędnych dla tej pozycji -> pomiń
            idx_map.append(None)
            continue
        px, py = tup
        pts.append([ [float(px), float(py)] ])   # shape (1,2) per entry for perspectiveTransform
        idx_map.append(len(pts)-1)

    if not pts:
        raise RuntimeError("Brak poprawnych współrzędnych pikselowych w parts_px.")

    pts_np = np.array(pts, dtype=np.float32)  # shape (N,1,2)

    # załaduj homografię jeśli nie podano
    if homography is None:
        try:
            H = load_homography(camera=camera)   # zakładamy, że ta funkcja istnieje i zwraca 3x3
        except Exception as e:
            raise RuntimeError(f"Nie udało się załadować homografii dla kamery '{camera}': {e}")
    else:
        H = np.array(homography, dtype=np.float64)

    if H is None or H.shape[0] != 3 or H.shape[1] != 3:
        raise RuntimeError("Homografia musi być macierzą 3x3.")

    # convert: pixel -> machine(mm)
    try:
        mm_pts = cv2.perspectiveTransform(pts_np, H)  # shape (N,1,2)
    except Exception as e:
        raise RuntimeError(f"Błąd przy cv2.perspectiveTransform: {e}")


    # wypisz wyniki do kopi słowników
    out = []
    for i, d in enumerate(parts_px):
        copy = dict(d)  # shallow copy
        map_idx = idx_map[i]
        if map_idx is None:
            # nie mieliśmy pikseli dla tej pozycji
            copy['machine_x'] = None
            copy['machine_y'] = None
        else:
            mx = float(mm_pts[map_idx,0,0]) + pnp_offset_x - 2.5
            my = float(mm_pts[map_idx,0,1]) + pnp_offset_y + 1.5
            copy['machine_x'] = mx
            copy['machine_y'] = my
        out.append(copy)

    return out


def debug_detected_parts(parts, n=6):
    """Wypisz proste info o strukturze pierwszych n elementów."""
    print("DEBUG detected_parts: type:", type(parts), "len:", (len(parts) if hasattr(parts, '__len__') else 'unknown'))
    for i, p in enumerate(parts if hasattr(parts, '__iter__') else []):
        if i >= n: break
        print(" item", i, "-> type:", type(p))
        if isinstance(p, dict):
            print("  keys:", list(p.keys()))
            # pokaż wartości krótko
            for k in list(p.keys())[:8]:
                v = p[k]
                if isinstance(v, (list,tuple)):
                    print(f"   {k}: {repr(v)[:80]}")
                else:
                    print(f"   {k}: {str(v)[:80]}")
        elif isinstance(p, (list,tuple)) and len(p) <= 6:
            print("  tuple/list:", p)
        else:
            print("  repr:", repr(p)[:200])
    print("----- end debug -----\n")

def _centroid_from_contour(contour):
    M = cv2.moments(contour)
    if M.get("m00",0) == 0:
        return None
    cx = M["m10"]/M["m00"]
    cy = M["m01"]/M["m00"]
    area = float(M["m00"])
    return (cx, cy, area)

def ensure_rect_on_detections(dets):
    """
    Uzupełnia pole 'rect' w każdym detecie jeśli go brak.
    - Jeśli jest 'contour' używa cv2.minAreaRect.
    - Jeśli contour wygląda na lokalny (ma małe współrzędne w stosunku do bbox),
      przesuwa go o (bbox[0], bbox[1]).
    - Jeśli nie ma contour używa prostego bbox -> rect = ((cx,cy),(w,h),0).
    Zwraca nową listę (kopię detekcji z dodanym polem 'rect').
    """
    out = []
    for d in dets:
        nd = dict(d)  # płytka kopia
        if 'rect' not in nd:
            # prefer contour if istnieje
            cnt = None
            if 'contour' in nd and nd['contour'] is not None and len(nd['contour'])>0:
                # contour może być w postaci np. ndarray [[x,y],...] lub list
                cnt_arr = np.asarray(nd['contour']).reshape(-1,2).astype(np.int32)
                # jeśli istnieje bbox, sprawdź czy contour jest lokalny (małe wartości)
                if 'bbox' in nd and nd['bbox']:
                    bx,by,bw,bh = nd['bbox']
                    # heurystyka: jeśli contour maks < min(bw,bh)*2 to traktujemy jako lokalny
                    if cnt_arr.max() < max(bw,bh)*3:
                        cnt_abs = cnt_arr + np.array([bx,by])
                    else:
                        cnt_abs = cnt_arr
                else:
                    cnt_abs = cnt_arr
                try:
                    rect = cv2.minAreaRect(cnt_abs)
                except Exception:
                    rect = ((float(nd.get('px',0)), float(nd.get('py',0))),
                            (float(nd.get('bbox', (10,10))[2] if 'bbox' in nd else 10),
                             float(nd.get('bbox', (10,10))[3] if 'bbox' in nd else 10)),
                            0.0)
                nd['rect'] = rect
            elif 'bbox' in nd and nd['bbox']:
                bx,by,bw,bh = nd['bbox']
                cx = bx + bw/2.0
                cy = by + bh/2.0
                nd['rect'] = ((cx, cy),(bw, bh), 0.0)
            else:
                # ostatecznie użyj px,py i ustaw mały rect
                px = float(nd.get('px', 0.0))
                py = float(nd.get('py', 0.0))
                nd['rect'] = ((px, py), (10.0, 10.0), 0.0)
        out.append(nd)
    return out

def normalize_detected_parts(detected, merge_dist_px=40, min_area_px=300):
    """
    Normalizuje listę dictów wykryć:
      - odrzuca te poniżej min_area_px
      - scala bliskie centroidy (merge_dist_px)
    Zwraca listę w formacie oczekiwanym przez UI: pola 'px','py','area','bbox','contour','rect'
    """
    # filtr
    filt = [d for d in detected if d['area'] >= min_area_px]
    if not filt:
        return []

    # sort by area descending (merge small ones into big ones)
    filt.sort(key=lambda x: x['area'], reverse=True)

    merged = []
    used = [False]*len(filt)
    for i, a in enumerate(filt):
        if used[i]:
            continue
        group = [a]
        used[i] = True
        ax, ay = a['px'], a['py']
        for j in range(i+1, len(filt)):
            if used[j]:
                continue
            bx, by = filt[j]['px'], filt[j]['py']
            if math.hypot(ax-bx, ay-by) <= merge_dist_px:
                group.append(filt[j])
                used[j] = True
        # merge group -> compute weighted centroid by area
        total_area = sum(g['area'] for g in group)
        cx = sum(g['px']*g['area'] for g in group)/total_area
        cy = sum(g['py']*g['area'] for g in group)/total_area
        # choose bbox that encloses all bboxes
        boxes = [g['bbox'] for g in group]
        xs = [b[0] for b in boxes]; ys=[b[1] for b in boxes]
        ws = [b[0]+b[2] for b in boxes]; hs=[b[1]+b[3] for b in boxes]
        x0 = min(xs); y0 = min(ys); x1 = max(ws); y1 = max(hs)
        merged_contour = np.vstack([g['contour'] for g in group]) if group else np.array([[]])
        merged_rect = group[0]['rect']  # approx (could do better)
        merged.append({
            'px': float(cx), 'py': float(cy),
            'area': float(total_area),
            'bbox': (int(x0), int(y0), int(x1-x0), int(y1-y0)),
            'contour': merged_contour,
            'rect': merged_rect
        })
    return merged

def display_detected_parts_interactive(img, detected_parts,
                                       window_name="Detected parts (f=fullscreen, q/Space/Enter=accept)",
                                       draw_scale=1.0):
    """
    Interaktywny podgląd: pokazuje indeksy, pozwala na:
      - f: fullscreen toggle
      - q/Space/Enter: accept i wyjdź
      - p: wpisz indeks -> pokaż crop (zoom) tego wykrycia
      - v: odśwież widok
    Zwraca listę wykryć (w tej samej kolejności).
    """
    vis_base = img.copy()
    h,w = vis_base.shape[:2]

    def make_vis(highlight_idx=None):
        vis = vis_base.copy()
        # legenda
        cv2.putText(vis, "f=fullscreen, q/Space/Enter=accept, p=view single idx, v=view all", (8,23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,0), 2)
        for i,d in enumerate(detected_parts):
            x = int(round(d['px'])); y = int(round(d['py']))
            col = (200,200,200)
            if highlight_idx is not None and highlight_idx == i:
                col = (0,255,0)
            # marker + index
            cv2.circle(vis, (x,y), 6, col, -1)
            cv2.putText(vis, str(i), (x+8, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            if d.get('bbox'):
                bx,by,bw,bh = map(int, d['bbox'])
                cv2.rectangle(vis, (bx,by), (bx+bw, by+bh), col, 1)
        return vis

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, make_vis())
    # krótka pętla eventów by GUI się zrenderowało
    for _ in range(5): cv2.waitKey(30)

    fullscreen = False
    while True:
        key = cv2.waitKey(0)
        if key == -1:
            continue
        if key in (27, 32, 13, ord('q'), ord('Q')):
            break
        if key in (ord('f'), ord('F')):
            fullscreen = not fullscreen
            try:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
            except:
                # fallback resize
                if fullscreen:
                    cv2.resizeWindow(window_name, 1920, 1080)
                else:
                    cv2.resizeWindow(window_name, int(w*0.75), int(h*0.75))
            cv2.imshow(window_name, make_vis())
            continue
        if key in (ord('v'), ord('V')):
            cv2.imshow(window_name, make_vis())
            continue
        if key in (ord('p'), ord('P')):
            s = input("Podaj indeks detekcji do powiększenia (liczba): ").strip()
            try:
                idx = int(s)
            except:
                print("niepoprawny indeks")
                continue
            if idx < 0 or idx >= len(detected_parts):
                print("poza zakresem")
                continue
            d = detected_parts[idx]
            bx,by,bw,bh = (d.get('bbox') or (int(d['px']-50), int(d['py']-50), 100, 100))
            # safe crop
            bx = max(0,int(bx)); by = max(0,int(by))
            bw = max(1,int(bw)); bh = max(1,int(bh))
            ex = min(img.shape[1], bx+bw); ey = min(img.shape[0], by+bh)
            crop = img[by:ey, bx:ex].copy()
            # scale it up for easy viewing
            mag = 3
            crop_big = cv2.resize(crop, (int(crop.shape[1]*mag), int(crop.shape[0]*mag)), interpolation=cv2.INTER_LINEAR)
            win = f"zoom idx {idx}"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.imshow(win, crop_big)
            cv2.waitKey(0)
            cv2.destroyWindow(win)
            cv2.imshow(window_name, make_vis(highlight_idx=idx))
            continue
        # ignoruj inne klawisze
    try:
        cv2.destroyWindow(window_name)
    except:
        pass
    return detected_parts

def select_roi_interactive(img, window_name="Zaznacz ROI myszką, zamknij okno aby zatwierdzić (bez zaznaczenia = cały obraz)"):
    """
    Interaktywnie wybierz prostokąt ROI. Zwraca (x,y,w,h) lub None.
    Używa matplotlib RectangleSelector zamiast cv2.selectROI — patrz komentarz
    w _show_rotation_grid_and_choose wyżej w pliku: cv2-HighGUI ma trwały konflikt
    z górną kamerą (strumień sieciowy) w tym środowisku.
    """
    from matplotlib.widgets import RectangleSelector

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.imshow(img_rgb)
    ax.set_title(window_name)

    box = {}

    def onselect(eclick, erelease):
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        if None in (x1, y1, x2, y2):
            return
        x, y = int(min(x1, x2)), int(min(y1, y2))
        w, h = int(abs(x2 - x1)), int(abs(y2 - y1))
        box['roi'] = (x, y, w, h)

    selector = RectangleSelector(ax, onselect, useblit=True,
                                  button=[1], interactive=True)
    plt.show()

    roi = box.get('roi')
    if not roi or roi[2] == 0 or roi[3] == 0:
        return None
    return roi

def debug_workbench_roi_pipeline(img, roi,
                                           canny=(20,80),
                                           morph_kernel=(3,3),
                                           min_area_px_try=10,
                                           save_dir="debug_pnp"):
    """
    Pokazuje krok-po-kroku przetwarzanie ROI (crop, gray, clahe, blur, thresh, edges, closed)
    w jednym dużym oknie. Zapisuje obrazy do folderu save_dir.
    Funkcja blokuje się dopóki użytkownik nie naciśnie:
      - SPACE lub Enter -> kontynuuj / zamknij debug i zwróć kontury
      - s -> zapisz dodatkowo obrazy (już zapisujemy domyślnie)
      - q -> zamknij i rzuć wyjątek KeyboardInterrupt
      - f -> toggle fullscreen (przydatne)
    Zwraca listę konturów znalezionych na etapie "closed" (w współrzędnych crop).
    """
    if roi is None:
        print("ROI = None -> anulowano.")
        return []
    rx, ry, rw, rh = roi
    crop = img[ry:ry+rh, rx:rx+rw].copy()
    os.makedirs(save_dir, exist_ok=True)
    timestamp = int(time.time())
    base = os.path.join(save_dir, f"roi_{timestamp}")

    # przygotuj obrazy
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray2 = clahe.apply(gray)
    except Exception:
        gray2 = gray
    blur = cv2.GaussianBlur(gray2, (5,5), 0)

    # adaptive threshold (good fallback)
    th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY_INV, 11, 2)

    # Canny + morphology
    edges = cv2.Canny(blur, canny[0], canny[1])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    closed = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel, iterations=1)

    # znajdź kontury (z użyciem closed jako maski)
    cnts, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # overlay wykryć
    overlay = crop.copy()
    for i, c in enumerate(cnts):
        a = cv2.contourArea(c)
        color = (0,255,0) if a >= min_area_px_try else (180,180,180)
        # obrys minimalnym prostokątem
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(overlay, [box], 0, color, 2)
        # centroid
        M = cv2.moments(c)
        if M.get('m00',0) != 0:
            cx = int(M['m10']/M['m00']); cy = int(M['m01']/M['m00'])
        else:
            cx,cy = box[0]
        cv2.putText(overlay, f"{i}:{int(a)}", (cx+3, cy+3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # przygotuj siatkę do wyświetlenia (skalowanie jeśli trzeba)
    def to3ch(x):
        if len(x.shape)==2:
            return cv2.cvtColor(x, cv2.COLOR_GRAY2BGR)
        return x
    imgs = [("crop", crop), ("gray", to3ch(gray)), ("clahe", to3ch(gray2)),
            ("blur", to3ch(blur)), ("thresh", to3ch(th)), ("edges", to3ch(edges)),
            ("closed", to3ch(closed)), ("overlay", overlay)]
    # ułóż w siatkę 2x4
    rows = []
    row_imgs = []
    for i,(_,im) in enumerate(imgs):
        # dopasuj szerokość do crop szerokości dla ładnego layoutu
        row_imgs.append(im)
        if (i%4)==3:
            rows.append(row_imgs); row_imgs=[]
    if row_imgs:
        # dopisz pusty filler jeśli trzeba
        while len(row_imgs)<4:
            row_imgs.append(np.zeros_like(row_imgs[0]))
        rows.append(row_imgs)

    # scale each tile to reasonable size if huge
    max_tile_w = 640
    tiles = []
    for r in rows:
        row_tiles = []
        for t in r:
            h,w = t.shape[:2]
            scale = 1.0
            if w > max_tile_w:
                scale = max_tile_w / w
            t2 = cv2.resize(t, (int(w*scale), int(h*scale)))
            row_tiles.append(t2)
        # horizontally concat, pad heights
        hmax = max(tt.shape[0] for tt in row_tiles)
        padded = []
        for tt in row_tiles:
            if tt.shape[0] < hmax:
                pad = np.zeros((hmax-tt.shape[0], tt.shape[1], 3), dtype=tt.dtype)
                tt = np.vstack([tt, pad])
            padded.append(tt)
        row_cat = np.hstack(padded)
        tiles.append(row_cat)
    full = np.vstack(tiles)

    # zapisz poszczególne obrazy i pełny widok na dysk
    for name, im in imgs:
        fn = f"{base}_{name}.png"
        cv2.imwrite(fn, im)
    cv2.imwrite(f"{base}_full.png", full)

    print("DEBUG: zapisałem obrazy do:", save_dir)

    # Podgląd przez matplotlib zamiast cv2.imshow — patrz komentarz w _show_rotation_grid_and_choose
    # wyżej w pliku: cv2-HighGUI ma trwały konflikt z górną kamerą (strumień sieciowy) w tym
    # środowisku. Prosty model interakcji: zamknij okno aby kontynuować.
    full_rgb = cv2.cvtColor(full, cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(figsize=(14, 7))
    ax.imshow(full_rgb)
    ax.set_title("crop | gray | clahe | blur | thresh | edges | closed | overlay — zamknij okno aby kontynuować")
    ax.axis('off')
    plt.show()

    # zwróć kontury (wcrop coords)
    return cnts

def _centroid_from_mask(mask):
    M = cv2.moments(mask.astype(np.uint8))
    if M.get('m00',0) == 0:
        return None
    cx = M['m10']/M['m00']; cy = M['m01']/M['m00']
    return (float(cx), float(cy))

def _maxdist_point_from_mask(mask):
    # distance transform + argmax (gives center of largest inscribed circle)
    mask_u = (mask>0).astype(np.uint8)
    if mask_u.sum() == 0:
        return None
    dist = cv2.distanceTransform(mask_u, cv2.DIST_L2, 5)
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(dist)
    return (float(maxLoc[0]), float(maxLoc[1]))

def _hull_centroid(contour):
    if contour is None or len(contour)==0:
        return None
    hull = cv2.convexHull(contour)
    M = cv2.moments(hull)
    if M.get('m00',0)==0:
        return None
    return (float(M['m10']/M['m00']), float(M['m01']/M['m00']))

def _minrect_center(rect):
    if rect is None:
        return None
    (cx,cy),(w,h),ang = rect
    return (float(cx), float(cy))

def _find_metal_pads_midpoint(crop_bgr, mask_body=None, min_pad_area=20):
    """
    Heurystyka: znajdź jasne obszary (metal), zwróć środek między dwoma największymi padami.
    crop_bgr - obraz crop (BGR)
    mask_body - (opcjonalnie) maska ciała by ograniczyć rozpoznanie
    """
    lab = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2LAB)
    L = lab[:,:,0]
    # adapt threshold: jasne obszary (metal) -> wysokie L
    _, th = cv2.threshold(L, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # optionally intersect with body mask
    if mask_body is not None:
        th = cv2.bitwise_and(th, th, mask=(mask_body>0).astype(np.uint8))
    # morphology to clean
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, ker, iterations=1)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pads = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_pad_area: 
            continue
        M = cv2.moments(c)
        if M.get('m00',0)==0:
            continue
        cx = M['m10']/M['m00']; cy = M['m01']/M['m00']
        pads.append((a, (cx,cy), c))
    if not pads:
        return None
    pads.sort(key=lambda x: x[0], reverse=True)
    if len(pads) >= 2:
        # take two largest pads
        (a1, (x1,y1), _), (a2, (x2,y2), _) = pads[0], pads[1]
        return ((x1+x2)/2.0, (y1+y2)/2.0)
    # if only one pad found, fallback to its centroid
    return pads[0][1]

def refine_detection_center(img, detection, pad_px=12, min_area_px=10, debug=False, dbg_prefix="refine_v2"):
    """
    Ulepszona wersja refinementu, zwracająca kilka metod środka i wybierając najlepszą.
    Zwraca detection z nowymi polami:
      refined_px, refined_py (global coords)
      centers_candidates: dict { 'moments':(x,y), 'minrect':..., 'hull':..., 'mask_centroid':..., 'maxdist':..., 'pads_mid':... }
    """
    if img is None or 'bbox' not in detection:
        return detection

    H,W = img.shape[:2]
    bx,by,bw,bh = detection['bbox']
    x0 = max(0, bx - pad_px); y0 = max(0, by - pad_px)
    x1 = min(W, bx + bw + pad_px); y1 = min(H, by + bh + pad_px)
    crop = img[y0:y1, x0:x1].copy()
    if crop.size == 0:
        return detection

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
    except:
        pass
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    # produce mask for body: combine Otsu and adaptive
    _, th_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    th_adapt = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 11, 2)
    mask = cv2.bitwise_or(th_otsu, th_adapt)
    # close to fill interior
    ker_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker_close, iterations=2)
    # remove small noise
    ker_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker_open, iterations=1)

    # find contours
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        detection['refined_px'] = detection.get('px', None)
        detection['refined_py'] = detection.get('py', None)
        detection['centers_candidates'] = {}
        return detection

    # choose largest contour (area) within crop
    best = max(cnts, key=cv2.contourArea)
    area_best = cv2.contourArea(best)
    if area_best < min_area_px:
        detection['refined_px'] = detection.get('px', None)
        detection['refined_py'] = detection.get('py', None)
        detection['centers_candidates'] = {}
        return detection

    # ensure mask is only for best contour (fill it)
    mask_best = np.zeros_like(mask)
    cv2.drawContours(mask_best, [best], -1, 255, thickness=cv2.FILLED)

    # compute rect and centers (in crop coords)
    rect = cv2.minAreaRect(best)  # ((cx,cy),(rw,rh),angle)
    c_mom = _centroid_from_mask(mask_best)
    c_maxd = _maxdist_point_from_mask(mask_best)
    c_hull = _hull_centroid(best)
    c_minr = _minrect_center(rect)

    # try metal pads midpoint (in crop coords)
    c_pads = _find_metal_pads_midpoint(crop, mask_body=mask_best, min_pad_area=max(8, min_area_px//4))

    # convert all found to global coords
    def to_global(pt):
        if pt is None: 
            return None
        return (pt[0] + x0, pt[1] + y0)

    centers = {
        'moments': to_global(c_mom),
        'maxdist': to_global(c_maxd),
        'hull': to_global(c_hull),
        'minrect': to_global(c_minr),
        'pads_mid': to_global(c_pads)
    }

    # heurystyka wyboru:
    # prefer pads_mid if exists (most reliable for components with two metal ends)
    chosen = None
    if centers['pads_mid'] is not None:
        chosen = centers['pads_mid']
        reason = 'pads_mid'
    else:
        # if shape roughly rectangular (aspect ratio near detection bbox) prefer minrect center
        (rw,rh) = rect[1]
        if rw*rh > 0:
            ar_rect = max(rw/rh, rh/rw)
        else:
            ar_rect = 1.0
        # if area is fairly compact choose maxdist or moments
        if centers['maxdist'] is not None and area_best > 5:
            chosen = centers['maxdist']; reason = 'maxdist'
        elif centers['hull'] is not None:
            chosen = centers['hull']; reason = 'hull'
        elif centers['moments'] is not None:
            chosen = centers['moments']; reason = 'moments'
        else:
            chosen = centers['minrect']; reason = 'minrect'

    # finally: write into detection
    detection['refined_px'] = float(chosen[0]) if chosen is not None else detection.get('px', None)
    detection['refined_py'] = float(chosen[1]) if chosen is not None else detection.get('py', None)
    detection['centers_candidates'] = centers
    detection['refine_reason'] = reason if chosen is not None else 'fallback'
    detection['refined_area'] = float(area_best)
    detection['refined_contour'] = (best + np.array([[[x0, y0]]])).astype(int)

    if debug:
        vis = img.copy()
        # draw bbox and crop rect
        cv2.rectangle(vis, (bx,by), (bx+bw, by+bh), (0,255,255), 1)
        cv2.rectangle(vis, (x0,y0), (x1,y1), (255,255,0), 1)
        # draw best contour global
        try:
            cv2.drawContours(vis, [detection['refined_contour']], -1, (0,255,0), 2)
        except:
            pass
        # draw candidate centers
        colors = {'moments':(0,255,0),'maxdist':(0,192,255),'hull':(255,0,0),'minrect':(0,0,255),'pads_mid':(255,0,255)}
        for k,v in centers.items():
            if v is None: continue
            cv2.circle(vis, (int(round(v[0])), int(round(v[1]))), 4, colors.get(k,(255,255,255)), -1)
            cv2.putText(vis, k, (int(round(v[0]))+6, int(round(v[1]))-6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors.get(k,(255,255,255)),1)
        # draw chosen
        if chosen is not None:
            cv2.circle(vis, (int(round(detection['refined_px'])), int(round(detection['refined_py']))), 6, (0,0,0), -1)
            cv2.circle(vis, (int(round(detection['refined_px'])), int(round(detection['refined_py']))), 4, (0,255,255), -1)
            cv2.putText(vis, f"choose:{reason}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0),2)
        cv2.imwrite(f"{dbg_prefix}_candidates_overlay.png", vis)
        cv2.imwrite(f"{dbg_prefix}_crop_mask.png", mask)
        cv2.imwrite(f"{dbg_prefix}_crop.png", crop)

    return detection

def parse_status_response(response):
    """
    Parsuje odpowiedź GRBL w formacie:
    <Idle|WPos:1.000,1.000,3.000|FS:0,0>
    i zwraca wartość Z jako float.
    """
    pattern = r"WPos:\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*)"
    match = re.search(pattern, response)
    if match:
        return float(match.group(3))
    return None


def get_z_from_status(ser, timeout=10, retry_delay=0.2):
    """
    Wysyła '?' do GRBL i zwraca Z z MPos.
    Jeśli status to Run, ponawia odczyt aż do Idle albo timeoutu.
    """
    start_time = time.time()

    while time.time() - start_time < timeout:
        print("Sending: ?")
        ser.write(b"?")   # bez newline

        line = ser.readline().strip().decode("utf-8", errors="ignore")
        if not line:
            time.sleep(retry_delay)
            continue

        print(" ->", line)

        if line.startswith("<"):
            # jeśli maszyna nadal jedzie, odczekaj i spróbuj jeszcze raz
            if line.startswith("<Run|"):
                time.sleep(retry_delay)
                continue

            z = parse_status_response(line)
            if z is not None:
                return z

        time.sleep(retry_delay)

    return None

# Zmienne na wybrane ścieżki zapisu plików wyjściowych
front_output = None
back_output = None
outline_output = None
vias_output = None
throughholes_output = None


levelfeed = format_value(config['levelfeed'])
delta = format_value(config['delta'])
zlevelmax = format_value(config['zlevelmax'])
zlevelmin = format_value(config['zlevelmin'])
cut_z = format_value(config['cut_z'])
while True:
    cls()
    print("*Menu*\n1. Wygeneruj gcode\n2. Zmień ustawienia\n3. Kalibracja\n4. Połączenie\n"
          "5. Wygeneruj podstawkę STL pod elementy z BOM\n6. Wyjście")
    wybor = int(readchar.readchar())
    if wybor == 1:
        if polaczenie < 3:
            print("Nie wybrano połączenia z maszyną lub jedną z kamer!")
            input("Prosze nacisnac enter i w menu dokonać połączenia z obydwoma (teraz resetuje się licznik)")
            polaczenie = 0
        else:
            stream_gcode_list(ser, ["$X"])
            cls()
            cmd = ["pcb2gcode"]
            gcode_paths = []
            sciezki = {}
            # Wybór plików wejściowych
            for plik in pliki:
                print(f"Czy chcesz wybrać plik {plik}? (t/n): ")
                odp = str(readchar.readchar())
                if odp.lower() == "t":
                    sciezka = filedialog.askopenfilename(title=f"Wybierz plik {plik}")
                    if sciezka:
                        sciezki[f"sciezka_{plik}"] = sciezka
                        gcode_paths.append(sciezka)

            # Dla poszczególnych warstw zapytaj użytkownika o miejsce zapisu plików NC
            if "sciezka_Górna warstwa" in sciezki:
                front_output = filedialog.asksaveasfilename(title="Wybierz miejsce zapisu dla front.nc",
                                                             defaultextension=".nc",
                                                             filetypes=[("NC Files", "*.nc")])
                cmd.append(f"--front '{sciezki['sciezka_Górna warstwa']}'")
                cmd.append(f"--front-output '{front_output}'")

            if "sciezka_Dolna warstwa" in sciezki:
                back_output = filedialog.asksaveasfilename(title="Wybierz miejsce zapisu dla back.nc",
                                                            defaultextension=".nc",
                                                            filetypes=[("NC Files", "*.nc")])
                cmd.append(f"--back '{sciezki['sciezka_Dolna warstwa']}'")
                cmd.append(f"--back-output '{back_output}'")

            if "sciezka_outline" in sciezki:
                outline_output = filedialog.asksaveasfilename(title="Wybierz miejsce zapisu dla outline.nc",
                                                               defaultextension=".nc",
                                                               filetypes=[("NC Files", "*.nc")])
                cmd.append(f"--outline '{sciezki['sciezka_outline']}'")
                cmd.append(f"--outline-output '{outline_output}'")

            if "sciezka_vias" in sciezki:
                vias_output = filedialog.asksaveasfilename(title="Wybierz miejsce zapisu dla vias.nc",
                                                            defaultextension=".nc",
                                                            filetypes=[("NC Files", "*.nc")])
                vias.append(f"--drill '{sciezki['sciezka_vias']}'")
                vias.append(f"--drill-output '{vias_output}'")
                
                if "sciezka_Górna warstwa" in sciezki:
                    
                    vias.append(f"--front '{sciezki['sciezka_Górna warstwa']}'")
                    vias.append(f"--front-output tymfzasowyplikokreslajacywymiaryoniemozliwejnazwie.nc")
                else:
                    print("nie wybrano pliku który pozwala oszacować wymiarów płytki, wygenerowany plik gcode najprawdopodobniej będzie missaligned!!")


            if "sciezka_through holes" in sciezki:
                throughholes_output = filedialog.asksaveasfilename(title="Wybierz miejsce zapisu dla through_holes.nc",
                                                                   defaultextension=".nc",
                                                                   filetypes=[("NC Files", "*.nc")])
                cmd.append(f"--drill '{sciezki['sciezka_through holes']}'")
                cmd.append(f"--drill-output '{throughholes_output}'")

            final_command = " ".join(cmd)
            final_vias_command = " ".join(vias)
            millproject_content = save_millproject(config)
            with open("millproject", "w") as f:
                f.write(millproject_content)
            existing_svgs = set(glob.glob("*.svg"))
            start_time = time.time()
            subprocess.run(final_command, shell=True)
            if "sciezka_vias" in sciezki:
                subprocess.run(final_vias_command, shell=True)
                os.remove('tymfzasowyplikokreslajacywymiaryoniemozliwejnazwie.nc')

            # Modyfikacja pliku front, jeśli istnieje
            if front_output and os.path.exists(front_output):
                temp_file_path = "temp.txt"
                with open(front_output, "r") as f, open(temp_file_path, "w") as temp_f:
                    for i, line in enumerate(f, start=1):
                        if not (11 <= i <= 21):  # pomijamy linie 11-21
                            temp_f.write(line)
                os.replace(temp_file_path, front_output)

            new_svgs = {f for f in glob.glob("*.svg") if os.path.getmtime(f) >= start_time}

            for file in new_svgs:
                os.remove(file)
                print(f"Usunięto: {file}")

            input("\nNaciśnij Enter, aby kontynuować...")
            cls()
            input("Program wykona teraz zdjęcie by określić punkt zerowy płytki.\nUpewnij się że maszyna oraz kamera są włączone, podłączone oraz gotowe do pracy.\nNaciśnij enter, aby kontynuować")
            print("\nWykonwyanie zdjęcia...")
            print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
            wybor = str(readchar.readchar())
            if wybor == str("t"):
                stream_gcode_list(ser, home)
                stream_gcode_list(ser, gcode_zdj)
            capture = capture_write_unified()
            zdj = cv2.imread("image.jpeg", cv2.IMREAD_COLOR)
            if zdj is None:
                raise FileNotFoundError("Nie udało się wczytać image.jpeg")

            h, w = zdj.shape[:2]
            fig, ax = plt.subplots(figsize=(8, 6))
            ax.imshow(cv2.cvtColor(zdj, cv2.COLOR_BGR2RGB))
            ax.set_title("Podgląd — zamknij okno aby kontynuować")
            ax.axis('off')
            plt.show()
            input("teraz program wykona detekcje punktu 0")
            max_w, max_h = 0.0, 0.0
            for path in gcode_paths:
                w, h = parse_dimensions(path)
                print(f"Plik {os.path.basename(path)}: szer. {w:.3f} mm, wys. {h:.3f} mm")
                if w > max_w: max_w = w
                if h > max_h: max_h = h
            print(f"\n➡️ Największa szerokość spośród wybranych: {max_w:.3f} mm")
            print(f"➡️ Największa wysokość spośród wybranych: {max_h:.3f} mm")
            X0, Y0 = detect_copper_origin(zdj, max_w, max_h)
            Xpnp = X0
            Ypnp = Y0-max_h
            xa=-X0+72.6 #-18.4
            ya=192-Y0+max_h       
            gcode_rog = [
            f"G92 X{xa} Y{ya}"
            ]
            stream_gcode_list(ser, gcode_rog)
            if gcode_paths:
                if max_w>5:
                    w_points=max_w//int(delta)
                    delta_w=max_w/w_points
                    w_points+=1
                else:
                    w_points=2
                    delta_w=max_w
                if max_h>5:
                    h_points=max_h//int(delta)
                    delta_h=max_h/h_points
                    h_points+=1
                else:
                    h_points=2
                    delta_h=max_h
                input(f"\nGrid o wielkości {w_points}x{h_points} z punktami oddalonymi od siebie o {delta_w} i {delta_h}\nmaszyna przejedzie teraz do rogu płytki. Naciśnij enter by wykonać")
                stream_gcode_list(ser, ["G90 G0 X0 Y0"])
                input("maszyna określi teraz wysokość zerową. Przymocuj do laminatu i frezu przewodniki i naciśnij enter")
                Z_RR=determine_z_zero(ser, 60, 1, 10)
                auto_level(ser, w_points, h_points, delta_w, delta_h, zlevelmax, zlevelmin, levelfeed, Z_RR)
                gcode_files = [front_output, back_output, outline_output, vias_output, throughholes_output]
                gcode_files = [p for p in gcode_files if p]    # tylko te, które wybrał użytkownik
                if gcode_files:
                    batch_apply_levels(gcode_files)
                    leveled_jobs = []
                    if front_output:
                        leveled_jobs.append((front_output, "górnej warstwy"))
                    if back_output:
                        leveled_jobs.append((back_output, "dolnej warstwy"))
                    if outline_output:
                        leveled_jobs.append((outline_output, "konturu"))
                    if vias_output:
                        leveled_jobs.append((vias_output, "vias"))
                    if throughholes_output:
                        leveled_jobs.append((throughholes_output, "wierceń"))

                    # 3) Dla każdego wygeneruj nazwę pliku leveled i wyślij
                    for orig_path, label in leveled_jobs:
                        base, ext = os.path.splitext(orig_path)
                        leveled_path = f"{base}_leveled{ext}"
                        if os.path.exists(leveled_path):
                            input(f"\nTeraz zostanie wysłany plik {label}.\nZałóż odpowiedni frez i naciśnij Enter, aby kontynuować…")
                            send_message("DREMEL_ON")
                            stream_gcode_file_pipelined(ser, leveled_path)
                            send_message("DREMEL_OFF")
                        else:
                            print(f"⚠️ Nie znaleziono pliku {os.path.basename(leveled_path)}, pomijam.")
                    
                    input("\n➡️ Wszystkie pliki wysłane.")
                    print("Czy chcesz wykonać operacje PnP na wykonanej płytce? (t/n)")
                    wybor = str(readchar.readchar())
                    if wybor.lower() != "t":
                        # użytkownik nie chce PnP -> wychodzimy
                        break
                    else:
                        # 1) Upewnij się, że X0,Y0 są w pamięci (wyznaczone przy frezowaniu)
                        try:
                            Xpnp, Ypnp
                        except NameError:
                            raise RuntimeError("Brak zdefiniowanych zmiennych xa/ya — najpierw wykonaj detekcję punktu 0 przed frezowaniem.")
                        
                        X0 = Xpnp
                        Y0 = Ypnp
                        
                        # 2) Wczytaj plik PnP
                        sciezka_pnp = filedialog.askopenfilename(title="Wybierz plik PnP")
                        if not sciezka_pnp:
                            print("Nie wybrano pliku PnP.")
                        else:
                            placements = parse_pick_and_place_auto(sciezka_pnp)
                            print(f"rows: {len(placements)}")
                            xs = [r['x'] for r in placements if r.get('x') is not None]
                            ys = [r['y'] for r in placements if r.get('y') is not None]
                            if xs and ys:
                                print(f"span_x = {max(xs)-min(xs):.3f} mm, span_y = {max(ys)-min(ys):.3f} mm")
                            print("first 8 rows:")
                            for r in placements[:8]:
                                print(" ", r)

                            print("Czy wszystko się zgadza? (t/n)")
                            wybor = str(readchar.readchar())
                            if wybor.lower() != "t":
                                print("Anulowano PnP przez użytkownika.")
                            else:
                                input("Proszę rozłożyć odpowiednie elementy SMD na blacie roboczym. Naciśnij Enter aby kontynuować...")


                                print("Należy wykonać zdjęcie.")
                                send_message("HOME")
                                z = get_z_from_status(ser)
                                pnp_z_zero = z + 3
                                camera_distance = 10-pnp_z_zero
                                stream_gcode_list(ser, unlock)
                                stream_gcode_list(ser, home)
                                stream_gcode_list(ser, gcode_zdj)   # gcode_zdj powinien ustawiać maszynę nad miejscem robienia zdjęcia 'top'

                                # 3) capture top camera
                                capture_write_unified(filename="img_top.jpeg", camera='top')
                                img_top = cv2.imread("img_top.jpeg", cv2.IMREAD_COLOR)
                                if img_top is None:
                                    raise RuntimeError("Nie udało się wczytać img_top.jpeg")
                                
                                # wybierz ROI (jeśli chcesz skupić się na obszarze z taśmą)
                                print("Wybierz ROI (obszar z częściami) — zamknij oknem lub anuluj, jeśli chcesz użyć całego obrazu.")
                                roi = select_roi_interactive(img_top)   # zwraca (x,y,w,h) lub None
                                
                                cnts = debug_workbench_roi_pipeline(img_top, roi, canny=(20,80), morph_kernel=(3,3), min_area_px_try=10)

                                input("Program wykona teraz detekcję elementów SMD. Naciśnij Enter aby kontynuować...")
                                # 1) wykryj elementy na blacie (raw)
                                dets_raw = detect_parts_on_workbench(img_top, roi=roi, min_area_px=20, morph_kernel_close=(5,5), debug_dir="debug_pnp", save_debug=True)

                                # 2) (opcjonalnie) pokaż surowe debugowanie jeśli chcesz
                                print("Raw detections:", len(dets_raw))
                                debug_detected_parts(dets_raw, n=8)

                                # 3) normalizuj / odfiltruj i scal drobne artefakty
                                dets_with_rect = ensure_rect_on_detections(dets_raw)
                                dets_norm = normalize_detected_parts(dets_with_rect, merge_dist_px=0, min_area_px=100)
                                print("After normalize -> count:", len(dets_norm))
                                if not dets_norm:
                                    raise RuntimeError("Nie wykryto części po normalizacji. Sprawdź ROI/parametry/detekcję.")
                                # 4) interaktywnie obejrzyj i zaakceptuj/zbadaj wykrycia
                                #    display_detected_parts_interactive obsługuje sterowanie (f, p, v, q/Enter)
                                dets_norm = display_detected_parts_interactive(img_top, dets_norm)

                                # 5) po akceptacji wywołaj istniejący UI do przypisania wykryć do pozycji z PnP
                                detected_centroid_pixels = show_parts_and_ask_label(placements, dets_norm, img=img_top)

                                # 6) przelicz piksele -> współrzędne maszyny (mm)
                                # jeśli masz gotową funkcję parts_pixels_to_machine -> użyj jej; inaczej użyj homografii (przykładowa funkcja poniżej)
                                pick_coords = parts_pixels_to_machine(detected_centroid_pixels, camera='top')
                               # 7) Wyświetl innformacje i wypisz plan (suchy run)
                                print("Plan PnP - pick coords (machine mm):")
                                for p in pick_coords:
                                    print(p.get('ref'), "->", p.get('machine_x'), p.get('machine_y'))
                                pnp_session_loop(pick_coords=pick_coords, placements=placements, ser=ser, camera_x=pnp_cam_x, camera_y=pnp_cam_y, pnp_z_zero=pnp_z_zero, camera_distance=camera_distance)


                else:
                    print("Brak plików G‑code do post‑procesu.")
                    input("a")
            else:
                print("Brak wybranych plików do analizy wymiarów.")
                input("a")
        
    elif wybor == 2:
        while True:
            cls()
            options = {
                1: "tool_diam", 2: "passes", 3: "pass_overlap", 4: "cut_z",
                5: "travel_z", 6: "end_move", 7: "xyfeedrate", 8: "zfeedrate",
                9: "spindle", 10: "multi_depth", 11: "zdrill",
                12: "milldrill_diameter", 13: "zcut_outline", 14: "outline_xyfeedrate",
                15: "bridges", 16: "bridgesnum", 17: "zbridges", 18: "delta", 19: "zlevelmax", 20: "zlevelmin",
                21: "holder_pocket_scale"
            }
            for num, key in options.items():
                print(f"{num}. {key.replace('_', ' ').capitalize()}: {config[key]}")
            print("22. Powrót do menu głównego")
            wybor_ustawienia = int(input())
            if wybor_ustawienia == 22:
                millproject_content = save_millproject(config)
                with open("millproject", "w") as f:
                    f.write(millproject_content)
                break
            elif wybor_ustawienia in options:
                key = options[wybor_ustawienia]
                config[key] = float(input(f"Podaj nową wartość dla {key.replace('_', ' ')}: "))
                save_config(config)
    elif wybor == 3:
        if polaczenie < 3:
            print("Nie wybrano połączenia z maszyną lub jedną z kamer!")
            input("Prosze nacisnac enter i w menu dokonać połączenia z obydwoma (teraz resetuje się licznik)")
            polaczenie = 0
        else:
            while True:
                cls()
                print("1. Kamera górna\n2. Kamera w blacie\n3. Wyjście")
                wybor = int(readchar.readchar())
                if wybor == 1:
                    cls()
                    while True:
                        print("1. Kalibracja kolorów\n2. kalibracja rogów\n3. Test kolorów\n4. Orientacja kamery\n5. Wyjście")
                        wybor = int(readchar.readchar())
                        if wybor == 1:
                            cls()
                            print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
                            wybor = str(readchar.readchar())
                            if wybor == str("t"):
                                stream_gcode_list(ser, home)
                                stream_gcode_list(ser, gcode_zdj)
                            input("Uwaga, program za chwile wykona zdjęcie. Naciśnij enter gdy gotowy")
                            capture_write_unified()  # zapisz obraz jako 'image.jpeg'
                            zdj = cv2.imread('image.jpeg', cv2.IMREAD_COLOR)
                            if zdj is None:
                                raise FileNotFoundError("Nie udało się wczytać image.jpeg")
                            manual_color_calibration(zdj)
                        
                        elif wybor == 2:
                            cls()
                            print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
                            wybor = str(readchar.readchar())
                            if wybor == str("t"):
                                stream_gcode_list(ser, home)
                                stream_gcode_list(ser, gcode_zdj)
                            input("Uwaga, program za chwile wykona zdjęcie. Naciśnij enter gdy gotowy")
                            capture_write_unified()  # zapisz obraz jako 'image.jpeg'
                            zdj = cv2.imread('image.jpeg', cv2.IMREAD_COLOR)
                            if zdj is None:
                                raise FileNotFoundError("Nie udało się wczytać image.jpeg")
                            manual_calibration(zdj)
                        elif wybor == 3:
                            cls()
                            if not os.path.isfile(CALIBRATION_FILE):
                                input("Brak zapisanej homografii — wykonaj najpierw kalibrację homografii (opcja kalibracji kamery top), zanim uruchomisz test kolorów. Naciśnij Enter aby wrócić do menu...")
                            else:
                                print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
                                wybor = str(readchar.readchar())
                                if wybor == str("t"):
                                    stream_gcode_list(ser, home)
                                    stream_gcode_list(ser, gcode_zdj)
                                input("Uwaga, program za chwile wykona zdjęcie. Naciśnij enter gdy gotowy")
                                capture = capture_write_unified()
                                zdj = cv2.imread("image.jpeg", cv2.IMREAD_COLOR)
                                if zdj is None:
                                    raise FileNotFoundError("Nie udało się wczytać image.jpeg")

                                h, w = zdj.shape[:2]
                                fig, ax = plt.subplots(figsize=(8, 6))
                                ax.imshow(cv2.cvtColor(zdj, cv2.COLOR_BGR2RGB))
                                ax.set_title("Podgląd — zamknij okno aby kontynuować")
                                ax.axis('off')
                                plt.show()
                                #detect_copper_corner(zdj)
                                xa, ya = detect_copper_origin(zdj, 5, 5)
                                print(xa)
                                print(ya)
                                input("A")
                            cls()
                        elif wybor == 4:
                            cls()
                            while True:
                                print("1. Podaj orientacje kamery \n2. Określ orientacje kamery \n3. Wyjście")
                                wybor = int(readchar.readchar())
                                if wybor == 1:
                                    cfg = configparser.ConfigParser()
                                    cfg.read(CONFIG_FILE)
                                    if "Connection" not in cfg:
                                        cfg["Connection"] = {}
                                    cur = cfg["Connection"].get("camera_rotation_deg", "0")
                                    print(f"Aktualna rotacja kamery: {cur} stopni.")
                                    val = input("Podaj rotację kamery (0, 90, 180, 270) lub Enter aby zostawić: ").strip()
                                    if not val:
                                        print("Nie zmieniono.")
                                    try:
                                        v = int(val) % 360
                                        if v not in (0,90,180,270):
                                            print("Zalecane wartości: 0,90,180,270. Zaokrąglam do najbliższej z nich.")
                                            # znajdź najbliższą z [0,90,180,270]
                                            cand = min((0,90,180,270), key=lambda x: abs(x - v))
                                            v = cand
                                        cfg["Connection"]["camera_rotation_deg"] = str(v)
                                        with open(CONFIG_FILE, "w") as f:
                                            cfg.write(f)
                                        print(f"Zapisano camera_rotation_deg = {v} w {CONFIG_FILE} (sekcja [Connection]).")
                                    except Exception as e:
                                        print("Błąd:", e)
                                elif wybor == 2:
                                    print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
                                    wybor = str(readchar.readchar())
                                    if wybor == str("t"):
                                        stream_gcode_list(ser, home)
                                        stream_gcode_list(ser, gcode_zdj)
                                    capture = capture_write_unified()
                                    # Kamera 'top' to źródło sieciowe (camera_ip) — jego backend (FFMPEG/GStreamer)
                                    # potrafi zostawić wątki dekodujące jeszcze chwilę po cap.release(), co koliduje
                                    # z pętlą zdarzeń okien OpenCV i powoduje, że waitKey wraca natychmiast.
                                    # Krótka pauza daje im czas się w pełni zamknąć przed otwarciem okien podglądu.
                                    time.sleep(0.5)
                                    zdj = cv2.imread("image.jpeg", cv2.IMREAD_COLOR)
                                    if zdj is None:
                                        raise FileNotFoundError("Nie udało się wczytać image.jpeg")
                                    variants = [(0, zdj),
                                                (90, rotate_image(zdj, 90)),
                                                (180, rotate_image(zdj, 180)),
                                                (270, rotate_image(zdj, 270))]
                                    # pokazuj kolejno i daj wybór przez klawisz
                                    print("Pokażę 4 warianty (0,90,180,270) w jednym oknie.")
                                    choice = _show_rotation_grid_and_choose(variants)
                                    try:
                                        choice = int(choice) % 360
                                        if choice not in (0,90,180,270):
                                            print("Wybrana nieprawidłowa, ustawiam 0.")
                                            choice = 0
                                    except:
                                        choice = 0
                                elif wybor == 3:
                                    break
                        elif wybor == 5:
                            break
                elif wybor == 2:
                    cls()
                    while True:    
                        print("1. Orientacja kamery\n2. Kalibracja px->mm\n3. Kalibracja offsetu głowicy PnP\n4. Wyjście")
                        wybor = int(readchar.readchar())
                        if wybor == 1:
                            cls()
                            print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
                            wybor = str(readchar.readchar())
                            if wybor == str("t"):
                                stream_gcode_list(ser, home)
                                stream_gcode_list(ser, gcode_zdj_up)
                            capture = capture_write_unified(camera='up')
                            zdj = cv2.imread("image.jpeg", cv2.IMREAD_COLOR)
                            if zdj is None:
                                raise FileNotFoundError("Nie udało się wczytać image.jpeg")
                            variants = [(0, zdj),
                                        (90, rotate_image(zdj, 90)),
                                        (180, rotate_image(zdj, 180)),
                                        (270, rotate_image(zdj, 270))]
                            # pokazuj kolejno i daj wybór przez klawisz
                            print("Pokażę 4 warianty (0,90,180,270) w jednym oknie.")
                            choice = _show_rotation_grid_and_choose(variants)
                            try:
                                choice = int(choice) % 360
                                if choice not in (0,90,180,270):
                                    print("Wybrana nieprawidłowa, ustawiam 0.")
                                    choice = 0
                            except:
                                choice = 0
                            cfg = configparser.ConfigParser()
                            cfg.read(CONFIG_FILE)
                            if "Connection" not in cfg:
                                cfg["Connection"] = {}
                            cur = cfg["Connection"].get("camera_up_rotation_deg", "0")
                            try:
                                cfg["Connection"]["camera_up_rotation_deg"] = str(choice)
                                with open(CONFIG_FILE, "w") as f:
                                    cfg.write(f)
                                print(f"Zapisano camera_rotation_deg = {choice} w {CONFIG_FILE} (sekcja [Connection]).")
                            except Exception as e:
                                print("Błąd:", e)                           
                        elif wybor == 2:
                            cls()
                            print("Czy chcesz ustawić maszyne na odpowiednią pozycje? (t/n): ")
                            wybor = str(readchar.readchar())
                            if wybor == str("t"):
                                stream_gcode_list(ser, home)
                                send_message("HOME")
                                stream_gcode_list(ser, gcode_zdj_up)
                            calibrate_up_via_tool(ser)
                        elif wybor == 3:
                            cls()
                            print("Czy chcesz ustawić maszynę na odpowiednią pozycje? (t/n): ")
                            wybor = str(readchar.readchar())
                            if wybor == str("t"):
                                stream_gcode_list(ser, home)
                                send_message("HOME")
                                z = get_z_from_status(ser)
                                x = 25 + z
                                send_message(f"GO:{-x}")
                                stream_gcode_list(ser, gcode_zdj_up)
                            else:
                                stream_gcode_list(ser, unlock)
                            input("naciśnij enter aby kontynuować")
                            ok = pnpp_calibrate_tool_offset(ser, tol_mm=0.01, max_iter=12, show_debug=True)
                            if ok:
                                input("Kalibracja PnP zakończona. Naciśnij Enter aby kontynuować...")
                                cls()
                            else:
                                input("Kalibracja PnP przerwana lub nieudanaw. Naciśnij Enter aby kontynuować...")
                                cls()
                        elif wybor == 4:
                            break
                elif wybor == 3:
                    break
    elif wybor == 4:
        cls()
        while True:
            print("1. Połączenie z maszyną\n2. Połączenie z kamerą\n3. Wyjście")
            wybor_pol = int(readchar.readchar())
            if wybor_pol == 1:
                cls()
                port = choose_serial_port()
                ser = serial.Serial(port, BAUD_RATE)
                send_wake_up(ser)
                print("Wybrano port:", port)
                polaczenie += 1
                saved_port = port    # zaktualizuj zmienną w pamięci, inaczej kolejny zapis nadpisze ją starą wartością
                save_connections(saved_port, saved_ip, saved_up_ip)    # zapisz nowy port
                time.sleep(3)
            elif wybor_pol == 2:
                cls()
                while True:
                    print("1. Kamera górna\n 2. Kamera w blacie\n3. Wyjście")
                    wybor = int(readchar.readchar())
                    if wybor == 1:
                        ip = input("Podaj adres IP kamery (włącznie z portem): ")
                        cls()
                        polaczenie += 1
                        saved_ip = ip    # zaktualizuj zmienną w pamięci
                        save_connections(saved_port, saved_ip, saved_up_ip)    # zapisz nowe IP
                    elif wybor == 2:
                        ip_up = input("Podaj adres IP kamery (włącznie z portem): ")
                        cls()
                        polaczenie += 1
                        saved_up_ip = ip_up    # zaktualizuj zmienną w pamięci
                        save_connections(saved_port, saved_ip, saved_up_ip)    # zapisz nowe IP
                    elif wybor == 3:
                        break
            elif wybor_pol == 3:
                break
            else:
                input("podano błędne dane")
    elif wybor == 5:
        menu_generate_component_holder_stl()
    elif wybor == 6:
        break

