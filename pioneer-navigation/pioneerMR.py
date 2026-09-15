#!/usr/bin/env python3

# Pioneer P3-DX / MobileSim 

from __future__ import annotations

import csv
import glob
import math
import os
from pathlib import Path
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np


try:
    from openpyxl import Workbook  # type: ignore
except Exception:
    Workbook = None

# Optional deps
try:
    import cv2  # type: ignore
except Exception:
    cv2 = None

try:
    import serial  # type: ignore
    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        list_ports = None
except Exception:
    serial = None
    list_ports = None


# ============================================================
# CONFIG
# ============================================================

DEFAULT_SERIAL_PORT = os.environ.get("P3DX_SERIAL", "COM3" if os.name == "nt" else "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("P3DX_BAUD", "9600"))

MOBILE_SIM_HOST = os.environ.get("P3DX_HOST", "127.0.0.1")
MOBILE_SIM_PORT = int(os.environ.get("P3DX_PORT", "8101"))

# Waypoints (mm) relative to start
NODES_REL_ALL: List[Tuple[float, float]] = [
    (0.0, 0.0),
    (2060.0, 0.0),
    (2060.0, -2530.0),
    (4060.0, -2530.0),
    (4060.0, -3790.0),
    (-1880.0, -3790.0),
]
NODES_ODOM: List[Tuple[float, float]] = NODES_REL_ALL[1:]
POINT_NAMES: List[str] = [f"P{i+1}" for i in range(len(NODES_ODOM))]

CONTROL_HZ = 20
WATCHDOG_SEC = 1.0

MAX_V_MM_S = 300
MAX_W_DEG_S = 60
P3DX_WHEEL_TRACK_MM = float(os.environ.get("P3DX_WHEEL_TRACK_MM", "330.0"))  # override if your robot differs

# Avoidance thresholds (mm)
AVOID_GAMMA_A_MM = 150.0

# Tracking thresholds + slower speeds
TRACK_SAFE_STOP_MM = 350.0        # stop if front too close in tracking
TRACK_REACH_MM = 320.0            # "mission complete" distance (front sonar)
TRACK_REACH_HOLD_SEC = 0.7        # must remain close for this time to declare complete

TRACK_V_MM_S = 120.0              # slower forward
TRACK_W_DEG_S = 30.0              # slower turn
TRACK_TURN_FWD_MM_S = 25.0        # tiny forward while turning
TRACK_DEADBAND_PX = 30

# Avoidance timed primitives
DECOLLIDE_BACK_MM_S = 220.0
DECOLLIDE_TURN_DEG_S = 45.0
DECOLLIDE_BACK_SEC = 2.0
DECOLLIDE_TURN_SEC = 2.0
TRAP_TURN_SEC = 2.0

# Camera processing
BLOB_MIN_AREA = 250
HSV_H_TOL = 12
HSV_S_TOL = 80
HSV_V_TOL = 80
HSV_S_MIN = 40
HSV_V_MIN = 40


# ============================================================
# Helpers
# ============================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_deg_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def deg360(th: float) -> float:
    return th - 360.0 * math.floor(th / 360.0)


def wrap_rad_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def angle_mean_deg(angles_deg: np.ndarray, weights: np.ndarray) -> float:
    if len(angles_deg) == 0:
        return 0.0
    ang = np.deg2rad(angles_deg)
    s = float(np.sum(weights * np.sin(ang)))
    c = float(np.sum(weights * np.cos(ang)))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return 0.0
    return deg360(math.degrees(math.atan2(s, c)))


def list_serial_ports_manual() -> List[str]:
    """
    Manual port list for Real Robot mode.
    Uses pyserial list_ports if available (best for Windows).
    """
    ports: List[str] = []
    if list_ports is not None:
        try:
            for p in list_ports.comports():
                if p.device:
                    ports.append(p.device)
        except Exception:
            pass

    # Fallback (still manual list, NOT auto-trying them)
    if not ports:
        if os.name == "nt":
            ports = [f"COM{i}" for i in range(1, 21)]
        else:
            for pat in ("/dev/serial/by-id/*", "/dev/serial/by-path/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
                ports += sorted(glob.glob(pat))

    # unique
    seen = set()
    out = []
    for p in ports:
        if p not in seen:
            out.append(p)
            seen.add(p)
    return out


# ============================================================
# P2OS / ARIA protocol basics
# ============================================================

HEADER = bytes([0xFA, 0xFB])

SYNC0 = bytes([0xFA, 0xFB, 0x03, 0x00, 0x00, 0x00])
SYNC1 = bytes([0xFA, 0xFB, 0x03, 0x01, 0x00, 0x01])
SYNC2 = bytes([0xFA, 0xFB, 0x03, 0x02, 0x00, 0x02])

CMD_PULSE = 0
CMD_OPEN = 1
CMD_ENABLE = 4
CMD_VEL = 11
CMD_RVEL = 21
CMD_SONAR = 28
CMD_STOP = 29

ARG_POS_INT = 0x3B
ARG_NEG_INT = 0x1B


def calc_checksum(packet: bytes) -> int:
    c = 0
    i = 3
    n = packet[2] - 2
    while n > 1:
        c += (packet[i] << 8) | packet[i + 1]
        c &= 0xFFFF
        n -= 2
        i += 2
    if n > 0:
        c ^= packet[i]
    return c & 0xFFFF


# ============================================================
# SIP parsing
# ============================================================

@dataclass
class RobotState:
    x_mm: float = 0.0
    y_mm: float = 0.0
    th_deg_wrap: float = 0.0
    th_deg_360: float = 0.0

    left_vel_mm_s: float = 0.0
    right_vel_mm_s: float = 0.0

    v_mm_s: float = 0.0
    w_deg_s: float = 0.0

    sonars_mm: List[float] = None

    def __post_init__(self):
        if self.sonars_mm is None:
            self.sonars_mm = [0.0] * 16


def _i16_le(buf: bytes, off: int) -> int:
    return struct.unpack_from("<h", buf, off)[0]


def parse_sip(packet: bytes, prev_sonars: List[float]) -> Optional[Tuple[float, float, float, float, float, List[float]]]:
    if len(packet) < 6 or packet[:2] != HEADER:
        return None
    bytecount = packet[2]
    if len(packet) < 3 + bytecount:
        return None

    rx_chk = int.from_bytes(packet[3 + bytecount - 2:3 + bytecount], "big")
    calc_chk = calc_checksum(packet[:3 + bytecount - 2] + b"\x00\x00")
    if rx_chk != calc_chk:
        return None

    payload = packet[3:3 + bytecount]
    if len(payload) < 5:
        return None

    data = payload[1:-2]
    if len(data) < 2 * 5 + 1 + 2 * 3 + 1 + 1:
        return None

    off = 0
    x_raw = float(_i16_le(data, off)); off += 2
    y_raw = float(_i16_le(data, off)); off += 2
    th_raw = float(_i16_le(data, off)); off += 2
    lvel = float(_i16_le(data, off)); off += 2
    rvel = float(_i16_le(data, off)); off += 2

    _battery = data[off]; off += 1
    _stall = _i16_le(data, off); off += 2
    _control = _i16_le(data, off); off += 2
    _flags = _i16_le(data, off); off += 2
    _compass = data[off]; off += 1
    sonar_count = int(data[off]); off += 1

    sonars = prev_sonars[:] if prev_sonars else [0.0] * 16
    for _ in range(sonar_count):
        if off + 3 > len(data):
            break
        sn = int(data[off]); off += 1
        sr = float(_i16_le(data, off)); off += 2
        if 0 <= sn < len(sonars):
            sonars[sn] = sr

    return x_raw, y_raw, th_raw, lvel, rvel, sonars


# ============================================================
# Link layer (TCP + Serial explicit)
# ============================================================

class PioneerLink:
    def __init__(self):
        self.use_serial = False
        self.sock: Optional[socket.socket] = None
        self.ser = None

        self._rx_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._buf = bytearray()

        self._lock = threading.Lock()
        self._latest: Optional[RobotState] = None

        self._origin_set = False
        self._x0 = 0.0
        self._y0 = 0.0
        self._th0 = 0.0
        self._sonars = [0.0] * 16

        self._last_pulse = 0.0

        self.endpoint = ""
        self.transport = ""

    def _theta_raw_to_deg(self, th_raw: float) -> float:
        # Default is 4095 ticks/rev for both MobileSim and common Pioneer setups.
        # Override P3DX_THETA_TICKS_PER_REV if your hardware/firmware differs.
        ticks_per_rev = float(os.environ.get("P3DX_THETA_TICKS_PER_REV", "4095.0"))
        return float(th_raw) * 360.0 / ticks_per_rev

    def _write(self, b: bytes) -> None:
        try:
            if self.use_serial and self.ser is not None:
                self.ser.write(b)
            elif self.sock is not None:
                self.sock.sendall(b)
        except Exception as e:
            print(f"[WARN] write failed ({e}); disconnecting")
            self.close()

    def _read_some(self) -> bytes:
        if self.use_serial and self.ser is not None:
            return self.ser.read(4096)
        if self.sock is not None:
            try:
                data = self.sock.recv(4096)
                if data == b"":
                    raise ConnectionError("TCP closed")
                return data
            except socket.timeout:
                return b""
        return b""

    def _start_reader_and_wait_sip(self, timeout_s: float = 0.9) -> bool:
        self._stop_evt.clear()
        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_thread.start()

        self._origin_set = False
        with self._lock:
            self._latest = None
        self._last_pulse = time.time()

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.get_state() is not None:
                return True
            time.sleep(0.02)
        return False

    def connect_tcp(self, host: str, port: int) -> bool:
        self.close()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((host, port))
            self.sock.settimeout(0.05)
            self.use_serial = False
            self.transport = "tcp"
            self.endpoint = f"{host}:{port}"
            print(f"[OK] TCP connected: {self.endpoint}")
        except Exception as e:
            print(f"[ERR] TCP connect failed: {e}")
            self.sock = None
            return False

        if not self._handshake():
            self.close()
            return False

        if not self._start_reader_and_wait_sip():
            print("[ERR] TCP connected but no SIP received (MobileSim not streaming?)")
            self.close()
            return False

        return True

    def connect_serial(self, port: str, baud: int) -> bool:
        self.close()
        if serial is None:
            print("[ERR] pyserial not installed; cannot use Serial")
            return False
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.use_serial = True
            self.transport = "serial"
            self.endpoint = f"{port} @ {baud}"
            print(f"[OK] Serial connected: {self.endpoint}")
        except Exception as e:
            print(f"[ERR] Serial connect failed: {e}")
            self.ser = None
            return False

        if not self._handshake():
            self.close()
            return False

        if not self._start_reader_and_wait_sip():
            print("[ERR] Serial connected but no SIP received (wrong port/baud?)")
            self.close()
            return False

        return True

    def close(self) -> None:
        self._stop_evt.set()
        try:
            self.stop_motion()
        except Exception:
            pass

        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

        rx_thread = self._rx_thread
        if rx_thread is not None and rx_thread.is_alive() and rx_thread is not threading.current_thread():
            try:
                rx_thread.join(timeout=0.2)
            except Exception:
                pass
        self._rx_thread = None

        self.ser = None
        self.sock = None
        self.use_serial = False
        self._buf.clear()
        self._sonars = [0.0] * 16
        self._origin_set = False
        self._x0 = 0.0
        self._y0 = 0.0
        self._th0 = 0.0

        with self._lock:
            self._latest = None

    def is_connected(self) -> bool:
        return self.ser is not None or self.sock is not None

    def send_cmd(self, cmd: int, arg: Optional[int] = None) -> None:
        if arg is None:
            bytecount = 1 + 2
            pkt_wo = HEADER + bytes([bytecount]) + bytes([cmd])
            chk = calc_checksum(pkt_wo + b"\x00\x00").to_bytes(2, "big")
            self._write(pkt_wo + chk)
            return

        if arg < 0:
            argtype = ARG_NEG_INT
            aval = abs(int(arg))
        else:
            argtype = ARG_POS_INT
            aval = int(arg)

        arg_bytes = struct.pack("<H", aval & 0xFFFF)
        bytecount = 1 + 1 + 2 + 2
        pkt_wo = HEADER + bytes([bytecount]) + bytes([cmd, argtype]) + arg_bytes
        chk = calc_checksum(pkt_wo + b"\x00\x00").to_bytes(2, "big")
        self._write(pkt_wo + chk)

    def _handshake(self) -> bool:
        try:
            for p in (SYNC0, SYNC1, SYNC2):
                self._write(p)
                time.sleep(0.05)
                _ = self._read_some()

            self.send_cmd(CMD_OPEN)
            self.send_cmd(CMD_ENABLE, 1)
            self.send_cmd(CMD_SONAR, 1)
            self.send_cmd(CMD_PULSE)
            print("[OK] Handshake done (SYNC/OPEN/ENABLE/SONAR)")
            return True
        except Exception as e:
            print(f"[ERR] Handshake failed: {e}")
            return False

    def _reader_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                data = self._read_some()
                if data:
                    self._buf.extend(data)
                    self._consume_packets()
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"[WARN] reader stopped ({e}); disconnecting")
                self.close()
                break

    def _consume_packets(self) -> None:
        while True:
            if len(self._buf) < 4:
                return

            h = self._buf.find(HEADER)
            if h < 0:
                self._buf.clear()
                return
            if h > 0:
                del self._buf[:h]
                if len(self._buf) < 4:
                    return

            bytecount = self._buf[2]
            total_len = 3 + bytecount
            if len(self._buf) < total_len:
                return

            pkt = bytes(self._buf[:total_len])
            del self._buf[:total_len]

            ptype = pkt[3] if len(pkt) > 3 else 0
            if not (0x30 <= ptype <= 0x33):
                continue

            parsed = parse_sip(pkt, self._sonars)
            if parsed is None:
                continue

            x_raw, y_raw, th_raw, lvel, rvel, sonars = parsed
            th_deg_abs = self._theta_raw_to_deg(th_raw)
            self._sonars = sonars[:]

            if not self._origin_set:
                self._origin_set = True
                self._x0, self._y0, self._th0 = x_raw, y_raw, th_deg_abs

            x_rel = x_raw - self._x0
            y_rel = y_raw - self._y0
            th_rel = th_deg_abs - self._th0

            st = RobotState(
                x_mm=x_rel,
                y_mm=y_rel,
                th_deg_wrap=wrap_deg_180(th_rel),
                th_deg_360=deg360(th_rel),
                left_vel_mm_s=lvel,
                right_vel_mm_s=rvel,
                v_mm_s=(lvel + rvel) / 2.0,
                w_deg_s=((rvel - lvel) / P3DX_WHEEL_TRACK_MM) * 180.0 / math.pi,
                sonars_mm=sonars[:],
            )
            with self._lock:
                self._latest = st

    def get_state(self) -> Optional[RobotState]:
        with self._lock:
            if self._latest is None:
                return None
            s = self._latest
            return RobotState(
                x_mm=s.x_mm,
                y_mm=s.y_mm,
                th_deg_wrap=s.th_deg_wrap,
                th_deg_360=s.th_deg_360,
                left_vel_mm_s=s.left_vel_mm_s,
                right_vel_mm_s=s.right_vel_mm_s,
                v_mm_s=s.v_mm_s,
                w_deg_s=s.w_deg_s,
                sonars_mm=s.sonars_mm[:],
            )

    def drive(self, v_mm_s: float, w_deg_s: float) -> None:
        v = int(clamp(v_mm_s, -MAX_V_MM_S, MAX_V_MM_S))
        w = int(clamp(w_deg_s, -MAX_W_DEG_S, MAX_W_DEG_S))
        self.send_cmd(CMD_VEL, v)
        self.send_cmd(CMD_RVEL, w)

    def stop_motion(self) -> None:
        self.send_cmd(CMD_STOP)
        self.send_cmd(CMD_VEL, 0)
        self.send_cmd(CMD_RVEL, 0)

    def pulse_if_needed(self) -> None:
        now = time.time()
        if now - self._last_pulse >= WATCHDOG_SEC:
            try:
                self.send_cmd(CMD_PULSE)
            except Exception:
                pass
            self._last_pulse = now


# ============================================================
# Camera tracker (ROI HSV learned once)
# ============================================================

@dataclass
class Blob:
    found: bool
    x: int = 0
    y: int = 0
    img_w: int = 0
    img_h: int = 0
    area: float = 0.0


class CameraTracker:
    def __init__(self):
        self.enabled = False
        self._cap = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._latest_blob = Blob(False)
        self._roi: Optional[Tuple[int, int, int, int]] = None
        self._hsv_center: Optional[Tuple[float, float, float]] = None

    def has_roi(self) -> bool:
        return self._roi is not None

    def start(self) -> bool:
        if cv2 is None:
            print("[WARN] OpenCV not installed; camera disabled")
            return False

        if self._thread is not None and self._thread.is_alive() and self.enabled:
            return True

        if self._cap is not None or (self._thread is not None and self._thread.is_alive()):
            self.stop()

        cap = None
        for idx in (0, 1, 2, 3):
            c = cv2.VideoCapture(idx)
            if c is not None and c.isOpened():
                cap = c
                break
            if c is not None:
                c.release()

        if cap is None:
            print("[WARN] Camera not available (busy?)")
            return False

        self._cap = cap
        self.enabled = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self.enabled = False
        self._stop.set()

        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            try:
                thread.join(timeout=0.5)
            except Exception:
                pass
        self._thread = None

        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        self._cap = None
        self._roi = None
        self._hsv_center = None
        with self._lock:
            self._latest_blob = Blob(False)
        try:
            if cv2 is not None:
                cv2.destroyWindow("P3DX Camera")
        except Exception:
            pass

    def get_blob(self) -> Blob:
        with self._lock:
            return self._latest_blob

    def _make_mask(self, hsv_img, center_hsv: Tuple[float, float, float]):
        assert cv2 is not None
        h0, s0, v0 = center_hsv

        s_lo = max(HSV_S_MIN, int(s0 - HSV_S_TOL))
        v_lo = max(HSV_V_MIN, int(v0 - HSV_V_TOL))
        s_hi = min(255, int(s0 + HSV_S_TOL))
        v_hi = min(255, int(v0 + HSV_V_TOL))

        h_low = int(h0 - HSV_H_TOL)
        h_high = int(h0 + HSV_H_TOL)

        if h_low < 0:
            mask1 = cv2.inRange(hsv_img, (0, s_lo, v_lo), (h_high, s_hi, v_hi))
            mask2 = cv2.inRange(hsv_img, (179 + h_low, s_lo, v_lo), (179, s_hi, v_hi))
            return cv2.bitwise_or(mask1, mask2)
        if h_high > 179:
            mask1 = cv2.inRange(hsv_img, (h_low, s_lo, v_lo), (179, s_hi, v_hi))
            mask2 = cv2.inRange(hsv_img, (0, s_lo, v_lo), (h_high - 179, s_hi, v_hi))
            return cv2.bitwise_or(mask1, mask2)

        return cv2.inRange(hsv_img, (h_low, s_lo, v_lo), (h_high, s_hi, v_hi))

    def _loop(self) -> None:
        assert cv2 is not None
        win = "P3DX Camera"
        cv2.namedWindow(win)

        selecting = {"on": False, "x0": 0, "y0": 0, "x1": 0, "y1": 0}

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                selecting["on"] = True
                selecting["x0"], selecting["y0"] = x, y
                selecting["x1"], selecting["y1"] = x, y
            elif event == cv2.EVENT_MOUSEMOVE and selecting["on"]:
                selecting["x1"], selecting["y1"] = x, y
            elif event == cv2.EVENT_LBUTTONUP:
                selecting["on"] = False
                selecting["x1"], selecting["y1"] = x, y
                x0, y0, x1, y1 = selecting["x0"], selecting["y0"], selecting["x1"], selecting["y1"]
                x0, x1 = sorted((x0, x1))
                y0, y1 = sorted((y0, y1))
                if abs(x1 - x0) > 10 and abs(y1 - y0) > 10:
                    self._roi = (x0, y0, x1, y1)
                    self._hsv_center = None  # learn once

        cv2.setMouseCallback(win, on_mouse)

        while not self._stop.is_set():
            if self._cap is None:
                break

            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            h, w = frame.shape[:2]
            blob = Blob(False, img_w=w, img_h=h)

            # Learn HSV ONCE after ROI is set
            if self._roi is not None and self._hsv_center is None:
                x0, y0, x1, y1 = self._roi
                roi = frame[y0:y1, x0:x1]
                if roi.size > 0:
                    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mean = hsv_roi.reshape(-1, 3).mean(axis=0)
                    self._hsv_center = (float(mean[0]), float(mean[1]), float(mean[2]))

            if self._hsv_center is not None:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = self._make_mask(hsv, self._hsv_center)

                mask = cv2.medianBlur(mask, 5)
                kernel = np.ones((5, 5), dtype=np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    area = float(cv2.contourArea(c))
                    if area >= BLOB_MIN_AREA:
                        M = cv2.moments(c)
                        if M["m00"] > 1e-6:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            blob = Blob(True, cx, cy, w, h, area)
                            x, y, bw, bh = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            # ROI box
            if selecting["on"]:
                cv2.rectangle(frame, (selecting["x0"], selecting["y0"]), (selecting["x1"], selecting["y1"]), (255, 0, 0), 1)
            if self._roi is not None:
                x0, y0, x1r, y1r = self._roi
                cv2.rectangle(frame, (x0, y0), (x1r, y1r), (255, 0, 0), 1)

            # instructions
            status = "Drag ROI to learn target | R=reset ROI/HSV | ESC=close"
            if self._roi is None:
                status += " | ROI NOT SET"
            elif self._hsv_center is None:
                status += " | learning..."
            else:
                status += " | tracking ready"

            cv2.putText(frame, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

            with self._lock:
                self._latest_blob = blob

            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            if k in (ord("r"), ord("R")):
                self._roi = None
                self._hsv_center = None

        self.enabled = False
        self._stop.set()
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        self._cap = None
        self._thread = None
        with self._lock:
            self._latest_blob = Blob(False)
        try:
            cv2.destroyWindow(win)
        except Exception:
            pass


# ============================================================
# Smoother filters (optional belief layer)
# ============================================================

class EKFLocalizer:
    """
    Lightweight pose EKF smoother.

    This is intentionally a *smoother*, not true localisation:
    - prediction uses v/w from wheel odometry
    - correction uses the robot-reported pose, but more weakly and less often
    - optional waypoint anchoring can gently pull the belief near a goal

    Compared with the previous version, this one is tuned to show a visible
    smoothing effect instead of collapsing almost exactly onto the raw pose.
    """

    def __init__(self):
        self.initialized = False
        self.x = np.zeros(3, dtype=float)  # x_mm, y_mm, th_rad
        self.P = np.diag([220.0**2, 220.0**2, math.radians(20.0)**2]).astype(float)

        # Trust raw pose LESS so the belief can deviate and smooth.
        self.meas_sigma_xy = 180.0
        self.meas_sigma_th_deg = 10.0

        # Trust the motion model MORE.
        self.proc_sigma_xy = 8.0
        self.proc_sigma_th_deg = 0.8

        # Do not correct from raw pose every tick.
        self.pose_update_stride = 4
        self._pose_update_count = 0

        # Reject obviously bad raw jumps.
        self.max_innov_xy_mm = 600.0
        self.max_innov_th_deg = 35.0

        # Optional waypoint pseudo-measurement.
        self.goal_gate_mm = 220.0
        self.goal_sigma_xy = 45.0
        self.goal_enabled = True

    def reset(self) -> None:
        self.initialized = False
        self.x[:] = 0.0
        self.P = np.diag([220.0**2, 220.0**2, math.radians(20.0)**2]).astype(float)
        self._pose_update_count = 0

    def _init_from_state(self, st: RobotState) -> None:
        self.x = np.array([st.x_mm, st.y_mm, math.radians(st.th_deg_360)], dtype=float)
        self.P = np.diag([120.0**2, 120.0**2, math.radians(10.0)**2]).astype(float)
        self.initialized = True
        self._pose_update_count = 0

    def predict(self, st: RobotState, dt: float) -> None:
        if not self.initialized:
            self._init_from_state(st)
            return

        dt = max(1e-3, min(0.25, float(dt)))
        v = float(st.v_mm_s)
        w_rad = math.radians(float(st.w_deg_s))
        th = float(self.x[2])

        # Slightly better differential-drive integration than plain Euler.
        if abs(w_rad) < 1e-6:
            dx = v * dt * math.cos(th)
            dy = v * dt * math.sin(th)
        else:
            th2 = th + w_rad * dt
            dx = (v / w_rad) * (math.sin(th2) - math.sin(th))
            dy = -(v / w_rad) * (math.cos(th2) - math.cos(th))

        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = wrap_rad_pi(self.x[2] + w_rad * dt)

        F = np.array([
            [1.0, 0.0, -v * dt * math.sin(th)],
            [0.0, 1.0,  v * dt * math.cos(th)],
            [0.0, 0.0, 1.0],
        ], dtype=float)

        q_xy = (self.proc_sigma_xy + 0.012 * abs(v)) ** 2
        q_th = math.radians(self.proc_sigma_th_deg + 0.008 * abs(st.w_deg_s)) ** 2
        Q = np.diag([q_xy, q_xy, q_th]).astype(float)
        self.P = F @ self.P @ F.T + Q

    def update_pose(self, st: RobotState) -> None:
        if not self.initialized:
            self._init_from_state(st)
            return

        z = np.array([st.x_mm, st.y_mm, math.radians(st.th_deg_360)], dtype=float)
        H = np.eye(3, dtype=float)
        R = np.diag([
            self.meas_sigma_xy ** 2,
            self.meas_sigma_xy ** 2,
            math.radians(self.meas_sigma_th_deg) ** 2,
        ]).astype(float)

        y = z - self.x
        y[2] = wrap_rad_pi(y[2])

        # Innovation gating: ignore raw pose jumps that are too large.
        if math.hypot(float(y[0]), float(y[1])) > self.max_innov_xy_mm:
            return
        if abs(math.degrees(float(y[2]))) > self.max_innov_th_deg:
            return

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = wrap_rad_pi(self.x[2])

        # Joseph-form covariance update for numerical stability.
        I = np.eye(3, dtype=float)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    def update_waypoint(self, goal: Optional[Tuple[float, float]]) -> None:
        if (not self.initialized) or (not self.goal_enabled) or (goal is None):
            return

        gx, gy = goal
        d = math.hypot(gx - self.x[0], gy - self.x[1])
        if d > self.goal_gate_mm:
            return

        z = np.array([gx, gy], dtype=float)
        H = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float)
        R = np.diag([self.goal_sigma_xy ** 2, self.goal_sigma_xy ** 2]).astype(float)

        y = z - (H @ self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = wrap_rad_pi(self.x[2])

        I = np.eye(3, dtype=float)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    def step(self, st: RobotState, dt: float, goal: Optional[Tuple[float, float]]) -> RobotState:
        self.predict(st, dt)

        self._pose_update_count += 1
        if self._pose_update_count >= self.pose_update_stride:
            self.update_pose(st)
            self._pose_update_count = 0

        self.update_waypoint(goal)
        return self.as_state(st)

    def as_state(self, raw: RobotState) -> RobotState:
        th_deg_360 = deg360(math.degrees(float(self.x[2])))
        return RobotState(
            x_mm=float(self.x[0]),
            y_mm=float(self.x[1]),
            th_deg_wrap=wrap_deg_180(th_deg_360),
            th_deg_360=th_deg_360,
            left_vel_mm_s=raw.left_vel_mm_s,
            right_vel_mm_s=raw.right_vel_mm_s,
            v_mm_s=raw.v_mm_s,
            w_deg_s=raw.w_deg_s,
            sonars_mm=raw.sonars_mm[:],
        )


class ParticleFilterLocalizer:
    """Simple particle filter pose smoother with optional waypoint anchoring."""

    def __init__(self, n_particles: int = 250):
        self.n = int(max(80, n_particles))
        self.initialized = False
        self.particles = np.zeros((self.n, 3), dtype=float)  # x_mm, y_mm, th_deg360
        self.weights = np.full(self.n, 1.0 / self.n, dtype=float)

        self.init_sigma_xy = 90.0
        self.init_sigma_th_deg = 8.0
        self.proc_sigma_xy = 14.0
        self.proc_sigma_th_deg = 2.0
        self.meas_sigma_xy = 45.0
        self.meas_sigma_th_deg = 4.0

        self.goal_gate_mm = 240.0
        self.goal_sigma_xy = 55.0
        self.goal_enabled = True
        self._rng = np.random.default_rng()

    def reset(self) -> None:
        self.initialized = False
        self.particles = np.zeros((self.n, 3), dtype=float)
        self.weights = np.full(self.n, 1.0 / self.n, dtype=float)

    def _init_from_state(self, st: RobotState) -> None:
        self.particles[:, 0] = st.x_mm + self._rng.normal(0.0, self.init_sigma_xy, self.n)
        self.particles[:, 1] = st.y_mm + self._rng.normal(0.0, self.init_sigma_xy, self.n)
        self.particles[:, 2] = np.mod(st.th_deg_360 + self._rng.normal(0.0, self.init_sigma_th_deg, self.n), 360.0)
        self.weights.fill(1.0 / self.n)
        self.initialized = True

    def predict(self, st: RobotState, dt: float) -> None:
        if not self.initialized:
            self._init_from_state(st)
            return

        dt = max(1e-3, min(0.25, float(dt)))
        v = float(st.v_mm_s)
        w = float(st.w_deg_s)

        v_noise = self._rng.normal(0.0, self.proc_sigma_xy + 0.03 * abs(v), self.n)
        w_noise = self._rng.normal(0.0, self.proc_sigma_th_deg + 0.02 * abs(w), self.n)

        th = np.deg2rad(self.particles[:, 2])
        v_eff = v + v_noise
        w_eff = w + w_noise
        self.particles[:, 0] += v_eff * dt * np.cos(th)
        self.particles[:, 1] += v_eff * dt * np.sin(th)
        self.particles[:, 2] = np.mod(self.particles[:, 2] + w_eff * dt, 360.0)

    def update(self, st: RobotState, goal: Optional[Tuple[float, float]]) -> None:
        if not self.initialized:
            self._init_from_state(st)
            return

        dx = self.particles[:, 0] - st.x_mm
        dy = self.particles[:, 1] - st.y_mm
        dth = np.array([wrap_deg_180(float(a - st.th_deg_360)) for a in self.particles[:, 2]], dtype=float)

        logw = -0.5 * (dx / self.meas_sigma_xy) ** 2
        logw += -0.5 * (dy / self.meas_sigma_xy) ** 2
        logw += -0.5 * (dth / self.meas_sigma_th_deg) ** 2

        if self.goal_enabled and goal is not None:
            gx, gy = goal
            raw_goal_dist = math.hypot(st.x_mm - gx, st.y_mm - gy)
            if raw_goal_dist <= self.goal_gate_mm:
                gdist = np.hypot(self.particles[:, 0] - gx, self.particles[:, 1] - gy)
                logw += -0.5 * (gdist / self.goal_sigma_xy) ** 2

        logw -= float(np.max(logw))
        w = np.exp(logw) + 1e-300
        self.weights = w / np.sum(w)

        neff = 1.0 / float(np.sum(self.weights ** 2))
        if neff < 0.5 * self.n:
            self._resample()

    def _resample(self) -> None:
        cdf = np.cumsum(self.weights)
        start = float(self._rng.uniform(0.0, 1.0 / self.n))
        points = start + np.arange(self.n, dtype=float) / self.n
        idx = np.searchsorted(cdf, points)
        idx = np.clip(idx, 0, self.n - 1)
        self.particles = self.particles[idx].copy()
        self.weights.fill(1.0 / self.n)

    def estimate(self) -> Tuple[float, float, float]:
        x = float(np.sum(self.weights * self.particles[:, 0]))
        y = float(np.sum(self.weights * self.particles[:, 1]))
        th = angle_mean_deg(self.particles[:, 2], self.weights)
        return x, y, th

    def step(self, st: RobotState, dt: float, goal: Optional[Tuple[float, float]]) -> RobotState:
        self.predict(st, dt)
        self.update(st, goal)
        return self.as_state(st)

    def as_state(self, raw: RobotState) -> RobotState:
        x, y, th_deg_360 = self.estimate()
        return RobotState(
            x_mm=x,
            y_mm=y,
            th_deg_wrap=wrap_deg_180(th_deg_360),
            th_deg_360=th_deg_360,
            left_vel_mm_s=raw.left_vel_mm_s,
            right_vel_mm_s=raw.right_vel_mm_s,
            v_mm_s=raw.v_mm_s,
            w_deg_s=raw.w_deg_s,
            sonars_mm=raw.sonars_mm[:],
        )

# ============================================================
# Behaviours
# ============================================================

class OdomAlignMoveNavigator:
    """
    Two-stage odometry controller (ASSIGNMENT requirement):

      - ALIGN (turn in place): v=0,  w = kp_rot * err  until |err| <= gamma_theta
      - MOVE  (drive + steer): v=v_cruise, w = kp_steer * err  (bounded)

    Tunable from GUI:
      - gamma_theta (deg)
      - gamma_dist (mm)
    """

    def __init__(self, nodes: List[Tuple[float, float]]):
        self.nodes = nodes[:]
        self.i = 0

        # Tunables (GUI)
        self.gamma_theta = 20.0
        self.gamma_dist = 120.0
        self.gamma_dist_final = 120.0

        # Gains / limits (safe defaults)
        self.kp_rot = 1.3
        self.kp_steer = 1.2
        self.v_cruise = 220.0
        self.w_max = 45.0
        self.min_w = 10.0

        # Hysteresis to prevent stop/start jitter near gamma_theta
        self.hysteresis_deg = 5.0
        self._stage = "ALIGN"  # ALIGN or MOVE

    def reset(self):
        self.i = 0
        self._stage = "ALIGN"

    def done(self) -> bool:
        return self.i >= len(self.nodes)

    def current_goal(self) -> Optional[Tuple[float, float]]:
        return None if self.done() else self.nodes[self.i]

    @staticmethod
    def _bearing_deg360(st: RobotState, gx: float, gy: float) -> float:
        a = math.degrees(math.atan2(gy - st.y_mm, gx - st.x_mm))
        if a < 0:
            a += 360.0
        return a

    def step(self, st: RobotState) -> Tuple[float, float, str]:
        if self.done():
            return 0.0, 0.0, "ODOM: DONE"

        gx, gy = self.nodes[self.i]
        dist = math.hypot(gx - st.x_mm, gy - st.y_mm)

        is_last = (self.i == len(self.nodes) - 1)
        hit = self.gamma_dist_final if is_last else self.gamma_dist

        if dist <= hit:
            self.i += 1
            self._stage = "ALIGN"
            if self.done():
                return 0.0, 0.0, "ODOM: FINISH"
            return 0.0, 0.0, f"ODOM: ARRIVED -> next ({self.i}/{len(self.nodes)})"

        target = self._bearing_deg360(st, gx, gy)
        err = wrap_deg_180(target - st.th_deg_360)

        # stage switching with hysteresis
        if self._stage == "MOVE":
            if abs(err) > (self.gamma_theta + self.hysteresis_deg):
                self._stage = "ALIGN"
        else:
            if abs(err) <= self.gamma_theta:
                self._stage = "MOVE"

        if self._stage == "ALIGN":
            w = clamp(self.kp_rot * err, -self.w_max, self.w_max)
            if 0 < abs(w) < self.min_w:
                w = self.min_w if w > 0 else -self.min_w
            return 0.0, w, f"ODOM: ALIGN err={err:.1f} dist={dist:.0f}"

        # MOVE with steering (this is what makes it behave like pioneer2.py)
        w = clamp(self.kp_steer * err, -self.w_max, self.w_max)
        return self.v_cruise, w, f"ODOM: MOVE err={err:.1f} dist={dist:.0f}"


class JavaObstacleAvoidance:
    """Simple, deterministic avoidance to prevent 'random walk'.

    - No random turn direction
    - No backing / de-collide phases (these cause oscillation on real robots)
    - Uses a short hold time so sonar noise doesn't flicker commands

    Intended use: in TRACK mode, as an override when a SIDE obstacle is too close.
    (Front stopping is handled by the tracking controller using s3/s4.)
    """

    # Use SIDE sonars (avoid using 3/4 here because they are front in your setup)
    LEFT_IDXS = [1, 2]
    RIGHT_IDXS = [5, 6]

    def __init__(self, gamma_mm: float = AVOID_GAMMA_A_MM, w_deg_s: float = 35.0, hold_sec: float = 0.6):
        self.gamma = float(gamma_mm)
        self.w = float(w_deg_s)
        self.hold = float(hold_sec)

        self._active_until = 0.0
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._info = ""

    @staticmethod
    def _min_sonar(sonars: List[float], idxs: List[int]) -> float:
        vals = [sonars[i] for i in idxs if 0 <= i < len(sonars) and sonars[i] > 0]
        return float(min(vals)) if vals else 99999.0

    def reset(self) -> None:
        self._active_until = 0.0
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._info = ""

    def step(self, st: RobotState) -> Tuple[Optional[float], Optional[float], str]:
        now = time.time()

        # Hold the last avoidance command briefly (prevents jitter)
        if now < self._active_until:
            return self._cmd_v, self._cmd_w, self._info

        son = st.sonars_mm or []
        l_min = self._min_sonar(son, self.LEFT_IDXS)
        r_min = self._min_sonar(son, self.RIGHT_IDXS)

        if (l_min >= self.gamma) and (r_min >= self.gamma):
            return None, None, "AVOID: clear"

        # Both sides close -> stop briefly (safe)
        if (l_min < self.gamma) and (r_min < self.gamma):
            self._cmd_v, self._cmd_w = 0.0, 0.0
            self._info = f"AVOID: STOP both (L={l_min:.0f} R={r_min:.0f})"
            self._active_until = now + self.hold
            return self._cmd_v, self._cmd_w, self._info

        # Obstacle left -> turn RIGHT (negative w). Obstacle right -> turn LEFT (positive w).
        if l_min < r_min:
            self._cmd_v, self._cmd_w = 0.0, -self.w
            self._info = f"AVOID: turn RIGHT (L={l_min:.0f})"
        else:
            self._cmd_v, self._cmd_w = 0.0, +self.w
            self._info = f"AVOID: turn LEFT (R={r_min:.0f})"

        self._active_until = now + self.hold
        return self._cmd_v, self._cmd_w, self._info

class FrontEmergencyAvoid:
    SAFE_DISTANCE_MM = 300.0
    FRONT_SONARS = (3, 4)

    BACK_V_MM_S = -150.0
    BACK_SEC = 1.2

    PIVOT_SEC = 3.0
    KP = 1.5
    MAX_W = 40.0
    MIN_W = 15.0
    DONE_ERR_DEG = 5.0

    def __init__(self):
        self.phase = "IDLE"
        self.t_end = 0.0
        self.goal: Optional[Tuple[float, float]] = None
        self.last_front = 9999.0

    def reset(self):
        self.phase = "IDLE"
        self.t_end = 0.0
        self.goal = None
        self.last_front = 9999.0

    @staticmethod
    def _angle_to_goal_deg360(st: RobotState, gx: float, gy: float) -> float:
        deg = math.degrees(math.atan2(gy - st.y_mm, gx - st.x_mm))
        if deg < 0:
            deg += 360.0
        return deg

    def step(self, st: RobotState, goal: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float, str]]:
        if goal is None:
            self.reset()
            return None

        now = time.time()

        if self.phase == "BACK":
            if now < self.t_end:
                return self.BACK_V_MM_S, 0.0, f"ODOM_AVOID: BACK front={self.last_front:.0f}"
            self.phase = "PIVOT"
            self.t_end = now + self.PIVOT_SEC
            return 0.0, 0.0, "ODOM_AVOID: PIVOT"

        if self.phase == "PIVOT":
            gx, gy = self.goal if self.goal is not None else goal
            err = wrap_deg_180(self._angle_to_goal_deg360(st, gx, gy) - st.th_deg_360)
            if abs(err) <= self.DONE_ERR_DEG or now >= self.t_end:
                self.reset()
                return 0.0, 0.0, "ODOM_AVOID: DONE"
            w = clamp(err * self.KP, -self.MAX_W, self.MAX_W)
            if abs(w) < self.MIN_W:
                w = self.MIN_W if w > 0 else -self.MIN_W
            return 0.0, w, f"ODOM_AVOID: PIVOT err={err:.1f}"

        front_vals = [st.sonars_mm[i] for i in self.FRONT_SONARS if 0 <= i < len(st.sonars_mm)]
        front_vals = [v for v in front_vals if v > 0]
        min_front = min(front_vals) if front_vals else 9999.0

        if min_front < self.SAFE_DISTANCE_MM:
            self.last_front = min_front
            self.goal = goal
            self.phase = "BACK"
            self.t_end = now + self.BACK_SEC
            return 0.0, 0.0, f"ODOM_AVOID: STOP front={min_front:.0f}"

        return None


class StableZoneTracker:
    FRONT_IDXS = [3, 4]

    def __init__(self):
        self._last_zone = "MID"
        self._ema_x: Optional[float] = None
        self._reach_t0: Optional[float] = None

        # Tunables (overridden from GUI each tick)
        self.safe_stop_mm = float(TRACK_SAFE_STOP_MM)
        self.reach_mm = float(TRACK_REACH_MM)
        self.reach_hold_sec = float(TRACK_REACH_HOLD_SEC)
        self.deadband_px = float(TRACK_DEADBAND_PX)

    @staticmethod
    def _min_front(sonars: List[float]) -> float:
        vals = [sonars[i] for i in StableZoneTracker.FRONT_IDXS if 0 <= i < len(sonars) and sonars[i] > 0]
        return float(min(vals)) if vals else 99999.0

    def reset(self):
        self._last_zone = "MID"
        self._ema_x = None
        self._reach_t0 = None

    def step(self, st: RobotState, blob: Blob, roi_ready: bool) -> Tuple[float, float, str, bool]:
        """
        returns: v, w, info, done
        """
        fmin = self._min_front(st.sonars_mm or [])

        # Tracking must NOT move without ROI selection
        if not roi_ready:
            self.reset()
            return 0.0, 0.0, "TRACK: WAIT ROI (no movement)", False

        # Safety stop (too close)
        if fmin <= self.safe_stop_mm:
            # mission complete check uses smaller threshold + hold
            if fmin <= self.reach_mm:
                if self._reach_t0 is None:
                    self._reach_t0 = time.time()
                if time.time() - self._reach_t0 >= self.reach_hold_sec:
                    return 0.0, 0.0, f"TRACK: COMPLETE (frontMin={fmin:.0f})", True
            else:
                self._reach_t0 = None

            return 0.0, 0.0, f"TRACK: STOP (frontMin={fmin:.0f})", False

        # If blob lost => stop (no scanning)
        if (not blob.found) or (blob.img_w <= 0):
            self._ema_x = None
            self._last_zone = "MID"
            self._reach_t0 = None
            return 0.0, 0.0, "TRACK: waiting target (no blob -> stop)", False

        # Smooth x
        x = float(blob.x)
        w_img = float(blob.img_w)
        if self._ema_x is None:
            self._ema_x = x
        else:
            self._ema_x = 0.7 * self._ema_x + 0.3 * x
        x = self._ema_x

        left_b = w_img / 3.0
        right_b = 2.0 * w_img / 3.0
        h = float(self.deadband_px)

        # hysteresis zones
        if self._last_zone == "LEFT":
            if x > left_b + h:
                self._last_zone = "MID"
        elif self._last_zone == "RIGHT":
            if x < right_b - h:
                self._last_zone = "MID"
        else:
            if x < left_b - h:
                self._last_zone = "LEFT"
            elif x > right_b + h:
                self._last_zone = "RIGHT"

        self._reach_t0 = None  # only count hold when close

        if self._last_zone == "LEFT":
            return TRACK_TURN_FWD_MM_S, +TRACK_W_DEG_S, "TRACK: LEFT -> turn + (slow)", False
        if self._last_zone == "RIGHT":
            return TRACK_TURN_FWD_MM_S, -TRACK_W_DEG_S, "TRACK: RIGHT -> turn - (slow)", False
        return TRACK_V_MM_S, 0.0, "TRACK: MID -> approach (slow)", False


# ============================================================
# GUI Application
# ============================================================

class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Pioneer Control Panel (Python)")

        self.link = PioneerLink()
        self.cam = CameraTracker()

        self.odom = OdomAlignMoveNavigator(NODES_ODOM)
        self.avoid = JavaObstacleAvoidance()
        self.front_avoid = FrontEmergencyAvoid()
        self.track = StableZoneTracker()

        self.ekf = EKFLocalizer()
        self.pf = ParticleFilterLocalizer()
        self._last_ekf_state: Optional[RobotState] = None
        self._last_pf_state: Optional[RobotState] = None

        self.point_records: List[dict] = []
        self.last_saved_record: Optional[dict] = None

        self.mode = tk.StringVar(value="MANUAL")  # MANUAL / ODOM / TRACK
        self.loc_mode = tk.StringVar(value="RAW")  # RAW ODOM / EKF smoother / PF experimental
        self.goal_anchor_var = tk.BooleanVar(value=True)
        self.auto = tk.BooleanVar(value=False)

        # Connection selection
        self.conn_mode = tk.StringVar(value="Simulation (TCP)")  # default for Windows students
        self.host_var = tk.StringVar(value=MOBILE_SIM_HOST)
        self.port_var = tk.StringVar(value=str(MOBILE_SIM_PORT))
        self.baud_var = tk.StringVar(value=str(SERIAL_BAUD))
        self.serial_var = tk.StringVar(value=DEFAULT_SERIAL_PORT)

        self._manual_keys: set[str] = set()
        self._manual_pad: Optional[tk.Toplevel] = None
        self._tick_job: Optional[str] = None
        self._shutting_down = False

        self.track_done = False
        self._last_print = 0.0
        self._last_state_t = time.time()

        # -------- GUI tunables (requested) --------
        # ODOM
        self.gamma_theta_var = tk.DoubleVar(value=20.0)      # deg
        self.gamma_dist_var = tk.DoubleVar(value=120.0)      # mm
        # TRACK
        self.track_safe_stop_var = tk.DoubleVar(value=TRACK_SAFE_STOP_MM)   # mm
        self.track_reach_var = tk.DoubleVar(value=TRACK_REACH_MM)           # mm
        self.track_deadband_var = tk.IntVar(value=TRACK_DEADBAND_PX)        # px
        # Localisation
        self.loc_goal_gate_var = tk.DoubleVar(value=220.0)     # mm
        self.loc_goal_sigma_var = tk.DoubleVar(value=45.0)     # mm
        self.pf_particles_var = tk.IntVar(value=250)
        # ------------------------------------------

        self._build_ui()
        self._bind_keys()
        self._tick()

    def _build_ui(self):
        f = ttk.Frame(self.root, padding=8)
        f.grid(row=0, column=0, sticky="nsew")

        connf = ttk.LabelFrame(f, text="Connection", padding=6)
        connf.grid(row=0, column=0, columnspan=3, sticky="ew")

        ttk.Label(connf, text="Mode:").grid(row=0, column=0, sticky="w")
        self.cmb_conn = ttk.Combobox(connf, textvariable=self.conn_mode, state="readonly",
                                    values=["Simulation (TCP)", "Real Robot (Serial)"])
        self.cmb_conn.grid(row=0, column=1, padx=4, sticky="ew")

        ttk.Label(connf, text="Host:").grid(row=1, column=0, sticky="w")
        ttk.Entry(connf, textvariable=self.host_var, width=14).grid(row=1, column=1, padx=4, sticky="w")
        ttk.Label(connf, text="Port:").grid(row=1, column=2, sticky="w")
        ttk.Entry(connf, textvariable=self.port_var, width=8).grid(row=1, column=3, padx=4, sticky="w")

        ttk.Label(connf, text="Serial:").grid(row=2, column=0, sticky="w")
        self.cmb_serial = ttk.Combobox(connf, textvariable=self.serial_var, width=18)
        self.cmb_serial.grid(row=2, column=1, padx=4, sticky="w")
        ttk.Button(connf, text="Scan Ports", command=self.on_scan_ports).grid(row=2, column=2, padx=4, sticky="w")
        ttk.Label(connf, text="Baud:").grid(row=2, column=3, sticky="w")
        ttk.Entry(connf, textvariable=self.baud_var, width=8).grid(row=2, column=4, padx=4, sticky="w")

        ttk.Button(connf, text="Connect", command=self.on_connect).grid(row=0, column=4, padx=6)
        ttk.Button(connf, text="Disconnect", command=self.on_disconnect).grid(row=0, column=5, padx=6)
        ttk.Button(connf, text="Help", command=self._show_help).grid(row=0, column=6, padx=6)

        self.lbl_conn = ttk.Label(connf, text="Status: DISCONNECTED")
        self.lbl_conn.grid(row=3, column=0, columnspan=7, sticky="w", pady=(6, 0))

        modef = ttk.LabelFrame(f, text="Mode", padding=6)
        modef.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Radiobutton(modef, text="Manual", value="MANUAL", variable=self.mode).grid(row=0, column=0, padx=4)
        ttk.Radiobutton(modef, text="Odometry", value="ODOM", variable=self.mode).grid(row=0, column=1, padx=4)
        ttk.Radiobutton(modef, text="Tracking", value="TRACK", variable=self.mode).grid(row=0, column=2, padx=4)
        ttk.Checkbutton(modef, text="Auto", variable=self.auto).grid(row=1, column=0, padx=4, pady=(6, 0), sticky="w")

        locf = ttk.LabelFrame(f, text="Smoother (not localisation)", padding=6)
        locf.grid(row=1, column=1, sticky="ew", pady=(6, 0))
        ttk.Radiobutton(locf, text="Raw Odom", value="RAW", variable=self.loc_mode, command=self.on_localization_mode_change).grid(row=0, column=0, padx=4, sticky="w")
        ttk.Radiobutton(locf, text="EKF Smoother", value="EKF", variable=self.loc_mode, command=self.on_localization_mode_change).grid(row=0, column=1, padx=4, sticky="w")
        ttk.Radiobutton(locf, text="PF Experimental", value="PF", variable=self.loc_mode, command=self.on_localization_mode_change).grid(row=0, column=2, padx=4, sticky="w")
        ttk.Checkbutton(locf, text="Waypoint anchor", variable=self.goal_anchor_var).grid(row=1, column=0, columnspan=2, padx=4, pady=(6, 0), sticky="w")
        ttk.Label(locf, text="Goal gate (mm):").grid(row=2, column=0, sticky="w")
        ttk.Entry(locf, textvariable=self.loc_goal_gate_var, width=10).grid(row=2, column=1, sticky="w")
        ttk.Label(locf, text="Goal σ (mm):").grid(row=3, column=0, sticky="w")
        ttk.Entry(locf, textvariable=self.loc_goal_sigma_var, width=10).grid(row=3, column=1, sticky="w")
        ttk.Label(locf, text="PF particles:").grid(row=4, column=0, sticky="w")
        ttk.Entry(locf, textvariable=self.pf_particles_var, width=10).grid(row=4, column=1, sticky="w")

        actf = ttk.LabelFrame(f, text="Actions", padding=6)
        actf.grid(row=1, column=2, sticky="ew", pady=(6, 0))
        ttk.Button(actf, text="Open Manual Pad", command=self.open_manual_pad).grid(row=0, column=0, padx=4)
        ttk.Button(actf, text="Reset Nodes", command=self.on_reset_nodes).grid(row=0, column=1, padx=4)
        ttk.Button(actf, text="Reset Smoother", command=self.reset_localizers).grid(row=0, column=2, padx=4)
        ttk.Button(actf, text="Stop", command=self.on_stop).grid(row=1, column=0, padx=4, pady=(6, 0))
        ttk.Button(actf, text="Reset Track Mission", command=self.on_reset_track).grid(row=1, column=1, padx=4, pady=(6, 0))
        ttk.Button(actf, text="Export Excel/CSV", command=self.export_point_records).grid(row=2, column=0, padx=4, pady=(6, 0))
        ttk.Button(actf, text="Clear Point Log", command=self.clear_point_records).grid(row=2, column=1, padx=4, pady=(6, 0))
        ttk.Button(actf, text="Shutdown", command=self.on_shutdown).grid(row=3, column=0, columnspan=3, padx=4, pady=(6, 0), sticky="ew")

        # ---- Tunables (ODOM + TRACK) ----
        ttk.Separator(actf, orient="horizontal").grid(row=4, column=0, columnspan=3, sticky="ew", pady=(10, 8))

        ttk.Label(actf, text="ODOM γθ (deg):").grid(row=5, column=0, sticky="w")
        ttk.Entry(actf, textvariable=self.gamma_theta_var, width=10).grid(row=5, column=1, sticky="w")

        ttk.Label(actf, text="ODOM γd (mm):").grid(row=6, column=0, sticky="w")
        ttk.Entry(actf, textvariable=self.gamma_dist_var, width=10).grid(row=6, column=1, sticky="w")

        ttk.Separator(actf, orient="horizontal").grid(row=7, column=0, columnspan=3, sticky="ew", pady=(10, 8))

        ttk.Label(actf, text="TRACK Stop (mm):").grid(row=8, column=0, sticky="w")
        ttk.Entry(actf, textvariable=self.track_safe_stop_var, width=10).grid(row=8, column=1, sticky="w")

        ttk.Label(actf, text="TRACK Reach (mm):").grid(row=9, column=0, sticky="w")
        ttk.Entry(actf, textvariable=self.track_reach_var, width=10).grid(row=9, column=1, sticky="w")

        ttk.Label(actf, text="TRACK Deadband (px):").grid(row=10, column=0, sticky="w")
        ttk.Entry(actf, textvariable=self.track_deadband_var, width=10).grid(row=10, column=1, sticky="w")
        # -------------------------------

        camf = ttk.LabelFrame(f, text="Camera", padding=6)
        camf.grid(row=1, column=3, sticky="ew", pady=(6, 0))
        ttk.Button(camf, text="Enable Camera", command=self.on_cam_start).grid(row=0, column=0, padx=4)
        ttk.Button(camf, text="Disable Camera", command=self.on_cam_stop).grid(row=0, column=1, padx=4)
        self.lbl_cam = ttk.Label(camf, text="Tracking: OFF")
        self.lbl_cam.grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

        self.lbl_keys = ttk.Label(
            f,
            text="Keys: W/A/S/D or arrows. SPACE=Stop. O=Odometry, T=Tracking, M=Manual. Smoother can be Raw Odom / EKF Smoother / PF Experimental.",
        )
        self.lbl_keys.grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 0))

        self.lbl_pose = ttk.Label(f, text="Pose: X=--  Y=--  Th=--")
        self.lbl_pose.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.lbl_belief = ttk.Label(f, text="Smoothed: --")
        self.lbl_belief.grid(row=4, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_goal = ttk.Label(f, text="Goal: --")
        self.lbl_goal.grid(row=5, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_sonar = ttk.Label(f, text="Sonar: --")
        self.lbl_sonar.grid(row=6, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_info = ttk.Label(f, text="Info: --")
        self.lbl_info.grid(row=7, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_points = ttk.Label(f, text="Point log: 0 / 5")
        self.lbl_points.grid(row=8, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_last_saved = ttk.Label(f, text="Last saved point: --")
        self.lbl_last_saved.grid(row=9, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_model_note = ttk.Label(f, text="Note: ODOM now always uses Raw Odom for control. EKF/PF are display/log smoothers only. EKF predicts from velocity (v,w) and applies a weak raw-odometry correction.")
        self.lbl_model_note.grid(row=10, column=0, columnspan=4, sticky="w", pady=(2, 0))

        connf.columnconfigure(1, weight=1)

    def _bind_keys(self):
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.bind_all("<FocusOut>", self._on_focus_out)
        try:
            self.root.focus_force()
        except Exception:
            pass
        self.root.protocol("WM_DELETE_WINDOW", self.on_shutdown)

    def _manual_direction_keys_active(self) -> bool:
        return any(k in self._manual_keys for k in ("w", "a", "s", "d", "up", "down", "left", "right"))

    def _stop_manual_motion_if_needed(self) -> None:
        if self.link.is_connected() and not self._manual_direction_keys_active():
            self.link.stop_motion()

    def _on_focus_out(self, _e: tk.Event):
        self._manual_keys.clear()
        self._stop_manual_motion_if_needed()

    def _show_help(self):
        msg = (
            "GUI guide:\n"
            "- Mode = how the robot behaves: Manual / Odometry / Tracking\n"
            "- Auto must be ON for Odometry or Tracking to move\n\n"
            "Smoother guide:\n"
            "- Raw Odom: uses the robot packet pose directly\n"
            "- EKF Smoother: predicts motion from velocity model (v,w) and applies a weak correction from raw odometry\n"
            "- PF Experimental: particle-based smoother for comparison only\n\n"
            "Important:\n"
            "- In ODOM mode, the robot is controlled ONLY by Raw Odom again\n"
            "- EKF and PF are used only for live display and the saved student comparison table\n"
            "- This restores the old working odometry behaviour on the real robot\n"
            "- The live labels may look similar at the final stop because the smoother converges toward raw when the robot is no longer moving\n"
            "- The saved P1..P5 rows are the correct values to compare for students\n\n"
            "Point log:\n"
            "- A row is saved each time the robot reaches P1..P5 in ODOM mode\n"
            "- Export Excel/CSV saves the table for student reports in the program folder\n"
            "- Last saved point shows the most recent saved comparison directly in the GUI\n- Export now shows a popup with the exact saved file path\n- Autosave files are also written after each captured point\n\n"
            "Manual drive:\n"
            "- W/A/S/D or arrows, SPACE stops\n\n"
            "Tracking rules:\n"
            "- No ROI => robot will NOT move\n"
            "- ROI set but no blob => robot will NOT move\n"
            "- Avoidance has priority\n"
            "- Mission complete when front sonar stays <= reach distance\n"
        )
        top = tk.Toplevel(self.root)
        top.title("Help")
        frm = ttk.Frame(top, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        txt = tk.Text(frm, width=100, height=24, wrap="word")
        txt.grid(row=0, column=0, sticky="nsew")
        txt.insert("1.0", msg)
        txt.configure(state="disabled")
        ttk.Button(frm, text="Close", command=top.destroy).grid(row=1, column=0, sticky="e", pady=(8, 0))

    def reset_localizers(self):
        self.ekf.reset()
        self.pf.reset()
        self._last_ekf_state = None
        self._last_pf_state = None
        self._last_state_t = time.time()

    def clear_point_records(self):
        self.point_records.clear()
        self.last_saved_record = None
        if hasattr(self, "lbl_points"):
            self.lbl_points.config(text=f"Point log: {len(self.point_records)} / {len(POINT_NAMES)}")
        if hasattr(self, "lbl_last_saved"):
            self.lbl_last_saved.config(text="Last saved point: --")
        self.lbl_info.config(text="Info: point log cleared")

    def _get_export_dir(self) -> Path:
        candidates = []
        try:
            candidates.append(Path(__file__).resolve().parent)
        except Exception:
            pass
        try:
            candidates.append(Path.cwd())
        except Exception:
            pass
        try:
            candidates.append(Path.home() / "Documents")
            candidates.append(Path.home())
        except Exception:
            pass

        seen = set()
        for d in candidates:
            try:
                d = d.resolve()
            except Exception:
                d = Path(d)
            key = str(d)
            if key in seen:
                continue
            seen.add(key)
            try:
                d.mkdir(parents=True, exist_ok=True)
                test_file = d / ".p3dx_export_test.tmp"
                with open(test_file, "w", encoding="utf-8") as f:
                    f.write("ok")
                test_file.unlink(missing_ok=True)
                return d
            except Exception:
                continue
        return Path(".")

    def _export_records_csv(self, csv_path: Path) -> None:
        with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
            writer = csv.DictWriter(fcsv, fieldnames=list(self.point_records[0].keys()))
            writer.writeheader()
            writer.writerows(self.point_records)

    def _export_records_xlsx(self, xlsx_path: Path) -> None:
        if Workbook is None:
            raise RuntimeError("openpyxl not installed")
        wb = Workbook()
        ws = wb.active
        ws.title = "Point Log"
        headers = list(self.point_records[0].keys())
        ws.append(headers)
        for rec in self.point_records:
            ws.append([rec.get(h, "") for h in headers])
        for col in ws.columns:
            max_len = 0
            col_letter = col[0].column_letter
            for cell in col:
                val = "" if cell.value is None else str(cell.value)
                if len(val) > max_len:
                    max_len = len(val)
            ws.column_dimensions[col_letter].width = min(max(12, max_len + 2), 24)
        wb.save(xlsx_path)

    def _show_export_result(self, title: str, message: str, error: bool = False) -> None:
        try:
            if error:
                messagebox.showerror(title, message, parent=self.root)
            else:
                messagebox.showinfo(title, message, parent=self.root)
        except Exception:
            pass

    def _autosave_point_records(self) -> None:
        if not self.point_records:
            return
        try:
            export_dir = self._get_export_dir()
            csv_path = export_dir / "odom_smoother_points_autosave.csv"
            self._export_records_csv(csv_path)
            if Workbook is not None:
                xlsx_path = export_dir / "odom_smoother_points_autosave.xlsx"
                self._export_records_xlsx(xlsx_path)
        except Exception as e:
            print(f"[AUTOSAVE] failed: {e}")

    def export_point_records(self):
        count = len(self.point_records)
        total = len(POINT_NAMES)
        if not self.point_records:
            msg = "No point records to export yet.\n\nRun ODOM until at least one point is saved."
            self.lbl_info.config(text="Info: no point records to export")
            self._show_export_result("Export Point Log", msg, error=True)
            return

        stamp = time.strftime("%Y%m%d_%H%M%S")
        export_dir = self._get_export_dir()
        csv_path = export_dir / f"odom_smoother_points_{stamp}.csv"
        xlsx_path = export_dir / f"odom_smoother_points_{stamp}.xlsx"

        try:
            self._export_records_csv(csv_path)
        except Exception as e:
            msg = f"CSV export failed.\n\nPath: {csv_path}\n\nError: {e}"
            self.lbl_info.config(text=f"Info: CSV export failed ({e})")
            self._show_export_result("Export Point Log", msg, error=True)
            return

        if Workbook is None:
            msg = (
                f"Exported partial/full point log: {count}/{total} point(s).\n\n"
                f"CSV saved to:\n{csv_path}\n\n"
                f"Excel file was not created because openpyxl is not available."
            )
            self.lbl_info.config(text=f"Info: exported CSV -> {csv_path}")
            print(f"[EXPORT] CSV: {csv_path}")
            self._show_export_result("Export Point Log", msg)
            return

        try:
            self._export_records_xlsx(xlsx_path)
            msg = (
                f"Exported point log: {count}/{total} point(s).\n\n"
                f"Excel:\n{xlsx_path}\n\nCSV:\n{csv_path}"
            )
            self.lbl_info.config(text=f"Info: exported Excel/CSV -> {xlsx_path}")
            print(f"[EXPORT] Excel: {xlsx_path}")
            print(f"[EXPORT] CSV:   {csv_path}")
            self._show_export_result("Export Point Log", msg)
        except Exception as e:
            msg = (
                f"CSV was saved, but Excel export failed.\n\n"
                f"CSV:\n{csv_path}\n\n"
                f"Excel error: {e}"
            )
            self.lbl_info.config(text=f"Info: Excel failed, CSV saved -> {csv_path}")
            print(f"[EXPORT] CSV:   {csv_path}")
            print(f"[EXPORT] Excel failed: {e}")
            self._show_export_result("Export Point Log", msg, error=True)
    def _record_arrived_point(self, point_idx: int, raw_st: RobotState, ekf_st: RobotState, pf_st: RobotState):
        if not (0 <= point_idx < len(self.odom.nodes)):
            return
        point_name = POINT_NAMES[point_idx] if point_idx < len(POINT_NAMES) else f"P{point_idx+1}"
        gx, gy = self.odom.nodes[point_idx]
        rec = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "transport": self.link.transport,
            "endpoint": self.link.endpoint,
            "point": point_name,
            "target_x_mm": round(gx, 3),
            "target_y_mm": round(gy, 3),
            "raw_x_mm": round(raw_st.x_mm, 3),
            "raw_y_mm": round(raw_st.y_mm, 3),
            "raw_th_deg": round(raw_st.th_deg_wrap, 3),
            "ekf_x_mm": round(ekf_st.x_mm, 3),
            "ekf_y_mm": round(ekf_st.y_mm, 3),
            "ekf_th_deg": round(ekf_st.th_deg_wrap, 3),
            "pf_x_mm": round(pf_st.x_mm, 3),
            "pf_y_mm": round(pf_st.y_mm, 3),
            "pf_th_deg": round(pf_st.th_deg_wrap, 3),
            "raw_err_mm": round(math.hypot(gx - raw_st.x_mm, gy - raw_st.y_mm), 3),
            "ekf_err_mm": round(math.hypot(gx - ekf_st.x_mm, gy - ekf_st.y_mm), 3),
            "pf_err_mm": round(math.hypot(gx - pf_st.x_mm, gy - pf_st.y_mm), 3),
        }
        self.point_records.append(rec)
        self.last_saved_record = rec
        if hasattr(self, "lbl_points"):
            self.lbl_points.config(text=f"Point log: {len(self.point_records)} / {len(POINT_NAMES)}")
        if hasattr(self, "lbl_last_saved"):
            self.lbl_last_saved.config(
                text=(
                    f"Last saved point: {point_name} | raw err={rec['raw_err_mm']:.1f} mm | "
                    f"EKF err={rec['ekf_err_mm']:.1f} mm | PF err={rec['pf_err_mm']:.1f} mm"
                )
            )
        self._autosave_point_records()
        self.lbl_info.config(text=f"Info: saved {point_name} ({len(self.point_records)}/{len(POINT_NAMES)})")

    def _smoother_mode_name(self) -> str:
        mode = self.loc_mode.get()
        return {"RAW": "Raw Odom", "EKF": "EKF Smoother", "PF": "PF Experimental"}.get(mode, mode)

    def on_localization_mode_change(self):
        self.reset_localizers()

    def _apply_localization_tunables(self) -> None:
        try:
            gate = max(50.0, float(self.loc_goal_gate_var.get()))
            sig = max(10.0, float(self.loc_goal_sigma_var.get()))
            goal_enabled = bool(self.goal_anchor_var.get())

            self.ekf.goal_gate_mm = gate
            self.ekf.goal_sigma_xy = sig
            self.ekf.goal_enabled = goal_enabled

            n_pf = int(max(80, self.pf_particles_var.get()))
            if n_pf != self.pf.n:
                self.pf = ParticleFilterLocalizer(n_pf)
            self.pf.goal_gate_mm = gate
            self.pf.goal_sigma_xy = sig
            self.pf.goal_enabled = goal_enabled
        except Exception:
            pass

    def _localized_state(self, raw_st: RobotState, dt: float, goal: Optional[Tuple[float, float]]) -> RobotState:
        self._apply_localization_tunables()
        self._last_ekf_state = self.ekf.step(raw_st, dt, goal)
        self._last_pf_state = self.pf.step(raw_st, dt, goal)

        mode = self.loc_mode.get()
        if mode == "EKF":
            return self._last_ekf_state
        if mode == "PF":
            return self._last_pf_state
        return raw_st

    def on_scan_ports(self):
        ports = list_serial_ports_manual()
        self.cmb_serial["values"] = ports
        if ports and (self.serial_var.get() not in ports):
            self.serial_var.set(ports[0])

    def on_connect(self):
        self.track_done = False
        self.reset_localizers()
        mode = self.conn_mode.get()
        ok = False

        if mode.startswith("Simulation"):
            host = self.host_var.get().strip()
            try:
                port = int(self.port_var.get().strip())
            except Exception:
                port = MOBILE_SIM_PORT
                self.port_var.set(str(port))
            ok = self.link.connect_tcp(host, port)

        else:
            portname = self.serial_var.get().strip()
            try:
                baud = int(self.baud_var.get().strip())
            except Exception:
                baud = SERIAL_BAUD
                self.baud_var.set(str(baud))

            if not portname:
                self.on_scan_ports()
                portname = self.serial_var.get().strip()

            ok = self.link.connect_serial(portname, baud)

        self.lbl_conn.configure(text=f"Status: {'CONNECTED' if ok else 'FAILED'} ({self.link.transport} {self.link.endpoint})" if ok else "Status: FAILED")

    def on_disconnect(self):
        self.auto.set(False)
        self._manual_keys.clear()
        self.reset_localizers()
        self.link.close()
        self.lbl_conn.configure(text="Status: DISCONNECTED")

    def on_reset_nodes(self):
        self.odom.reset()
        self.reset_localizers()
        self.clear_point_records()

    def on_reset_track(self):
        self.track_done = False
        self.track.reset()

    def on_stop(self):
        self.auto.set(False)
        self._manual_keys.clear()
        if self.link.is_connected():
            self.link.stop_motion()

    def on_shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        self.auto.set(False)

        if self._tick_job is not None:
            try:
                self.root.after_cancel(self._tick_job)
            except Exception:
                pass
            self._tick_job = None

        self.on_cam_stop()
        self.on_disconnect()
        try:
            self.root.destroy()
        except Exception:
            pass

    def on_cam_start(self):
        ok = self.cam.start()
        self.lbl_cam.configure(text=f"Tracking: {'ON' if ok else 'OFF'}")

    def on_cam_stop(self):
        self.cam.stop()
        self.lbl_cam.configure(text="Tracking: OFF")

    def open_manual_pad(self):
        if self._manual_pad is not None and self._manual_pad.winfo_exists():
            self._manual_pad.lift()
            return

        top = tk.Toplevel(self.root)
        top.title("Manual Drive")
        self._manual_pad = top

        frm = ttk.Frame(top, padding=10)
        frm.grid(row=0, column=0)

        ttk.Label(frm, text="Hold buttons OR use keyboard: W/A/S/D, arrows. SPACE = Stop").grid(
            row=0, column=0, columnspan=3, pady=(0, 8)
        )

        def bind_hold(btn: ttk.Button, v: float, w: float):
            btn.bind("<ButtonPress-1>", lambda e: self._manual_hold(v, w))
            btn.bind("<ButtonRelease-1>", lambda e: self._manual_hold(0.0, 0.0))

        b_up = ttk.Button(frm, text="Forward", width=12)
        b_left = ttk.Button(frm, text="Left", width=12)
        b_stop = ttk.Button(frm, text="Stop", width=12, command=self.on_stop)
        b_right = ttk.Button(frm, text="Right", width=12)
        b_down = ttk.Button(frm, text="Backward", width=12)

        b_up.grid(row=1, column=1, padx=4, pady=4)
        b_left.grid(row=2, column=0, padx=4, pady=4)
        b_stop.grid(row=2, column=1, padx=4, pady=4)
        b_right.grid(row=2, column=2, padx=4, pady=4)
        b_down.grid(row=3, column=1, padx=4, pady=4)

        bind_hold(b_up, +200.0, 0.0)
        bind_hold(b_down, -200.0, 0.0)
        bind_hold(b_left, 0.0, +45.0)
        bind_hold(b_right, 0.0, -45.0)

        top.protocol("WM_DELETE_WINDOW", lambda: top.destroy())

    def _manual_hold(self, v: float, w: float):
        self.auto.set(False)
        if self.link.is_connected():
            self.link.drive(v, w)

    def _on_key_press(self, e: tk.Event):
        k = (e.keysym or "").lower()
        self._manual_keys.add(k)
        if k == "space":
            self.on_stop()
        elif k == "o":
            self.mode.set("ODOM")
        elif k == "t":
            self.mode.set("TRACK")
        elif k == "m":
            self.mode.set("MANUAL")

    def _on_key_release(self, e: tk.Event):
        k = (e.keysym or "").lower()
        self._manual_keys.discard(k)
        if k in ("w", "a", "s", "d", "up", "down", "left", "right"):
            self._stop_manual_motion_if_needed()

    def _manual_command_from_keys(self) -> Tuple[float, float]:
        v = 0.0
        w = 0.0
        keys = self._manual_keys

        if "w" in keys or "up" in keys:
            v += 220.0
        if "s" in keys or "down" in keys:
            v -= 220.0
        if "a" in keys or "left" in keys:
            w += 50.0
        if "d" in keys or "right" in keys:
            w -= 50.0

        if any(k in keys for k in ("w", "a", "s", "d", "up", "down", "left", "right")):
            self.auto.set(False)

        return v, w

    def _tick(self):
        if self._shutting_down:
            return
        try:
            self.lbl_cam.configure(text=f"Tracking: {'ON' if self.cam.enabled else 'OFF'}")
            self._control_step()
        finally:
            if self._shutting_down:
                self._tick_job = None
                return
            try:
                if self.root.winfo_exists():
                    self._tick_job = self.root.after(int(1000 / CONTROL_HZ), self._tick)
                else:
                    self._tick_job = None
            except tk.TclError:
                self._tick_job = None

    def _control_step(self):
        if not self.link.is_connected():
            self.lbl_pose.config(text="Pose: X=--  Y=--  Th=--")
            self.lbl_belief.config(text="Smoothed: --")
            self.lbl_goal.config(text="Goal: --")
            self.lbl_sonar.config(text="Sonar: --")
            self.lbl_info.config(text="Info: not connected")
            return

        self.link.pulse_if_needed()
        raw_st = self.link.get_state()
        if raw_st is None:
            self.lbl_pose.config(text="Pose: (waiting for SIP...)")
            self.lbl_belief.config(text=f"Smoothed: waiting ({self._smoother_mode_name()})")
            self.lbl_goal.config(text="Goal: --")
            self.lbl_sonar.config(text="Sonar: --")
            self.lbl_info.config(text="Info: waiting for sensor packets (SIP)")
            return

        now = time.time()
        dt = max(1e-3, min(0.25, now - self._last_state_t))
        self._last_state_t = now
        goal = self.odom.current_goal()
        st = self._localized_state(raw_st, dt, goal)

        self.lbl_pose.config(
            text=f"Pose(raw): X={raw_st.x_mm:7.1f}  Y={raw_st.y_mm:7.1f}  Th={raw_st.th_deg_wrap:6.1f} (360={raw_st.th_deg_360:6.1f})"
        )
        self.lbl_belief.config(
            text=f"Smoothed({self._smoother_mode_name()}): X={st.x_mm:7.1f}  Y={st.y_mm:7.1f}  Th={st.th_deg_wrap:6.1f}"
        )

        # Apply GUI tunables (ODOM + TRACK)
        try:
            self.odom.gamma_theta = float(self.gamma_theta_var.get())
            gd = float(self.gamma_dist_var.get())
            gd = max(10.0, gd)
            self.odom.gamma_dist = gd
            self.odom.gamma_dist_final = gd
        except Exception:
            pass

        try:
            self.track.safe_stop_mm = max(50.0, float(self.track_safe_stop_var.get()))
            self.track.reach_mm = max(50.0, float(self.track_reach_var.get()))
            self.track.deadband_px = max(0.0, float(self.track_deadband_var.get()))
        except Exception:
            pass

        goal = self.odom.current_goal()
        if goal is None:
            self.lbl_goal.config(text="Goal: (done)")
        else:
            gx, gy = goal
            dist_bel = math.hypot(gx - st.x_mm, gy - st.y_mm)
            dist_raw = math.hypot(gx - raw_st.x_mm, gy - raw_st.y_mm)
            self.lbl_goal.config(text=f"Goal: node {self.odom.i+1}/{len(self.odom.nodes)} -> ({gx:.0f},{gy:.0f})  raw={dist_raw:.0f}mm smooth={dist_bel:.0f}mm")

        s = st.sonars_mm[:] if st.sonars_mm else [0.0] * 16
        self.lbl_sonar.config(text="Sonar: " + " ".join([f"s{i}={int(s[i]):4d}" for i in range(8)]))

        # Manual keys override any mode
        mv, mw = self._manual_command_from_keys()
        if mv != 0.0 or mw != 0.0:
            self.link.drive(mv, mw)
            self.lbl_info.config(text="Info: MANUAL (keys)")
            return

        if not self.auto.get():
            self.lbl_info.config(text=f"Info: idle (mode={self.mode.get()}, Auto=OFF)")
            return

        if self.track_done and self.mode.get() == "TRACK":
            self.link.stop_motion()
            self.auto.set(False)
            self.lbl_info.config(text="Info: TRACK COMPLETE -> STOP")
            return

        mode = self.mode.get()
        v_cmd = 0.0
        w_cmd = 0.0
        info = ""

        if mode == "ODOM":
            # IMPORTANT: keep the original working odometry path on the real robot.
            # ODOM and front avoidance use raw packet odometry only.
            # EKF/PF are display/log smoothers only.
            prev_idx = self.odom.i
            fa = self.front_avoid.step(raw_st, self.odom.current_goal())
            if fa is not None:
                v_cmd, w_cmd, info = fa
            else:
                v_cmd, w_cmd, info = self.odom.step(raw_st)
                if self.odom.i > prev_idx:
                    ekf_st = self._last_ekf_state if self._last_ekf_state is not None else raw_st
                    pf_st = self._last_pf_state if self._last_pf_state is not None else raw_st
                    self._record_arrived_point(prev_idx, raw_st, ekf_st, pf_st)
                    info += f" | saved {POINT_NAMES[prev_idx] if prev_idx < len(POINT_NAMES) else prev_idx+1}"
                if self.odom.done():
                    self.link.stop_motion()
                    self.auto.set(False)
                    info = "ODOM complete -> STOP (raw odom control restored)"
                    v_cmd, w_cmd = 0.0, 0.0
        elif mode == "TRACK":
            # Tracking first (keeps blob-following stable). Avoidance only overrides when needed.
            roi_ready = self.cam.enabled and self.cam.has_roi()
            blob = self.cam.get_blob() if self.cam.enabled else Blob(False)

            v_cmd, w_cmd, info, done = self.track.step(st, blob, roi_ready)
            if done:
                self.track_done = True
                self.link.stop_motion()
                self.auto.set(False)
                self.lbl_info.config(text=f"Info: {info}")
                return

            # If we are stopped due to safety (ROI missing / target lost / too close), do NOT override with avoidance.
            if (not roi_ready) or (not blob.found) or (blob.img_w <= 0) or info.startswith("TRACK: STOP") or info.startswith("TRACK: WAIT"):
                pass
            else:
                # Side avoidance override (deterministic, no backing)
                av_v, av_w, av_info = self.avoid.step(st)

                # Only override if avoidance has a command AND we are trying to move forward.
                if (av_v is not None) and (av_w is not None) and (v_cmd > 0.0):
                    v_cmd, w_cmd, info = av_v, av_w, av_info


        else:
            v_cmd, w_cmd, info = 0.0, 0.0, "MANUAL(auto): idle"

        self.link.drive(v_cmd, w_cmd)
        self.lbl_info.config(text=f"Info: {info}")


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()

