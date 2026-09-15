#!/usr/bin/env python3
"""pioneer_odom_only_good.py

Odometry-only waypoint driver for Pioneer P3-DX / MobileSim using the P2OS protocol.

- Pure Python (no ARIA/ROS required)
- Connects either:
    * MobileSim via TCP (default 127.0.0.1:8101)
    * Real robot via Serial (e.g. COM3 @ 9600)
- No obstacle avoidance, no sonar logic (we don't request sonar)
- Drives through a list of waypoints using an ALIGN->MOVE state machine

This script is designed for teaching labs where MobileSim works first, then the
same logic is used on the real robot.

Usage examples
--------------

1) MobileSim (Windows):
    python pioneer_odom_only_good.py --sim

2) Real robot (Windows):
    python pioneer_odom_only_good.py --serial COM3 --baud 9600 --theta ticks

If the robot turns the *wrong* way (common when simulator/robot differ in sign
convention), try:
    --invert-rotcmd

If the pose heading increases opposite to the physical rotation, try:
    --invert-theta

If your real robot uses tenths-of-degree heading (0..3599), try:
    --theta tenths

Safety: Ctrl+C stops the robot.

Dr Salem Ameen (Salford) — odometry-only lab helper
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

try:
    import serial  # type: ignore
    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        list_ports = None
except Exception:
    serial = None
    list_ports = None

# =====================
# P2OS packet constants
# =====================

HEADER = bytes([0xFA, 0xFB])

SYNC0 = bytes([0xFA, 0xFB, 0x03, 0x00, 0x00, 0x00])
SYNC1 = bytes([0xFA, 0xFB, 0x03, 0x01, 0x00, 0x01])
SYNC2 = bytes([0xFA, 0xFB, 0x03, 0x02, 0x00, 0x02])

CMD_PULSE = 0
CMD_OPEN = 1
CMD_ENABLE = 4
CMD_VEL = 11
CMD_RVEL = 21
CMD_STOP = 29

ARG_POS_INT = 0x3B
ARG_NEG_INT = 0x1B


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_deg_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def deg360(th: float) -> float:
    return th - 360.0 * math.floor(th / 360.0)


def calc_checksum(packet: bytes) -> int:
    """P2OS checksum (16-bit) used by SYNC/command packets."""
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


def _i16_le(buf: bytes, off: int) -> int:
    return struct.unpack_from("<h", buf, off)[0]


def parse_sip(packet: bytes) -> Optional[Tuple[int, int, int, int, int]]:
    """Parse a standard SIP-like packet: returns (x, y, th_raw, lvel, rvel).

    This matches the common Pioneer/MobileSim layout used in many labs.
    """
    if len(packet) < 8 or packet[:2] != HEADER:
        return None

    bytecount = packet[2]
    if len(packet) < 3 + bytecount:
        return None

    payload = packet[3 : 3 + bytecount]
    if len(payload) < 1 + 2 * 5 + 2:
        return None

    # payload = [ptype][data...][checksum_hi][checksum_lo]
    data = payload[1:-2]
    if len(data) < 10:
        return None

    x = _i16_le(data, 0)
    y = _i16_le(data, 2)
    th = _i16_le(data, 4)
    lvel = _i16_le(data, 6)
    rvel = _i16_le(data, 8)
    return x, y, th, lvel, rvel


# =====================
# Robot state & odometry
# =====================


@dataclass
class RobotState:
    x_mm: float
    y_mm: float
    th_deg_wrap: float
    th_deg_360: float
    th_raw: int
    lvel: float
    rvel: float


class ThetaDecoder:
    """Stable heading conversion.

    Important difference vs your earlier GUI code:
    - AUTO mode does *not* flip between representations during the run.
    - We decide once (or you set it explicitly).

    Modes:
      - deg    : th_raw is degrees
      - tenths : th_raw is 0.1 degrees
      - ticks  : th_raw is 0..4095 ticks mapped to 0..360 degrees
      - auto   : decide once based on observed raw values
    """

    def __init__(self, mode: str = "auto", invert_theta: bool = False):
        self.mode = (mode or "auto").strip().lower()
        self.invert_theta = bool(invert_theta)

        self._decided_mode: Optional[str] = None
        self._samples: List[int] = []
        self._t0 = time.time()

    def decided_mode(self) -> str:
        return self._decided_mode or (self.mode if self.mode != "auto" else "auto(pending)")

    def _decide_from_samples(self) -> Optional[str]:
        if not self._samples:
            return None
        maxabs = max(abs(s) for s in self._samples)
        # Heuristic:
        #  - values near 4095 strongly suggest ticks
        #  - values > 360 suggest tenths or ticks; values > ~3700 => ticks
        if maxabs >= 3700:
            return "ticks"
        if maxabs > 360:
            return "tenths"
        # If we only saw small headings so far, delay the decision.
        return None

    def to_deg(self, th_raw: int) -> float:
        if self.mode != "auto":
            mode = self.mode
        else:
            # Gather evidence and decide once.
            if self._decided_mode is None:
                self._samples.append(int(th_raw))
                # Decide if enough evidence OR after a short time window.
                decided = self._decide_from_samples()
                if decided is not None:
                    self._decided_mode = decided
                elif (time.time() - self._t0) > 2.0 and len(self._samples) >= 20:
                    # still no evidence: assume degrees for now
                    self._decided_mode = "deg"
            mode = self._decided_mode or "deg"

        th = float(th_raw)
        if mode == "deg":
            out = th
        elif mode == "tenths":
            out = th * 0.1
        elif mode == "ticks":
            out = th * 360.0 / 4095.0
        else:
            # fallback
            out = th

        if self.invert_theta:
            out = -out
        return out


# =====================
# Link layer (TCP/Serial)
# =====================


class PioneerLink:
    def __init__(
        self,
        theta_mode: str = "auto",
        xy_scale: float = 1.0,
        invert_theta: bool = False,
        invert_rotcmd: bool = False,
        flip_x: bool = False,
        flip_y: bool = False,
    ):
        self.sock: Optional[socket.socket] = None
        self.ser = None
        self.use_serial = False

        self._buf = bytearray()
        self._stop_evt = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

        self._lock = threading.Lock()
        self._latest: Optional[RobotState] = None

        self._origin_set = False
        self._x0 = 0.0
        self._y0 = 0.0
        self._th0 = 0.0

        self.xy_scale = float(xy_scale)
        self.flip_x = bool(flip_x)
        self.flip_y = bool(flip_y)

        self.theta = ThetaDecoder(mode=theta_mode, invert_theta=invert_theta)

        self.invert_rotcmd = bool(invert_rotcmd)

        self._last_keepalive = 0.0
        self._last_rx_time = time.time()

    # ---- connect / close ----

    def connect_tcp(self, host: str, port: int) -> None:
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((host, port))
        s.settimeout(0.05)
        self.sock = s
        self.use_serial = False
        self._handshake()
        self._start_reader()

    def connect_serial(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install with: pip install pyserial")
        self.close()
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.use_serial = True
        self._handshake()
        self._start_reader()

    def close(self) -> None:
        self._stop_evt.set()
        try:
            self.stop()
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

        self.ser = None
        self.sock = None
        self.use_serial = False

        with self._lock:
            self._latest = None
        self._buf = bytearray()

    # ---- low-level IO ----

    def _write(self, b: bytes) -> None:
        if self.use_serial and self.ser is not None:
            self.ser.write(b)
        elif self.sock is not None:
            self.sock.sendall(b)

    def _read_some(self) -> bytes:
        if self.use_serial and self.ser is not None:
            return self.ser.read(4096)
        if self.sock is not None:
            try:
                d = self.sock.recv(4096)
                if d == b"":
                    raise ConnectionError("TCP closed")
                return d
            except socket.timeout:
                return b""
        return b""

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

    def _handshake(self) -> None:
        # SYNC sequence
        for p in (SYNC0, SYNC1, SYNC2):
            self._write(p)
            time.sleep(0.05)
            _ = self._read_some()

        # OPEN + ENABLE
        self.send_cmd(CMD_OPEN)
        self.send_cmd(CMD_ENABLE, 1)
        self.send_cmd(CMD_PULSE)

    def keepalive(self, period_s: float = 0.8) -> None:
        now = time.time()
        if now - self._last_keepalive >= period_s:
            try:
                self.send_cmd(CMD_PULSE)
            except Exception:
                pass
            self._last_keepalive = now

    # ---- SIP reader ----

    def _start_reader(self) -> None:
        self._stop_evt.clear()
        self._origin_set = False
        self._last_rx_time = time.time()

        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_thread.start()

        # Wait briefly for first state
        t0 = time.time()
        while time.time() - t0 < 1.8:
            if self.get_state() is not None:
                return
            time.sleep(0.02)
        raise RuntimeError("Connected but no SIP received (check MobileSim streaming or serial settings)")

    def _reader_loop(self) -> None:
        try:
            while not self._stop_evt.is_set():
                d = self._read_some()
                if d:
                    self._buf.extend(d)
                    self._consume_packets()
                else:
                    time.sleep(0.01)
        except Exception:
            self.close()

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

            # accept common SIP types (0x30..0x33)
            ptype = pkt[3] if len(pkt) > 3 else 0
            if not (0x30 <= ptype <= 0x33):
                continue

            parsed = parse_sip(pkt)
            if parsed is None:
                continue

            self._last_rx_time = time.time()

            x_raw, y_raw, th_raw, lvel, rvel = parsed

            # scaling & axis options
            x = float(x_raw) * self.xy_scale
            y = float(y_raw) * self.xy_scale
            if self.flip_x:
                x = -x
            if self.flip_y:
                y = -y

            th_deg_abs = self.theta.to_deg(int(th_raw))

            if not self._origin_set:
                self._origin_set = True
                self._x0, self._y0, self._th0 = x, y, th_deg_abs

            x_rel = x - self._x0
            y_rel = y - self._y0
            th_rel = th_deg_abs - self._th0

            st = RobotState(
                x_mm=x_rel,
                y_mm=y_rel,
                th_deg_wrap=wrap_deg_180(th_rel),
                th_deg_360=deg360(th_rel),
                th_raw=int(th_raw),
                lvel=float(lvel),
                rvel=float(rvel),
            )

            with self._lock:
                self._latest = st

    def get_state(self) -> Optional[RobotState]:
        with self._lock:
            if self._latest is None:
                return None
            return RobotState(**self._latest.__dict__)

    def rx_age(self) -> float:
        return time.time() - self._last_rx_time

    # ---- motion ----

    def drive(self, v_mm_s: float, w_deg_s: float, v_limit: float, w_limit: float) -> None:
        v = int(clamp(v_mm_s, -abs(v_limit), abs(v_limit)))
        w = float(clamp(w_deg_s, -abs(w_limit), abs(w_limit)))
        if self.invert_rotcmd:
            w = -w
        self.send_cmd(CMD_VEL, v)
        self.send_cmd(CMD_RVEL, int(w))

    def stop(self) -> None:
        try:
            self.send_cmd(CMD_STOP)
            self.send_cmd(CMD_VEL, 0)
            self.send_cmd(CMD_RVEL, 0)
        except Exception:
            pass


# =====================
# Odometry waypoint nav
# =====================


class OdomNavigator:
    """ALIGN -> MOVE controller (your assignment logic)."""

    def __init__(
        self,
        waypoints_mm: Sequence[Tuple[float, float]],
        gamma_theta_deg: float = 20.0,
        gamma_dist_mm: float = 120.0,
        kp_rot: float = 1.3,
        kp_steer: float = 1.2,
    ):
        self.waypoints = list(waypoints_mm)
        self.i = 0
        self.gamma_theta = float(gamma_theta_deg)
        self.gamma_dist = float(gamma_dist_mm)
        self.kp_rot = float(kp_rot)
        self.kp_steer = float(kp_steer)

    def done(self) -> bool:
        return self.i >= len(self.waypoints)

    def goal(self) -> Optional[Tuple[float, float]]:
        return None if self.done() else self.waypoints[self.i]

    @staticmethod
    def _bearing_deg360(x0: float, y0: float, x1: float, y1: float) -> float:
        a = math.degrees(math.atan2(y1 - y0, x1 - x0))
        if a < 0:
            a += 360.0
        return a

    def step(self, st: RobotState, v_cruise: float, w_max: float) -> Tuple[float, float, str]:
        if self.done():
            return 0.0, 0.0, "DONE"

        gx, gy = self.waypoints[self.i]
        dx = gx - st.x_mm
        dy = gy - st.y_mm
        dist = math.hypot(dx, dy)

        if dist <= self.gamma_dist:
            self.i += 1
            if self.done():
                return 0.0, 0.0, "ARRIVED final"
            return 0.0, 0.0, f"ARRIVED -> next {self.i+1}/{len(self.waypoints)}"

        target = self._bearing_deg360(st.x_mm, st.y_mm, gx, gy)
        err = wrap_deg_180(target - st.th_deg_360)

        # Angular transition δθ
        if abs(err) > self.gamma_theta:
            w = clamp(self.kp_rot * err, -w_max, w_max)
            # minimum to overcome stiction
            if 0 < abs(w) < 10.0:
                w = 10.0 if w > 0 else -10.0
            return 0.0, w, f"ALIGN err={err:.1f} deg dist={dist:.0f}"

        # Linear transition δd
        w = clamp(self.kp_steer * err, -w_max, w_max)
        v = float(v_cruise)
        return v, w, f"MOVE err={err:.1f} deg dist={dist:.0f}"


# =====================
# Utilities
# =====================


def scan_serial_ports() -> List[str]:
    out: List[str] = []
    if list_ports is not None:
        try:
            for p in list_ports.comports():
                if getattr(p, "device", None):
                    out.append(p.device)
        except Exception:
            pass
    return out


def parse_points(s: str) -> List[Tuple[float, float]]:
    """Parse points as: "x1,y1;x2,y2;..." in mm."""
    pts: List[Tuple[float, float]] = []
    for part in s.split(";"):
        part = part.strip()
        if not part:
            continue
        xs, ys = part.split(",")
        pts.append((float(xs), float(ys)))
    if not pts:
        raise ValueError("No points parsed")
    return pts


# =====================
# Main
# =====================


DEFAULT_POINTS = [
    (2060.0, 0.0),
    (2060.0, -2530.0),
    (4060.0, -2530.0),
    (4060.0, -3790.0),
    (-1880.0, -3790.0),
]


def main() -> int:
    ap = argparse.ArgumentParser(add_help=True)

    conn = ap.add_argument_group("connection")
    conn.add_argument("--sim", action="store_true", help="Use MobileSim TCP (default if no --serial)")
    conn.add_argument("--host", default="127.0.0.1", help="MobileSim host")
    conn.add_argument("--port", type=int, default=8101, help="MobileSim port")
    conn.add_argument("--serial", default="", help="Serial port for real robot (e.g. COM3 or /dev/ttyUSB0)")
    conn.add_argument("--baud", type=int, default=9600, help="Serial baud (often 9600)")

    od = ap.add_argument_group("odometry")
    od.add_argument(
        "--points",
        default="",
        help='Waypoints in mm: "x1,y1;x2,y2;..." (default: the 5 lab points)',
    )
    od.add_argument("--hz", type=float, default=20.0, help="Control loop rate")
    od.add_argument("--gamma-theta", type=float, default=20.0, help="Angle threshold (deg) to switch ALIGN/MOVE")
    od.add_argument("--gamma-dist", type=float, default=120.0, help="Distance threshold (mm) to accept waypoint")

    motion = ap.add_argument_group("motion")
    motion.add_argument("--v", type=float, default=220.0, help="Cruise speed (mm/s)")
    motion.add_argument("--w", type=float, default=45.0, help="Max rotation speed (deg/s)")
    motion.add_argument("--invert-rotcmd", action="store_true", help="Invert sign of rotation command sent to robot")

    decode = ap.add_argument_group("decoding")
    decode.add_argument("--theta", choices=["auto", "deg", "tenths", "ticks"], default="auto", help="Heading raw units")
    decode.add_argument("--invert-theta", action="store_true", help="Invert heading sign when decoding")
    decode.add_argument("--xy-scale", type=float, default=1.0, help="Multiply raw x/y by this")
    decode.add_argument("--flip-x", action="store_true", help="Flip x sign")
    decode.add_argument("--flip-y", action="store_true", help="Flip y sign")

    dbg = ap.add_argument_group("debug")
    dbg.add_argument("--print-every", type=float, default=0.8, help="Console print period (s)")
    dbg.add_argument("--no-motion", action="store_true", help="Decode/print only; never send motion commands")

    args = ap.parse_args()

    pts = DEFAULT_POINTS if not args.points else parse_points(args.points)

    use_serial = bool(args.serial)
    use_sim = args.sim or not use_serial

    link = PioneerLink(
        theta_mode=args.theta,
        xy_scale=args.xy_scale,
        invert_theta=args.invert_theta,
        invert_rotcmd=args.invert_rotcmd,
        flip_x=args.flip_x,
        flip_y=args.flip_y,
    )

    try:
        if use_sim:
            print(f"[CONNECT] TCP {args.host}:{args.port}")
            link.connect_tcp(args.host, args.port)
        else:
            if not args.serial:
                ports = scan_serial_ports()
                raise RuntimeError(f"No --serial provided. Available ports: {ports}")
            print(f"[CONNECT] Serial {args.serial} @ {args.baud}")
            link.connect_serial(args.serial, args.baud)

        print(f"[OK] Connected. theta_mode={link.theta.decided_mode()} invert_rotcmd={args.invert_rotcmd}")
        print("[INFO] Ctrl+C to stop.")

        nav = OdomNavigator(
            pts,
            gamma_theta_deg=args.gamma_theta,
            gamma_dist_mm=args.gamma_dist,
        )

        dt = 1.0 / float(args.hz)
        t_last = 0.0

        while True:
            link.keepalive()

            st = link.get_state()
            if st is None:
                if link.rx_age() > 1.5:
                    raise RuntimeError("No SIP packets received (timeout)")
                time.sleep(0.02)
                continue

            if time.time() - t_last >= float(args.print_every):
                t_last = time.time()
                g = nav.goal()
                if g is None:
                    gtxt = "(done)"
                else:
                    gx, gy = g
                    gtxt = f"goal=({gx:.0f},{gy:.0f})"
                print(
                    f"[POSE] x={st.x_mm:7.0f} y={st.y_mm:7.0f} th={st.th_deg_wrap:6.1f} (360={st.th_deg_360:6.1f}) rawTh={st.th_raw:6d}"
                    f"  {gtxt}  theta_decoded={link.theta.decided_mode()}"
                )

            if nav.done():
                if not args.no_motion:
                    link.stop()
                print("[DONE] Waypoints complete.")
                return 0

            v_cmd, w_cmd, info = nav.step(st, v_cruise=args.v, w_max=args.w)

            if not args.no_motion:
                link.drive(v_cmd, w_cmd, v_limit=args.v, w_limit=args.w)
            # tiny sleep to keep a stable loop
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[STOP] KeyboardInterrupt -> stopping robot")
        try:
            link.stop()
        except Exception:
            pass
        return 0

    except Exception as e:
        print(f"[ERR] {type(e).__name__}: {e}")
        try:
            link.stop()
        except Exception:
            pass
        return 2

    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
