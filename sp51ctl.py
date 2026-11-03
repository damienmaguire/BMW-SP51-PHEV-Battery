#!/usr/bin/env python3
"""sp51ctl — bench controller for a BMW SP51 Gen5 PHEV pack (Preh EA443 SME, SWFL 00003e6f).

!!!  HV WARNING  !!!  `close` energises the pack output terminals (~380 V DC, >200 A capable).
Only use on a bench with safe, insulated HV wiring, HVIL closed, a precharge-capable load and
an emergency 12 V disconnect within reach. You are responsible for what is connected.

Bench only: this tool synthesises vehicle frames (0x12F KL15, 0x1A1, 0x328, 0x330, ...). In the
F11 those come from CAS/BDC/KOMBI/DSC — do NOT run it (or let a ZombieVerter do this) in the car.

Commands
  wake     start cyclic frames, wait for 0x112; retry with the CAN restart pattern (stop, silence,
           restart with fresh counters) — the pack can be LV-awake yet CAN-application-asleep.
  status   wake (or --listen-only) and show the live panel (~2 Hz).
  close    wake, pre-check (0x112 alive, 0x1FA b1==0x45, no open request), idle, 0x53A A->B->C->D +
           0x10B CLOSE; wait for HV state 6; hold (--hold-seconds, 0 = until Ctrl-C); then OPEN.
  open     send 0x10B OPEN + 0x53A A for a few seconds, wait for state 4.
  dtc      read-only 19 02 <mask>, decoded with names and status bits.
  powercycle  run the --power-off-cmd/--power-on-cmd relay hooks, then wake (start CAN early).
  shell    interactive prompt (status/close/open/dtc/quit) with frames kept running.

Safety: Ctrl-C, SIGTERM, any exception, repeated bus TX errors, loss of 0x112 or a pack open
request while closed -> 0x10B OPEN + 0x53A A is sent for OPEN_FLUSH_S, then TX stops.
UDS: read-only allow-list only (10 03, 3E 80, 22, 19 02). No SecurityAccess, writes, routines,
IO control, ECUReset or ClearDTC. 0x380 (VIN) is never transmitted.

Tags: [CONFIRMED] seen on this pack, [ANALYSIS]/[FALLBACK] layout from public maps. Panel fields
not confirmed on this pack are shown with '?'.

Requires: python-can>=4 (pip install python-can). Optional: none (cantools not needed).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    import can  # python-can
except ImportError:  # allow import for --help / unit tests of pure functions
    can = None

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

# ----------------------------------------------------------------------------- frame builders
def _crc_table(poly=0x1D):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


CRC_TABLE = _crc_table()  # SAE-J1850 poly 0x1D, no final XOR [CONFIRMED AS4 table + SP51 frames]


def crc8(data, init):
    c = init & 0xFF
    for b in data[1:]:
        c = CRC_TABLE[c ^ b]
    return c


def with_crc(data, init):
    d = list(data)
    d[0] = crc8(d, init)
    return d


INIT_12F, INIT_10B, INIT_1A1 = 0x60, 0x3F, 0x77
KL15_TAIL = [0x8A, 0xDD, 0xF1, 0x05, 0x30, 0x01]
FRAME_53A = {
    "A": [0x40, 0x3A, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00],
    "B": [0x40, 0x3A, 0x04, 0x00, 0x00, 0x80, 0x01, 0x00],
    "C": [0x40, 0x3A, 0x04, 0x00, 0x00, 0x84, 0x01, 0x00],
    "D": [0x40, 0x3A, 0x04, 0x00, 0x00, 0x84, 0x00, 0x00],
}
FRAME_3A0 = [0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
FRAME_3CA = [0xB7, 0x60, 0x01, 0x0F, 0x0F, 0x31, 0xFF, 0xFF]
FRAME_2CA = [0x6E, 0x6F]
FRAME_3E8 = [0xF1, 0xFF]
FRAME_510 = [0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x10]
FRAME_512 = [0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x12]
FRAME_330 = [0xA0, 0x86, 0x01, 0x06, 0x00, 0x0D, 0xBD, 0x07]
SEED_328_SECONDS, SEED_328_DAYS = 243785948, 9244
CMD_OPEN, CMD_CLOSE = 0x0, 0x1
NEVER_TX = {0x380}
PACK_TX_IDS = {0x112, 0x1FA, 0x239, 0x295, 0x2A5, 0x2F5, 0x40D, 0x423, 0x430, 0x431, 0x432,
               0x4AD, 0x4B0, 0x587}
REQ_ID, RSP_ID, ECU, TESTER = 0x6F1, 0x607, 0x07, 0xF1


def b12f(c):
    return with_crc([0, 0x20 | (c % 15)] + KL15_TAIL, INIT_12F)


def b10b(cmd, c):
    return with_crc([0, ((cmd & 0xF) << 4) | (c % 15), 0xFC], INIT_10B)


def b1a1(c):
    return with_crc([0, 0xC0 | (c % 15), 0x00, 0x00, 0x81], INIT_1A1)


class Cyclic:
    """Deterministic vehicle-frame scheduler. tick(t_ms) returns [(offset_us, id, data)] for one
    10 ms tick, byte- and order-identical to scripts/gen_sp51_close.py variant v4."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cmd = CMD_OPEN
        self.s53a = "A"
        self.reset()

    def reset(self):
        with self.lock:
            self.c12f = self.c10b = self.c1a1 = 0
            self.sec, self.days = SEED_328_SECONDS, SEED_328_DAYS

    def set(self, cmd=None, s53a=None):
        with self.lock:
            if cmd is not None:
                self.cmd = cmd
            if s53a is not None:
                self.s53a = s53a

    def tick(self, t_ms):
        out = []
        with self.lock:
            if t_ms % 100 == 0:
                out.append((0, 0x12F, b12f(self.c12f)))
                self.c12f = (self.c12f + 1) % 15
            if t_ms % 20 == 0:
                out.append((1, 0x10B, b10b(self.cmd, self.c10b)))
                self.c10b = (self.c10b + 1) % 15
                out.append((12, 0x1A1, b1a1(self.c1a1)))
                self.c1a1 = (self.c1a1 + 1) % 15
            if t_ms % 200 == 0:
                out.append((2, 0x53A, list(FRAME_53A[self.s53a])))
            if t_ms % 1000 == 0:
                s, d = self.sec, self.days
                out.append((3, 0x328, [s & 0xFF, s >> 8 & 0xFF, s >> 16 & 0xFF, s >> 24 & 0xFF,
                                       d & 0xFF, d >> 8 & 0xFF]))
                self.sec += 1
                if self.sec % 86400 == 0:
                    self.days += 1
                out += [(4, 0x3A0, list(FRAME_3A0)), (5, 0x3CA, list(FRAME_3CA)),
                        (6, 0x2CA, list(FRAME_2CA)), (7, 0x3E8, list(FRAME_3E8)),
                        (8, 0x510, list(FRAME_510)), (9, 0x512, list(FRAME_512)),
                        (11, 0x330, list(FRAME_330))]
        out.sort(key=lambda e: (e[0], e[1]))
        return out


def v4_phase(t_s):
    """The proven v4 close timeline (relative to TX start)."""
    if t_s < 20.0 or t_s >= 35.0:
        return CMD_OPEN, "A"
    r = t_s - 20.0
    if r < 2.0:
        return CMD_OPEN, "B"
    if r < 2.2:
        return CMD_CLOSE, "C"
    return CMD_CLOSE, "D"


def v7_phase(t_s):
    """v7: continuous traffic, 25 s OPEN/A idle, B 25-27, CLOSE+C 27, D 27.2, OPEN+A 40, end 50."""
    return v4_phase(t_s - 5.0) if t_s >= 5.0 else (CMD_OPEN, "A")


def simulate(duration_s, phase_fn=v4_phase, start_us=0):
    """Offline schedule for tests: [(ts_us, id, data)] for a phase function."""
    cy = Cyclic()
    rows = []
    for t_ms in range(0, int(round(duration_s * 1000)), 10):
        cmd, s = phase_fn(t_ms / 1000)
        cy.set(cmd, s)
        for off, cid, d in cy.tick(t_ms):
            rows.append((start_us + t_ms * 1000 + off, cid, d))
    return rows


# ----------------------------------------------------------------------------- pack state
DTC_NAMES = {}


def _load_names():
    p = ROOT / "analysis" / "sp51_cad_public_names.json"
    try:
        DTC_NAMES.update({k.upper(): v for k, v in json.loads(p.read_text())["codes"].items()})
    except Exception:
        pass
    try:
        sys.path.insert(0, str(ROOT / "scripts"))
        import decode_sp51_diag as dd  # noqa
        for k, v in dd.CAD_MSG_NAMES.items():
            key = (f"{k:06X}" if isinstance(k, int) else str(k).upper())
            DTC_NAMES.setdefault(key, v)
    except Exception:
        pass


STATUS_BITS = [(0x01, "TF"), (0x02, "TFTOC"), (0x04, "PDTC"), (0x08, "CDTC"), (0x10, "TNCSLC"),
               (0x20, "TFSLC"), (0x40, "TNCTOC"), (0x80, "WIR")]
HV_STATE = {4: "OPEN", 5: "PRECHARGE", 6: "CLOSED"}


@dataclass
class Pack:
    lock: threading.Lock = field(default_factory=threading.Lock)
    last: dict = field(default_factory=dict)  # id -> (mono_t, data)
    first_112: float | None = None
    v: float | None = None
    i: float | None = None
    v_link: float | None = None
    open_req: int | None = None
    hv_state: int | None = None
    b1_1fa: int | None = None
    t_min: int | None = None
    t_max: int | None = None
    lim: dict = field(default_factory=dict)
    cells: dict = field(default_factory=dict)
    soc: float | None = None
    dtc_active: int | None = None
    dtcs: list = field(default_factory=list)
    any_app: bool = False            # any pack application frame seen this session
    stable_since: float | None = None  # 0x112 continuously present (no gap > 0.1 s) since
    n_1fa_45: int = 0                # consecutive 0x1FA frames with b1 == 0x45

    def feed(self, cid, d, t=None):
        t = time.monotonic() if t is None else t
        with self.lock:
            prev = self.last.get(cid)
            self.last[cid] = (t, bytes(d))
            if cid in PACK_TX_IDS:
                self.any_app = True
            if cid == 0x112 and len(d) >= 8:
                if prev is None or t - prev[0] > 0.1 or self.stable_since is None:
                    self.stable_since = t
                if self.first_112 is None:
                    self.first_112 = t
                self.i = ((d[0] | d[1] << 8) - 8192) * 0.1
                self.v = (d[2] | d[3] << 8) * 0.1
                self.open_req = d[5] >> 6
                self.v_link = d[7] * 4
            elif cid == 0x1FA and len(d) >= 8:
                self.b1_1fa = d[1]
                self.n_1fa_45 = self.n_1fa_45 + 1 if d[1] == 0x45 else 0
                self.hv_state = d[1] >> 4
                self.t_min, self.t_max = d[6] - 50, d[7] - 50
            elif cid == 0x2F5 and len(d) >= 8:
                u = lambda k: d[k] | d[k + 1] << 8  # noqa
                self.lim.update(u_chg=u(0) * 0.1, i_chg=u(2) * 0.1 - 819.2, u_dch=u(4) * 0.1,
                                i_dch=u(6) * 0.1)
            elif cid == 0x40D and len(d) >= 8:
                u = lambda k: d[k] | d[k + 1] << 8  # noqa
                self.lim.update(p_chg_st=u(0) * 3e-3, p_dch_st=u(2) * 3e-3)

    def age(self, cid):
        with self.lock:
            x = self.last.get(cid)
        return None if x is None else time.monotonic() - x[0]

    def alive(self, max_age=0.5):
        a = self.age(0x112)
        return a is not None and a < max_age

    def stable_for(self):
        """Seconds 0x112 has been continuously present (0 if not alive)."""
        if not self.alive(0.1) or self.stable_since is None:
            return 0.0
        return time.monotonic() - self.stable_since

    def state_name(self):
        return HV_STATE.get(self.hv_state, "?" if self.hv_state is None else f"0x{self.hv_state:X}")


def f(x, fmt="{:.1f}", q=""):
    return "--" if x is None else fmt.format(x) + q


# 0x112 b5[7:6] open request: 1 = no request [CONFIRMED normal: 0x7F in every SP51 log, states 4/5/6,
# incl. all successful closes]; other values = pack wants the contactors open [FALLBACK BE enum].
OPEN_REQ_TXT = {1: "(none, normal)", 0: "(0 ?)", 2: "(PACK WANTS OPEN)", 3: "(invalid?)"}


def panel_lines(p: Pack, note=""):
    with p.lock:
        pw = None if p.v is None or p.i is None else p.v * p.i / 1000
        c = p.cells
        L = p.lim
        lines = [
            f"SP51 {time.strftime('%H:%M:%S')}  {note}",
            f" contactors : {p.state_name():<10} (0x1FA b1 {f(p.b1_1fa, '{:02X}')})   0x112 age "
            f"{f(p.age_unlocked(0x112), '{:.2f}', ' s')}",
            f" pack V     : {f(p.v, '{:.1f}', ' V')}   link V? {f(p.v_link, '{:.0f}', ' V')}",
            f" current    : {f(p.i, '{:+.1f}', ' A')}   power {f(pw, '{:+.2f}', ' kW')}",
            f" SOC?       : {f(p.soc, '{:.1f}', ' %')} (cell-SOC avg E59A)",
            f" cells      : min {f(c.get('min'), '{:d}')} max {f(c.get('max'), '{:d}')} "
            f"delta {f(None if not c else c['max'] - c['min'], '{:d}', ' mV')}  (E554, {c.get('n', 0)} cells)",
            f" temps?     : min {f(p.t_min, '{:d}')} max {f(p.t_max, '{:d}', ' °C')} (0x1FA b6/b7)",
            f" limits     : U chg max {f(L.get('u_chg'), '{:.1f}', ' V')}  U dch min "
            f"{f(L.get('u_dch'), '{:.1f}', ' V')}",
            f"              I chg max? {f(L.get('i_chg'), '{:.1f}', ' A')}  I dch max? "
            f"{f(L.get('i_dch'), '{:.1f}', ' A')}",
            f"              P chg? {f(L.get('p_chg_st'), '{:.1f}', ' kW')}  P dch? "
            f"{f(L.get('p_dch_st'), '{:.1f}', ' kW')} (0x40D short-term)",
            f" DTC active : {f(p.dtc_active, '{:d}')} (status bit0 TF, last 19 02)   open req "
            f"{f(p.open_req, '{:d}')} {OPEN_REQ_TXT.get(p.open_req, '')}",
        ]
    return lines


def _age_unlocked(self, cid):
    x = self.last.get(cid)
    return None if x is None else time.monotonic() - x[0]


Pack.age_unlocked = _age_unlocked

LOG_FIELDS = ["time", "state", "b1_1fa", "v", "v_link", "i", "p_kw", "soc", "cell_min", "cell_max",
              "t_min", "t_max", "u_chg", "u_dch", "i_chg", "i_dch", "p_chg", "p_dch", "dtc_active"]


def log_row(p: Pack):
    with p.lock:
        pw = None if p.v is None or p.i is None else round(p.v * p.i / 1000, 3)
        return [time.strftime("%Y-%m-%d %H:%M:%S"), p.state_name(), p.b1_1fa, p.v, p.v_link, p.i, pw,
                p.soc, p.cells.get("min"), p.cells.get("max"), p.t_min, p.t_max, p.lim.get("u_chg"),
                p.lim.get("u_dch"), p.lim.get("i_chg"), p.lim.get("i_dch"), p.lim.get("p_chg_st"),
                p.lim.get("p_dch_st"), p.dtc_active]


# ----------------------------------------------------------------------------- UDS (read-only)
ALLOWED = {(0x10, 0x03), (0x10, 0x01), (0x3E, 0x80), (0x3E, 0x00), (0x19, 0x02)}


def uds_allowed(payload):
    if not payload:
        return False
    if payload[0] == 0x22:
        return True
    return (payload[0], payload[1] if len(payload) > 1 else None) in ALLOWED


def decode_dtcs(resp):
    """resp = 59 02 <avail> (DTC3 status)*"""
    out = []
    if len(resp) < 3 or resp[0] != 0x59:
        return out
    b = resp[3:]
    for k in range(0, len(b) - 3, 4):
        code = f"{b[k]:02X}{b[k+1]:02X}{b[k+2]:02X}"
        st = b[k + 3]
        out.append({"dtc": code, "status": st, "bits": [n for m, n in STATUS_BITS if st & m],
                    "name": DTC_NAMES.get(code, "")})
    return out


# ----------------------------------------------------------------------------- controller
class Abort(Exception):
    pass


class Controller:
    OPEN_FLUSH_S = 2.0

    def __init__(self, bus, tx_log=None, verbose=True):
        self.bus = bus
        self.cy = Cyclic()
        self.pack = Pack()
        self.tx_on = threading.Event()
        self.stop_ev = threading.Event()
        self.abort_reason = None
        self.tx_errors = 0
        self.uds_q = queue.Queue()
        self.uds_lock = threading.Lock()
        self.tx_log = tx_log
        self.tx_log_lock = threading.Lock()
        self.t0 = None
        self.verbose = verbose
        self.closed_by_us = False
        self.events = []
        self.last_diag = None
        self.t_power_on = None
        self.rx_t = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_t = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_t.start()
        self.tx_t.start()

    def log(self, *a):
        self.events.append(time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a))
        del self.events[:-6]
        if self.verbose:
            print(time.strftime("%H:%M:%S"), *a, flush=True)

    # --- TX
    def _send(self, cid, data):
        assert cid not in NEVER_TX
        msg = can.Message(arbitration_id=cid, data=bytes(data), is_extended_id=False)
        try:
            self.bus.send(msg, timeout=0.05)
            self.tx_errors = 0
        except Exception as e:  # can.CanError, OSError (ENOBUFS), ...
            self.tx_errors += 1
            if self.tx_errors >= 25 and not self.abort_reason:
                self.abort_reason = f"bus TX error x{self.tx_errors}: {e}"
        if self.tx_log is not None:
            with self.tx_log_lock:
                ts = int((time.monotonic() - (self.t0 or 0)) * 1e6)
                self.tx_log.append((ts, cid, list(data)))

    def _tx_loop(self):
        t_ms = 0
        nxt = None
        while not self.stop_ev.is_set():
            if not self.tx_on.wait(0.05):
                nxt = None
                continue
            if nxt is None:
                nxt, t_ms = time.monotonic(), 0
            for _, cid, d in self.cy.tick(t_ms):
                self._send(cid, d)
            t_ms += 10
            nxt += 0.010
            dt = nxt - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            elif dt < -0.5:  # fell far behind (suspended?) — resync, keep counters
                nxt = time.monotonic()

    def start_tx(self, fresh=True):
        if fresh:
            self.cy.reset()
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.tx_on.set()

    def stop_tx(self):
        self.tx_on.clear()
        time.sleep(0.03)

    # --- RX
    def _rx_loop(self):
        while not self.stop_ev.is_set():
            try:
                m = self.bus.recv(0.1)
            except Exception as e:
                if not self.abort_reason:
                    self.abort_reason = f"bus RX error: {e}"
                time.sleep(0.1)
                continue
            if m is None or m.is_extended_id or m.is_error_frame:
                continue
            if m.arbitration_id == RSP_ID:
                self.uds_q.put(list(m.data))
            else:
                self.pack.feed(m.arbitration_id, list(m.data))

    # --- UDS ISO-TP (BMW extended addressing, b0 = target/source)
    def uds(self, payload, timeout=1.5):
        if not uds_allowed(payload):
            raise ValueError(f"UDS service {payload[:2]} not in read-only allow-list")
        assert len(payload) <= 6
        with self.uds_lock:
            while not self.uds_q.empty():
                self.uds_q.get_nowait()
            self._send(REQ_ID, [ECU, len(payload)] + list(payload) + [0] * (6 - len(payload)))
            deadline = time.monotonic() + timeout
            buf, need, sn = [], None, 1
            while time.monotonic() < deadline:
                try:
                    d = self.uds_q.get(timeout=0.05)
                except queue.Empty:
                    continue
                if d[0] != TESTER:
                    continue
                pci = d[1] >> 4
                if pci == 0:
                    n = d[1] & 0xF
                    r = d[2:2 + n]
                    if len(r) >= 3 and r[0] == 0x7F and r[2] == 0x78:
                        deadline = time.monotonic() + 5
                        continue
                    return r
                if pci == 1:
                    need = (d[1] & 0xF) << 8 | d[2]
                    buf = d[3:8]
                    sn = 1
                    self._send(REQ_ID, [ECU, 0x30, 0, 0, 0, 0, 0, 0])
                elif pci == 2 and need is not None:
                    if d[1] & 0xF != sn % 16:
                        return None
                    sn += 1
                    buf += d[2:8]
                    if len(buf) >= need:
                        return buf[:need]
            return None

    def read_did(self, did):
        r = self.uds([0x22, did >> 8, did & 0xFF])
        if r and r[0] == 0x62 and (r[1] << 8 | r[2]) == did:
            return r[3:]
        return None

    def poll_cells(self):
        r = self.read_did(0xE554)
        if r:
            c = [r[k] << 8 | r[k + 1] for k in range(0, len(r) - 1, 2)]
            real = [x for x in c if x != 0x7FFF]
            if real:
                with self.pack.lock:
                    self.pack.cells = {"min": min(real), "max": max(real), "n": len(real)}
        r = self.read_did(0xE59A)
        if r:
            s = [r[k] << 8 | r[k + 1] for k in range(0, len(r) - 1, 2)]
            s = [x for x in s if x not in (0x1228, 0x7FFF, 0xFFFF)]
            if s:
                with self.pack.lock:
                    self.pack.soc = sum(s) / len(s) / 100

    def read_dtcs(self, mask=0xFF):
        r = self.uds([0x19, 0x02, mask], timeout=3)
        if r is None or r[0] != 0x59:
            return None
        d = decode_dtcs(r)
        with self.pack.lock:
            self.pack.dtcs = d
            self.pack.dtc_active = sum(1 for x in d if x["status"] & 1)
        return d

    # --- safety
    def check(self):
        if self.abort_reason:
            raise Abort(self.abort_reason)

    def safe_open(self, why=""):
        self.log(f"SAFE OPEN ({why}) -> 0x10B OPEN + 0x53A A for {self.OPEN_FLUSH_S:.0f} s")
        self.cy.set(CMD_OPEN, "A")
        if not self.tx_on.is_set():
            self.start_tx(fresh=False)
        t_end = time.monotonic() + self.OPEN_FLUSH_S
        while time.monotonic() < t_end:
            time.sleep(0.05)
        self.closed_by_us = False
        self.log(f"state now {self.pack.state_name()}")

    def shutdown(self):
        self.stop_tx()
        self.stop_ev.set()
        self.rx_t.join(1)
        self.tx_t.join(1)

    # --- commands
    def power_cycle(self, off_cmd, on_cmd, off_s=5.0):
        """Optional relay/GPIO hook (user-supplied shell commands, e.g. `gpioset gpiochip0 17=0`).
        Bus is silent while off; TX restarts right after power-on (start CAN early — owner
        hypothesis: SME wants traffic within a window after power-up)."""
        import subprocess
        self.stop_tx()
        self.log(f"12 V OFF via hook: {off_cmd}")
        subprocess.run(off_cmd, shell=True, check=True, timeout=10)
        time.sleep(off_s)
        self.pack.first_112 = None
        self.pack.any_app = False
        self.pack.stable_since = None
        self.pack.n_1fa_45 = 0
        self.log(f"12 V ON via hook: {on_cmd}")
        subprocess.run(on_cmd, shell=True, check=True, timeout=10)
        self.t_power_on = time.monotonic()
        self.cy.set(CMD_OPEN, "A")
        self.start_tx(fresh=True)

    def wake(self, wake_timeout=3.0, gap=2.0, retries=4, power_off_cmd=None, power_on_cmd=None,
             power_cycles=0, power_off_s=5.0):
        """Start TX; if no 0x112, stop -> silence -> restart (fresh counters) [ANALYSIS loop3/wake1].
        If still mute: diagnose (UDS read-only) and, if hooks are given, power-cycle 12 V."""
        for pc in range(power_cycles + 1):
            if self._wake_once(wake_timeout, gap, retries):
                if getattr(self, "t_power_on", None) and self.pack.first_112:
                    self.log(f"0x112 {self.pack.first_112 - self.t_power_on:.2f} s after 12 V ON")
                return True
            kind, msg = self.diagnose_no_wake()
            self.log(f"NO WAKE [{kind}]: {msg}")
            self.last_diag = (kind, msg)
            if pc < power_cycles and power_off_cmd and power_on_cmd and kind != "hvil-open?":
                self.power_cycle(power_off_cmd, power_on_cmd, power_off_s)
                retries = max(retries, 1)
            else:
                if kind == "com-latched" and not (power_off_cmd and power_on_cmd):
                    self.log(">>> POWER-CYCLE 12 V NOW (off >=5 s), then re-run within a few seconds. "
                             "(or pass --power-off-cmd/--power-on-cmd/--auto-powercycle 1)")
                break
        return False

    def _wake_once(self, wake_timeout=3.0, gap=2.0, retries=4):
        if self.pack.alive():
            if not self.tx_on.is_set():
                self.start_tx(fresh=True)
            return True
        for k in range(retries + 1):
            self.cy.set(CMD_OPEN, "A")
            self.pack.first_112 = None
            t_start = time.monotonic()
            self.start_tx(fresh=True)
            self.log(f"wake attempt {k+1}: cyclic TX started, waiting {wake_timeout:.1f} s for 0x112")
            while time.monotonic() - t_start < wake_timeout:
                self.check()
                if self.pack.alive(0.3):
                    self.log(f"0x112 up after {self.pack.first_112 - t_start:.3f} s "
                             f"(state {self.pack.state_name()})")
                    return True
                time.sleep(0.02)
            if self.pack.any_app:
                # v6 lesson (close_v6_run1): once the SME has been CAN-active, bus silence of
                # ~1.2-1.8 s makes it drop COM and it does not come back on traffic -> never go
                # silent here; keep TX running and tell the user to power-cycle.
                self.log("pack application TX was seen and has stopped — NOT going silent "
                         "(silence after activity latches COM off); keep TX, power-cycle 12 V")
                return False
            if k < retries:
                self.log(f"no 0x112 at all — CAN restart: bus silent {gap:.1f} s, then fresh start")
                self.stop_tx()
                time.sleep(gap)
        return False

    def diagnose_no_wake(self):
        """Classify a pack that does not send 0x112 (read-only UDS). Signatures:
        - no UDS reply              -> SME unpowered / CAN wiring / bitrate          [CONFIRMED probes]
        - UDS OK, CAD4B0+CAD9A6 TF=0 -> COM RX alive, app TX off = HVIL OPEN          [CONFIRMED run2]
        - UDS OK, CAD4B0/CAD9A6 TF=1 while we send 0x1A1/0x10B -> SME application COM
          latched OFF ('LV-awake, CAN-asleep'); only a 12 V power cycle cleared it   [CONFIRMED
          wake_window_test 2026-11-03; mechanism HYPOTHESIS]"""
        if not self.tx_on.is_set():
            self.start_tx(fresh=True)
        time.sleep(1.5)  # give RX deadline monitors time with our frames present
        r = self.uds([0x10, 0x03])
        if not r:
            return "no-uds", "SME does not answer UDS: check 12 V (pin 1/2), 30C (pin 3), CAN H/L, 500k"
        d = self.read_dtcs(0x0C) or self.read_dtcs(0xFF) or []
        st = {x["dtc"]: x["status"] for x in d}
        tf = [c for c in ("CAD4B0", "CAD9A6") if st.get(c, 0) & 1]
        op_new = st.get("21F4AF") == 0x68
        if tf:
            return "com-latched", (f"SME answers UDS but its application COM is OFF ({'/'.join(tf)} "
                                   "testFailed although 0x1A1/0x10B are on the bus). Power-cycle 12 V "
                                   "(KL30+30C off >=5 s), then start CAN traffic immediately.")
        return "hvil-open?", ("SME answers UDS and receives our frames, but sends no application TX: "
                              f"HVIL open signature (pins 7/8, HV connector).{' Fresh op cycle.' if op_new else ''}")

    def precheck(self, wait=3.0):
        t_end = time.monotonic() + wait
        while time.monotonic() < t_end and self.pack.age(0x1FA) is None:
            self.check()
            time.sleep(0.05)
        p = self.pack
        res = {
            "0x112 alive": p.alive(),
            "0x1FA seen": p.age(0x1FA) is not None and p.age(0x1FA) < 2.5,
            "0x1FA b1 == 0x45 (OPEN, LockErr=1, Precharge=1)": p.b1_1fa == 0x45,
            "0x112 open request == 1": p.open_req == 1,
            "pack V > 200 V": (p.v or 0) > 200,
        }
        return all(res.values()), res

    def wait_stable(self, stable_s=5.0, timeout=15.0, panel=None):
        """Gate: 0x112 continuously present >= stable_s AND >= 3 consecutive 0x1FA == 0x45."""
        t_end = time.monotonic() + max(timeout, stable_s + 3)
        while time.monotonic() < t_end:
            self.check()
            if self.pack.stable_for() >= stable_s and self.pack.n_1fa_45 >= 3:
                self.log(f"stable: 0x112 continuous {self.pack.stable_for():.1f} s, "
                         f"0x1FA 0x45 x{self.pack.n_1fa_45}")
                return True
            if panel:
                panel.maybe("waiting for stable 0x112/0x1FA")
            time.sleep(0.05)
        self.log(f"NOT stable: 0x112 continuous {self.pack.stable_for():.1f} s, "
                 f"0x1FA 0x45 x{self.pack.n_1fa_45}")
        return False

    def close(self, idle_s=20.0, b_hold=2.0, close_timeout=5.0, hold_s=10.0, panel=None,
              force=False, stable_s=5.0):
        if not self.wait_stable(stable_s, panel=panel) and not force:
            self.log("0x112/0x1FA not stable — not closing (guide §3a)")
            return False
        ok, res = self.precheck()
        for k, v in res.items():
            self.log(f"  pre-check {'OK  ' if v else 'FAIL'} {k}")
        if not ok and not force:
            self.log("PRE-CHECK FAIL — not closing (HVIL? wake? see guide §10)")
            return False
        t_wake = time.monotonic()
        self.log(f"idle OPEN/A for {idle_s:.1f} s (v4 timeline)")
        self._wait(idle_s, panel, "idle", watch_closed=False, need_alive=True)
        self.cy.set(CMD_OPEN, "B")
        self.log("0x53A B")
        self._wait(b_hold, panel, "53A B", watch_closed=False, need_alive=True)
        ok, res = self.precheck(wait=0.5)
        if not ok and not force:
            self.cy.set(CMD_OPEN, "A")
            self.log(f"pre-close re-check FAIL {[k for k, v in res.items() if not v]} — no CLOSE sent")
            return False
        self.closed_by_us = True
        self.cy.set(CMD_CLOSE, "C")
        t_cmd = time.monotonic()
        self.log("0x10B CLOSE + 0x53A C")
        self._wait(0.2, panel, "close C", watch_closed=False)
        self.cy.set(CMD_CLOSE, "D")
        self.log("0x53A D — waiting for state 6")
        t5 = None
        while True:
            self.check()
            if self.pack.hv_state == 5 and t5 is None:
                t5 = time.monotonic()
                self.log(f"state 5 PRECHARGE +{t5 - t_cmd:.3f} s")
            if self.pack.hv_state == 6:
                self.log(f"state 6 CLOSED +{time.monotonic() - t_cmd:.3f} s after CLOSE "
                         f"({time.monotonic() - t_wake:.1f} s after pre-check)")
                break
            if time.monotonic() - t_cmd > close_timeout:
                self.log(f"close timeout {close_timeout:.1f} s (state {self.pack.state_name()})")
                self.safe_open("close timeout")
                return False
            if panel:
                panel.maybe("closing")
            time.sleep(0.02)
        self.log("HOLDING CLOSED — HV LIVE" + ("" if hold_s > 0 else " until Ctrl-C"))
        self._wait(hold_s if hold_s > 0 else 1e9, panel, "CLOSED — HV LIVE", watch_closed=True)
        self.do_open()
        return True

    def _wait(self, secs, panel, note, watch_closed, need_alive=False):
        t_end = time.monotonic() + secs
        lost = None
        while time.monotonic() < t_end:
            self.check()
            if need_alive and not self.pack.alive(0.5):
                raise Abort(f"0x112 lost during '{note}' — pack dropped CAN; power-cycle 12 V")
            if panel:
                panel.maybe(note)
            if watch_closed:
                if not self.pack.alive(0.5):
                    raise Abort("0x112 lost while closed")
                if self.pack.open_req not in (None, 1):
                    raise Abort(f"pack open request {self.pack.open_req}")
                if self.pack.hv_state not in (6,):
                    lost = lost or time.monotonic()
                    if time.monotonic() - lost > 1.0:
                        raise Abort(f"pack left CLOSED (state {self.pack.state_name()})")
                else:
                    lost = None
            time.sleep(0.02)

    def do_open(self, wait=3.0):
        self.cy.set(CMD_OPEN, "A")
        if not self.tx_on.is_set():
            self.start_tx(fresh=True)
        t = time.monotonic()
        self.log("0x10B OPEN + 0x53A A")
        while time.monotonic() - t < wait:
            if self.pack.hv_state == 4:
                self.log(f"state 4 OPEN +{time.monotonic() - t:.3f} s")
                break
            time.sleep(0.02)
        else:
            self.log(f"no state 4 within {wait:.1f} s (state {self.pack.state_name()})")
        self.closed_by_us = False
        time.sleep(0.5)


class Panel:
    """~2 Hz panel; ANSI redraw on a TTY, plain blocks otherwise. Optional CSV log. Low-rate
    read-only UDS polling (cells/SOC every `uds_period` s, DTC count every 4x that)."""

    def __init__(self, ctl: Controller, hz=2.0, log_path=None, uds_period=5.0, tty=None):
        self.ctl, self.dt, self.next = ctl, 1.0 / hz, 0.0
        self.tty = sys.stdout.isatty() if tty is None else tty
        self.log_f = None
        if log_path:
            new = not Path(log_path).exists()
            self.log_f = open(log_path, "a", newline="")
            self.w = csv.writer(self.log_f)
            if new:
                self.w.writerow(LOG_FIELDS)
        self.uds_period = uds_period
        self.uds_next = time.monotonic() + 1.0
        self.dtc_next = time.monotonic() + 2.0
        self.uds_busy = False
        self.frames = 0

    def _uds_worker(self, what):
        try:
            if what == "cells":
                self.ctl.poll_cells()
            else:
                self.ctl.read_dtcs(0xFF)
        except Exception:
            pass
        finally:
            self.uds_busy = False

    def maybe(self, note=""):
        now = time.monotonic()
        if self.uds_period and not self.uds_busy and self.ctl.pack.alive():
            what = None
            if now >= self.dtc_next:
                what, self.dtc_next = "dtc", now + 4 * self.uds_period
            elif now >= self.uds_next:
                what, self.uds_next = "cells", now + self.uds_period
            if what:
                self.uds_busy = True
                threading.Thread(target=self._uds_worker, args=(what,), daemon=True).start()
        if now < self.next:
            return
        self.next = now + self.dt
        lines = panel_lines(self.ctl.pack, note)
        if self.tty:
            lines += [" events:"] + ["   " + e for e in self.ctl.events]
            sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lines) + "\n  ('?' = layout not confirmed on "
                             "this pack)  Ctrl-C = OPEN + stop\n")
        else:
            sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
        self.frames += 1
        if self.log_f:
            self.w.writerow(log_row(self.ctl.pack))
            self.log_f.flush()


# ----------------------------------------------------------------------------- fake pack (dry-run)
class FakePack(threading.Thread):
    """Replays pack TX frames of a real SavvyCAN log on a virtual bus (open-loop, real timing),
    and answers UDS requests with the response frames captured after the same request in the log."""

    def __init__(self, bus, log_path, t_offset_s=0.0, needs_restart=False, start_file=None):
        super().__init__(daemon=True)
        self.start_file = start_file  # test hook: frames start only once this file exists
        self.needs_restart = needs_restart  # emulate 'LV-awake, CAN-asleep': start only after
        # traffic -> >=1 s silence -> traffic again (owner observation 2026-11-03)
        self.bus, self.stop_ev = bus, threading.Event()
        rows = list(csv.DictReader(open(log_path)))
        self.frames = []
        self.uds = {}
        last_req = None
        for r in rows:
            if not r.get("ID") or not r.get("LEN"):
                continue
            cid = int(r["ID"], 16)
            d = [int(r[f"D{k}"], 16) for k in range(1, int(r["LEN"]) + 1)]
            ts = int(r["Time Stamp"]) / 1e6
            if cid == REQ_ID:
                if d[1] >> 4 == 0:
                    last_req = tuple(d[2:2 + (d[1] & 0xF)])
                    self.uds.setdefault(last_req, [])
                    self._cur = last_req
                    self._fresh = self.uds[last_req] == []
                continue
            if cid == RSP_ID:
                if last_req is not None and self._fresh:
                    self.uds[last_req].append(d)
                continue
            if cid not in PACK_TX_IDS:
                continue
            self.frames.append((ts, cid, d))
        t0 = self.frames[0][0] if self.frames else 0
        self.frames = [(t - t0 - t_offset_s, c, d) for t, c, d in self.frames if t - t0 >= t_offset_s]
        self.req_q = queue.Queue()

    def run(self):
        start = None
        k = 0
        pend = None
        seen = last = None
        armed = not self.needs_restart
        while not self.stop_ev.is_set():
            try:
                m = self.bus.recv(0.0)
            except Exception:
                m = None
            now = time.monotonic()
            if m is not None and start is None:
                if not armed and seen is not None and now - last >= 1.0:
                    armed = True
                seen = seen or now
                last = now
                if armed and (not self.start_file or os.path.exists(self.start_file)):
                    start = now
            if m is not None and m.arbitration_id == REQ_ID:
                d = list(m.data)
                if d[1] >> 4 == 0:
                    req = tuple(d[2:2 + (d[1] & 0xF)])
                    resp = self.uds.get(req) or ([[TESTER, 0x03, 0x7F, req[0], 0x31, 0, 0, 0]]
                                                 if req else [])
                    if resp:
                        self.bus.send(can.Message(arbitration_id=RSP_ID, data=bytes(resp[0]),
                                                  is_extended_id=False))
                        pend = resp[1:] if resp[0][1] >> 4 == 1 else None
                elif d[1] == 0x30 and pend:
                    for cf in pend:
                        self.bus.send(can.Message(arbitration_id=RSP_ID, data=bytes(cf),
                                                  is_extended_id=False))
                    pend = None
            if start is None:
                time.sleep(0.001)
                continue
            t = now - start
            while k < len(self.frames) and self.frames[k][0] <= t:
                _, cid, d = self.frames[k]
                self.bus.send(can.Message(arbitration_id=cid, data=bytes(d), is_extended_id=False))
                k += 1
            time.sleep(0.001)


# ----------------------------------------------------------------------------- CLI
def write_savvy_csv(path, rows):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["Time Stamp", "ID", "Extended", "Dir", "Bus", "LEN"] + [f"D{k}" for k in range(1, 9)])
        for ts, cid, d in rows:
            w.writerow([ts, f"{cid:08X}", "false", "Tx", 0, len(d)] + [f"{x:02X}" for x in d]
                       + [""] * (8 - len(d)))


HV_BANNER = """
=====================================================================
  HV WARNING: 'close' connects ~380 V DC to the pack output terminals.
  Bench only. HVIL closed, insulated wiring, load precharge-safe,
  12 V emergency disconnect in reach. Ctrl-C sends OPEN and stops.
=====================================================================
"""


def make_bus(a):
    if can is None:
        sys.exit("python-can not installed: pip install python-can")
    if a.dry_run:
        return can.Bus(interface="virtual", channel=a.virtual_channel, receive_own_messages=False)
    return can.Bus(interface=a.interface, channel=a.channel, bitrate=a.bitrate)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="sp51ctl", description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=HV_BANNER)
    ap.add_argument("command", choices=["wake", "status", "close", "open", "dtc", "shell", "powercycle"])
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--dry-run", action="store_true", help="python-can virtual bus, no hardware")
    ap.add_argument("--virtual-channel", default="sp51")
    ap.add_argument("--fake-pack", help="dry-run: replay pack frames from this SavvyCAN log")
    ap.add_argument("--fake-offset", type=float, default=0.0, help="skip seconds of the fake log")
    ap.add_argument("--fake-needs-restart", action="store_true",
                    help="dry-run: fake pack only starts after traffic, >=1 s silence, traffic")
    ap.add_argument("--fake-start-file", help="dry-run test hook: fake pack TX starts once file exists")
    ap.add_argument("--tx-log", help="write everything we transmitted as SavvyCAN CSV")
    ap.add_argument("--log", help="append panel rows to this CSV (~2 Hz)")
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--no-uds", action="store_true", help="no UDS polling in the panel")
    ap.add_argument("--uds-period", type=float, default=5.0)
    ap.add_argument("--listen-only", action="store_true", help="status/dtc: do not transmit cyclic frames")
    ap.add_argument("--wake-timeout", type=float, default=3.0)
    ap.add_argument("--wake-gap", type=float, default=1.0,
                    help="silence before a restart; only used if NO pack frame was ever seen")
    ap.add_argument("--wake-retries", type=int, default=4)
    ap.add_argument("--power-off-cmd", help="relay/GPIO hook: shell command that switches 12 V OFF")
    ap.add_argument("--power-on-cmd", help="relay/GPIO hook: shell command that switches 12 V ON")
    ap.add_argument("--power-off-seconds", type=float, default=5.0)
    ap.add_argument("--auto-powercycle", type=int, default=0,
                    help="if still mute and COM looks latched, power-cycle via the hooks N times")
    ap.add_argument("--idle-seconds", type=float, default=20.0)
    ap.add_argument("--stable-seconds", type=float, default=5.0,
                    help="close gate: 0x112 continuous this long + 3x 0x1FA 0x45")
    ap.add_argument("--close-timeout", type=float, default=5.0)
    ap.add_argument("--hold-seconds", type=float, default=10.0, help="0 = hold until Ctrl-C")
    ap.add_argument("--status-seconds", type=float, default=0.0, help="status: 0 = until Ctrl-C")
    ap.add_argument("--dtc-mask", type=lambda s: int(s, 0), default=0xFF)
    ap.add_argument("--force", action="store_true", help="close even if pre-check fails (not advised)")
    ap.add_argument("--yes", action="store_true", help="skip the HV confirmation prompt")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    _load_names()

    bus = make_bus(a)
    fake = None
    if a.dry_run and a.fake_pack:
        fbus = can.Bus(interface="virtual", channel=a.virtual_channel, receive_own_messages=False)
        fake = FakePack(fbus, a.fake_pack, a.fake_offset, a.fake_needs_restart,
                        a.fake_start_file)
    tx_log = [] if a.tx_log else None
    ctl = Controller(bus, tx_log=tx_log)

    def on_sig(signum, _frm):
        ctl.abort_reason = f"signal {signal.Signals(signum).name}"

    signal.signal(signal.SIGTERM, on_sig)
    signal.signal(signal.SIGINT, on_sig)
    rc = 0
    if a.command == "close":
        print(HV_BANNER)
        if not a.yes and not a.dry_run:
            if input("Type CLOSE to continue: ").strip() != "CLOSE":
                print("aborted")
                return 1
    try:
        if fake:
            fake.start()
        panel = Panel(ctl, a.hz, a.log, 0 if a.no_uds else a.uds_period) \
            if a.command in ("status", "close") else None
        if a.command in ("status", "dtc") and a.listen_only:
            pass
        elif a.command == "powercycle":
            if not (a.power_off_cmd and a.power_on_cmd):
                ctl.log("powercycle needs --power-off-cmd and --power-on-cmd (relay/GPIO hook). "
                        "Manual: switch 12 V off >=5 s, on, then run `sp51ctl wake` straight away.")
                return 2
            ctl.power_cycle(a.power_off_cmd, a.power_on_cmd, a.power_off_seconds)
            ok = ctl.wake(a.wake_timeout, a.wake_gap, a.wake_retries)
            return 0 if ok else 2
        elif a.command != "open":
            if not ctl.wake(a.wake_timeout, a.wake_gap, a.wake_retries, a.power_off_cmd,
                            a.power_on_cmd, a.auto_powercycle, a.power_off_seconds):
                ctl.log("no 0x112 after all wake attempts — see message above (guide §10a)")
                rc = 2
                return rc
        if a.command == "wake":
            ctl.log("awake; frames stop on exit (Ctrl-C).")
            while True:
                ctl.check()
                time.sleep(0.1)
        elif a.command == "status":
            t_end = time.monotonic() + (a.status_seconds or 1e9)
            while time.monotonic() < t_end:
                ctl.check()
                panel.maybe("status")
                time.sleep(0.02)
        elif a.command == "close":
            rc = 0 if ctl.close(a.idle_seconds, 2.0, a.close_timeout, a.hold_seconds, panel, a.force,
                                a.stable_seconds) else 3
        elif a.command == "open":
            ctl.do_open()
        elif a.command == "dtc":
            ctl.uds([0x10, 0x03])
            d = ctl.read_dtcs(a.dtc_mask)
            if d is None:
                ctl.log("no 59 02 reply")
                rc = 4
            elif a.json:
                print(json.dumps(d, indent=1))
            else:
                for x in d:
                    print(f"  {x['dtc']}  {x['status']:02X} {','.join(x['bits']):<28} {x['name']}")
                print(f"  {len(d)} DTCs, {sum(1 for x in d if x['status'] & 1)} testFailed now")
        elif a.command == "shell":
            rc = shell(ctl, a)
    except Abort as e:
        ctl.log(f"ABORT: {e}")
        rc = 5
    except Exception as e:  # noqa
        ctl.log(f"EXCEPTION: {type(e).__name__}: {e}")
        rc = 6
    finally:
        if ctl.tx_on.is_set() or ctl.closed_by_us:
            ctl.abort_reason = None
            try:
                ctl.safe_open("exit")
            except Exception as e:  # noqa
                print("safe_open failed:", e)
        had_app = ctl.pack.any_app
        ctl.shutdown()
        if had_app and not a.dry_run:
            print("NOTE: TX stopped. The SME drops CAN ~1.5 s after the bus goes quiet and will not "
                  "come back on traffic: power-cycle 12 V before the next session (or use `shell` "
                  "to keep frames running between closes).")
        if fake:
            fake.stop_ev.set()
        if a.tx_log:
            t0 = tx_log[0][0] if tx_log else 0
            write_savvy_csv(a.tx_log, [(ts - t0, c, d) for ts, c, d in tx_log])
        if panel and panel.log_f:
            panel.log_f.close()
        bus.shutdown()
    return rc


def shell(ctl, a):
    print("sp51 shell: status [s] | close | open | dtc | quit   (frames keep running)")
    panel = Panel(ctl, a.hz, a.log, 0 if a.no_uds else a.uds_period, tty=False)
    while True:
        ctl.check()
        try:
            cmd = input("sp51> ").strip().split()
        except EOFError:
            return 0
        if not cmd:
            continue
        if cmd[0] in ("q", "quit", "exit"):
            return 0
        if cmd[0] == "status":
            for line in panel_lines(ctl.pack):
                print(line)
        elif cmd[0] == "close":
            if input("HV LIVE. Type CLOSE: ").strip() == "CLOSE":
                ctl.close(a.idle_seconds, 2.0, a.close_timeout, a.hold_seconds, panel,
                          stable_s=a.stable_seconds)
        elif cmd[0] == "open":
            ctl.do_open()
        elif cmd[0] == "dtc":
            for x in ctl.read_dtcs(a.dtc_mask) or []:
                print(f"  {x['dtc']}  {x['status']:02X} {','.join(x['bits']):<28} {x['name']}")
        else:
            print("?")


if __name__ == "__main__":
    sys.exit(main())
