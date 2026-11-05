#!/usr/bin/env python3
"""sp51_bench — wake a BMW SP51 bench pack and open/close its contactors from a simple prompt.

For people NOT running a ZombieVerter. One command, no sub-commands:

    python3 sp51_bench.py                 # socketcan can0 @ 500 kbit/s: wake, then prompt

!!!  HV WARNING  !!!  'c' (close) puts ~380 V DC (>200 A capable) on the pack output terminals.
Bench only: HVIL closed, insulated HV wiring, a precharge-safe load, 12 V emergency disconnect in
reach. This script synthesises vehicle frames (0x12F KL15, 0x1A1, 0x10B, 0x328, ...): never run it
on a pack that is in a car.

GOLDEN RULE: keep this script running for the whole session. Once the pack has talked on CAN,
~1.2-1.8 s of bus silence latches the SME's CAN off and it will not come back on traffic
[CONFIRMED close_v6_run1 / wake_window_test]. So: power-cycle 12 V (off >= 5 s), start this script
straight away, and open/close as often as you like from the prompt. After you quit, power-cycle
12 V again before the next session.

Prompt keys (frames keep running the whole time):
  c / close   pre-checks + stable-0x112 gate, then the proven close (0x53A A->B->C->D + 0x10B CLOSE,
              ~7 s total incl. 5 s stability gate, no idle wait), then HOLD CLOSED until you open.
              Asks you to type CLOSE first (skip with --yes). A one-line status prints while it runs.
  o / open    0x10B OPEN + 0x53A A (also cancels a close that is still in progress).
  s / status  one-shot status panel (V, I, state, cells, limits, DTC count).
  d / dtc     read-only DTC read (UDS 19 02).
  q / quit    open if closed, stop TX, exit.        Ctrl-C / SIGTERM: same, immediately.
  h / ?       help.

Safety (all logic is sp51ctl's, imported, not copied): Ctrl-C, SIGTERM, any exception, repeated bus
TX errors, loss of 0x112 while closed, the pack leaving state 6 or a pack open request while closed
-> 0x10B OPEN + 0x53A A for 2 s, then TX stops and the script exits. UDS is the read-only allow-list
(10 03, 3E 80, 22, 19 02); 0x380 (VIN) is never transmitted. Tags: [CONFIRMED] seen on this pack,
[ANALYSIS]/[FALLBACK] layout from public maps; panel fields marked '?' are not confirmed.

Flags (all optional):
  --interface socketcan  --channel can0  --bitrate 500000      CAN adapter (python-can names)
  --yes                 do not ask for 'CLOSE' before each close
  --panel line|full|off  live view while closing/closed (default: compact one line every ~2 s)
  --log FILE.csv        append ~2 Hz panel rows while closing/closed
  --no-uds              no cell/SOC/DTC polling
  --dry-run --fake-pack LOG.csv   no hardware: python-can virtual bus + replay of a real SP51 log
  (tuning, leave alone unless you know why: --idle-seconds 0 --b-hold 2 --stable-seconds 5
   --close-timeout 5 --wake-timeout 3 --wake-retries 4 --wake-gap 1 --tx-log FILE.csv)

Requires python-can >= 4 (pip install python-can) and sp51ctl.py in the same folder.
"""
from __future__ import annotations

import argparse
import queue
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sp51ctl as S  # noqa: E402  (frame maths, Controller, Panel, FakePack: single source of truth)


class Cancelled(Exception):
    """User asked to open while a close/hold was running (not a fault)."""


class BenchController(S.Controller):
    """sp51ctl.Controller + a 'user wants OPEN' event that ends a close/hold at the next check()."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.open_ev = threading.Event()

    def log(self, *a):
        msg = " ".join(str(x) for x in a)
        if msg.startswith("HOLDING CLOSED"):
            msg = "HOLDING CLOSED — HV LIVE until you type o (Enter)"
        super().log(msg)

    def check(self):
        super().check()  # faults (signal, TX errors, ...) first: those exit the script
        if self.open_ev.is_set():
            raise Cancelled()


class LinePanel(S.Panel):
    """Compact view: one line when the phase/state changes, else every `period` s. Keeps
    sp51ctl.Panel's ~2 Hz CSV logging and low-rate read-only UDS polling."""

    def __init__(self, *a, period=2.0, **k):
        super().__init__(*a, **k)
        self.period, self.last_key, self.last_t = period, None, 0.0

    def render(self, note=""):
        p = self.ctl.pack
        key = (note, p.hv_state)
        now = time.monotonic()
        if key == self.last_key and now - self.last_t < self.period:
            return
        self.last_key, self.last_t = key, now
        with p.lock:
            age = p.age_unlocked(0x112)
            print(f"  [{note}] {p.state_name():<9} {S.f(p.v, '{:.1f}', ' V')}  "
                  f"{S.f(p.i, '{:+.1f}', ' A')}  0x112 age {S.f(age, '{:.2f}', ' s')}  "
                  f"open-req {S.f(p.open_req, '{:d}')}", flush=True)


HELP = ("keys: c=close (and hold)  o=open  s=status  d=DTCs  q=quit   "
        "(frames keep running; Ctrl-C = OPEN + exit)")


def parse(argv=None):
    ap = argparse.ArgumentParser(
        prog="sp51_bench", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Prompt keys: c=close+hold, o=open, s=status, d=DTCs, q=quit.\n"
               "Golden rule: keep it running; power-cycle 12 V between sessions.\n" + S.HV_BANNER)
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--yes", action="store_true", help="skip the 'type CLOSE' confirmation")
    ap.add_argument("--panel", choices=["line", "full", "off"], default="line",
                    help="live view while closing/closed (default: one compact line)")
    ap.add_argument("--hz", type=float, default=2.0, help="panel/log rate while closing/closed")
    ap.add_argument("--log", help="append panel rows to this CSV while closing/closed")
    ap.add_argument("--no-uds", action="store_true", help="no cell/SOC/DTC polling")
    ap.add_argument("--uds-period", type=float, default=5.0)
    ap.add_argument("--dtc-mask", type=lambda s: int(s, 0), default=0xFF)
    ap.add_argument("--dry-run", action="store_true", help="python-can virtual bus, no hardware")
    ap.add_argument("--virtual-channel", default="sp51")
    ap.add_argument("--fake-pack", help="dry-run: replay pack frames from this SavvyCAN log")
    ap.add_argument("--fake-offset", type=float, default=0.0)
    ap.add_argument("--fake-needs-restart", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--tx-log", help="write everything we transmitted as SavvyCAN CSV on exit")
    ap.add_argument("--wake-timeout", type=float, default=3.0)
    ap.add_argument("--wake-gap", type=float, default=1.0)
    ap.add_argument("--wake-retries", type=int, default=4)
    ap.add_argument("--idle-seconds", type=float, default=0.0)
    ap.add_argument("--b-hold", type=float, default=2.0)
    ap.add_argument("--stable-seconds", type=float, default=5.0)
    ap.add_argument("--close-timeout", type=float, default=5.0)
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    print(S.HV_BANNER)
    S._load_names()
    bus = S.make_bus(a)  # exits with a pip hint if python-can is missing
    fake = None
    if a.dry_run and a.fake_pack:
        fbus = S.can.Bus(interface="virtual", channel=a.virtual_channel, receive_own_messages=False)
        fake = S.FakePack(fbus, a.fake_pack, a.fake_offset, a.fake_needs_restart)
    tx_log = [] if a.tx_log else None
    ctl = BenchController(bus, tx_log=tx_log)
    uds_p = 0 if a.no_uds else a.uds_period
    if a.panel == "full":
        panel = S.Panel(ctl, a.hz, a.log, uds_p, tty=False)
    elif a.panel == "line":
        panel = LinePanel(ctl, a.hz, a.log, uds_p, tty=False)
    else:
        panel = S.Panel(ctl, a.hz, a.log, uds_p, tty=False)
        panel.render = lambda note="": None

    def on_sig(signum, _frm):
        ctl.abort_reason = f"signal {signal.Signals(signum).name}"

    signal.signal(signal.SIGTERM, on_sig)
    signal.signal(signal.SIGINT, on_sig)

    # stdin on a daemon thread so the main loop can watch faults/signals every 100 ms
    lines: queue.Queue = queue.Queue()

    def reader():
        for ln in sys.stdin:
            lines.put(ln.strip())
        lines.put(None)  # EOF -> quit

    threading.Thread(target=reader, daemon=True).start()

    def read_line(prompt):
        print(prompt, end="", flush=True)
        while True:
            ctl.check_fault()
            try:
                return lines.get(timeout=0.1)
            except queue.Empty:
                pass

    worker = {"t": None, "fault": None}

    def close_job():
        try:
            ok = ctl.close(a.idle_seconds, a.b_hold, a.close_timeout, 0, panel, False, a.stable_seconds)
            if not ok:
                ctl.log("close not completed — contactors OPEN, frames still running")
        except Cancelled:
            ctl.log("open requested")
            ctl.do_open()
        except Exception as e:  # Abort or anything else: OPEN on the bus NOW, main loop exits
            ctl.cy.set(S.CMD_OPEN, "A")
            worker["fault"] = e
        finally:
            ctl.open_ev.clear()
            if not worker["fault"]:
                print("sp51> ", end="", flush=True)

    def busy():
        return worker["t"] is not None and worker["t"].is_alive()

    ctl.check_fault = lambda: S.Controller.check(ctl)  # faults only, never Cancelled
    rc = 0
    lost_warned = False
    try:
        if fake:
            fake.start()
        if not ctl.wake(a.wake_timeout, a.wake_gap, a.wake_retries):
            ctl.log("no 0x112 after all wake attempts — see the message above. Usual fix: switch 12 V "
                    "OFF (>= 5 s), ON, and start this script again within a few seconds.")
            return 2
        t_end = time.monotonic() + 2.0  # let 0x1FA (state) arrive so the first report is meaningful
        while ctl.pack.hv_state is None and time.monotonic() < t_end:
            ctl.check_fault()
            time.sleep(0.05)
        ctl.log(f"pack awake: state {ctl.pack.state_name()}, frames running. Keep this open.")
        print(HELP)
        while True:
            if worker["fault"]:
                raise worker["fault"]
            if not busy() and not ctl.pack.alive(1.0):
                if not lost_warned:
                    ctl.log("WARNING: 0x112 gone — pack dropped CAN. Frames still running; "
                            "q, then power-cycle 12 V and restart.")
                    lost_warned = True
            else:
                lost_warned = False
            ctl.check_fault()
            try:
                ln = lines.get(timeout=0.1)
            except queue.Empty:
                continue
            if ln is None:
                ctl.log("stdin closed — quitting")
                break
            cmd = ln.lower().split()[0] if ln.split() else ""
            if not cmd:
                print("sp51> ", end="", flush=True)
                continue
            if cmd in ("q", "quit", "exit"):
                break
            if cmd in ("h", "help", "?"):
                print(HELP)
            elif cmd in ("s", "status"):
                if not a.no_uds and not busy():
                    try:
                        ctl.poll_cells()
                    except Exception:
                        pass
                for line in S.panel_lines(ctl.pack, "status"):
                    print(line)
                print(f"  {'close/hold running' if busy() else 'idle'}; ('?' = layout not confirmed "
                      f"on this pack)")
            elif cmd in ("d", "dtc"):
                ctl.uds([0x10, 0x03])
                d = ctl.read_dtcs(a.dtc_mask)
                if d is None:
                    print("  no 59 02 reply")
                else:
                    for x in d:
                        print(f"  {x['dtc']}  {x['status']:02X} {','.join(x['bits']):<28} {x['name']}")
                    print(f"  {len(d)} DTCs, {sum(1 for x in d if x['status'] & 1)} testFailed now")
            elif cmd in ("c", "close"):
                if busy():
                    print("  already closing/closed — 'o' to open")
                else:
                    ok = a.yes or read_line("HV WILL BE LIVE. Type CLOSE to continue: ") == "CLOSE"
                    if ok:
                        ctl.open_ev.clear()
                        worker["t"] = threading.Thread(target=close_job, daemon=True)
                        worker["t"].start()
                        continue  # prompt is re-printed when the close job ends
                    print("  not closing")
            elif cmd in ("o", "open"):
                if busy():
                    ctl.open_ev.set()
                    worker["t"].join(10)
                    continue
                ctl.do_open()
            else:
                print(f"  ? {HELP}")
            print("sp51> ", end="", flush=True)
    except S.Abort as e:
        ctl.log(f"ABORT: {e}")
        rc = 5
    except Exception as e:  # noqa
        ctl.log(f"EXCEPTION: {type(e).__name__}: {e}")
        rc = 6
    finally:
        ctl.cy.set(S.CMD_OPEN, "A")  # OPEN on the bus immediately, before anything else
        ctl.open_ev.set()
        if worker["t"] is not None:
            worker["t"].join(5)
        if ctl.tx_on.is_set() or ctl.closed_by_us:
            ctl.abort_reason = None
            try:
                ctl.safe_open("exit")
            except Exception as e:  # noqa
                print("safe_open failed:", e)
        had_app = ctl.pack.any_app
        ctl.shutdown()
        if had_app:
            print("NOTE: TX stopped. The SME drops CAN ~1.5 s after the bus goes quiet and will not "
                  "come back on traffic: power-cycle 12 V (off >= 5 s) before the next session.")
        if fake:
            fake.stop_ev.set()
            fake.join(1)
            fake.bus.shutdown()
        if a.tx_log:
            t0 = tx_log[0][0] if tx_log else 0
            S.write_savvy_csv(a.tx_log, [(ts - t0, c, d) for ts, c, d in tx_log])
        if panel.log_f:
            panel.log_f.close()
        bus.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
