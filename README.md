
# SP51 Gen5 PHEV pack: how to make the contactors close (bench guide)

<img width="2304" height="4096" alt="SP51_plate" src="https://github.com/user-attachments/assets/ec4931dc-685e-4973-af20-065b7228a8b7" />



**Pack:** BMW SP51 Gen5 PHEV HV battery, SME = Preh EA443 AS4 (SWFL 00003e6f), owned unit.
**Goal:** close the main contactors from a bench CAN source (SavvyCAN, SocketCAN or ZombieVerter), hold them closed, and open them again cleanly.
**Status (2026-11-03):** closed **7 times** with the v4 playback (run 1, run 3, loop 3 ×4, wake-window test). Run 2 failed because HVIL was open. Wake failures are now explained by a **CAN-wake vs LV-wake** effect (§3a): the SME can be LV-awake but CAN-asleep, and sometimes only a 12 V power cycle fixes it. **v6 failed (run 1): its 2 s silent gaps latched COM off after the pack had woken — use v7 (continuous traffic), never pause the bus.**
**Companion files:**
- `analysis/sp51.dbc`: all known messages and signals.
- `analysis/sp51_close_v4_result.md`: run-by-run analysis.
- `analysis/sp51_dtc_notes.md`: DTC sources.
- `playback/sp51_close_attempt.csv`: the v4 playback that worked.
- `playback/sp51_close_v7_continuous.csv`: **recommended**: v4 frames, continuous from t = 0, close at 27 s (§6.1).
- `playback/sp51_close_v6_reliable.csv`: **DO NOT USE** (its silent gaps latch COM off, §6.2).
- `tools/sp51ctl.py`: SocketCAN CLI (wake / status / close / open / dtc / shell / powercycle), §15.

### Confidence tags used throughout

| Tag | Meaning |
|---|---|
| **[CONFIRMED SP51]** | Seen on this pack in our logs. |
| **[CONFIRMED AS4]** | Read out of the EA443 AS4 firmware image. |
| **[CONFIRMED F11]** | Seen in the owner's F11 vehicle logs. |
| **[ANALYSIS]** | Our inference from data. |
| **[HYPOTHESIS]** | Plausible but untested. |
| **[FALLBACK]** | From a public map (Battery-Emulator "BE", carcoding, forum). Used only where the firmware and logs say nothing. |

> No VIN, keys, SecurityAccess, coding, RoutineControl, IO control, ECUReset or ClearDTC appear anywhere in this procedure. Diagnostics are **read-only** (`10 03`, `3E 80`, `22`, `19 02`). 0x380 (VIN) is never transmitted.

---

## 0. Short version

1. LV: 12 V on KL30 **and** 30C, GND, wake line, **HVIL loop closed**. Connect CAN-H/L to a 500 kbit/s classic CAN with 120 Ω termination at each end.
2. Get the CAN tool ready first. Then **12 V off ≥ 5 s → on**, and **start the vehicle frames within about 5 s** (proven ≤ ~14 s) [ANALYSIS, §3a]. Don't wait 60 s (old advice, withdrawn).
3. Within about 0.12 s the pack must start sending **0x112 every 20 ms** and **0x1FA = `F5 45 …`** (state 4 = OPEN). **Never let the bus go quiet once 0x112 has appeared**: ~1.2–1.8 s of silence after that makes the SME drop CAN until a 12 V cycle [CONFIRMED v6 run 1]. If 0x112 never appeared at all: stop, ~1 s silence, restart once. Still nothing → read DTCs (§3a): CAD4B0/CAD9A6 = 27 → power-cycle 12 V; both 26 → HVIL open.
4. Keep the vehicle frames running, with no gaps, for at least 20 s (v7: 25 s), with 0x10B = OPEN and 0x53A = state A.
5. Set 0x53A to state **B** and hold it for 2 s. Then switch 0x10B to **CLOSE** and 0x53A to **C**, and 200 ms later to **D**.
6. 0x1FA goes 4 → **5** (precharge, about 70 ms after the request) → **6** (closed): about **1.8–1.9 s** after the request on the first close after a 12 V power-up, **0.56–0.62 s** on later closes in the same power cycle [CONFIRMED SP51 3/3 and 4/4]. 0x112 b7 rises to about 0x5F (≈380 V).
7. To open: 0x10B = OPEN and 0x53A = A. 0x1FA goes back to 4 within about 80 ms.

---

## 1. Hardware

### 1.1 LV connector (A332*11B, owner's diagram) [user diagram; ANALYSIS for pin roles]

| Pin | Signal | Bench connection | Notes |
|---|---|---|---|
| 1 | KL30 | +12 V (fused, 5 A) | Permanent supply. |
| 2 | KL31 | GND | Common ground with the CAN interface. |
| 3 | 30C | +12 V | **Needed for application TX.** Without it the SME draws idle current and stays silent [ANALYSIS BE/Damien; matches early wake tests]. |
| 4 | ETH_WUP / wake | +12 V | Wake assist. |
| 7 / 8 | HVIL loop | Bridge 7↔8 (or route through your HVIL chain) | **Open HVIL = no application CAN at all.** UDS still answers [CONFIRMED SP51, HVIL A/B and run 2]. |
| 11/12 or 13/14 | K-CAN5 H/L | CAN interface | 500 kbit/s classic (not CAN-FD) [CONFIRMED SP51]. **Verify which pair with a scope before relying on it.** |

- **Termination:** 120 Ω at each end of the bus, so about 60 Ω measured H↔L with power off. It is not known whether the pack has internal termination. Measure it first.
- **12 V supply:** a bench PSU at 12.5–13.5 V with at least 3 A, or a battery. The contactor coils draw more than the idle current, so set the current limit at 3 A or higher.

### 1.2 CAN interface

- SavvyCAN with GVRET/CANable at 500 kbit/s. Our playback CSVs are SavvyCAN "GVRET" format with timestamps in µs, `Dir=Tx`, bus 0.
- SocketCAN (Linux):

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 500000 restart-ms 100   # classic CAN, auto-recover from bus-off
sudo ip link set can0 up
ip -details -statistics link show can0     # check state ERROR-ACTIVE, no bus-off counters
candump -ta -l can0                        # log everything to candump-*.log, start BEFORE 12 V
candump -ta can0,112:7FF,1FA:7FF           # live view of the two gate frames
cansend can0 10B#CD01FC                    # single test frame (0x10B OPEN, alive 1)
```

**Always start logging before you apply 12 V, and keep it running through failed attempts too.** The run-3 log captured only the successful attempt, so we can't yet say why the earlier ones failed (§9).

---

## 2. HV safety

- **Once closed, the HV output terminals carry the full pack voltage: about 380 V DC** (380.8 V at about 19 % SOC) [CONFIRMED SP51 0x112 U_Pack]. This is lethal. Treat the output as live whenever 12 V is on and frames are running.
- The pack output is **floating** (isolated from chassis). Don't ground either pole: that defeats the isolation monitor and is a shock path.
- **Measuring the output without breaking HVIL:**
  - On our bench the HVIL loop is closed at LV pins 7↔8, so the HV output connector can be probed without touching HVIL [ANALYSIS: the run-1/3 closes happened with HVIL bridged on the LV side].
  - If your HVIL chain also runs through the HV connector (an HVIL bridge in the mating plug), use a **mating plug or adaptor** that keeps that bridge, and take your probe leads off its HV pins.
  - Don't pull the HV plug to get access: that opens HVIL, which **mutes the pack**, so nothing will close.
  - Fit the meter **before** 12 V goes on. Use a CAT III 600 V or better meter with insulated clip leads, and don't touch anything during the close window.
- **Load and capacitor:**
  - Runs 1 and 3 had **nothing** on the output. Precharge still completed (state 5 → 6 in 1.86 s) and no precharge DTC set [CONFIRMED SP51].
  - The ≈220 V pulse decays with τ ≈ 45 ms (§7), and the output falls to 0 V within about 55 ms of opening [CONFIRMED SP51 0x112 b7].
  - Recommended for the next test [ANALYSIS, engineering advice]: add a **small DC-link capacitor** (100–470 µF, at least 500 V DC rating) with a **permanent bleed resistor across it** (for example 100 kΩ, 3 W: 1.4 W at 380 V; τ = 22 s with 220 µF, so 5τ ≈ 2 min). This gives a realistic precharge and proves the voltage reading.
  - **With a capacitor fitted, the output stays charged after the contactors open.** The pack doesn't discharge your capacitor. Wait 5τ and **measure 0 V** before touching it.
  - Don't fit a capacitor much larger than an inverter's (a few hundred µF to about 1 mF). The pack's precharge resistor and timeout are sized for that [HYPOTHESIS].
- **Abort:**
  - Stop the frames, or send 0x10B OPEN. The pack opens within about 80 ms [CONFIRMED SP51].
  - Removing 12 V also drops the contactors, which are coil-driven. That is a hard abort, but it may log DTCs and stress the contacts under load.
  - With no load there is no current to interrupt.

---

## 3. Power-up and settle

| Step | Time | Why |
|---|---|---|
| CAN tool ready | before 12 V | Logging running, playback / `sp51ctl` loaded. |
| 12 V off | **≥ 5 s** | Reboots the SME (21F4AF shows 68 = new operation cycle afterwards) [CONFIRMED SP51 wake-window test]. |
| 12 V on (KL30 + 30C) | t₀ | |
| Start frames | **within ~5 s of t₀** (proven ≤ ~14 s) | Owner hypothesis, supported by the wake-window test: the SME wants CAN traffic within a window after power-up, otherwise application COM stays off [ANALYSIS; window length not measured]. The old "settle 60 s" advice is withdrawn. |
| First 0x112 | +0.11–0.12 s after our first frame | [CONFIRMED SP51 runs 1, 3, wake-window seg 6] |
| Keep frames running | the whole session | Once the pack has sent 0x112, a **silent bus of ~2 s** latched COM off (v6 run 1: app TX stopped 1.2–1.76 s after our last frame and never came back) [CONFIRMED]. 0.16 s loop gaps were fine (loop 3) [CONFIRMED]. The owner's ~5 min idle before the refused wake is consistent. The earlier '≤ 32 s recovers' note is **withdrawn**: that case never had 0x112 before the silence. |

## 3a. CAN wake vs LV wake

**LV-awake** = 12 V + 30C present, SME draws ~200 mA and **answers UDS**. **CAN-awake** = the SME runs its application COM: it **sends** 0x112/0x1FA/… and **receives** 0x10B/0x1A1/…. You need both. LV-awake alone has been seen in three forms:

| Form | Evidence | Signature | Fix |
|---|---|---|---|
| **First onset ignored** | loop 3 (owner): first playback no 0x112, abort + restart → 0x112 at once, 4 closes | no 0x112 on the first try only | stop, ~1 s silence, restart — **only if no pack frame was ever seen** (`sp51ctl wake` does this) [ANALYSIS] |
| **COM latched off** | wake-window test: 5 restarts over 101 s, all mute; 12 V cycle → wake in 0.12 s | UDS OK, **CAD4B0 and CAD9A6 = 27** (0x1A1/0x10B "missing") **although we are sending them**; 21F4AF still 28 (no reset) [CONFIRMED SP51] | **12 V off ≥ 5 s**, then start traffic within ~5 s |
| **COM dropped by a gap** | v6 run 1: 0x112 at 0.09 s, normal state 4; our 2 s gap at 4.96 s → all pack app TX stops at 6.73 s, never resumes (pulse 2 and v4 part ignored); UDS still answers | as COM latched: CAD4B0/CAD9A6 **27** while sending (v4 successes: 26); 21F4AF 68 (fresh cycle, so not 'left powered') [CONFIRMED SP51] | 12 V cycle; then continuous traffic (v7) |
| HVIL open (not a wake issue) | run 2, HVIL A/B | UDS OK, CAD4B0/CAD9A6 **26** (RX works), no app TX [CONFIRMED SP51] | close HVIL (pins 7/8) |

**Step order (use this every time):**
1. CAN interface up, logging on.
2. 12 V off ≥ 5 s, then on (KL30 + 30C).
3. Within ~5 s: start `playback/sp51_close_v7_continuous.csv` (original timing, not looped, **no pauses**), or `sp51ctl close`.
4. Pre-check (§4): 0x112 must appear within ~0.2 s and stay up. If it appears and then stops, or never appears → stop the file, read DTCs → table above; the next try needs a 12 V cycle.
5. Close window (§6), end OPEN.
6. When the file ends the bus goes quiet → the pack drops CAN within ~2 s. **Every new attempt = 12 V cycle first.** To close several times in one cycle, use `sp51ctl shell` (frames keep running) or a seamless loop.

**Why [HYPOTHESIS]:** with 30C holding the MCU in RUN there is no new wake-up event, so once ComM/NM has dropped to no-communication nothing restarts it short of an EcuM reboot. Pin 4 (wake line) has not been tested as an alternative. The SME's NM PDU 0x507 is in the AS4 TX table but has never been sent; the NM/ComM/EcuM timeouts were **not found** in the firmware (searched; see `sp51_close_v4_result.md`).

---

## 4. Pre-check (do this every attempt)

Within **1 s** of starting the frames you must see all of these:

| Check | Expected | Meaning if missing |
|---|---|---|
| 0x112 every 20 ms | e.g. `FF 1F E0 0E FF 7F 75 00` → I = −0.1 A, U = 380.8 V, b7 = 0 (output open) | No 0x112 means HVIL is open or the SME hasn't started its application (settle/restart, §9). **Don't continue.** |
| 0x1FA | `F5 45 45 7F 3A FF 39 3A` → b1 = **0x45**: state nibble **4 = OPEN**, lock_err = 1 (not active), precharge_locked = 1 (not active) | |
| 0x2F5, 0x40D, 0x239, 0x430/431/432, 0x2A5, 0x295 | present | Same as above. |
| UDS (optional) | `07 02 10 03` → `F1 06 50 03 00 32 01 F4` | UDS answering **without** 0x112 = HVIL open **or** COM latched off. Tell them apart with 19 02: CAD4B0/CAD9A6 = 26 → HVIL; = 27 while you send 0x1A1/0x10B → power-cycle 12 V (§3a). |

Decoder check of a log (§11): `decode_sp51_diag.py LOG --can-hvil --close-report` prints `PRE-CHECK FAIL` when there are zero pack application frames.

---
## 5. Frames to send (bench, v4, the version that closed)

All frames are 11-bit standard IDs on classic CAN at 500 kbit/s. Byte numbering is b0 = first data byte. Multi-byte fields are little-endian.

| ID | Name | DLC | Period | Payload | CRC/alive | Needed? |
|---|---|---|---|---|---|---|
| **0x10B** | SPEC_DCSW_HVSTO (contactor request) | 3 | **20 ms** | `CRC, (cmd<<4)\|alive, FC` | CRC init **0x3F**, alive 0..14 | **Yes**: the close command. In AS4 RX [CONFIRMED AS4]; close accepted [CONFIRMED SP51]. |
| **0x53A** | HV request state | 8 | **200 ms** | state A/B/C/D (§6) | none | **Yes** in v4: the A→B→C→D sequence worked [CONFIRMED SP51]. Whether it is strictly needed is not tested (v3 = 0x10B only, never run). In AS4 RX group 2 [CONFIRMED AS4]. |
| 0x12F | Terminal/KL15 | 8 | 100 ms | `CRC, 0x20\|alive, 8A DD F1 05 30 01` | CRC init **0x60**, alive 0..14 | Sent in every working run. **Not in the AS4 RX table** [CONFIRMED AS4], so it may only act as a bus-wake. |
| 0x1A1 | V_VEH | 5 | 20 ms | `CRC, 0xC0\|alive, 00 00 81` | CRC init **0x77**, alive 0..14 | Clears CAD4B0 [CONFIRMED SP51]. A close without it has not been tried. |
| 0x330 | KILOMETERSTAND | 8 | 1000 ms | `A0 86 01 06 00 0D BD 07` (100000 km placeholder) | none | Clears CAD558 [CONFIRMED SP51]. |
| 0x328 | RELATIVZEIT | 6 | 1000 ms | u32 seconds LE (+1 each frame), u16 days LE: `DC E0 87 0E 1C 24` first | none | Sent [FALLBACK BE]. |
| 0x2CA | A_TEMP | 2 | 1000 ms | `6E 6F` | none | [FALLBACK BE] |
| 0x3A0 | FZZSTD | 8 | 1000 ms | `FF FF CF FF FF FF FF FD` | none | [FALLBACK BE] |
| 0x3CA | FRC_RD_INFO | 8 | 1000 ms | `B7 60 01 0F 0F 31 FF FF` | none | [FALLBACK BE] |
| 0x3E8 | DIAG_OBD_ENGMG_EL | 2 | 1000 ms | `F1 FF` | none | [FALLBACK BE] |
| 0x510 | NM (F11 copy) | 8 | 1000 ms | `00 00 00 00 FF FF 00 10` | none | Possible wake trigger [HYPOTHESIS]. |
| 0x512 | NM (F11 copy) | 8 | 1000 ms | `00 00 00 00 FF FF 00 12` | none | Possible wake trigger [HYPOTHESIS]. |
| 0x6F1 | UDS request (optional) | 4–8 | ~150 ms | read-only only | ISO-TP | Monitoring only. |

**Never send** 0x380 (VIN). Don't send any SecurityAccess, write, routine, IO, reset or clear service.

### 5.1 CRC recipe (all E2E frames) [CONFIRMED AS4 poly table; per-ID init values as tagged]

- CRC-8 SAE-J1850 polynomial **0x1D**, table-driven, **no final XOR**.
- Start value = **per-ID init**.
- Computed over **b1 … b(DLC−1)** and stored in **b0**.
- Alive counter = low nibble of b1, counting **0,1,…,14,0** (modulo 15; 15 is never used).

```python
def crc8_tbl(poly=0x1D):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t
T = crc8_tbl()

def bmw_crc(data, init):          # data = full frame, b0 ignored
    c = init
    for b in data[1:]:
        c = T[c ^ b]
    return c

INIT = {0x12F: 0x60, 0x10B: 0x3F, 0x1A1: 0x77, 0x295: 0x45}   # 0x295 is pack TX (for checking)
```

```c
uint8_t bmw_crc(const uint8_t *d, uint8_t dlc, uint8_t init) {
    uint8_t c = init;
    for (uint8_t i = 1; i < dlc; i++) c = crc_tbl[c ^ d[i]];   /* crc_tbl = J1850 poly 0x1D */
    return c;
}
```

### 5.2 Worked examples (each matches a frame in the run-3 log)

**0x10B CLOSE, alive 5, init 0x3F.** Frame `?? 15 FC`.
- Step 1: T[0x3F ^ 0x15 = 0x2A] = 0x55.
- Step 2: T[0x55 ^ 0xFC = 0xA9] = **0x54**.
- Result: **`54 15 FC`** (sent at 21.994 s).

**0x10B OPEN, alive 1.** Frame `?? 01 FC`.
- Step 1: T[0x3F ^ 0x01 = 0x3E] = 0xEC.
- Step 2: T[0xEC ^ 0xFC = 0x10] = **0xCD**.
- Result: **`CD 01 FC`**.

**0x12F, alive 1, init 0x60.** Frame `?? 21 8A DD F1 05 30 01`.
- The chain runs T[60^21=41]=0E → T[0E^8A=84]=52 → T[52^DD=8F]=9D → T[9D^F1=6C]=08 → T[08^05=0D]=81 → T[81^30=B1]=71 → T[71^01=70]=**59**.
- Result: **`59 21 8A DD F1 05 30 01`**.

**0x1A1, alive 1, init 0x77.** Frame `?? C1 00 00 81`.
- The chain runs T[77^C1=B6]=22 → T[22^00]=BD → T[BD^00]=ED → T[ED^81=6C]=**08**.
- Result: **`08 C1 00 00 81`**.

**0x295 (pack TX, for checking your CRC code), init 0x45.** Frame `?? 50 FE FF 14 FF FF FF`.
- The chain runs T[45^50=15]=A4 → T[A4^FE=5A]=0C → T[0C^FF=F3]=58 → T[58^14=4C]=8F → T[8F^FF=70]=59 → T[59^FF=A6]=EF → T[EF^FF=10]=**CD**.
- Result: **`CD 50 FE FF 14 FF FF FF`**.

Full 15-frame alive cycles, which you can paste straight into a schedule:

- 0x10B CLOSE: `35 10 FC, 79 11 FC, AD 12 FC, E1 13 FC, 18 14 FC, 54 15 FC, 80 16 FC, CC 17 FC, 6F 18 FC, 23 19 FC, F7 1A FC, BB 1B FC, 42 1C FC, 0E 1D FC, DA 1E FC`
- 0x10B OPEN: `81 00 FC, CD 01 FC, 19 02 FC, 55 03 FC, AC 04 FC, E0 05 FC, 34 06 FC, 78 07 FC, DB 08 FC, 97 09 FC, 43 0A FC, 0F 0B FC, F6 0C FC, BA 0D FC, 6E 0E FC`
- 0x12F b0 for alive 0..14 (tail `8A DD F1 05 30 01`): `04 59 BE E3 6D 30 D7 8A D6 8B 6C 31 BF E2 05`
- 0x1A1 b0 for alive 0..14 (tail `00 00 81`): `95 08 B2 2F DB 46 FC 61 09 94 2E B3 47 DA 60`

**Keep the alive counter continuous when switching 0x10B between OPEN and CLOSE.** Only the high nibble changes. Our playback never reset the counter, and CAD9A7/9A8 (not-current/checksum) never set [CONFIRMED SP51].

---

## 6. Close sequence (exact v4 timing)

0x53A states [FALLBACK BE payloads; CONFIRMED SP51 to work]:

| State | Payload | Meaning [FALLBACK BE] |
|---|---|---|
| A | `40 3A 04 00 00 00 01 00` | idle / no HV request |
| B | `40 3A 04 00 00 80 01 00` | HV requested (announce) |
| C | `40 3A 04 00 00 84 01 00` | escalate (one 200 ms tick) |
| D | `40 3A 04 00 00 84 00 00` | steady HV on |

| t (s) | 0x10B | 0x53A | What happens on the pack [CONFIRMED SP51 run 1 / run 3] |
|---|---|---|---|
| 0.000 | OPEN (hi nibble 0) every 20 ms | A every 200 ms | First 0x112 at 0.110 / 0.118 s. 0x1FA `F5 45` (OPEN). |
| ~20.0 | OPEN | **B** | No change. |
| **21.994** | **CLOSE (hi nibble 1)** | **C** (same instant) | Close request. |
| 22.06–22.07 | CLOSE | C | **0x1FA b1 = 0x55 (state 5 PRECHARGE)**, event frame (+70 ms). |
| 22.394 | CLOSE | **D** | |
| 23.36–23.56 | CLOSE | D | 0x112 b7 pulse to 0x36–0x38 (≈216–224 V), decaying to 0 (τ ≈ 45 ms) [ANALYSIS: precharge-relay test pulse into an unloaded output]. |
| 23.58–23.83 | CLOSE | D | b7 rises to **0x5F** (≈380 V). One-frame markers on 0x239 b1/b3, 0x431 b5 A0→AF, 0x432 b0 03→FB, 0x40D b6. |
| **23.85–23.86** | CLOSE | D | **0x1FA b1 = 0x65 (state 6 CLOSED)**, event frame (**+1.86 s** after the request). |
| 23.91–23.92 | CLOSE | D | 0x587 `06 C4 09 FF…` then `06 00 00 …`. |
| 24.86, 34.87 | CLOSE | D | **0x423 `14 04 00 00 00 00 00 00` every 10 s, only while CLOSED.** |
| **35.000** | **OPEN** | **A** | Open request. |
| 35.07–35.08 | OPEN | A | **0x1FA b1 = 0x45 (state 4 OPEN)** (+72 to 80 ms). b7 → 0 by 35.13. |
| 35.21 | OPEN | A | DTC 21F4AF status 68 → 28 (the open/weld test completed, passed). |
| 37.41 | OPEN | A | 0x587 `06 32 00 FF…`. |
| 45.0 | stop | stop | The file ends OPEN. |

The two successful runs agree to within about 10 ms. The hold length (11 s) is our choice; nothing in the logs suggests a maximum.

### 6.1 v7 "continuous" playback (`playback/sp51_close_v7_continuous.csv`, 50 s) — recommended

| t (s) | What |
|---|---|
| 0–25 | full v4 vehicle set from the first frame, 0x10B OPEN, 0x53A A (no gaps; max inter-frame gap 20 ms) |
| 25–27 | 0x53A B |
| 27.0 | 0x10B CLOSE + 0x53A C; 27.2 D |
| 40.0 | 0x10B OPEN + 0x53A A; end at 50 s, OPEN |

- Same bytes as v4 with a 5 s longer idle (0x112 has ~25 s to be stable before the close). Read-only diag from 2 s (10 03, 3E 80, 22, 19 02); 0x380 never sent.
- 12 V off ≥ 5 s → on → start v7 within ~5 s, original timing, **not looped, never paused**. Watch 0x112 in the first second: absent or dropping → stop, read DTCs, 12 V cycle.
- After the file ends the pack drops CAN; power-cycle 12 V before the next run.
- Unit test: `scripts/test_close_v7.py` (generator reproducible, CRC/alive, no gap > 20 ms, first 0x12F at 0, close at 27.0 within window, ends OPEN, no 0x380, read-only SIDs, < 50 s). Generator: `scripts/gen_sp51_close_v7.py`.

### 6.2 v6 "reliable" playback — **DO NOT USE** (`playback/sp51_close_v6_reliable.csv`)

**Failed in v6 run 1 [CONFIRMED]:** the pack woke on pulse 1 and its 2 s silences then latched COM off, so the CLOSE at 36 s went to a mute pack. Kept only for reference.

| t (s) | What |
|---|---|
| 0–5 | pulse 1: full vehicle set, 0x10B OPEN, 0x53A A, counters from 0 |
| 5–7 | **bus silent** |
| 7–12 | pulse 2: same again (fresh counters) |
| 12–14 | **bus silent** |
| 14–59 | the v4 file **byte-identical**, shifted by 14 s: B at 34, CLOSE+C at 36, D at 36.2, OPEN+A at 49, ends OPEN at 59 |

- Covers the "first onset ignored" case (§3a); it **cannot** fix a latched SME, which needs a 12 V cycle first.
- Play it with **original timing** (the gaps must be real silence), **not looped**, within ~5 s of 12 V on.
- Contactors are only commanded in the v4 part. No UDS in the pulses; the v4 part has the same read-only diag (10 03, 3E 80, 22, 19 02).
- Unit test: `scripts/test_close_v6.py` (CRC/alive, silent gaps, ends OPEN, no 0x380, read-only services, < 60 s, main part identical to v4). Generator: `scripts/gen_sp51_close_v6.py`.

---

## 7. What success looks like

1. **0x1FA b1 high nibble: 4 → 5 → 6** (raw b1 `45 → 55 → 65`) [CONFIRMED SP51]. This is the primary indicator.
2. **0x112 b7 ≈ 0x5F** while closed. That is about 380 V at ≈4 V/bit [ANALYSIS; voltmeter cross-check still pending]. b7 is 0 when open.
3. **0x423** appears about 1 s after CLOSED, then every 10 s [CONFIRMED SP51].
4. **0x587 `06 …`** event after the close and after the open.
5. Audible contactor clunks (owner, run 1).
6. 0x112 OpenContactorReq/Instant/Fast stay **1 (not active)**. If one goes 2 (active), the pack wants to open [FALLBACK BE].

---

## 8. Opening and aborting

- **Normal open:** 0x10B high nibble 0 (keep alive counting) and 0x53A = A. Keep sending for at least 3 s so the pack logs the open cleanly. The 0x587 `06 32 00` came 2.4 s after the open.
- **Abort at any time:** send OPEN, or simply stop all frames. Loss of 0x10B (CAD9A6 "missing") should open the contactors, but **that is not tested**: we always opened explicitly. So **prefer an explicit OPEN**, then a 12 V cut as the hard backstop.
- **Pack-initiated open:** watch 0x112 b5[7:6] / b6[1:0] / b6[3:2] (open, instant-open and fast-open requests, BE enums) [FALLBACK BE].

---
## 9. Power limits: what the pack reports

**Short answer:** yes. The pack sends **voltage and current limits in 0x2F5** and **charge/discharge available-power figures in 0x40D**.
- Both IDs are in the SME's own vehicle TX PDU table [CONFIRMED AS4: `analysis/as4_tx_pdu_table.csv`, idx 5 and 9].
- The firmware table gives the **ID list only, not the bit packing**, so the layouts are Battery-Emulator's [FALLBACK]. Our logs check them for plausibility [ANALYSIS].
- All values below were taken cold (about 7–8 °C cell temperature, 0x1FA / DID DDC0) at about 19 % cell SOC, with no load.
- Open and closed values are the same, apart from the one-frame switching markers.

| Signal (DBC name) | ID / bytes | Scaling | Run 1 | Run 3 | Confidence |
|---|---|---|---|---|---|
| Max charge voltage `U_ChargeMax` | 0x2F5 b0-1 u16 LE | ×0.1 V | 453.1 V | 453.1–453.2 V | **[ANALYSIS] strong**: equals UDS DID E54C (453.1 V). Layout [FALLBACK BE]. |
| Max charge current `I_ChargeMax` | 0x2F5 b2-3 | ×0.1 − 819.2 A | 202.0 → 202.8 A | 204.0 → 204.4 A | [FALLBACK BE] scaling. The value fits 0x40D (about 77 kW at 380 V vs 70 kW). |
| Min discharge voltage `U_DischargeMin` | 0x2F5 b4-5 | ×0.1 V | 303.0 V | 303.0 V | **[ANALYSIS] strong**: equals DID E54C (302.9/303.0 V). |
| Max discharge current `I_DischargeMax` | 0x2F5 b6-7 | ×0.1 A, **no offset** | 297.2 A | 297.2 A | [ANALYSIS]. BE subtracts 819.2, which gives an impossible −522 A on this pack. 297 A × ~336 V ≈ 100 kW matches 0x40D. **Confirm under load.** |
| Short-term charge power `P_Avail_Chg_ShortTerm` | 0x40D b0-1 | ×3 W | 69.5–69.8 kW | 70.2–70.3 kW | [FALLBACK BE BEV layout; ANALYSIS plausible]. Creeps up slowly during rest. |
| Short-term discharge power `P_Avail_Dchg_ShortTerm` | 0x40D b2-3 | ×3 W | 85.7–86.0 kW | 85.3–85.5 kW | Same as above. The low byte jitters by about ±16 counts every frame. |
| Long-term charge power `P_Avail_Chg_LongTerm` | 0x40D b4-5 | ×3 W | 46.2–46.4 kW | 46.6–46.7 kW | Same as above. |
| Long-term discharge power `P_Avail_Dchg_LongTerm` | 0x40D b6-7 | ×3 W | 100.1–100.3 kW | 99.8–100.0 kW | [HYPOTHESIS]. Long-term is higher than short-term here, so which word is which is doubtful. |
| Predicted terminal voltages | 0x430 b0-1 / b2-3 | ×0.1 V | 414.3 / 302.4 V | 414.5 / 302.4 V | [FALLBACK BE]. BE calls these predictions, not hard limits. Long-term words are 0xFFFF. |
| Predicted energy | 0x239 b1-2 | Wh | 1664 Wh | 1665 Wh | [FALLBACK BE] |

**Not limits / no data:**
- 0x2A5 is static `E4 33 FF FF FF FF FF FF` [CONFIRMED SP51], meaning unknown.
- The AS4 TX table also lists **0x33E, 0x3C2, 0x3EA and 0x426**. None of these appeared on the bench, so they are probably charging-mode or gated PDUs [HYPOTHESIS].
- BE's "hard limits" DID **DD7E returns NRC 31** on this pack. **E54C** (U max/min) works.

**For the ZombieVerter [ANALYSIS]:**
- Use `U_ChargeMax`/`U_DischargeMin` as hard voltage limits.
- Use `I_DischargeMax`/`I_ChargeMax` as current limits, and **treat both current scalings as unproven until a load test**.
- Check them against the 0x112 current while drawing a known current, ideally with the limits watched as the pack warms.
- The 0x40D powers make a useful soft limit once confirmed.

---

## 10. Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| **No 0x112 at all**, UDS answers, CAD4B0/CAD9A6 = 26 | **HVIL open** [CONFIRMED SP51 run 2 + HVIL A/B] | Check the 7↔8 bridge and any HV-connector HVIL bridge. Wiggle-test while watching 0x112. |
| **No 0x112**, UDS answers, **CAD4B0/CAD9A6 = 27** although 0x1A1/0x10B are sent | **COM latched off** (pack left powered with a silent bus) [CONFIRMED SP51 wake-window test] | 12 V off ≥ 5 s → on → start frames within ~5 s. Restarting the playback alone does **not** help (5 tries). `sp51ctl wake` detects this and tells you. |
| No 0x112 on the **first** playback only | first onset ignored [CONFIRMED owner loop 3; mechanism HYPOTHESIS] | stop, ~1 s silence, restart (only if 0x112 never appeared). |
| **0x112 for a few s, then all pack frames stop**, UDS still answers, CAD4B0/CAD9A6 = 27 | our traffic paused ≥ ~1.2 s after the pack was CAN-active → COM latched [CONFIRMED v6 run 1] | 12 V cycle; use v7/sp51ctl (no pauses). Check the tool isn't looping with a gap or stalling (USB adapter, SavvyCAN pause). |
| No 0x112, no UDS either | 12 V/30C/wake missing, wrong CAN pair, bit rate, termination, bus-off | Check pins 1–4, ~60 Ω H↔L, `ip -s link show can0` for bus-off/error counters. |
| **0x112 sometimes appears, sometimes not, same file** (owner's run-3 session) | now explained by §3a (first onset ignored / COM latched) [ANALYSIS]; a marginal HVIL contact is still possible | follow the §3a step order and use the DTC discriminator. Log failed tries too. |
| 0x112 present but 0x1FA stays 4 after CLOSE | 0x10B wrong CRC/alive/rate, 0x53A sequence missing, or pack-side lock | Check CRC with §5.2. Read DTCs and look for CAD9A7/9A8 (0x10B not-current/checksum). Check 0x1FA lock_err/precharge_locked ≠ 1. |
| 0x1FA goes to 5 but never 6 | precharge didn't reach its threshold (large or shorted load) | Remove the load or cap, and check for a short on the output. Never seen on SP51. |
| Closes, then opens on its own | the pack requested an open (0x112 open-req bits = 2) or isolation fault | Read 0x112 b5/b6 and 0x1FA iso bits, then read DTCs. |

**Run-3 findings on the intermittent 0x112** [details in `sp51_close_v4_result.md`]:
- **Once 0x112 is up, the close is repeatable.** Run 3 matches run 1 to within about 10 ms at every step.
- **The SME was already in a running operation cycle at the start of run 3.**
  - CAD900/CAD9B5 were already status 27 at 2.2 s; on a fresh start they mature at 5.5 s.
  - There was no 0x587 `A2` start-up burst.
  - 21F152 was 26, not 27.
  - So the "no 0x112" tries had woken the SME's diagnostics without starting its application TX. That is the same picture as HVIL open [ANALYSIS].
- No gaps, bus errors or missing IDs during the successful attempt. **The failed attempts were not logged**, so bus-off during them can't be excluded.
- In the successful runs, pack application TX started 26–38 ms after our first 0x12F/0x510/0x512 frames. Which one is the wake trigger is unknown; none of them is in the AS4 RX table [HYPOTHESIS: a separate NM/wake filter].

**Experiments to measure the window** (log from before 12 V on):
1. 12 V on, start traffic after 2 / 10 / 30 / 60 / 120 s → startup window.
2. After a good wake, stop traffic 10 / 30 / 60 / 120 / 300 s, then restart → bus-silence latch time.
3. In the latched state, toggle pin 4 (wake) or 30C only, KL30 kept → which input re-arms COM.
4. `sp51ctl wake --auto-powercycle 1` with a relay hook prints "0x112 x.xx s after 12 V ON".

---

## 11. Fault codes (DTCs)

### 11.1 DTC table

The status values below are the first→last value of each run: run 1 (closed), run 2 (HVIL open, no close), run 3 (closed). A single value means it never changed. Status bits are explained in §11.3.

**Clearing:**
- "Monitor passes" means the testFailed bit (0x01) drops once the condition is fixed. The history bits (0x08 confirmed, 0x20 failed-since-clear) stay in memory.
- Only ClearDTC (UDS `14`) wipes them. **It exists, but we don't use it in this project.**
- A stored history code alone has never stopped a close.

**Later sessions** [CONFIRMED SP51]: loop 3: 0317F5 and 21F393 → 27, CAD4A0 → 27 (55 s), **new CAD8FC** (STAT_DRUCK_KAELTE, refrigerant pressure missing, public 0x2D2 [FALLBACK]) = 27. Wake-window test: **new 020780** (unknown) = 27 while latched; **CAD4B0/CAD9A6 = 27 while latched** although we sent 0x1A1/0x10B (diagnostic for "COM off", §3a); after the 12 V cycle 21F4AF = 68 (new op cycle) and the late codes drop to 24. None of these stopped a close (7 closes).

**Blocks a close?** Evidence: **all 20 stored codes were present during both successful closes** (and the new ones during the later closes). So none of them blocks a close in its observed state [CONFIRMED SP51].

| DTC | Meaning | Source | Run 1 / Run 2 / Run 3 | Bench: what makes the monitor pass | In the F11 | Blocks close? |
|---|---|---|---|---|---|---|
| **CADC00** | unknown | none found (the FW byte hit is a pointer false positive) | 2F / 2E / 2F | unknown | unknown | **No** (present, testFailed, in both closes) |
| **21F4AF** | unknown; [HYPOTHESIS] contactor open/weld plausibility test | none public; runs at each contactor open | 68→28 / 68 / 68→28 | Completes (68→28) at every successful open | same | **No** |
| **CAD4B0** | V_VEH (vehicle speed) missing, 0x1A1 | owner text [pt88u] + [FALLBACK carcoding]; ID in AS4 RX [CONFIRMED AS4] | 26 / 26 / 26 | **Send 0x1A1** (v4) [CONFIRMED SP51] | F11 DSC sends 0x1A1 (F11 PT-CAN, DLC5, 20 ms) [CONFIRMED F11] | No |
| **CAD9AA** | STAT_ENG_STA_AUTO missing, 0x30B? | [FALLBACK carcoding]; 0x30B [HYPOTHESIS] | 27 / 26 / 27 | not supplied | 0x30B not in F11 logs | No |
| **CAD714** | CON_VEH (vehicle status) missing, 0x03C | [FALLBACK carcoding]; 0x03C in AS4 RX [CONFIRMED AS4] | 27 / 26 / 27 | not supplied (layout unknown) | F11 has no 0x03C (G-series PDU), so it stays set | **No** [CONFIRMED SP51] |
| **CAD99E** | RQ_HVSTO missing (ID ?) | [FALLBACK carcoding] | 26→27 / 26 / 26→27 | unknown ID | unknown | No |
| **CAD7F3** | DIAG_OBD_ENG missing, 0x397 | [FALLBACK carcoding] + AS4 RX | 2E→2F / 2E / 2E→2F | not supplied; could add 0x397 | F11 sends 0x397 (DLC3, 60 ms) [CONFIRMED F11] | No |
| **21F152** | unknown | in AS4 DEM [CONFIRMED FW]; no text | 27 / 27 / **26** | unknown | unknown | No |
| **21F04B** | **Internal CAN between SME and CSC (cell supervision): communication fault** | [PUBLIC] BMW SI B61 28 17 / NHTSA; owner text; in AS4 DEM | 27 / 27 / 27 | internal pack bus, not vehicle CAN | same; SI says it can be stored erroneously | **No**, but **watch for cell-plausibility codes** |
| **CAD900** | StatusDiagnosisEtxvCCU missing (ID ?) | owner text + [FALLBACK] | 24→27 / 24→27 / 27 | unknown (v5 probes 0x0B8/0x0CC untested) | unknown | **No** |
| **CAD9B5** | ECU_DOM_MOT (electric-motor domain) missing (ID ?) | owner text + [FALLBACK] | 24→27 / 24→27 / 27 | unknown (v5 probes) | unknown | **No** |
| **CAD9B2** | CHGNG_ST (charging status) missing, 0x3E9 | [FALLBACK] + AS4 RX | 27 / 24 / 27 | not supplied | not in F11 | No |
| **CAD9A2** | SPEC_HVSTO missing, 0x433 (sender EME) | [FALLBACK] + AS4 RX | 24→27 / 24 / 24→27 | not supplied (EME role; ZombieVerter candidate) | not in F11 | No |
| **CAD856** | RQ_AIC_1 missing (ID ?) | [FALLBACK] | 24→27 / 24 / 24→27 | unknown | unknown | No |
| **CAD7DE** | STAT_KLIMA_INTERN missing (public 0x32E, not in AS4 RX) | [FALLBACK] | 24→27 / 24 / 24→27 | unknown | unknown | No |
| **CAD558** | KILOMETERSTAND (mileage) missing, 0x330 | owner text + [FALLBACK]; AS4 RX | 24 / 24 / 24 | **Send 0x330** (v4) [CONFIRMED SP51] | F11 KOMBI sends 0x330 (event-driven) [CONFIRMED F11] | No |
| **CAD4A0** | RLS_COOL_HVSTO (cooling release) missing, 0x37B | [FALLBACK] + AS4 RX | 24 / 24 / 24 (not failing) | not supplied, but not failing [ANALYSIS: monitor may be inactive] | not in F11 | No |
| **CAD9A6** | SPEC_DCSW_HVSTO (**0x10B**) missing | [FALLBACK] + AS4 RX | 26 / 26 / 26 | send 0x10B. 26 = failed earlier this cycle (before our frames), passing now | ZombieVerter sends 0x10B | No. **CAD9A7 (not current) / CAD9A8 (checksum) never set, so our CRC/alive is accepted** |
| **0317F5** | unknown | none | 24 / 24 / 24 | unknown | unknown | No |
| **21F393** | Transport/crash-related lockout (forum consensus, unverified) | [PUBLIC secondary] forums | 24 / 24 / 24 | — | — | **No**: closed twice with it stored [CONFIRMED SP51] |

**Codes we know about but have never seen on this pack:**

| DTC | Meaning | Source | Relevance |
|---|---|---|---|
| 21F148 | HVIL line disconnected | [FALLBACK public] | **Not set even with HVIL open** (runs 2 and HVIL-B). Don't use it as an HVIL indicator; use 0x112 presence. |
| CAD715 / CAD716 | CON_VEH not current / checksum | pack catalogue [FALLBACK names] | Would appear if we sent a wrong 0x03C. We don't send 0x03C. |
| CAD9A7 / CAD9A8 | 0x10B not current / checksum | pack catalogue | Would flag a bad 0x10B alive/CRC. Never set. |
| CAD415 / CAD416 | EME status / e-motor (HV spec + alive), 0x10B family | [CONFIRMED AS4 DEM] + [FALLBACK BE ID binding] | In the AS4 DEM, but this pack reports CAD9A6/7/8 for 0x10B instead [ANALYSIS: the pack runs a newer SME build than the drop]. |
| CAD402 | RELATIVZEIT (0x328) | AS4 DEM + [FALLBACK BE] | Not set; we send 0x328. |
| CAD418 | 0x12F terminal status not current | [PUBLIC] F18 SME | Not set. |
| CAD400 / CAD404 / CAD401 / CAD405 / CAD408 / CAD409 / CAD40A / CAD413 / CAD429 | 0x10F / 0xCC / 0x2CA / 0x3E8 / 0x3A0 / 0x37B / 0x1A1 / 0x433 / 0x3CA missing (BE PHEV family) | AS4 DEM + [FALLBACK BE] | Not reported by this pack. |

### 11.2 Reading DTCs (read-only)

**Addressing [CONFIRMED SP51]:**
- Tester → SME: ID **0x6F1** with **b0 = 0x07** (SME address), followed by ISO-TP.
- SME → tester: ID **0x607** with **b0 = 0xF1**.
- PCI nibbles: 0 = single frame (SF), 1 = first frame (FF), 2 = consecutive frame (CF), 3 = flow control (FC).

1. **Optional extended session:**
   - Request: `6F1: 07 02 10 03`
   - Reply: `607: F1 06 50 03 00 32 01 F4` (P2 = 50 ms, P2* = 5000 ms).
   - Keep it alive with `6F1: 07 02 3E 80` every 2 s (no reply, because of suppressPosRsp).
2. **Request the DTCs.** `19 02 <mask>`:
   - `6F1: 07 03 19 02 FF` means every DTC with any status bit set.
   - Our playback uses mask `0C` (pending|confirmed): `07 03 19 02 0C`. It returned the same 20 codes.
3. **First frame from the SME (multi-frame reply):**
   `607: F1 10 53 59 02 FF CA DC`
   - `10 53` = FF, total length **0x053 = 83 bytes**.
   - First payload bytes: `59 02 FF CA DC`.
4. **Tester flow control (within about 1 s; we send it about 20 ms after the FF):**
   `6F1: 07 30 00 00 00 00 00 00`
   - `30` = ContinueToSend, BS = 0 (no further FC), STmin = 0.
   - SavvyCAN playback is open-loop, so the FC is scheduled blindly. A stray FC after the reply is ignored.
5. **Consecutive frames** `607: F1 21 …` to `F1 2D …`. That is 13 CFs × 6 bytes, plus 5 bytes in the FF, = 83 bytes.

**Run-3 reply at 43.47 s:**

```
607  F1 10 53 59 02 FF CA DC     FF: len 83, data 59 02 FF CA DC
6F1  07 30 00 00 00 00 00 00     FC from tester
607  F1 21 00 2F 21 F4 AF 28     CF1
607  F1 22 CA D4 B0 26 CA D9     CF2
607  F1 23 AA 27 CA D7 14 27     CF3
...  (CF4..CF13 = F1 24 .. F1 2D)
```

**Reassembled message:** `59 02 FF | CA DC 00 2F | 21 F4 AF 28 | CA D4 B0 26 | CA D9 AA 27 | CA D7 14 27 | …`

| Bytes | Decode |
|---|---|
| `59` | positive response to 0x19 |
| `02` | subfunction reportDTCByStatusMask |
| `FF` | DTCStatusAvailabilityMask (all 8 bits supported) |
| `CA DC 00 2F` | DTC **CADC00**, status 0x2F = testFailed + failedThisOpCycle + pending + confirmed + failedSinceClear (**active now**) |
| `21 F4 AF 28` | DTC **21F4AF**, status 0x28 = confirmed + failedSinceClear (history only; its test ran this cycle and passed) |
| `CA D4 B0 26` | **CAD4B0**, 0x26 = failedThisOpCycle + pending + failedSinceClear (failed earlier this cycle, **passing now**) |
| `CA D9 AA 27` | **CAD9AA**, 0x27 = 0x26 + testFailed (**failing now**) |

Each DTC is 3 bytes (big-endian, shown as 6 hex digits) plus 1 status byte.

A **negative reply** looks like `F1 03 7F 19 nn`. The `nn` codes are: 0x31 requestOutOfRange, 0x22 conditionsNotCorrect, 0x78 responsePending (wait and listen).

### 11.3 Status byte in plain language (ISO 14229)

| Bit | Value | Name | Plain meaning |
|---|---|---|---|
| 0 | 0x01 | testFailed | **failing right now** |
| 1 | 0x02 | testFailedThisOperationCycle | failed at least once since this power-up/wake |
| 2 | 0x04 | pendingDTC | failed in this or the last cycle (not yet "healed") |
| 3 | 0x08 | confirmedDTC | failed long enough to be stored permanently |
| 4 | 0x10 | testNotCompletedSinceLastClear | test hasn't run since memory was last cleared |
| 5 | 0x20 | testFailedSinceLastClear | has failed at some point since the last clear (history) |
| 6 | 0x40 | testNotCompletedThisOperationCycle | **test hasn't run yet this cycle** |
| 7 | 0x80 | warningIndicatorRequested | wants a warning lamp |

**Common values:**
- `24` = pending + history, not failing.
- `26` = failed earlier this cycle, passing now.
- `27` = failing now.
- `2E`/`2F` = the same plus confirmed.

**Example, 21F4AF 0x68 → 0x28:**
- 0x68 = 0x40 + 0x20 + 0x08: "stored in the past, and **this cycle's test hasn't run yet**".
- At 35.21 s, just after the contactors opened, bit 6 cleared, giving 0x28 with bit 0 still clear. So the test ran on the open and **passed**.
- That is why we think 21F4AF is a contactor open/weld check [HYPOTHESIS].
- Run 2 (no close) stayed at 68: the test never ran.

### 11.4 Decoding a SavvyCAN log with `decode_sp51_diag.py`

```bash
cd /workspace/sp51
.venv/bin/python scripts/decode_sp51_diag.py logs/YOURLOG.csv --can-hvil --close-report
#   --can-hvil      pack TX presence per ID + HVIL verdict + latest 0x1FA bits
#   --close-report  0x112/0x1FA snapshots, contactor timeline (cmd TX + 1FA 4/5/6 + link V),
#                   19 02 status changes over time, PRE-CHECK FAIL if zero pack app TX
.venv/bin/python scripts/decode_sp51_diag.py logs/YOURLOG.csv --all            # every UDS reply, incl. each 19 02 snapshot
.venv/bin/python scripts/decode_sp51_diag.py logs/YOURLOG.csv --json out.json  # machine-readable (DTC list per snapshot)
.venv/bin/python scripts/decode_sp51_diag.py logs/YOURLOG.csv --probe-report probe.csv   # one row per DID: pos/NRC/len/hex
```

- The decoder reassembles ISO-TP on 0x607 (b0 = 0xF1), pairs replies with our 0x6F1 requests, handles 7F/0x78, prints DIDs with scaling, and **redacts identity DIDs** (VIN etc.).
- Options `--tester`, `--rsp-id` and `--req-id` change the addressing.
- The input is a SavvyCAN GVRET CSV, as saved by SavvyCAN "Save Frames → GVRET".
- With `cantools`, you can also decode any frame through `analysis/sp51.dbc` (use `allow_truncated=True` for UDS frames of DLC < 8).

---

## 12. Unknowns (open questions for Sam)

1. **Wake trigger.** Is it 0x12F, 0x510/0x512, or any frame (NM/partial-network filter)? None of these is in the AS4 RX table.
2. **Is 0x53A required**, or is 0x10B alone enough? (v3 has not been run.)
3. **0x112 b7 scale** (≈4 V/bit): confirm with a voltmeter.
4. **Current-limit scaling in 0x2F5** (offset on b2-3 vs none on b6-7), plus the st/lt assignment in 0x40D. Needs a load test.
5. **IDs and layouts of CON_VEH 0x03C, ECU_DOM_MOT, StatusDiagnosisEtxvCCU and RQ_HVSTO.** They don't block a close, but they will stay as DTCs.
6. **0x423, 0x4AD, 0x4B0, 0x587, 0x2A5 content**, and why 0x423/0x4AD/0x4B0 aren't in the drop's AS4 TX table (the pack runs a newer build).
7. **Meanings of CADC00, 21F4AF, 21F152, 0317F5 and 21F393**: ISTA text wanted.
8. **The wake window** (§3a): startup window after power-up, bus-silence latch time, role of pin 4 / 30C, NM 0x507 never sent, NM/ComM timeouts.
9. Behaviour when 0x10B is simply lost while closed (timeout open), and the maximum hold time.
10. Live insulation resistance: DID E4D4 reads a 2000 kΩ placeholder.

---

## 13. ZombieVerter / in-car notes

- In the F11, **the car's own CAS/DSC/KOMBI already send 0x12F, 0x1A1, 0x330, 0x328, 0x2CA, 0x3A0 and 0x397** [CONFIRMED F11 logs, PT-CAN buses 2/3]. **The ZombieVerter must NOT synthesise them**, and must never send 0x380 (VIN).
  - The F11 has a second 0x12F variant (DLC 7, 1000 ms) on bus 1. Connect the pack to the bus carrying the DLC 8, 100 ms, CRC init 0x60 version.
- The pack must therefore sit **on the F11 bus that carries those frames**. If it can't, use a pass-through gateway that **forwards** the real frames. That is a design decision to agree with Sam; it is not synthesis [ANALYSIS].
- **The ZombieVerter plays the EME role.** It sends **0x10B** (20 ms, CRC 0x3F) and **0x53A** (200 ms, A/B/C/D).
  - Optional: 0x433 SPEC_HVSTO (CAD9A2) and 0x3E9 (CAD9B2), but their layouts are unknown.
  - 0x3CA, 0x3E8 and 0x510/0x512 were sent on the bench. 0x3CA, 0x3E8 and 0x3E9 aren't in the F11 logs. Whether the pack needs any of them is untested.
- Gate the close in the ZombieVerter on:
  - 0x112 present at 20 ms;
  - 0x1FA state 4 with lock_err/precharge_locked = 1 and iso bits = 1;
  - 0x112 open requests = 1.
- Treat 0x1FA = 6 as "HV ready", and give up if 6 hasn't arrived within about 3 s.
- Use the §9 limits once their scaling is confirmed.

---

## 14. Sources

| Item | Source |
|---|---|
| RX/TX PDU tables | EA443 AS4 3e6f seg0: RX `0x1522AC–0x152E1C` (`as4_rx_filter_table.csv`), TX `0x151E14–0x1522A0` (`as4_tx_pdu_table.csv`) [CONFIRMED AS4] |
| CRC poly 0x1D table | AS4 [CONFIRMED]. Inits: 0x60 for 0x12F [F11], 0x77 for 0x1A1 [F11], 0x3F for 0x10B [FALLBACK BE, accepted by SP51], 0x45 for 0x295 [SP51] |
| Close/open behaviour, timings, 0x1FA states, 0x423, 0x587 | `logs/close_v4_01M6GYPM9M7SMQWDN4TTZEYYJW.csv` (run 1), `logs/close_v4_run3.csv` (run 3) [CONFIRMED SP51] |
| HVIL gate | `logs/hvil_A/B…`, `logs/close_v4_run2_nocycle.csv` [CONFIRMED SP51] |
| Signal layouts 0x112/1FA/2F5/239/40D/430–432 and 0x53A/0x10B payloads | Battery-Emulator BMW PHEV [FALLBACK], checked against SP51 values/DIDs |
| DTC names | owner text, carcoding CA list [FALLBACK], BMW SI B61 28 17 [PUBLIC], AS4 DEM scan |
| Playback generator | `scripts/gen_sp51_close.py` (v4), `scripts/gen_sp51_close_v6.py` (v6), `scripts/gen_sp51_close_v7.py` (v7), `scripts/gen_sp51_wake.py` (CRC) |
| Wake behaviour | `logs/close_v4_loop3.csv`, `logs/wake_window_test.csv` (+ `_refused_part.csv`) [CONFIRMED SP51] |
| DBC | `analysis/sp51.dbc` (built by `scripts/build_sp51_dbc.py`, verified with cantools 45.1.2 on runs 1 and 3: every frame decodes) |

---

## 15. `tools/sp51ctl.py`: SocketCAN controller (README)

> **HV WARNING.** `close` puts ~380 V DC on the pack output. Bench only: insulated HV wiring, HVIL closed, precharge-safe load, 12 V emergency disconnect in reach. **Not for the car:** it synthesises 0x12F/0x1A1/0x328/0x330 etc., which in the F11 come from CAS/BDC/KOMBI/DSC.

**Install:** `python3 -m venv .venv && .venv/bin/pip install "python-can>=4"` (cantools is not needed). Interface: `sudo ip link set can0 up type can bitrate 500000`.

| Command | Does |
|---|---|
| `wake` | starts the cyclic set, waits for 0x112; no pack frame **ever** seen → `--wake-gap` (1 s) silence + fresh restart (×`--wake-retries`); if pack frames were seen and stopped it **never goes silent** and says power-cycle; still mute → read-only DTC diagnosis (no-UDS / HVIL-open / **COM-latched → "POWER-CYCLE 12 V NOW"**); optional relay hook power-cycles for you |
| `status` | live panel (~2 Hz), `--log panel.csv` |
| `close` | wake → **stability gate** (0x112 continuous ≥ `--stable-seconds` 5 + ≥ 3× 0x1FA 0x45) → pre-check (0x112 alive, 0x1FA b1 = 0x45, open-req = 1, pack V > 200 V) → idle `--idle-seconds` (20, aborts if 0x112 is lost) → 53A B 2 s → pre-check again → CLOSE + C → D → wait for state 6 (`--close-timeout` 5 s) → hold `--hold-seconds` (10; 0 = until Ctrl-C) → OPEN |
| `open` | 0x10B OPEN + 0x53A A, waits for state 4 |
| `dtc` | `10 03` + `19 02 <--dtc-mask>` decoded with names and status bits |
| `powercycle` | runs the relay hooks (off, `--power-off-seconds`, on), starts traffic at once, waits for 0x112 |
| `shell` | prompt with status/close/open/dtc while frames keep running |

**Cyclic TX** (background thread, 10 ms tick): 0x12F 100 ms, 0x10B + 0x1A1 20 ms, 0x53A 200 ms, 0x328/0x3A0/0x3CA/0x2CA/0x3E8/0x510/0x512/0x330 1 s, CRC/alive exactly as §5. The frame order and bytes are **identical** to the v4/v6 playback for the same timeline (tested).

**Safety:** Ctrl-C, SIGTERM, any exception, 25 consecutive TX errors, an RX bus error, 0x112 lost > 0.5 s while closed, a pack open request, or the pack leaving state 6 for > 1 s → **0x10B OPEN + 0x53A A for 2 s, then TX stops**. `close` asks you to type `CLOSE` (`--yes` skips). On exit with a woken pack it reminds you that the next session needs a 12 V cycle. UDS is an allow-list: 10 01/03, 3E 00/80, 22, 19 02. No SecurityAccess, writes, routines, IO control, ECUReset or ClearDTC. 0x380 is never sent.

**Relay hook (placeholder):** `--power-off-cmd "gpioset gpiochip0 17=0" --power-on-cmd "gpioset gpiochip0 17=1" --auto-powercycle 1` (any shell command; drive a relay in the 12 V KL30+30C feed). Without hooks the tool prints the manual instruction.

**Panel fields** ('?' = layout not confirmed on this pack):

| Field | Source |
|---|---|
| contactors | 0x1FA b1 high nibble 4/5/6 [CONFIRMED] |
| pack V, current, power | 0x112 [CONFIRMED]; P = V·I |
| link V? | 0x112 b7 × 4 V [ANALYSIS] |
| SOC? | mean of per-cell SOC, DID E59A (UDS, every `--uds-period` 5 s) [ANALYSIS] |
| cells min/max/delta | DID E554 [CONFIRMED 108 cells] |
| temps? | 0x1FA b6/b7 − 50 [FALLBACK] |
| U chg max / U dch min | 0x2F5 [ANALYSIS = E54C] |
| I chg?/I dch?, P chg?/P dch? | 0x2F5, 0x40D short-term [FALLBACK/ANALYSIS] |
| open req | 0x112 b5[7:6]: **1 = no open request (normal)**; b5 = 0x7F in every log and every state 4/5/6, including all 7 closes and the failed v6 run [CONFIRMED]. 2 = pack wants open → `close` aborts/opens [FALLBACK BE] |
| DTC active | count with testFailed in the last 19 02 FF (every 4 × uds-period) |

**Examples:**
```
.venv/bin/python tools/sp51ctl.py wake                     # bring the pack up, keep frames running
.venv/bin/python tools/sp51ctl.py status --log panel.csv   # live panel, logged
.venv/bin/python tools/sp51ctl.py dtc
.venv/bin/python tools/sp51ctl.py close --hold-seconds 30 --log close.csv
.venv/bin/python tools/sp51ctl.py close --hold-seconds 0   # hold until Ctrl-C (Ctrl-C = OPEN)
.venv/bin/python tools/sp51ctl.py open
# dry-run on a python-can virtual bus, fake pack replaying a real log:
.venv/bin/python tools/sp51ctl.py close --dry-run --fake-pack logs/close_v4_loop3.csv --hold-seconds 3 --tx-log /tmp/tx.csv
```

**Tests:** `.venv/bin/python tools/test_sp51ctl.py` (13 tests, ~2.5 min): byte-identity vs v4, v6 and v7, no-silence-after-activity (v6 run 1 replay: aborts, no CLOSE, no TX gap), known frames, UDS allow-list, DTC decode, fake-pack close with state-6 detection + panel CSV + E554 poll, SIGTERM → OPEN, wake with restart, latched-COM diagnosis (wake-window log), relay-hook power cycle, dtc. The fake pack is open-loop (it replays the log's timing), so it tests decoding and state handling, not the pack's real response to our commands.
