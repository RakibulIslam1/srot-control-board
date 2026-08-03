# Design review + doc drift — from the `duburi_ws` side, 2026-08-03

**Context:** SROT/Hengla/Bondor development has moved to the `duburi_ws` agent, so this is
written as a new owner reading the whole design set for the first time and checking it against
the source. **No firmware behaviour is changed by this document.** It records what the code
actually does where a doc says otherwise, and the design decisions worth preserving.

`AGENTS.md:117-121` is the governing rule and it is the right one: *"when a doc and the code
disagree, the code is ground truth — fix the doc, never 'fix' the code to match a stale
claim."* Everything below follows it.

**Verified this session — all six envs built, three of them do not exist:**

| env | result |
|---|---|
| `esp32doit-devkit-v1` | ✅ **SUCCESS** — RAM 24.3% (79612 B), Flash 29.6% (931573 B) |
| `pico` | ✅ **SUCCESS** (14m42s — the RP2350 toolchain is slow to fetch) |
| `second-board` | ✅ **SUCCESS** (12s) |
| ~~`groundstation-esp32`~~ | **REMOVED** — lives in `srot-ground-station/` |
| ~~`esp32_4way`~~ | **REMOVED** — lives in `srot-esc-flasher/` |
| ~~`esp32_4way_diag`~~ | **REMOVED** — lives in `srot-esc-flasher/` |

**The three failures are stale env definitions, not broken code.** `platformio.ini:104-170`
declares envs whose `build_src_filter` points at **`src/groundstation/` and `src/esp32_4way/`,
and neither directory exists in this repo** — `git log` shows they never did. Both sources live
in the sibling repos: `srot-esc-flasher/src/esp32_4way/` and `srot-ground-station/`.

**FIXED 2026-08-03.** The three dead envs are removed and the header now says THREE, with a
pointer to the repo that owns each of the others (both siblings already declare identical envs
with the sources actually present, so these were pure duplicates). All three remaining targets
re-verified green after the edit: `esp32doit-devkit-v1` 27 s, `pico` 9 s, `second-board` 12 s.

**`RPM_LOOP` is also fixed** — see §1. It turned out to be worse than a wrong mechanism.

---

## 1. Doc drift, with file:line

`ARCHITECTURE.md` is the stalest document **and the one a new developer opens first**. Four of
the six items below are in it.

| Claim | Where | What the code says |
|---|---|---|
| "SROT presents as **ArduSub 4.1.0**" | `ARCHITECTURE.md:134` | `MAV_AUTOPILOT_GENERIC` (`mav_stream.cpp:178`). The spoof was deleted in AUDIT B10; `README.md:9-11` is correct |
| §3 written around **BlueOS / QGC on a Pi** | `ARCHITECTURE.md:132-146` | There is no Pi and no BlueOS (`AGENTS.md:28`). GCS is Bondor; the companion is on USB |
| **7 flight modes** | `ARCHITECTURE.md:95-103` | **11** (`state_types.h:33-53`). The omissions include **`AUTO` (23) — the entire companion interface** |
| "all six tasks" for `STK_*` | `ARCHITECTURE.md:128` | **Seven**, per its own table at `:15-24` and `main.cpp:142-157` |
| `THR_RPM_CLOSED_LOOP = 1` / `RPM_LOOP` default 1 | `PARAMETERS.md:141,146`, `ARCHITECTURE.md:120` | `config.h:218` has **0**, and `DEF_RPM_LOOP = ((float)THR_RPM_CLOSED_LOOP)` = **0** (`config.h:507`) |
| "121 + 26 + 80 = **227**" params | `PARAMETERS.md:11-13` | ~**231** (125 scalar + 26 `CAL_*` + 80 servo, `params.cpp:314-362`) |

**Two that are worth more than a table row:**

**`RPM_LOOP` — RESOLVED 2026-08-03, and it was worse than a wrong mechanism.**

The code was never ambiguous: `THR_RPM_CLOSED_LOOP = 0` is deliberate and `config.h:206-217`
argues it well — an RPM setpoint inside the attitude path made the vehicle oscillate (1°
disturbance → spin-up/stop/spin-up), which is architectural rather than a tuning problem, with
citations (Yoerger/Cooke/Slotine 1990; Smogeli/Sørensen 2009) and the observation that no
mainstream flight or ROV firmware closes a thrust loop on RPM.

So the docs were fixed, not the code — `AGENTS.md:117-121`, code is ground truth.

**But checking the alternatives turned up the real problem: `THR_TRIM_EN` (0) and
`MOT_BAT_V_MAX` (0 = off) are ALSO disabled at defaults.** So it is not that
`ALGORITHMS.md §11.1` credited the wrong mechanism for voltage-independent distance — **no
mechanism is active at all**, and a timed AUTO move genuinely travels further on a full pack
than a flat one. That is a live behaviour the companion plans timed moves against, and it was
documented as solved. All three docs now state it plainly and list the two supported routes
(`THR_TRIM_EN`, recommended; battery feedforward, needs the 2nd board).

**`HARDWARE.md` gives two different RMT fallback pin lists** — `:82` says GPIO 0/12/13/14/15,
`:296-298` says 4/12/13/14/15/16/17/27. The second includes **GPIO4, which `:50` documents as
the BNO085 interrupt line**. Under that list, RMT mode drives DShot on the IMU's interrupt pin.
Almost certainly a stale list rather than a live hazard (the default backend is PICO, so
neither is exercised), but it should not be ambiguous in a pin map.

---

## 2. An undocumented cascade stage

**`thrust_trim` is live in the control path and absent from `ALGORITHMS.md`.** It is applied at
`mixer.cpp:157` and driven at ~10 Hz from `task_dshot_rmt.cpp:188`. `README.md:99` mentions
`THR_TRIM_EN` only as not water-validated.

Its header carries the best physics argument in the repository, and it deserves to be in the
algorithms document rather than discoverable only by reading the mixer:

> *"Throttle does not command thrust, it commands volts: `RPM ~ (duty · V_batt)/Kv`,
> `thrust ~ RPM²` — so a T200 at the SAME PWM makes 3.71 kgf at 12 V and 6.7 kgf at 20 V, a
> 1.8× spread purely from battery state… Measured thrust/RPM², by contrast, is invariant to
> ~3% across that whole range."*

And the reason it sits *outside* the fast loop, which is the part a future maintainer would
otherwise rediscover the hard way:

> *"Using RPM as a setpoint inside the stabilisation path made the vehicle **oscillate** (spin
> up / stop / spin up on a 1 degree disturbance)… So this runs at ~10 Hz with a multi-second
> time constant, applying only a bounded GAIN to the commanded duty. It can never fight the
> attitude controller: it is 100× slower than it."*

---

## 3. Design decisions worth preserving

Read as a new owner, these are the choices that are load-bearing and should not be casually
undone:

- **Excluding the magnetometer from the BNO085 fusion.** `ARCHITECTURE.md:68-70`: it is what
  makes attitude immune to thruster current — an in-hull compass reads the motors, not the
  earth. `MAG_YAW_REF` then buys back an absolute reference *once*, at boot, and is honest that
  it does not fix drift (`ALGORITHMS.md:70-73`) because a continuous mag correction would
  reintroduce exactly the sensitivity being avoided.
- **Applying the yaw reference at a single publish point** so heading-hold, absolute `TURN`,
  `ATTITUDE` and `VFR_HUD` cannot disagree (`ALGORITHMS.md:62-63`). One place to reason about.
  *(Our display bug this week was the companion-side version of the same class of problem, and
  we fixed it the same way — one shared formatter.)*
- **Core 1 is the flight core and nothing else runs there**, so comms can never preempt flight.
- **Take one mutex, copy fast, release; never nest; never hold across I/O** — deadlock-freedom
  by construction rather than by ordering discipline, which is the right trade at 500 Hz.
- **Uniform mixer saturation** preserving the relative mix instead of clipping one motor.
- **Derivative on measurement**, avoiding setpoint kick.
- **DShot 1048 neutral** — zero thrust with the ESC still armed.
- **`movement::Type` is append-only** (`AGENTS.md:61-63`): the wire mapping is
  `mv_type = wire + 1`, so inserting a value silently renumbers every verb after it.
- **The terminal-ACK invariant** (`AGENTS.md:65-74`) and the two specific mechanisms that hold
  it: AUTO being displaced calls `cancel()` **not `abort()`**, and the `mv_*` publish is
  unconditional. Both must stay; re-gating that publish on the mode reintroduces a companion
  hang.
- **NaN treated as an adversary** (`AGENTS.md:109-111`) — `constrain(NaN, …)` returns NaN,
  which reaches `(int16_t)NaN` on an armed thruster.

---

## 4. Two things we would put at the top of a new owner's list

**The Pico e-stop fails permissive** (`AGENTS.md:123-125`), on an eight-thruster vehicle. It is
recorded as a deliberate deferral, and it is the one deferral we would revisit first.

**The barometer plausibility band is per-sample, and that is structurally blind to variance.**
Covered fully in `BENCH_FINDINGS_FROM_DUBURI_WS_2026-08-02.md`: every reading in a 317..874
mbar spread was individually inside the band, so the health bit stayed set while the derived
depth wandered six metres. A jitter/peak-to-peak gate over N samples while stationary would
catch it, and the board is the better place for it — **you can refuse `AUTO`; we can only
refuse to arm.**

---

## 5. What changed on the companion side this week

Relevant to you only because it changes what our tools show:

- **Heading is `0..360` on every companion surface**, matching your OLED and
  `VFR_HUD.heading`. We had one path rendering the raw signed `ATTITUDE.yaw`, so it printed
  `-162.23°` beside your `197°` — the same angle, disagreeing by 360. One shared formatter now
  owns every conversion.
- **Depth is gated on the `SYS_STATUS` barometer health bit.** ⚠ This is worth knowing on your
  side: **`VFR_HUD` is sent unconditionally** (`mav_stream.cpp:258`), unlike `SCALED_PRESSURE2`
  and `WTEMP` which you *do* suppress. So a board with no Bar30 fitted still streams
  `alt = -0.000` forever, and an unguarded consumer publishes a confident `0.00 m` — which
  silently disables every depth guard on our side, since they all compare against negative
  constants. **Consider suppressing `VFR_HUD.alt` on the same condition as `SCALED_PRESSURE2`**,
  or documenting the asymmetry loudly. We have guarded it host-side either way.
- **A change-logger** now reports absent↔present transitions, so your deliberate suppression
  shows up as an event rather than as a number that quietly stops moving. On the bare-board
  bench it immediately caught the unwired PM1 pin oscillating **1.20 ↔ 5.96 V** — a floating
  ADC, which a periodic snapshot renders as a plausible-looking single voltage.

---

## 6. Open questions for whoever knows the history

1. **`RPM_LOOP`**: closed-loop RPM, or `THR_TRIM_EN`? The code and three docs disagree, and
   AUTO's repeatable-distance claim depends on the answer.
2. **The RMT fallback pin list** — which of the two is current, and does either really include
   GPIO4?
3. **Boot mode.** `ARCHITECTURE.md:96` says MANUAL (19); we have observed both MANUAL and
   SURFACE (9) disarmed on the bench. SURFACE is most likely the GCS-loss failsafe having
   already latched (no heartbeat → SURFACE after `GCS_FAILSAFE_MS`), but it is worth confirming
   because it changes how you reason about arm behaviour.
