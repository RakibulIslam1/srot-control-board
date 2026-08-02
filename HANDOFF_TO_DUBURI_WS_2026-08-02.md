# Handoff — SROT / Hengla development moves to the `duburi_ws` agent

**From:** the firmware side (`srot-control-board`, `srot-ground-station` + Bondor).
**Baseline:** `srot-control-board` @ `4fcfd1c` · **`SROT_FW_BEHAVIOUR_REV 4`** ·
`srot-ground-station` @ `7292d06`.
**Setup from here on:** the SROT board lives on the **`duburi_ws` dev box**. You drive it.

---

## Read these three, in this order

1. **`WATER_TEST_CONTROL_ONLY.md`** — the gate, the pre-dive checklist, the command sequence.
   This is the operational document.
2. **`AUDIT.md`** R46–R56 — what changed this round and why.
3. **`DUBURI_WS_INTEGRATION.md`** §0a — the depth gate, corrected wording.

---

## The four things that will bite you if you skip this page

### 1. Yaw is ABSOLUTE from rev 4
`ATTITUDE.yaw` / `VFR_HUD.heading` are a magnetic compass heading, not relative-to-boot. A heading
recorded against rev ≤ 3, **or compared across a vehicle reset**, is not the same number. Absolute
`MOVE_TURN` (p4=1) works for the first time — it always needed `MAG_YAW_REF=1`, which shipped off.

### 2. A missing value is NOT zero
`WTEMP` and `SCALED_PRESSURE2` are **suppressed** when the barometer is unhealthy or stale;
`SCALED_IMU2.temperature` sends `0` as MAVLink's "not provided" sentinel.

Before rev 3, a Bar30 read during a PROM reset race published fabricated pressure, depth **and**
temperature — `−51 °C` and `+2.87 m` in air were both observed on this hardware, with nothing
marking them wrong. **A consumer that renders absence as `0.0` reintroduces that failure.**

### 3. Payload — the channel mapping is FIRMWARE's, not yours
16 PCA9685 channels (0–15). Each channel's role is a **firmware parameter set in Bondor**:
`PCA_SERVO` (PWM) or `PCA_SWITCH` (HIGH/LOW MOSFET/relay).

**You need `set_channel(n, value)` over 0–15, not a wiring map.** The board already routes to
`DO_SET_SERVO` or `DO_SET_RELAY` by configured role. A host-side wiring table duplicates firmware
state and goes stale **silently** the moment a channel is re-roled. Keep semantic names
(`torpedo`, `dropper`) with the *mission*; the channel number is the only thing that crosses repos.

### 4. Check the behaviour rev before flying anything
`AUTOPILOT_VERSION.middleware_sw_version` via `MAV_CMD_REQUEST_MESSAGE(148)`. `0` means firmware
older than 2026-08-01 — fail closed on it. This interlock exists because you removed the host-side
`MOVE_STOP` brake; an old board coasts and nothing says so.

---

## ⛔ The gate — half closed, half standing

**Closed (bench, disarmed, 2026-08-02):** the depth controller's **sign**.
`DEPTH_CMD` mean −0.844 (descend toward a deeper target), correlation with measured depth **+0.719**
(deeper → less descent). The inversion `AUDIT.md` R1 warns about is **not present in the
controller**.

**Still standing, and these are separate failure modes:**
1. **The demand reaching the thrusters with the right sign.** The mixer columns and
   `MOT_n_DIRECTION` are downstream and can invert independently. A correct controller feeding an
   inverted mixer still dives when told to surface.
2. **Closed-loop stability** against real water, mass and buoyancy.

The two hand checks in `WATER_TEST_CONTROL_ONLY.md` gate **every `SROT_MOVE`, `move_forward`
included** — `SROT_MOVE` auto-enters AUTO and AUTO closes the depth loop under every primitive.
**An in-air `move_forward` is not partial validation:** at ~0 m the target and measurement agree,
so the loop never does any work.

Log `DEPTH_CMD`, `DEPTH_ERR` and `DEPTH_OUT` on the first dive. If the armed loop disagrees with
the disarmed preview, that difference is the diagnosis.

---

## Known-open, honestly

| item | status |
|---|---|
| PM1 analog voltage | reads ~0 — **nothing is wired to GPIO36**. Hardware, not firmware. |
| BNO085 dynamic calibration | `sh2_setCalConfig` is rejected and retries do not take, so `MAGACC` stays 0–1. Harmless: the yaw reference uses **our** `CAL_MAG_*` and ignores the flag. |
| `arc`, `style_yaw` | not built. You said not to spend time on them. |
| Vision (`LANDING_TARGET`) | spec'd, not built — blocked on **measured camera FOV**, which is yours. |
| LEAK on `SYS_STATUS` extended bits | on the wire and correct, but **pymavlink 2.4.49 cannot decode it** (13-field schema, no extensions). `NAMED_VALUE_FLOAT("LEAK")` is kept as a deprecated duplicate. |

---

## How to reach us

Same channel as before: this repo and `duburi_ws` branch `srot`. Open an issue or a PR here for
anything firmware-side. **Name the behaviour you want, not the implementation** — that is what
worked last round.

If you change a wire constant, bump `SROT_FW_BEHAVIOUR_REV` in the same commit. Our source-text
drift test stayed green through an entire `MOVE_STOP` behaviour change; a number bumped
deliberately is the thing that does not.

---

## One lesson from this round, offered because it will save you time

Four separate gates refused the magnetic heading reference, and **every one of them refused without
showing the number it was refusing on**: the feature defaulted off, then the IMU's own accuracy
flag (which the operator's calibration cannot move), then a field band tuned for open air, then the
accuracy flag again at a lower threshold. Each was individually defensible. Together they made a
working feature look permanently broken for weeks.

Every refusal in this firmware now quotes its measurement. If you add a gate, make it say what it
saw — otherwise the operator cannot tell "not configured" from "configured and rejected".
