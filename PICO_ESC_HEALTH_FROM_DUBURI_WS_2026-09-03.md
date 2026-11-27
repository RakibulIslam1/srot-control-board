# PR C — the Pico decodes ESC voltage, current and temperature, then discards them

**Repo:** `srot-control-board` (needs a **dual reflash**: ESP32 + Pico)
**Severity:** medium — but it is the failure this board was built to catch
**Evidence:** `src/pico/main.cpp:299-318`, `shared/thruster_link_proto.h:53-63`,
`src/comms/mav_stream.cpp:188`

---

## The finding

The Pico already receives Extended DShot Telemetry frames and already decodes
their payload into `tval`. Then it drops it:

```cpp
uint32_t tval = 0;
switch (s_esc[i]->getTelemetryPacket(&tval)) {
case BidirDshotTelemetryType::ERPM: {
    int16_t meas = signedRpm(i, s_prev_dir[i], tval);   // tval USED
    ...
}
case BidirDshotTelemetryType::VOLTAGE:
case BidirDshotTelemetryType::CURRENT:
case BidirDshotTelemetryType::TEMPERATURE:
case BidirDshotTelemetryType::STATUS:
case BidirDshotTelemetryType::STRESS:
    s_last_tlm_ms[i] = now;    // "not RPM, but the ESC IS answering"
    s_tc_edt[i]++;
    break;                     // tval discarded
}
```

The comment is right that any frame proves liveness. The cost is that the
*values* — which arrived, decoded, in a variable already in scope — are thrown
away one line before they could be stored.

Downstream, `mav_stream.cpp:188` packs those exact fields as literal zeros into
`ESC_TELEMETRY_1_TO_4` / `_5_TO_8`. Those are **real MAVLink fields that
`duburi_ws` already decodes today** — we read the message, we read the zeros,
and we cannot tell "this ESC reports 0 A" from "nobody filled this in".

## Why it matters more than a telemetry nicety

A fouled prop draws **current without RPM**. That is the signature — high
current, low or zero RPM, rising temperature — and it is exactly the fault a
board with per-thruster ESC telemetry exists to catch. Today we can see the RPM
half and not the current half, so the one measurement that distinguishes "the
prop is jammed" from "the ESC is dead" is the one being discarded.

It is also the gate we want before a torpedo run: refusing a shot with a
degraded thruster, rather than discovering it mid-manoeuvre.

## Requested change

**1. Capture the values on the Pico.** The switch arms already run; they need a
store rather than a `break`:

```cpp
case BidirDshotTelemetryType::VOLTAGE:
    s_volt_cv[i] = (uint16_t)tval;   // EDT volts are centivolts
    s_last_tlm_ms[i] = now; s_tc_edt[i]++; break;
case BidirDshotTelemetryType::CURRENT:
    s_curr_ca[i] = (uint16_t)tval;   // centiamps
    s_last_tlm_ms[i] = now; s_tc_edt[i]++; break;
case BidirDshotTelemetryType::TEMPERATURE:
    s_temp_c[i] = (uint8_t)tval;     // degrees C
    s_last_tlm_ms[i] = now; s_tc_edt[i]++; break;
```

**2. Carry them in `tl_tlm_t`.** It currently has `rpm[8]`, `status[8]`,
`fault_mask` and `uptime_ms` and no room for these. Adding
`uint16_t volt_cv[8]`, `uint16_t curr_ca[8]`, `uint8_t temp_c[8]` costs
**40 bytes per frame**. The link is 1 Mbaud and, by your own note in
`config.h`, "still <20% utilised" — this is not a bandwidth question.

**Append, do not insert.** `tl_tlm_t` is a packed struct shared by two
separately-flashed MCUs; inserting a field silently misaligns every one after
it, and the CRC would not catch a *consistent* misparse across a matched pair.
Append, and bump whatever the pair uses to detect a version skew.

**3. Fill the MAVLink fields.** `mav_stream.cpp:188` already writes the message;
it just needs the values instead of zeros. **No wire change** — the fields
exist and we decode them.

**4. Please state the units in the header.** EDT is centivolts / centiamps /
degrees; `ESC_TELEMETRY_*` wants centivolts, centiamps and degrees. If they
happen to match, say so, because a silent unit mismatch here reads as a
plausible number rather than an error — which is how `PM1_VMULT` cost two
rounds.

## The dual reflash, and why it is worth it

This is the one change in the set that needs both MCUs flashed together, which
makes it the most expensive to land and the clearest example of what owning
both firmwares buys: the data is already on the wire between them, and the only
reason it does not reach a mission is that nobody wrote it down.

**Bump `SROT_FW_BEHAVIOUR_REV`.** We gate on the rev; without a bump we will
keep treating those fields as unpopulated and your work will go unused. That is
not hypothetical — it is what happened to the `SYS_STATUS` leak bit you added
for us in rev 3, which we only adopted this round.

## What we will do with it

- populate `Telemetry.esc_voltage` / `esc_current` / `esc_temp` (the decode
  already exists on our side)
- a pre-fire gate: refuse a torpedo or dropper run on a thruster showing current
  without RPM, **gated on `esc_msgs > 0`** so "no telemetry" can never read as
  "healthy"
- surface all three per thruster in the mission console, which already has the
  panel and is currently drawing zeros
