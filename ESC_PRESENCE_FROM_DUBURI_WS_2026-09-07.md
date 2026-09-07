# PR F — thruster presence is computed, carried, and then dropped one line from the wire

**Repo:** `srot-control-board` — **ESP32 only, no Pico reflash, no new message,
no extra bandwidth.**
**Severity:** high — it is the difference between a thruster health gate that
can pass and one that is structurally incapable of passing.
**Evidence:** `src/comms/mav_stream.cpp:42-43, 176-200`,
`include/state_types.h:201-202`, `src/tasks/task_dshot_rmt.cpp:133-134`,
`shared/thruster_link_proto.h` (`TL_ST_PRESENT`)

> **Rank this ABOVE PR C.** PR C asks for volts/amps/°C and needs a dual
> reflash. This asks for one byte that is already sitting in the function that
> builds the message, and it unblocks more.

---

## The finding

`esc_present` and `esc_fault` are computed on the Pico from `TL_ST_PRESENT`,
cross the thruster link in `tl_tlm_t`, land in `g_state.thrusters`
(`task_dshot_rmt.cpp:133-134`), and are **already members of `Snap`**:

```cpp
// src/comms/mav_stream.cpp:42-43
uint8_t esc_fault   = 0;   // bit i = thruster i faulted
uint8_t esc_present = 0;   // bit i = thruster i sending telemetry (connected)
```

`Snap` is the struct `sendEscStatus(const Snap& s, uint32_t t)` receives. Both
fields are **in scope at the packing site** and neither is read. Meanwhile the
same function packs:

```cpp
uint16_t ... cnt[4] = {0};        // ESC_TELEMETRY .count — literal zeros
```

So the vehicle ships a `count` field of zeros in a message that already decodes
on our side, while the presence bitmask it could carry sits in a local variable
three lines above.

## Why this is the one that matters

**Measured on our vehicle: 958 CRC-valid `ESC_STATUS` frames across two
recorded sessions with no ESCs attached, both index blocks, every rpm exactly
`0`.** The board fills all eight slots regardless of what is connected.

That is the correct behaviour for a fixed-size message — but it means **no
signal on the current wire can distinguish "eight healthy thrusters idling"
from "no thrusters at all."** Frame count says the link works. `len(rpm) == 8`
is true either way. Zero rpm is what a still hull reports too.

Consequence on our side, which we have just had to design around: our thruster
health can only ever report **UNKNOWN**, because the only honest presence
evidence we have is the English STATUSTEXT summary emitted at the **first arm**
— so a disarmed bench session, and any hull whose ESCs are not Bluejay-flashed,
has nothing to grade. Our pre-fire payload gate therefore cannot refuse a
torpedo run with a dead thruster; it can only decline to refuse.

Your own comment at `mav_stream.cpp:172` already states the rule this violates:

> *"a consumer should treat 0 as 'not instrumented', not 'measured zero'."*

Exactly so — and with presence on the wire, we could.

## Requested change

**Option 1 (preferred, one line, zero cost).** Carry the presence bit in the
already-zero `count` field:

```cpp
for (int i = 0; i < 4; ++i) cnt[i] = (uint16_t)((s.esc_present >> i) & 1);
// and for the 5..8 block:
for (int i = 0; i < 4; ++i) cnt[i] = (uint16_t)((s.esc_present >> (4 + i)) & 1);
```

No new message, no new bytes, no Pico change. `count` is nominally an error
counter, so if that reuse is distasteful, `temperature` is equally zero today —
or:

**Option 2 (cleaner, still ESP32-only).** One `NAMED_VALUE_FLOAT` per second
carrying `esc_present` and `esc_fault` as a packed 16-bit value — call it
`ESCPRESENT`. ~20 B/s. Please keep the name ≤ 10 characters; the field
truncates silently, which is how we once read `BARO_HEALTH` as `BARO_HEALT`.

**Option 3 (best, if a `tl_tlm_t` bump is on the table anyway).** Ship the full
per-thruster `status[]` byte — `TL_ST_RPM_VALID / PRESENT / WARNING / ALERT`.
That subsumes PR C's health half without the ESC voltage plumbing, and it is
already computed.

## What we do with it the day it lands

- The thruster health reporter starts returning **OK / FAILED** instead of a
  permanent UNKNOWN, and the pre-fire gate can actually refuse a shot the hull
  cannot hold. Our side is already written and tested against exactly this
  contract (`health_reporters.thrusters`, `Duburi._thruster_fault`).
- `/duburi/esc_rpm` starts publishing. It is currently gated on announced
  presence precisely so it does not emit eight fabricated zeros.
- A **stall becomes actionable**: `esc_fault` plus our signed-RPM check
  distinguishes a fouled prop from an unwired one, which unsigned magnitude
  cannot.

## One thing we are NOT asking for

Do not synthesise presence from "rpm != 0". A thruster commanded to zero is
indistinguishable from an absent one under that rule, and it would put the
false-OK back one layer down.
