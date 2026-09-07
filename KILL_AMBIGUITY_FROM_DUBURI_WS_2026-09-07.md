# PR H — `KILL = 0` means two different things, and one of them is dangerous

**Repo:** `srot-control-board` — **ESP32 only, one line, no new message, no bandwidth**
**Severity:** high — it is a false safety statement on the line an operator reads before arming
**Evidence:** `src/drivers/espnow_link.cpp:49`, `src/comms/mav_stream.cpp:820-846`,
`src/control/arming.cpp:13-47`, `include/state_types.h:124`,
`srot-ground-station/src/groundstation/main.cpp:385`

---

## The finding

The thruster kill switch is a rotary knob on the **second board**; its state
crosses to the control board over ESP-NOW. On link loss you report it as
*clear*, deliberately:

```cpp
kill = f ? s_kill : false;   // link lost → don't assert kill (display-only)
                             //   espnow_link.cpp:49
```

**That justification is no longer true.** `kill_switch` is not display-only — it
is packed onto MAVLink at `mav_stream.cpp:846`:

```cpp
sendNamed(now, "KILL", s.kill ? 1.0f : 0.0f);
```

and re-emitted a second time by the ground station off the LoRa flags
(`groundstation/main.cpp:385`). `state_types.h:124` still carries the stale
comment `// thruster kill from 2nd board (display-only)`.

So on the wire **`KILL = 0` means either "thruster power is live" or "nobody is
telling us"**, and no consumer can tell them apart.

## Why it matters

Nothing refuses a cut kill switch, on either side. `canArm()` checks IMU, leak
and pack voltage and never looks at the kill state — so on a hull whose rotary
switch is outside its ON windows:

- arming succeeds,
- every motion primitive runs its full profile against unpowered ESCs,
- the payload fires,
- and the mission completes on a **motionless vehicle**.

That is the same silent-success shape you already fixed twice — the pre-rev-13
disarmed `SROT_MOVE`, and `MANUAL_CONTROL` in `SURFACE`.

**Observed on our vehicle**, in one status row: `thruster --` (no
`BATTERY_STATUS` instance 1, so no ESP-NOW link at all) printed next to
`KILL clear`.

## The ask — one line, and it is your own rule

You already do exactly this for two other values in the same function:

```cpp
// SCALED_PRESSURE2 is SUPPRESSED, not zeroed, when the baro is unhealthy or stale: the
// message has no validity field... Sending it would be asserting a pressure we know is
// unreliable. Absence is the signal.                     — mav_stream.cpp:813-815

if (s.pm2_present) sendBattery(1, s.pm2, 0);              // mav_stream.cpp:821
```

`s.pm2_present` is a member of the same `Snap`, in scope 25 lines above the
`KILL` send. Apply the rule you already wrote:

```cpp
- sendNamed(now, "KILL", s.kill ? 1.0f : 0.0f);
+ // Suppress rather than zero: the kill state arrives over ESP-NOW and the link
+ // may be down, in which case `s.kill` is a default and not a reading. Absence
+ // is the signal — same rule as SCALED_PRESSURE2 and BATTERY_STATUS id 1 above.
+ if (s.pm2_present) sendNamed(now, "KILL", s.kill ? 1.0f : 0.0f);
```

**Two follow-ons, both cheap:**

1. **`canArm()` should refuse on a KNOWN-engaged kill**, gated on
   `s.pm2_present` so a vehicle with no second board is unaffected — the same
   shape as your `FS_BAT_ENABLE` gate, which already refuses to arm into a flat
   pack "only when a fresh ESP-NOW source exists".
2. **The LoRa path cannot disambiguate at all.** `LT_FLAG_THR_LINK` looks like
   the freshness bit but is set from `g_state.thrusters.link_ok` — the **Pico**
   link, not ESP-NOW — so it does not help here, and the ground station never
   reads it anyway. A spare flag bit for ESP-NOW freshness would fix the LoRa
   half; without it a GCS on LoRa cannot know either.

Also worth correcting: `state_types.h:124`'s `(display-only)` comment, which is
what makes the current behaviour look correct on review.

## What we have already done on our side

We are not waiting on this. We fold the third state back in using
**`BATTERY_STATUS` instance 1 as the link-liveness proxy** — you suppress it
when ESP-NOW is stale, so its presence is a freshness signal we can read today.
`Telemetry.kill_switch` is now `Optional[bool]`, our status line reads
`KILL UNKNOWN (no 2nd-board link)`, and `arm()` refuses on **known**-engaged
only (never on unknown, or a bench vehicle with no second board could never arm).

We are sending this anyway for three reasons: the proxy is indirect and would
break the day `BATTERY_STATUS` gating changes; the **LoRa path has no proxy at
all**; and the board is the only place that can refuse to arm on its own.
