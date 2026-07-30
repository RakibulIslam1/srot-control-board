# AGENTS.md — srot-control-board (firmware "Hengla")

Instructions for AI agents and developers working in this repo. A matching file exists in each
sibling repo; the **Shared invariants** section below is deliberately identical in all of them.

---

## What this repo is

The **SROT** control board firmware: an **ESP32** flight core + an **RP2350 Pico** thruster/RPM
co-processor, for a `vectored_6dof` AUV (8× T200). It speaks **MAVLink 2** as sysid/compid
**1/1** over **USB serial at 115200**, and owns every real-time control loop at **500 Hz**:
attitude, depth, thrust allocation, the timed AUTO primitives, arming and all failsafes.

Read `ARCHITECTURE.md` first, then `ALGORITHMS.md`. `AUDIT.md` is the record of what has
already gone wrong — read it before assuming a piece of oddly-shaped code is accidental.

## Where it sits

```
Jetson Orin Nano  ──USB serial 115200──  SROT board  ──1 Mbaud UART──  RP2350 Pico ── 8× ESC
  (duburi_ws:                              (this repo)                   (thruster/RPM)
   ROS 2, YOLO11,
   missions, payload)                          │
                                               └── LoRa ── Bondor GCS (srot-ground-station)
```

There is **no Raspberry Pi and no BlueOS**. The Jetson connects to this board's USB-C port
directly. Bondor is a parallel, independent link — it is not in the control path.

**The division of labour is deliberate:** the Jetson does perception and mission logic; this
board does all control. See `VISION_API.md` for the vision half of that contract, which is
specified but not yet implemented.

## Companion documents

| File | What it is |
|---|---|
| `JETSON_COMMS.md` | the wire protocol the companion codes against — **treat as a contract** |
| `DUBURI_WS_INTEGRATION.md` | the companion-side migration guide |
| `JETSON_FEEDBACK.md` | findings sent **to us** by the companion team, ranked by what blocks them |
| `VISION_API.md` | the vision-guided control spec (not yet implemented) |
| `AUDIT.md` | three rounds of defect findings + what is deferred and why |
| `ROADMAP.md` | phase status and the ranked upgrade list |

---

## Shared invariants (identical in every repo's AGENTS.md)

These are **co-owned across repos**. Changing one unilaterally breaks a partner silently — no
exception is raised, the vehicle just behaves wrong.

1. **The wire constants are frozen unless changed on both sides in the same PR.**
   `MAV_CMD_SROT_MOVE = 31000`; the `SROT_MOVE` p1 type codes and their ordering; the
   `FlightMode` integers; `PCA_RELAY_BASE_CH = 8`; `MAVLINK_BAUD = 115200`;
   `GCS_FAILSAFE_MS = 5000`.
   `duburi_ws` mirrors all of these in `fc/srot_protocol.py` and has a test
   (`test_srot_protocol_drift.py`) that **reads this repo's headers directly** and fails on
   divergence. That test is the safety net — do not defeat it, and do expect it to catch you.

2. **`movement::Type` is append-only.** The wire mapping is `mv_type = wire + 1`, so inserting
   a value silently renumbers every verb after it — `forward` would become `strafe`. Add at the
   end and raise the bound in `mav_commands.cpp` in the same commit.

3. **Every command reaches exactly one terminal ACK.** `ACCEPTED` / `CANCELLED` / `FAILED` /
   `DENIED`, plus `TEMPORARILY_REJECTED` on a mutex miss. `IN_PROGRESS` is not terminal. A
   companion action client that never receives a terminal result **hangs**, which is worse
   than any error. (This guarantee is currently *violated* on a mid-move failsafe —
   `JETSON_FEEDBACK.md` §1.)

4. **Depth sign: `VFR_HUD.alt` is negative below the surface.** Positive-down internally is
   fine; the wire convention is what matters. The whole companion stack compares depth against
   negative constants, so a sign flip does not error — it silently disables every depth guard.

5. **A heartbeat ≥ 1 Hz is mandatory** or this board surfaces after `GCS_FAILSAFE_MS`.

6. **Never break the contract to fix a bug.** If the right fix changes the wire, say so and
   coordinate — do not add a compensating hack on one side. Both codebases are in active
   development; a clean change on both sides is cheaper than a workaround that outlives its
   reason.

---

## Rules specific to this repo

**Safety-first ordering.** The in-loop precedence is: safety-monitor disarm > failsafe SURFACE
> mode > motor-test override > mixer. Anything new must state where it sits and what
STATUSTEXT the operator sees when it is overridden.

**Any new automatic manoeuvre must be added to all of:** the mode-entry reset, the
ARMED→DISARMED abort list, and the safety-monitor gate list. Omitting the abort entry means a
panic-disarm mid-manoeuvre resumes on re-arm — see the comment in `task_control_loop.cpp`
explaining exactly that bug.

**Every external input gets the freshness triple** — `value` + `stamp_ms` + `valid`, with a
staleness window and a defined degraded behaviour. `ESPNOW_STALE_MS`, `DEPTH_STALE_MS`,
`PICO_LINK_TIMEOUT_MS`, `GCS_FAILSAFE_MS` are the precedents. A stale input that keeps being
used is the recurring bug class in this codebase.

**Core discipline.** Core 0 = comms/UI; Core 1 = sensors/control/DShot. Core 1 must not write
UART0 (use `queueStatusText()`) and must not touch flash. Never nest `StateLock`s. Real-time
callers use short timeouts and skip the cycle on a miss.

**NaN is a real attacker here.** Every MAVLink entry point must reject non-finite values —
there are three now (`dispatchCommand`, `onParamSet`, and any new message handler), and
`constrain(NaN,…)` returns NaN, which reaches `(int16_t)NaN` on an armed thruster.

**Do not bump `PARAM_DEFAULTS_VER` to add a parameter.** Adding is free — an absent NVS key
falls back to its default. A bump discards all user tuning and has already cost four pool
sessions.

**Docs are part of the change.** This repo's docs have drifted from the code repeatedly
(`ARCHITECTURE.md` still claimed an ArduSub identity that was deleted; parameter counts and
defaults are quoted wrong in three files). If you change behaviour, fix the doc in the same
commit. When a doc and the code disagree, **the code is ground truth** — fix the doc, never
"fix" the code to match a stale claim.

**Deferred, not forgotten** (`AUDIT.md`): the Pico e-stop fails *permissive*; the LoRa mission
upload has no CRC. Neither is on the ROS↔board path, but do not let them decay further.

---

## When your change crosses a repo boundary

1. Say so explicitly in the commit message, naming the other repo.
2. Update `JETSON_COMMS.md` — it is the contract the companion codes against.
3. Expect `duburi_ws`'s drift test to fail until its mirror is updated; that is the design.
4. If it changes a parameter's meaning or units, note whether existing tuning survives.
