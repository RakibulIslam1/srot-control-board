# PR B — log the decisions you are already making

**Repo:** `srot-control-board` · **Severity:** low code risk, high diagnostic value
**Costed, not wished:** every ask below has a byte count against a measured budget.

---

## 0. Context: your own doc understates what you have

`docs/VS_ARDUSUB.md:114` calls replay logging "the biggest practical gap" versus
ArduSub. That is **not true of this firmware** — `src/drivers/sd_log.{h,cpp}`
writes a packed `Record` every 50 ms behind a `"SROTLOG1"` header. (Our first
grep looked in `src/comms/` and missed it; the flawed check was ours, not your
doc's, and we are correcting both.)

The gap is not that the log is absent. It is that the log records **what the
vehicle did** and not **what it decided**:

```c
struct Record {                 // 62 bytes @ 20 Hz
    uint32_t t_ms;
    float    qw, qx, qy, qz;    // orientation
    float    gx, gy, gz;        // gyro
    float    depth;
    int16_t  thr[8];            // DShot values -- the OUTPUT
    uint8_t  mode, armed;
    float    pm1, pm2;
};
```

No setpoints, no PID terms, no saturation, no `norm[]`. After a bad dive you can
see the hull moved and what the ESCs were told, and you cannot see what the
controller was *trying* to do — which is the question.

---

## 1. `ControlState.out_*` has zero writers — six lines

`state_types.h:157-158` declares `out_roll, out_pitch, out_yaw, out_throttle,
out_forward, out_lateral`. The **entire tree contains one reference**:
`mav_stream.cpp:78` *reads* `c.out_throttle`. Nothing ever assigns them.

Measured consequence, on a 44 s capture from the live board:

```
VFR_HUD frames: 429      distinct throttle values: {0: 429}
```

**`VFR_HUD.throttle` is a hardcoded zero at 5 Hz**, and has been for every dive.

The locals are still in scope at the end of `Task_ControlLoop`
(`task_control_loop.cpp:902-924`) — this is six assignments before the mixer
call, and it fixes the field for free.

## 2. Enrich the `Record` you already write — **+54 bytes, 0.116 % of the card**

| field | bytes | why |
|---|---|---|
| `float sp[6]` — roll/pitch/yaw/thr/fwd/lat demands | 24 | the SETPOINT. Without it a log cannot distinguish "asked for the wrong thing" from "asked right, tracked badly" |
| `float out[6]` — post-mixer normalised, pre-DShot | 24 | `thr[8]` is after the DShot mapping; a saturated axis is unrecoverable from it |
| `uint8_t sat[2]` — the mixer's per-group scale-down | 2 | you compute `maxabs[2]` twice per cycle at 500 Hz and discard it. Saturation is invisible in the output by definition — every motor just looks smaller |
| `uint32_t loop_dt_us` | 4 | real loop jitter, from `loop_stamp_ms`'s delta |

**+54 bytes/record at 20 Hz = +1080 B/s.** A class-10 card sustains ~2 MB/s, so
this is **0.116 %** of it. **Zero MAVLink bandwidth** — it never leaves the card.

This is not "add logging". It is "write down the numbers already in registers".

## 3. Optional: the same twelve scalars live, if you want them

Only if you want them on the wire; the SD half above is the one we are asking
for.

| option | msgs | bytes/s @ 20 Hz | RX budget |
|---|---|---|---|
| `ATTITUDE_TARGET` (83) + `ACTUATOR_CONTROL_TARGET` (140) | 2 | **2040** | 37 % → **55 %** |
| the same twelve scalars as `NAMED_VALUE_FLOAT` | 12 | **7200** | → 62 % |

**3.5× worse** for identical information, which is the argument for structured
messages over more named channels. 55 % stays inside the 60 % mark we treat as
the ceiling on a 115200 line already carrying ~5.5 kB/s (measured).

## 4. What we do with it

`duburi_ws` now records the **raw MAVLink stream both ways** to a `.tlog` and
replays it (`tools/srot_replay.py`), reconstructing every console panel from the
file alone. That log holds the host's decisions; yours holds the board's. With
§1 and §2 the two together cover the whole loop.

**They cannot yet be put on one time axis** — see PR D: `TIMESYNC` (111) is
silent, so board `time_boot_ms` and host wall-clock have no relation. Ten lines
there make this section worth twice as much.

---

## 5. Filed separately

### Safety — the pre-arm bypass

`mav_commands.cpp:238`:

```cpp
case JS_ARM:        setArmed(true);  ...
case JS_ARM_TOGGLE: setArmed(!armed); ...
```

Neither consults `canArm()`. The `COMMAND_LONG` path at `:301` checks
correctly, so a joystick button can arm **into a detected leak** where the
documented command path refuses. `arming.cpp` exists precisely to prevent that.

Please route both through `canArm()` and report the refusal the same way the
command path does.

*(Related, from our side: `LEAK_EN = 0.0` on the vehicle's board right now, so
the leak pre-arm and the leak failsafe are both disabled. We verified the sensor
reads DRY with it enabled, so turning it on looks safe — but while it is 0 the
bypass above is moot and the failsafe is absent, which is worth knowing
together.)*

### Doc drift (low)

- `docs/VS_ARDUSUB.md:114` — says you have no replay logging; `sd_log.cpp` says
  otherwise.
- `include/config.h` §6 header — *"Rates: control loop 200 Hz; sensor read
  200 Hz; DShot output 200 Hz"*, while `CONTROL_LOOP_HZ`, `TASK_SENSOR_HZ` and
  `TASK_DSHOT_HZ` are all **500**.
- `PARAMETERS.md` never mentions **`FRAME_REVERSE`**, a parameter that negates
  all six axis demands — and it is **set to 1** on this hull.
- `MOT_n_DIRECTION` defaults documented as 1; the live banner reads
  `DIR=-1,1,1,1,1,1,1,-1`.
