# Hengla with BlueOS

**Short version:** BlueOS works well as *plumbing* — MAVLink routing, MAVLink2Rest, video,
networking, a web terminal. Its **vehicle-configuration pages will not work**, because those are
gated on an ArduPilot autopilot and Hengla honestly reports that it is not one. Bondor is the
configurator. Nothing here is a bug to be fixed; it is a consequence of not pretending to be
ArduSub.

## Why

Hengla's heartbeat is `MAV_TYPE_SUBMARINE` + **`MAV_AUTOPILOT_GENERIC`**
([`mav_stream.cpp`](../src/comms/mav_stream.cpp)). BlueOS's and QGroundControl's setup UIs load
their ArduPilot/ArduSub plugin from that autopilot id, and gate features on a reported ArduPilot
firmware version. With `GENERIC` they never load.

The firmware previously carried an `APM_COMPAT_*` block intended to spoof "ArduSub V4.1.0" for
exactly this reason. It was dead code — nothing referenced it — and has been removed rather than
revived. See [`AUDIT.md`](../AUDIT.md) B10.

## What works

| Capability | Notes |
|---|---|
| **mavlink-router** | Transport-agnostic. Routes Hengla's MAVLink to as many endpoints as you like (QGC, a Jetson, MAVLink2Rest) without caring what autopilot it is. |
| **MAVLink2Rest** | Exposes **every** message over HTTP/WebSocket, including `MAV_CMD_SROT_MOVE` (31000) and all `NAMED_VALUE_FLOAT`s. This is the cleanest way for a non-ROS script to drive the board. |
| **Telemetry display** | Attitude, depth, battery and armed state all populate — they are standard messages. |
| **Video streaming** | Entirely independent of the autopilot. |
| **Networking / WiFi / tether** | Independent. |
| **Web terminal, file manager, logs** | Independent. |
| **Serial passthrough** | Useful for pointing Bondor at the board *through* BlueOS. |

## What does not work, and cannot

| Feature | Why |
|---|---|
| **Vehicle Setup** pages | ArduPilot-gated. Use Bondor's Setup tab (calibration, motor test/reverse/detect). |
| **Parameter descriptions / ranges** | BlueOS looks these up in ArduPilot metadata, keyed by firmware version. Parameters still list and edit over the standard protocol, just without help text. Bondor carries its own metadata for all of them. |
| **Firmware upload / board detection** | Expects an ArduPilot board over its bootloader. Flash with PlatformIO. |
| **ArduSub-specific widgets** (lights, gripper presets, camera mount) | Keyed to ArduSub parameter names. Hengla's aux outputs are `SERVOn_ROLE` + `DO_SET_SERVO`/`DO_SET_RELAY`; drive them from Bondor. |
| **Autopilot manager health/restart** | It manages an ArduPilot process, not a separate MCU. |

## Setting it up

1. Wire the board's UART0 (GPIO1 TX / GPIO3 RX, **115200**) to the Pi — via a USB-serial adapter
   or directly to the Pi's UART at 3.3 V.
2. In BlueOS, **disable the ArduPilot autopilot manager** (or set the board type to
   *Other/Custom*) so it stops trying to manage a Pixhawk that is not there.
3. Add a **serial endpoint** at 115200 pointing at that device, and whatever UDP endpoints you
   want fanned out.
4. Confirm the vehicle appears with a heartbeat and that `ATTITUDE` is flowing (MAVLink
   Inspector, or MAVLink2Rest's message list).
5. Configure the vehicle from **Bondor** — connect it to a BlueOS UDP endpoint, or straight to
   the board over USB.

## Things that will bite you

- **A parameter download suppresses most telemetry.** During a `PARAM_REQUEST_LIST` the board
  sends only `HEARTBEAT` (1 Hz), `ESC_STATUS`, and move ACKs. Do not treat that as a dropout.
  (Move ACKs are exempt as of this revision — a move completing during a download used to lose
  its terminal ACK entirely; see [`AUDIT.md`](../AUDIT.md) B1.)
- **Telemetry rates are fixed.** Neither `REQUEST_DATA_STREAM` (66) nor
  `MAV_CMD_SET_MESSAGE_INTERVAL` (511) is implemented, so a GCS asking for different rates is
  refused. Harmless over USB/UDP; it does mean you cannot currently turn rates *down* for a LoRa
  link from the GCS side. On the roadmap.
- **`MAV_AUTOPILOT_GENERIC` may make some tools cautious.** Anything that branches on autopilot
  type to decide whether arming is safe may behave conservatively. MAVLink2Rest and
  mavlink-router do not care.
- **`BATTERY_STATUS` id 1 is the thruster pack**, and it only reports once the 2nd board is
  broadcasting over ESP-NOW (`env:second-board`) *and* `ESPNOW_EN = 1` on the control board.
  Before that it is silent — not zero.
- **Depth sign.** Internally depth is positive-down metres; on the wire `VFR_HUD.alt = −depth`,
  i.e. **negative underwater**, per MAVLink convention. Full table in
  [`JETSON_COMMS.md`](../JETSON_COMMS.md).

## If you decide you want the ArduSub pages back

It would take a runtime parameter that switches the heartbeat to `MAV_AUTOPILOT_ARDUPILOTMEGA`
and reports an ArduSub version — roughly 20 lines. It was deliberately **not** added, because it
reintroduces exactly the disguise that was just removed, and a half-working ArduSub UI driving a
firmware whose parameters only partly match is a good way to break a vehicle. If the need is
real, prefer extending Bondor.
