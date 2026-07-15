# fujitsu_ac — ESPHome external component

Native ESPHome port of the UART protocol from [Benas09/FujitsuAC](https://github.com/Benas09/FujitsuAC)
(UTY-TFSXW1 / FGLair dongle replacement). No MQTT bridge — the AC is exposed
directly through the ESPHome native API as a climate entity plus supporting
entities.

Validated against ESPHome 2026.6.5 (config validation, codegen, and a header
syntax check). Not yet tested against real hardware — watch the DEBUG log on
first boot; you should see `Handshake complete, polling started` within a
couple of seconds.

## Entities

| Entity | Maps to |
|---|---|
| `climate` | power + mode (off/auto/cool/dry/fan_only/heat), setpoint (16–30 °C, 0.5° steps), fan (auto/quiet/low/medium/high), swing (off/vertical/horizontal/both), current temperature |
| `sensor.outdoor_temperature` | register 0x2020, `(raw − 5025)/100 °C` |
| `sensor.indoor_temperature` | register 0x1033 (same probe the climate entity uses) |
| `select.vertical_airflow` / `select.horizontal_airflow` | louvre position 1–6 or Swing |
| `switch.*` | powerful, economy, energy_saving_fan, outdoor_unit_low_noise, coil_dry, human_sensor, minimum_heat |

Behavioural notes:

- Swing modes and feature switches are gated on the capability registers the
  unit reports during the handshake, same as the original dongle. Unsupported
  commands are refused with a log warning and the UI control snaps back.
- Mode/fan/setpoint changes are blocked while coil dry or minimum heat is
  active (mirrors upstream guards).
- `minimum_heat` writes are refused by default because upstream has never
  tested them (`setMinimumHeat` is stubbed out in the original library). Set
  `enable_minimum_heat_control: true` on the hub to opt in — at your own risk.
- Setpoint is snapped to 0.5 °C and clamped to 16.0 °C (heat) / 18.0 °C
  (other modes) minimum, 30.0 °C maximum, matching the original library.
- Fan-only mode reports setpoint `0xFFFF`; it is ignored rather than published.
- Writes are queued, coalesced into a single write frame at the next slot in
  the 400 ms poll cycle, acked, then read back immediately so HA state
  converges within ~1 s. Lost write frames are retried automatically.

## Wiring

Identical to the upstream project (12 V → 5 V buck, logic level shifter,
GPIO16 = AC→ESP, GPIO17 = ESP→AC on a classic ESP32). **The AC's UART is not
galvanically isolated — never connect its GND to an earth-referenced device.**

The one thing that is easy to miss: the bus is **inverted-logic 9600 8N1**.
The original library calls `uart_set_line_inverse()` on both lines; in
ESPHome that translates to `inverted: true` on **both** `rx_pin` and
`tx_pin`. Without it the handshake never completes.

## Install

Copy `components/fujitsu_ac` next to your YAML and use
`external_components` with a local source (see `fujitsu-ac-example.yaml`),
or host it in a git repo and point `source` at that.

## Differences from / fixes to the original UART code

The frame parser here is a hardened rewrite of `Buffer.cpp`:

1. **Bounds checking** — the original never checks `currentIndex` against the
   128-byte buffer, and a corrupted length byte (`buffer[4]` up to 255 ⇒
   expected frame of 262 bytes) both overruns the buffer and stalls the
   parser. This port validates the length byte, resets on overflow, and
   resyncs on line noise.
2. **Parser reset after a complete frame** — the original only resets its
   index on a ≥20 ms inter-byte gap, so two back-to-back frames would be
   concatenated and dropped. This port resets immediately after dispatching
   a frame.
3. **Handshake comparison bug** — the original validates init replies with
   `memcmp(...) > 0`, which accepts any reply that compares *less than* the
   expected bytes. This port compares for equality.
4. **Recovery instead of termination** — the original sets a permanent
   `terminated` flag on an unexpected init reply and permanently stops
   polling after an unacknowledged timeout. This port retries the handshake
   and re-initialises after repeated timeouts.
