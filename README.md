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

## Multiple ACs: shared package + substitutions

If you have more than one Fujitsu AC, don't copy-paste the platform config
into every device's YAML — use ESPHome's `packages:` + `substitutions:`
instead, so a sensor rename/add/removal is a single edit that every device
picks up on its next compile.

```
fujitsu_ac_esphome/
  common/
    fujitsu-ac-common.yaml   # all entities + uart/external_components — edit here
  devices/
    lounge.yaml              # ~10 lines: name, board, pins, secrets
    bedroom.yaml
    secrets.yaml              # wifi + per-device api key / ota password
```

`common/fujitsu-ac-common.yaml` holds everything: `external_components`,
`uart`, the `fujitsu_ac` hub, and the `climate` / `sensor` / `select` /
`switch` blocks — all parameterised with `${...}` substitutions.

Each file in `devices/` is just:

```yaml
substitutions:
  name: "lounge-ac"
  friendly_name: "Lounge AC"
  board: "esp32dev"
  rx_pin: "GPIO16"
  tx_pin: "GPIO17"
  api_encryption_key: !secret lounge_ac_api_key
  ota_password: !secret lounge_ac_ota_password

packages:
  fujitsu_ac: !include ../common/fujitsu-ac-common.yaml
```

Substitutions declared in the device file override the defaults of the
same name in the package — that's the whole mechanism. Compile/flash each
device the normal way, just pointed at its own file:

```
esphome run devices/lounge.yaml
esphome run devices/bedroom.yaml
```

To add a sensor for every AC at once, add it once under `sensor:` in
`fujitsu-ac-common.yaml`, then re-run/upload each `devices/*.yaml` — no
per-device edits needed. This was validated end-to-end (two devices with
different boards/pins, package merge, and a common-file edit propagating to
both) against ESPHome 2026.6.5.

`secrets.yaml` must live next to the files you actually compile (i.e. in
`devices/`, since that's what you pass to `esphome run`) — see
`devices/secrets.yaml.example`.

If you'd rather not maintain `components/fujitsu_ac` locally, the
`external_components` block in the common package can point at a git repo
instead (see the `type: git` example already in `fujitsu-ac-common.yaml`).
You can do the same for the package itself — `packages:` supports a `git`
source with the same `url`/`ref`/`refresh` fields, so the whole
`fujitsu-ac-common.yaml` can live in a shared repo too if you want it
versioned separately from your per-device configs.

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
