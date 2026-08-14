# Hardware wiring reference

Pin defaults are exactly the values `configLoadDefaults()` writes into `RouterConfig` (see
`src/config/config_types.h`); the web UI can remap any of them, but this is the as-shipped wiring
the system test checklist (`test/system/CHECKLIST.md`) assumes unless a step says otherwise.

## ESP32 dev board pin-out

| Signal              | GPIO | Notes                                              |
|---------------------|------|-----------------------------------------------------|
| Receiver A RX        | 16   | UART1 RX, receiver port 0 (priority 0 by default)   |
| Receiver A TX        | 17   | UART1 TX (telemetry back to receiver A, if used)    |
| Receiver B RX        | 18   | UART2 RX, receiver port 1 (priority 1 by default)   |
| Receiver B TX        | 19   | UART2 TX                                            |
| Output (to FC) RX    | 22   | UART0-alt RX — telemetry FROM the flight controller |
| Output (to FC) TX    | 23   | UART0-alt TX — RC frames TO the flight controller    |
| PWM 0                | 25   | Servo/switch output channel 0                        |
| PWM 1                | 26   | Servo/switch output channel 1                        |
| PWM 2                | 27   | Servo/switch output channel 2                        |
| PWM 3                | 32   | Servo/switch output channel 3                        |
| Voltage ADC          | 33   | Through an external resistor divider (see below)     |
| Status LED           | 2    | Active-high, on-board LED on most esp32dev boards    |

**Note on the output port sharing UART0:** `Esp32UartPort` reassigns UART0's RX/TX signals to
GPIO 22/23 via the ESP32 GPIO matrix when the output port begins. This means the hardware USB
debug console (also UART0, on GPIO1/3 by default) and this reassigned pair are the SAME peripheral
used differently — once `OutputManager::begin()` runs, `Serial`/Logger console output over USB
stops being visible. This was confirmed empirically on real hardware during Task 21's bring-up:
the boot log prints normally up through "config loaded, revision=..." and then goes silent the
moment the output port initializes; the device keeps running correctly (no crash/reboot), it's
just no longer observable via the USB serial console from that point on. Use the in-RAM `Logger`
ring buffer via the web UI's `/api/logs` page as the primary debug channel once the output port is
live, or the early boot window (before receivers/output are configured) for USB-console-based
checks.

## Voltage divider

Battery-positive -> R1 (e.g. 10 kΩ) -> ADC pin 33 -> R2 (e.g. 1.5 kΩ) -> GND. This gives a
divider_ratio of `(R1+R2)/R2 ≈ 7.667` for a rough starting config value; always run the Calibrate
procedure (Voltage page, or `POST /api/voltage/calibrate`) against a bench multimeter reading
rather than trusting resistor tolerances.

## Bench equipment (for full system test execution)

- 3x transmitter/receiver pairs or equivalent signal sources: one CRSF, one SBUS, one MAVLink
  (a companion computer or GCS emitting `RC_CHANNELS_OVERRIDE`/`HEARTBEAT` is sufficient for the
  MAVLink leg if a physical MAVLink RC receiver is unavailable).
- A flight controller (or a UART capture / logic analyzer standing in for one) able to receive
  each of CRSF, SBUS, and MAVLink and to source its own telemetry stream back.
- Two-channel oscilloscope, 20 MHz+ bandwidth, with two probes.
- Bench power supply capable of 2S-6S LiPo-equivalent voltages (7-25 V), current-limited.
- WiFi access point (or the router's own AP-mode fallback), laptop + phone browser.
- Attenuator or physical distance/orientation control for the CRSF/SBUS transmitter, to force
  RSSI/LQ degradation on demand for the receiver-switching tests.

## Wiring verified so far

Only a bare ESP32 dev board over USB has been used to date — no receivers, flight controller,
PWM loads, oscilloscope, or bench power supply have been connected. What has been confirmed on
real hardware:

- Firmware and filesystem image both flash successfully via `/dev/tty.*` (see the top-level
  `README.md` for the `cu.`-vs-`tty.` device-node gotcha encountered on macOS).
- Boot log is visible over USB serial up through early config load, then goes silent once the
  output port claims UART0 (expected, see note above) — device does not crash or reboot.
- Device falls back to WiFi AP mode (`RC-Router-XXXX`) as expected with no WiFi configured.

No receiver/FC/PWM/voltage-divider hardware has been wired yet, so nothing in the sections above
has been physically validated beyond the bare board itself.
