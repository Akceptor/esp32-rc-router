# System test checklist — ESP32 RC Signal Router

> **Status: NOT EXECUTED.** This checklist has not been run end-to-end. Executing it requires bench
> equipment not available in the environment this firmware was developed in: multiple RC
> transmitter/receiver pairs across CRSF/SBUS/MAVLink, a flight controller (or logic
> analyzer/UART capture standing in for one), a two-channel oscilloscope, a variable bench power
> supply with a wired voltage-divider circuit, and a multi-hour block of time for the soak test.
> Only a bare ESP32 dev board over USB has been used so far (see `docs/hardware-wiring.md`'s
> "Wiring verified so far" section for exactly what that covered). Section 10 (OTA update) also
> cannot pass as written — the OTA HTTP routes were intentionally left unwired in this build (see
> the top-level `README.md`'s "What's intentionally not wired" section); that section will need
> the OTA routes wired into `web_server.cpp` before it's meaningful to attempt.
>
> Every checkbox below is intentionally left unchecked. Do not check a box without personally
> observing the described result on real hardware, per this document's own instructions.

Run against a fully flashed device (firmware + filesystem image, see `README.md`) wired per
`docs/hardware-wiring.md`. Check every box only after personally observing the described result;
write the observed value next to any step marked "(record: ...)".

## 1. Wiring verification

- [ ] 1.1 Continuity-check every signal in `docs/hardware-wiring.md`'s pin-out table against the
      physical board before applying power.
- [ ] 1.2 Power up with only USB connected (no receivers/FC/PWM loads); confirm `pio device
      monitor -b 115200` shows the full boot log with no panics/reboots for 60 s.
- [ ] 1.3 Confirm the status LED lights in `DOUBLE_BLINK` (AP/config mode) on first boot with no
      WiFi configured yet.
- [ ] 1.4 Confirm `/api/status` is reachable over the fallback AP (`RC-Router-XXXX`) using the
      documented default password.

## 2. Per-protocol receiver input

- [ ] 2.1 CRSF receiver on port 0: bind a CRSF transmitter, confirm `dash-active-protocol` reads
      `CRSF`, RSSI/LQ move with transmitter distance, and `dash-active-receiver` shows `0`.
- [ ] 2.2 SBUS receiver on port 1: reconfigure port 1 to SBUS via the Receivers page, bind an SBUS
      receiver, confirm frames are decoded (`dash-lq`/`dash-rssi` non-zero, channels move on the
      Dashboard when sticks move — verify by temporarily wiring a PWM output and watching a
      servo).
- [ ] 2.3 MAVLink source on either port: point a MAVLink `RC_CHANNELS`/`RC_CHANNELS_OVERRIDE`
      stream at the configured port, confirm frames decode and `HEARTBEAT` keeps the link marked
      alive.

## 3. Cross-protocol conversion matrix (3 inputs x 3 outputs = 9 combinations)

For each cell: set the active input protocol, set the Output page's protocol, move all sticks
through their full range, and confirm the FC-side capture (scope/logic analyzer/FC's own RC input
page) shows correctly-scaled 988-2012 µs-equivalent values with no dropped frames over 30 s.

- [ ] 3.1 CRSF in -> CRSF out
- [ ] 3.2 CRSF in -> SBUS out
- [ ] 3.3 CRSF in -> MAVLink out
- [ ] 3.4 SBUS in -> CRSF out
- [ ] 3.5 SBUS in -> SBUS out
- [ ] 3.6 SBUS in -> MAVLink out
- [ ] 3.7 MAVLink in -> CRSF out
- [ ] 3.8 MAVLink in -> SBUS out
- [ ] 3.9 MAVLink in -> MAVLink out

## 4. Receiver switching

- [ ] 4.1 With receiver A active and healthy, unplug it; confirm the router fails over to receiver
      B within `switch_delay_ms` of the configured `SelectionConfig` and `switch_count` on the
      Dashboard increments by 1.
- [ ] 4.2 With both receivers healthy and A active (priority 0), degrade A's RSSI using an
      attenuator or increased distance below `rssi_threshold_percent`; confirm the FSM enters
      `EVALUATING`, then `SWITCHING`, then hands off to B once B has been better for
      `switch_delay_ms` continuously (measure this duration on a scope by toggling a spare GPIO
      or watching the LED transition from `SOLID` to `SLOW_BLINK` to failover) — (record: measured
      switch delay ___ ms, configured ___ ms).
- [ ] 4.3 Restore A; confirm the router does NOT immediately switch back until
      `min_active_time_ms` on B has elapsed AND A has beaten B by `hysteresis_percent` — (record:
      observed hold-off ___ ms).

## 5. Failsafe (all three modes)

- [ ] 5.1 `HOLD_LAST`: cut all receiver input; confirm PWM/output frames continue at the last
      good channel values indefinitely (no drift), and the Dashboard shows `failsafe: FAILSAFE`.
- [ ] 5.2 `STOP_PWM`: cut all receiver input; confirm PWM channels configured for this mode stop
      pulsing entirely (scope shows no more pulses) rather than holding or going to a fixed value.
- [ ] 5.3 `FAILSAFE_VALUES`: cut all receiver input; confirm every output channel snaps to the
      exact `failsafe_channels`/`failsafe_us` values configured on the Output/PWM pages.
- [ ] 5.4 Restore input after each mode; confirm normal operation resumes without a reboot.

## 6. PWM outputs

- [ ] 6.1 Scope-measure all 4 PWM pins in `SERVO` mode: confirm 50 Hz frame rate (20 ms period,
      +/-1%) — (record: measured Hz per pin).
- [ ] 6.2 Command full-low stick: confirm ~988 µs pulse width on the mapped channel — (record:
      measured µs).
- [ ] 6.3 Command center stick: confirm ~1500 µs — (record: measured µs).
- [ ] 6.4 Command full-high stick: confirm ~2012 µs — (record: measured µs).
- [ ] 6.5 Switch one PWM pin to `SWITCH` mode; confirm the digital output flips at
      `switch_threshold_us` with the configured `switch_active_high` polarity — (record: measured
      threshold crossing µs vs. configured value).

## 7. Voltage calibration procedure

- [ ] 7.1 Set the bench supply to a known voltage (e.g. 12.00 V measured independently with a
      trusted multimeter across the router's battery input terminals).
- [ ] 7.2 On the Voltage page, confirm the live reading is in the right ballpark given the
      as-wired divider ratio (it will likely be off before calibration).
- [ ] 7.3 Click Calibrate, enter the multimeter-measured value (12.00), confirm
      `calibration_factor` updates and the live reading now matches the multimeter within +/-1%.
- [ ] 7.4 Change the bench supply to at least two other voltages spanning the expected battery
      range (e.g. 7.4 V and 22.2 V); confirm the live reading tracks each within +/-1% without
      re-calibrating.
- [ ] 7.5 Power-cycle the router; confirm the calibration factor persisted (Voltage page still
      shows the calibrated value, live reading still accurate).

## 8. Telemetry override verification

- [ ] 8.1 With `telemetry_override` OFF, confirm the transmitter's telemetry screen shows the real
      FC-reported battery values passed through unmodified.
- [ ] 8.2 Enable `telemetry_override` on the Voltage page; confirm the transmitter's telemetry
      screen now shows the router's own ADC-derived voltage instead of the FC's, while all other
      telemetry fields (GPS, attitude, etc.) still pass through unaffected.
- [ ] 8.3 Disable it again; confirm the FC's own values return.

## 9. Web pages — desktop and phone

- [ ] 9.1 Dashboard: all fields populate and update live; log viewer shows recent lines.
- [ ] 9.2 Receivers: both ports + selection tuning save/reload correctly.
- [ ] 9.3 Output: protocol/pins/channel-map/failsafe save/reload correctly.
- [ ] 9.4 PWM: all 4 rows render, mode-dependent field show/hide works, save/reload correctly.
- [ ] 9.5 Voltage: live readings, config save/reload, calibrate button all work.
- [ ] 9.6 Network: DHCP toggle show/hide works; save/reload correctly; reboot-required is
      indicated for WiFi/static-IP changes.
- [ ] 9.7 Firmware: version displays; Restart and Factory Reset both prompt via `confirm()` before
      acting.
- [ ] 9.8 Repeat 9.1-9.7 on a phone browser: no horizontal scroll, nav wraps, controls are
      comfortably tappable.

## 10. OTA update

> **Blocked as written:** the OTA HTTP routes (`/api/ota/upload`, `/api/ota/status`) are not
> wired into `web_server.cpp` in this build (descoped per project decision — see `README.md`).
> This section cannot pass until that wiring is added.

- [ ] 10.1 Build a firmware image with a bumped `FIRMWARE_VERSION`, upload it via the Firmware
      page's file picker and Upload button; confirm the progress bar advances to 100%.
- [ ] 10.2 Confirm the status LED shows `TRIPLE_BLINK` throughout.
- [ ] 10.3 Confirm the RC link and PWM outputs keep running (no glitches, no dropped frames)
      during the entire upload.
- [ ] 10.4 Confirm the device restarts ~1.5 s after reaching 100% and comes back up running the
      new `firmware_version`.
- [ ] 10.5 Also verify via `curl -F "firmware=@.pio/build/esp32dev/firmware.bin"
      http://rc-router.local/api/ota/upload` from a terminal, polling `/api/ota/status` in
      parallel.

## 11. Factory reset

- [ ] 11.1 Change several settings across multiple pages, save each.
- [ ] 11.2 Trigger Factory Reset from the Firmware page (confirm dialog appears).
- [ ] 11.3 Confirm the device reboots into `configLoadDefaults()` values on every page.

## 12. Config persistence across power cycle

- [ ] 12.1 Change a setting on every one of the 7 pages, saving each.
- [ ] 12.2 Hard power-cycle the device (remove and reapply battery/USB power, not just Restart).
- [ ] 12.3 Confirm every changed setting survived and reloads correctly on every page.

## 13. Watchdog / soak test

- [ ] 13.1 Run the fully wired system continuously for 2 hours under representative load (active
      RC link, PWM outputs moving, web UI dashboard open and polling, telemetry flowing).
- [ ] 13.2 Confirm zero unexpected reboots (task watchdog must never fire under normal load) —
      (record: reboot count, expect 0).
- [ ] 13.3 Sample `dash-free-heap` at the start and every 15 minutes; confirm it stays flat within
      noise (no monotonic decline indicating a leak) — (record: heap samples).
- [ ] 13.4 Confirm `switch_count` does not increment during the soak unless a receiver was
      intentionally perturbed (i.e. no spurious failovers under stable signal).

## 14. Latency measurement (<5 ms added)

- [ ] 14.1 Set up a two-channel oscilloscope: channel 1 on the active receiver's UART TX line
      (input to the router), channel 2 on the router's output UART TX line (toward the FC).
- [ ] 14.2 Trigger on a sharp stick step on the transmitter (e.g. snap a switch channel from one
      extreme to the other) and capture both channels' corresponding frame edges.
- [ ] 14.3 Measure edge-to-edge time between "input frame containing the new value arrives" and
      "output frame containing the new value is transmitted."
- [ ] 14.4 Separately measure the output protocol's own native frame period with the router
      removed (receiver wired directly to the same protocol analyzer) to establish the baseline
      that is NOT attributable to the router.
- [ ] 14.5 Subtract the baseline from the end-to-end measurement; confirm the remainder — the
      router's own added latency — is under 5 ms across at least 10 trigger events — (record:
      min/avg/max added latency, expect avg ~1-2 ms, max < 5 ms per the Task 21 budget analysis).

## Acceptance criteria

| Requirement                                   | How verified                                   | Pass condition                                  |
|------------------------------------------------|-------------------------------------------------|--------------------------------------------------|
| 3 input protocols supported                    | Section 2                                       | All 3 decode frames correctly                     |
| 9-way protocol conversion matrix                | Section 3                                       | All 9 cells produce correct scaled output          |
| Automatic receiver failover                     | Section 4.1                                     | Fails over within configured `switch_delay_ms`     |
| Failover hysteresis prevents flapping           | Section 4.2-4.3                                 | No switch-back before `min_active_time_ms` + `hysteresis_percent` satisfied |
| All 3 failsafe modes behave per spec            | Section 5                                       | Each mode's documented output behavior observed    |
| PWM servo timing accurate                       | Section 6.1-6.4                                 | 50 Hz +/-1%, 988/1500/2012 µs +/-1% at range ends  |
| PWM switch mode threshold accurate              | Section 6.5                                     | Digital transition within a few µs of configured threshold |
| Voltage calibration accurate                    | Section 7                                       | Reading within +/-1% of multimeter across 3+ voltages, persists across power cycle |
| Telemetry override works without breaking passthrough | Section 8                                  | Only battery fields replaced when enabled, everything else passes through |
| All 7 web pages functional, desktop + phone     | Section 9                                       | Every field loads/saves/reloads correctly on both form factors |
| OTA update succeeds without disrupting RC link  | Section 10                                      | New version boots, zero RC/PWM glitches during flash (blocked — see note above) |
| Factory reset restores defaults                 | Section 11                                      | Every page matches `configLoadDefaults()` after reset |
| Config persists across power cycle              | Section 12                                      | Every changed setting survives a hard power cycle  |
| No watchdog reboots / stable heap over 2 h       | Section 13                                      | 0 reboots, heap flat within noise                  |
| Added latency < 5 ms                             | Section 14                                      | Measured added latency (input edge to output edge, minus protocol's own native frame period) < 5 ms worst case |

## Definition of done

- [ ] Every checkbox in sections 1-14 above is checked, with every `(record: ...)` field filled in
      with actually-observed values (not placeholders).
- [ ] The acceptance-criteria table's "Pass condition" column is satisfied for every row, with the
      corresponding recorded measurement written alongside it in this file.
- [ ] `pio test -e native -v` passes for the full suite (Tasks 1-18's native-testable modules).
- [ ] `pio run -e esp32dev` builds cleanly with zero warnings introduced by this plan's tasks.
- [ ] `pio run -t buildfs -e esp32dev` and `pio run -t uploadfs -e esp32dev` both succeed.
- [ ] The 2-hour soak test (section 13) has been run at least once end-to-end with zero reboots.
- [ ] An OTA update has been performed at least once successfully on real hardware (section 10)
      — requires wiring the OTA routes first, see the note under Section 10 above.
- [ ] A factory reset has been performed at least once and verified (section 11).
- [ ] This checklist and `docs/hardware-wiring.md` are committed to the repository as the
      permanent record of the validation run.
