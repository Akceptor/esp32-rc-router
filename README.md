# ESP32 RC Signal Router

Firmware that sits between two RC receivers and a flight controller, continuously scores both
receiver links, forwards the best one to the FC in any configured serial protocol (CRSF / SBUS /
MAVLink, any-in to any-out), and exposes PWM outputs, voltage telemetry, and a browser-based
configuration UI served offline from the device itself.

See `2026-08-03-esp32-rc-router.md` for the full implementation plan this firmware was built from.
See `docs/hardware-wiring.md` for pin-out and bench wiring, and `test/system/CHECKLIST.md` for the
end-to-end hardware validation checklist (not yet executed — see that file's status note).

## Prerequisites

- [PlatformIO Core](https://platformio.org/) (`pio` on your `PATH`).
- For hardware flashing: a USB-to-serial-capable ESP32 dev board (`board = esp32dev` in
  `platformio.ini`), connected via USB.

## Running the native test suite

All logic modules (protocols, receiver management, config, PWM, telemetry, etc.) are unit-tested
in a `native` PlatformIO environment with no Arduino/ESP32 dependency:

```sh
pio test -e native -v
```

To run a single test folder, PlatformIO's filter must include the folder path under `test/`, not
just the test's own name (this project's test folders all live under `test/native/<name>/`):

```sh
pio test -e native -f "native/test_logger" -v
```

The bare form (`-f test_logger`, without the `native/` prefix) silently reports every test as
`SKIPPED` in this environment — always use the prefixed form above.

## Building and flashing to hardware

The firmware has two separately-flashed images: the compiled program itself, and a LittleFS
filesystem image containing the web UI (`data/index.html`, `data/style.css`, `data/app.js`).
**Both must be flashed** — flashing only the firmware leaves the web server running with no files
to serve (you'll get an HTTP 404 "Not Found" at `/`).

1. **Build the firmware:**

   ```sh
   pio run -e esp32dev
   ```

2. **Find your board's serial port.** On macOS, USB-serial adapters show up as both a `/dev/cu.*`
   and a `/dev/tty.*` device for the same physical port:

   ```sh
   pio device list
   ```

   **Use the `/dev/tty.*` path, not `/dev/cu.*`, for flashing and monitoring.** On at least one
   configuration encountered during development, the `cu.` node accepted opens but rejected every
   `tcsetattr`/baud-rate change (`Invalid argument`) even from a bare `stty` command, while the
   `tty.` node for the identical physical device worked immediately. If flashing or the serial
   monitor fails with a baud-rate/termios error, try the `tty.` device path before assuming a
   missing driver.

3. **Flash the firmware:**

   ```sh
   pio run -e esp32dev -t upload --upload-port /dev/tty.usbserial-XXXX
   ```

4. **Build and flash the filesystem image (the web UI):**

   ```sh
   pio run -t buildfs -e esp32dev
   pio run -t uploadfs -e esp32dev --upload-port /dev/tty.usbserial-XXXX
   ```

   Re-run these two commands any time `data/*` changes; the firmware upload in step 3 does not
   touch the filesystem partition, and vice versa.

5. **Watch the boot log:**

   ```sh
   pio device monitor -p /dev/tty.usbserial-XXXX -b 115200
   ```

   **Known limitation:** the Output port (`OutputManager`, talking to the flight controller) and
   the USB debug console share the same physical UART0 peripheral. Early boot log lines ("firmware
   booting", "config loaded, revision=...", WiFi/mDNS status) print normally, but once
   `output_manager_.begin()` runs during `setup()`, UART0 is reassigned to the Output port's pins
   (GPIO 22/23) and further `Serial`/Logger console output stops being visible over USB — this is
   expected, not a fault. Once WiFi is up, use the web UI's `/api/logs` page (backed by an in-RAM
   ring buffer, see `src/logging/logger.h`) as the ongoing diagnostic channel instead of the live
   serial console.

## First boot and connecting to the web UI

On first boot (no WiFi configured yet), the device falls back to its own WiFi access point:

- **SSID:** `RC-Router-XXXX`, where `XXXX` is the last two bytes of the device's WiFi MAC address
  in hex (e.g. MAC `68:09:47:1e:a9:e8` -> SSID `RC-Router-A9E8`). The MAC/SSID is logged during the
  early boot window described above, or can be found via your OS's WiFi network list.
- **Password:** `rcrouter123` (documented default; the firmware intentionally logs this rather than
  hiding it, since it's the only way to reach a brand-new device — change it via the Network page
  once connected, if the deployment isn't physically secured).

Connect to that AP, then browse to `http://192.168.4.1/` (the ESP32 AP's default gateway IP) to
reach the dashboard and configuration pages. Once a real WiFi network is configured via the Network
page and the device is rebooted, it joins that network instead and becomes reachable at
`http://rc-router.local/` (via mDNS) or its DHCP/static-assigned IP.

## What's intentionally not wired

- **OTA firmware updates:** `OtaManager` exists as a class (flash-write state machine wrapping the
  Arduino `Update` API) but its HTTP routes (`/api/ota/upload`, `/api/ota/status`) are not
  registered in `web_server.cpp` — OTA was descoped for this build. The Firmware page's OTA upload
  widget is present in the UI but will fail (404/connection error) if used; this is expected.
- **End-to-end hardware validation:** `test/system/CHECKLIST.md` documents the full bench-test
  procedure (multi-protocol receivers, oscilloscope latency measurement, 2-hour soak test, voltage
  calibration, etc.) but has not been executed — see that file's status note for what bench
  equipment executing it would require.
