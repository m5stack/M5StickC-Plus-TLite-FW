# M5Stick T-Lite Firmware

Firmware for the [M5Stack T-Lite (SKU K126)](https://docs.m5stack.com/en/app/T-Lite),
a thermal camera built from an M5StickC Plus and an MLX90640 Thermal HAT.

![T-Lite](https://static-cdn.m5stack.com/resource/docs/products/app/T-Lite/img-1b8ebe07-4556-4641-a487-b6fe6bf8b0c6.jpg)

## Supported hardware

| Main unit | Thermal sensor |
| --- | --- |
| M5StickC Plus | MLX90640 Thermal HAT |
| M5StickC Plus SE | MLX90640 Thermal HAT |

The same firmware image runs on both main units. T-Lite does not use the internal
IMU, which is absent from the Plus SE.

## Features

- Thermal image display with multiple layouts and color palettes
- Temperature range, refresh rate, noise filter, and emissivity settings
- Wi-Fi setup from a phone using an access point and QR codes
- Browser control, temperature data, and MJPEG streaming over the local network
- Access by IP address or mDNS hostname
- Battery level and charging status in the upper-right status row
- Runtime heap, task stack, and Wi-Fi diagnostics at `/health`

## Wi-Fi setup

1. On T-Lite, open **Network > WiFi Setting > AP mode**.
2. Scan the displayed Wi-Fi QR code with a phone. To connect manually, select
   `T-Lite_xxxx` and enter `12341234` as the password.
3. After the phone joins the T-Lite network, scan the new QR code or open
   `http://192.168.4.1/wifi`.
4. Select a nearby network from the SSID list, enter its password, and save.
5. Reconnect the phone to the same local network as T-Lite.

If the SSID list is still being populated, wait for the page to refresh or select
**Refresh networks**. When all phones leave the setup access point, T-Lite shows
the Wi-Fi connection QR code again.

## Browser access

After Wi-Fi setup, the Network menu provides QR codes for local access.

| Method | Address |
| --- | --- |
| IP address | The address assigned by the local router |
| mDNS | `http://t-lite-xxxx.local/` |

The home page links to browser control, text and JSON temperature data, system
health, and the MJPEG stream. If the network or phone does not support mDNS, use
the IP address instead.

The mDNS hostname changed from `T-Lite_xxxx.local` to `t-lite-xxxx.local`. Update
saved bookmarks created with older firmware.

## Network use

T-Lite never contacts the internet. Its only network activity is on the local
network:

- DHCP, to obtain an IP address and register its `t-lite-xxxx` hostname
- mDNS, to announce `t-lite-xxxx.local` on the local link
- its own HTTP server (browser control, data, and the MJPEG stream)
- in AP setup mode, a DNS responder that answers phones joined to T-Lite's own
  access point with T-Lite's address

There is no cloud upload and no NTP time sync, so the device has no wall-clock
time; the text page shows uptime instead.

**Running Mode** (Network menu) is either **Offline** (Wi-Fi off) or **LAN**.
When upgrading from firmware with cloud support, a saved *LAN* or *LAN+Cloud*
mode becomes **LAN** and *Cloud Online* becomes **Offline**, and the stored cloud
settings and confirm code are erased from flash on first boot. The `/json`
output no longer includes the `pwd`, `interval`, and `datetime` fields.

## Battery indicator

The battery icon shares the upper-right status row with the Wi-Fi icon.

| Appearance | Meaning |
| --- | --- |
| Green | Battery is charging |
| White | Battery is not charging |
| Gray | Charging state is unavailable |
| Dash inside the icon | Battery level is unavailable |

The filled portion represents the estimated battery level and updates about once
per second. A USB connection does not always mean that the battery is actively
charging.

## Build

This project uses [PlatformIO](https://platformio.org/). The framework is pinned to
Espressif32 6.12.0, which includes Arduino-ESP32 2.0.17.

Build the release firmware:

```sh
pio run -e release
```

Upload it to a connected device:

```sh
pio run -e release -t upload
```

For verbose runtime logging, use the `debug` environment:

```sh
pio run -e debug
```

The application image is generated at `.pio/build/release/firmware.bin`. It is an
application-partition image for address `0x10000`; use the PlatformIO upload command
above when writing a complete device.

### Power-cycle after flashing

The thermal sensor's I2C data line (SDA) is on GPIO0, which the ESP32 also reads at
reset to choose between normal boot and download mode. If the chip is reset while a
sensor transfer is in progress (for example by the uploader's automatic reset), the
sensor can still be holding SDA low, so the ESP32 starts in download mode and the
screen stays dark. This is a property of the hardware wiring and cannot be avoided by
the firmware: such a reset bypasses it. Switch the device off and on after flashing.
The firmware clears the I2C bus whenever it (re)initialises the sensor.

If uploads fail at the default `upload_speed` (1.5 Mbaud), set a lower speed such as
`upload_speed = 115200` in `platformio.ini`.

## Links

- [T-Lite documentation and datasheet](https://docs.m5stack.com/en/app/T-Lite)
- [M5StickC Plus SE specifications](https://docs.m5stack.com/en/core/StickC-Plus_SE)

## License

[MIT](LICENSE)
