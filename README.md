# HexDrive2

Standalone EEPROM-side app source for Team RobotMad HexDrive2 boards.

This repository starts from the current BadgeBot `EEPROM/hexdrive.py` implementation, renamed to `hexdrive2.py`, so that BadgeBot and HexManager can consume the same HexDrive2 source as a submodule instead of carrying separate copies.

## What This Repo Contains

- `hexdrive2.py`: the MicroPython app that is compiled to `.mpy`, copied onto a hexpansion EEPROM, and run by BadgeOS as `app.mpy`.

The exported runtime class remains `HexDriveApp`; only the source filename has changed.

## Current Scope

The current implementation is the BadgeBot-derived HexDrive app with support for:

- HexDrive v1 and HexDrive2 EEPROM detection.
- Motor-only, servo-only, mixed motor/servo, and uncommitted HexDrive variants.
- Hardware-specific servo pin remapping for HexDrive2 variants.
- Keep-alive shutdown to de-energise outputs if updates stop arriving.
- HexDrive2-only helper controls for the distance-sensor XSHUT pin and sensor LED.

## Building

Compile the EEPROM app with `mpy-cross`:

```bash
mpy-cross -v hexdrive2.py -o hexdrive2.mpy
```

The resulting `hexdrive2.mpy` should then be copied into the consuming app's `EEPROM/` directory. When a host app writes that file to a hexpansion EEPROM it is renamed to `app.mpy` on the EEPROM so BadgeOS will discover it automatically.

## Public API

The current `HexDriveApp` implementation exposes these control methods:

- `set_power(state)`: enable or disable the HexDrive supply.
- `set_motors((left, right))`: set signed motor outputs in the range `-65535..65535` for supported motor channels.
- `set_servoposition(channel=None, position=None)`: set servo pulse positions; channels are logical servo channels, not always direct physical pin numbers.
- `set_servocentre(centre, channel=None)`: adjust servo centre trim.
- `set_freq(freq, channel=None, servo=False)`: adjust PWM frequency for either motor or servo channels.
- `set_keep_alive(period_ms)`: change the watchdog timeout.
- `set_dist_xshut(state)`: HexDrive2-only distance-sensor reset/power pin control.
- `set_sensor_led(state)`: HexDrive2-only sensor illumination LED control.

## Behaviour Notes

- The code does **not** currently provide the old `set_pwm()` helper that some earlier BadgeBot documentation mentioned.
- PWM outputs are allocated lazily so unused channels do not consume PWM resources.
- The keep-alive watchdog is refreshed by `set_motors()` and `set_servoposition()` updates.
- Calls to `set_dist_xshut()` and `set_sensor_led()` return `False` on older hardware that does not expose those lines.

## Intended Consumers

This repository is intended to be used as a git submodule by:

- BadgeBot
- HexManager

Those host apps are responsible for compiling `hexdrive2.py` to `EEPROM/hexdrive2.mpy` and deploying the artifact to the badge.