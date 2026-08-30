# HexDrive2

Standalone EEPROM-side app source for Team RobotMad HexDrive2 boards.

## What This Repo Contains

- `hexdrive2.py`: the MicroPython app that is compiled to `.mpy`, copied onto a hexpansion EEPROM, and run by BadgeOS as `app.mpy`.

The exported runtime class is `HexDriveApp`.

## Current Scope

The current implementation is the BadgeBot-derived HexDrive app with support for:

- HexDrive2 EEPROM detection.
- Motor-only, servo-only, mixed motor/servo, and uncommitted HexDrive variants.
- Keep-alive shutdown to de-energise outputs if updates stop arriving.
- HexDrive2 helper controls for the distance-sensor XSHUT pin and sensor 'flood' LED.
- HexDrive2 driver for the VL53L0X distance sensor.
- HexDrive2 driver for the OPT4060 colour sensor.

## Building

Compile the EEPROM app with `mpy-cross`:

```bash
mpy-cross -march=xtensawin -O2 -v hexdrive2.py -o hexdrive2.mpy
```

The resulting `hexdrive2.mpy` should then be copied into the consuming app's `EEPROM/` directory. When a host app writes that file to a hexpansion EEPROM it is renamed to `app.mpy` on the EEPROM so BadgeOS will discover it automatically.

## Testing

The compatibility tests use only the Python standard library:

```bash
python -m unittest discover -s tests -v
```

They load the app with and without a stub `i2c_mgr` module and cover cached manager reads, legacy direct I2C reads, readiness checks, and measurement sequence updates.

## Public API

The current `HexDriveApp` implementation exposes these control methods:

- `set_logging(state)`: enable or disable logging.
- `set_power(state)`: enable or disable the HexDrive supply.
- `set_motors((M1, M2))`: set signed motor outputs in the range `-65535..65535` for supported motor channels.
- `set_servoposition(channel=None, position=None)`: set servo pulse positions; channels are logical servo channels, not always direct physical pin numbers, position is an offset from the centre in microseconds.
- `set_servocentre(centre, channel=None)`: adjust servo centre trim (default 1500uS).
- `set_freq(freq, channel=None, servo=False)`: adjust PWM frequency for either motor or servo channels.
- `set_keep_alive(period_ms)`: change the watchdog timeout.
- `set_range_xshut(state)`: HexDrive2-only distance-sensor reset/power pin control.
- `set_flood_led(state)`: HexDrive2-only sensor illumination LED control.

- `set_range_period(period_ms)`: HexDrive2-only distance-sensor measurement period control (fastest in practice is about 100mS).
- `range_enable(enable)`: HexDrive2-only distance-sensor enable/disable; optionally you can request to be sent RangeEvent events - but only use this at very slow update rates otherwise the badge event system is overwhelmed.
- `range` property: returns the most recent distance measurement in millimetres.

- `set_colour_period(period_ms)`: HexDrive2-only colour-sensor measurement period control (fastest in practice is about 20mS) but you will only achieve this in practice under ideal conditions when NOT performing any screen drawing.
- `colour_enable(enable)`: HexDrive2-only colour-sensor enable/disable; optionally you can request to be sent ColourEvent events - but only use this at very slow update rates otherwise the badge event system is overwhelmed.
- `colour` property: returns the most recent colour measurement as an RGBW tuple of integers (see below for details of how to get more colour data from the colour sensor class).


Each of the sensors has a 'sequence' property which is incremented each time a new measurement is made.  You can use this to detect when a new measurement is available without having to rely on events.

### Range Data
- `range_sensor.range_sequence` property: returns the sequence number of the most recent distance measurement.  This is incremented each time a new measurement is made.
- `range_sensor.range` property: returns the most recent distance measurement in millimetres.


### Colour Data
- `colour_sensor.colour_sequence` property: returns the sequence number of the most recent colour measurement.  This is incremented each time a new measurement is made.
- `colour_sensor.colour` property: returns the most recent colour measurement as RGBW tuple of integers.
- `colour_sensor.colour_name` property: returns the most recent colour measurement as a string name of the colour (e.g. "Red", "Green", "Blue", "White", "Black", ...).
- `colour_sensor.colour_hue` property: returns the most recent colour measurement as a hue value in units of 1/10th degrees (0-3600).
- `colour_sensor.colour_saturation` property: returns the most recent colour measurement as a saturation value as a percentage (0-100).

### Colour Calibration
- `calibrated` property reports whether the colour sensor has been calibrated since the last power-on.
- `black_reference` and `white_reference` properties of the colour sensor can be set to a tuple of 4 integers representing the RGBW values measured under the current lighting conditions.
The colour sensor needs to be calibrated for the ambient light conditions.  The calibration is done by reading the values when over a known black surface and then passing them to the 'black_reference' property of the colour sensor. Then doing the same for a known white surface and passing those values to the 'white_reference' property.

### Power
The HexDrive incorporates a Switch Mode Power Supply which boosts the 3.3V provided by the badge up to 5V (or higher if your hexpansion has been modified) to drive the motors.  To turn this on or off call
```set_power(True | False)```

### Drive
Call ```set_motors()``` to control the two motors, providing a signed integer from -65535 to +65535 for each in a tuple.

Alternatively:
Call ```set_pwm()``` to set the duty cycle of the 4 PWM channels which control the motors. This function takes a tuple of 4 integers, each from 0 to 65535. e.g.
```set_pwm((0,1000,1000,0))```
note the extra set of brackets as the function argument is a single tuple of 4 values rather than being 4 individual values.

### Servos
You can control 1,2,3 or 4 RC hobby servos (centre pulse width 1500us).  The first time you set a pulse width for a channel using ```set_servoposition()``` the PWM frequency for that channel will be set to 50Hz.
The first two Channels take up signals that would otherwise control Motor 1 and the second two Channels take up the signals that are used for Motor 2.
You can use one motor and 1 or 2 servos simultaneously.

### Frequency
You can adjust the PWM frequency, default 20000Hz for motors and 50Hz for servos by calling the ```set_freq()``` function.

#### Keep Alive
To protect against most badge/software crashes causing the motors or servos to run out of control there is a keep alive mechanism which means that if you do not make a call to the ```set_pwm```, ```set_motors``` or ```set_servoposition``` functions the motors/servos will be turned off after 1000mS (default - which can be changed with a call to ```set_keep_alive()```).

## Behaviour Notes

- PWM outputs are allocated lazily so unused channels do not consume PWM resources.
- The keep-alive watchdog is refreshed by `set_motors()` and `set_servoposition()` updates.

## Intended Consumers

This repository is intended to be used as a git submodule by:

- BadgeBot
- HexManager

Those host apps are responsible for compiling `hexdrive2.py` to `EEPROM/hexdrive2.mpy` and deploying the artifact to the badge.
