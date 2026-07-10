"""HexDrive2 EEPROM app source for Team RobotMad apps."""

# This is the app to be installed from a HexDrive2 Hexpansion EEPROM.
# it is compiled and copied onto the EEPROM as app.mpy
# It is then run from the EEPROM by the BadgeOS.
import time # aim to eliminate this dependency
try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is an identity function on MicroPython
    const = lambda x: x  # noqa: E731
import struct
import ota
from machine import PWM, Pin, I2C
from events import Event
from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.hexpansion import app as hexpansion_app
from system.scheduler.events import RequestStopAppEvent
import app
from tildagon import Pin as ePin

# Define the minimum BadgeOS version required to run this app (e.g. if we need features that are only available in a certain version of BadgeOS)
_MIN_BADGEOS_VERSION = [2, 0, 0]     # v2.0.0 is required to be able to use the new hexpansion utilites

# HexDrive Hexpansion constants
# Hardware defintions:
_ENABLE_PIN  = const(0)         # First LS pin used to enable the SMPSU
_COLOUR_INT_PIN = const(1)      # Second LS pin used to detect interrupts from the colour sensor to trigger readings without polling
_LED_PIN  = const(2)            # Third LS pin used to control an LED to illuminate the area under the colour sensor for better readings of reflected light from the surface below.
_RANGE_INT_PIN = const(3)       # Fourth LS pin used to detect interrupts from the distance sensor to trigger readings without polling
_RANGE_XSHUT_PIN = const(4)     # Fifth LS pin used to control the XSHUT pin of the distance sensor to allow it to be power cycled for reset or power saving
_ALT_RANGE_INT_PIN = const(4)   # Some models of the VL52L0X sensor module have the interrupt and XSHUT pins swapped
_ALT_RANGE_XSHUT_PIN = const(3) # Some models of the VL52L0X sensor module have the interrupt and XSHUT pins swapped

_RANGE_SENSOR_XSHUT_RESPONSE_TIME_MS = const(20)  # Time to wait after changing the XSHUT pin state before the sensor is ready to respond to I2C commands
_SENSOR_CHECK_INTERVAL_MS = const(100)  # Interval to check for new sensor readings in continuous mode (ms) - as a fallback in case the interrupts are missed

# Hexpansion EEPROM constants
_ADDR_LEN = const(2)          # EEPROM I2C address length in bytes (1 or 2)
_ADDR = const(0x50)           # EEPROM I2C address (7-bit)


# EXTENDED Header constants (MUST fit within the first page of the EEPROM)
_EXTENDED_HEADER_ADDR = const(0x20)  # EEPROM address of the extended header
_EXTENDED_HEADER_SIZE = const(32)    # Size of the extended header in bytes
_EXTENDED_HEADER_MAGIC = b"HDR2"            # Magic bytes to identify the extended header
_EXTENDED_HEADER_VERSION = b"2026"          # Version of the extended header format

# EXTENDED header flags constants
_EXTENDED_HEADER_FLAG_INITIALISED = const(0x01)  # Flag indicating that the extended header has been initialised
_EXTENDED_HEADER_FLAG_RANGE_SENSOR = const(0x02)  # Flag indicating that the distance sensor pins are swapped
_EXTENDED_HEADER_FLAG_RANGE_PINS_SWAPPED = const(0x04)  # Flag indicating that the distance sensor pins are swapped
_EXTENDED_HEADER_FLAG_COLOUR_SENSOR = const(0x08)  # Flag indicating that the colour sensor is present
_EXTENDED_HEADER_FLAG_FLOOD_LEDS = const(0x10)  # Flag indicating that the flood LEDs are present


# Default values and limits:
_DEFAULT_PWM_FREQ = const(20000)           # 20kHz is a good default for motors as it is above the audible range for most people and works with most motors and ESCs
_DEFAULT_SERVO_FREQ = const(50)            # 50Hz = 20mS period
_DEFAULT_KEEP_ALIVE_PERIOD = const(1000)   # 1 second
_DEFAULT_RANGE_PERIOD_MS = const(0)        # default inter-measurement period (ms) for continuous distance ranging (~20 readings/s); 0 = back-to-back (as fast as the sensor allows)
_MAX_NUM_CHANNELS = const(4)               # Max number of PWM channels supported by any type of HexDrive (Hexpansion limitation, not BadgeBot limit)
_MAX_NUM_MOTORS = const(2)                 # Max number of motor channels supported by any type of HexDrive

# Servo Constants
_MAX_SERVO_FREQ = const(200)               # 200Hz = 5mS period (can work with some Servos but not all)
_SERVO_CENTRE    = const(1500)             # 1500us pulse width is the centre position for most RC servos (but some may be different, so we allow this to be trimmed)
_MAX_SERVO_RANGE = const(1400)             # 1400us either side of centre (VERY WIDE)
_SERVO_MAX_TRIM  = const(1000)             # 1000us either side of centre for trimming the centre position


class HexDriveType:
    """Represents a sub-type of HexDrive Hexpansion module."""
    __slots__ = ("pid", "name", "motors", "servos", "servo_pin_map", "ext_header")

    def __init__(self, pid_byte: int, name: str = "Uncommitted", motors: int = 0, servos: int = 0, servo_pins: tuple[int, int, int, int] = (-1, -1, -1, -1), ext_header: bool = True):
        self.pid: int = pid_byte            # Product ID byte read from the EEPROM to identify the type of HexDrive
        self.name: str = name               # A friendly name for the type of HexDrive
        self.motors: int = motors           # Number of motor channels supported by this type of HexDrive (0, 1 or 2)
        self.servos: int = servos           # Number of servo channels supported by this type of HexDrive (0, 2 or 4)
        self.servo_pin_map: tuple[int, int, int, int] = servo_pins # Map the logical servo channels to the physical pin index according to hardware version
        self.ext_header: bool = ext_header  # Flag indicating if this HexDrive type supports extended header in EEPROM

_HEXDRIVE_TYPES = (
    HexDriveType(0xC8, motors=2, servos=2, servo_pins=(3, 1, -1, -1)),  # uncommitted version can be used for anything
    HexDriveType(0xC9, servos=2, name="2 Servo", servo_pins=(3, 1, -1, -1)),
    HexDriveType(0xCA, motors=2, name="2 Motor"),
    HexDriveType(0xCE, motors=1, name="1 Motor"),
    HexDriveType(0xCF, motors=1, servos=1, name="1 Mot 1 Srvo", servo_pins=(1, -1, -1, -1)),
)


_DEFAULT_HEXDRIVE_TYPE = _HEXDRIVE_TYPES[0]  # default to the uncommitted version if we can't read the EEPROM for some reason


# --------------------------------------------------------------------------------------------------------------
# Extended Hexpansion Header class for reading and writing the extended header of the hexpansion EEPROM.
# This uses the fact that the standard Hexpansion header is 32 bytes long, and the extended header is
# stored in the spare bytes of the first sector of the EEPROM. As we know that our EEPROMS use 64-byte pages.
# --------------------------------------------------------------------------------------------------------------
class ExtendedHexpansionHeader:
    """ Represents the extended header of the hexpansion EEPROM, which is stored in the spare bytes of the first page of the EEPROM. """
    _header_format = "<4s4sI19s"
    _magic = _EXTENDED_HEADER_MAGIC

    def __init__(
        self,
        manifest_version: str = _EXTENDED_HEADER_VERSION.decode(),
        flags: int = 0xFFFF0000,
        spare: str = "\xFF" * 19
        ):
        self.manifest_version = manifest_version
        self.flags: int = flags
        self.spare: str = spare
        self.to_bytes()

    def __str__(self):
        return f"""ExtendedHexpansionHeader[
    manifest version: {self.manifest_version},
    flags: {'0x' + hex(self.flags)[4:].upper()},
    spare: {'0x' + hex(int.from_bytes(self.spare.encode(), 'little'))[2:].upper()}
]"""

    @classmethod
    def calc_checksum(cls, b):
        """ Calculate the checksum for the given bytes buffer. """
        checksum = 0x55
        for byte in b:
            checksum ^= byte
        return checksum

    def to_bytes(self):
        """ Convert the ExtendedHexpansionHeader object to a bytes buffer suitable for writing to the EEPROM. """
        b = struct.pack(
            self._header_format,
            self._magic,
            self.manifest_version,
            self.flags,
            self.spare
        )
        checksum = self.calc_checksum(b[1:])
        return b + bytes([checksum])

    @classmethod
    def from_bytes(cls, buf, validate_checksum=True):
        """ Create an ExtendedHexpansionHeader object from a bytes buffer read from the EEPROM. """
        if len(buf) != _EXTENDED_HEADER_SIZE:
            raise RuntimeError(f"Invalid extended header length, should be {_EXTENDED_HEADER_SIZE}")
        if buf[0:4] != _EXTENDED_HEADER_MAGIC:
            raise RuntimeError(f"Invalid magic in extended header: {buf[0:4]}")
        if buf[4:8] != _EXTENDED_HEADER_VERSION:
            raise RuntimeError(f"Unknown manifest version. Supported: [{_EXTENDED_HEADER_VERSION.decode()}]")
        unpacked = struct.unpack(cls._header_format, buf)

        if validate_checksum:
            header_checksum = buf[_EXTENDED_HEADER_SIZE - 1]
            bytes_checksum = cls.calc_checksum(buf[1:_EXTENDED_HEADER_SIZE - 1])
            if header_checksum != bytes_checksum:
                raise RuntimeError(
                    f"Extended header checksum mismatch: {header_checksum} != {bytes_checksum}"
                )

        return cls(
            manifest_version=unpacked[1].decode().split("\x00")[0],
            flags=unpacked[2],
            spare=unpacked[3],
        )


#----------------------------------------------------------------
# HexDriveApp class
#----------------------------------------------------------------
class HexDriveApp(app.App):         # pylint: disable=no-member
    """ HexDrive Hexpansion App for BadgeBot."""
    VERSION = 2         # Increment this when making changes to the app that require the hexpansion EEPROM app to be re-flashed with the new code.

    class RangeEvent(Event):
        """Emitted when a new ToF distance measurement is obtained, providing the distance to target in mm."""
        def __init__(self, distance: int):
            self.range = distance

        def __str__(self):
            return f"Range: {self.range}mm"


    class ColourEvent(Event):
        """Emitted when a new colour measurement is obtained, providing the RGBW values."""
        def __init__(self, colour: tuple[int, int, int, int], colour_name: str):
            self.colour = colour
            self.colour_name = colour_name

        def __str__(self):
            return f"Colour: {self.colour_name} (R={self.colour[0]}, G={self.colour[1]}, B={self.colour[2]}, W={self.colour[3]})"


    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()
        if config is None:
            raise TypeError("HexDriveApp requires a HexpansionConfig on initialisation")

        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            if ver >= _MIN_BADGEOS_VERSION:
                pass
            else:
                raise TypeError("BadgeOS version is too old for HexDriveApp")
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:Ver check failed {e}!")

        self.config: HexpansionConfig = config
        self._logging: bool = True
        self._i2c: I2C | None = None

        # What flavour of HexDrive Hexpansion module do we have plugged in?
        _hexdrive_type = self._check_port_for_hexdrive(self.config.port)

        # report app starting and which port it is running on
        print(f"D:{self.config.port}:HexDrive2 Type:'{_hexdrive_type.name}' App V{self.VERSION} by Team RobotMad")

        self._hexdrive_type: HexDriveType = _hexdrive_type
        self._keep_alive_period: int = _DEFAULT_KEEP_ALIVE_PERIOD
        self._power_state: bool = False
        self._pwm_setup: bool = False
        self._time_since_last_update: int = 0
        self._outputs_energised: bool = False
        self.PWMOutput: list[PWM | None] = [None] * _MAX_NUM_CHANNELS
        self._freq: list[int] = [0] * _MAX_NUM_CHANNELS
        if self._hexdrive_type.motors > 0:
            self._motor_output: list[int] = [0] * self._hexdrive_type.motors

        if self._hexdrive_type.ext_header:
            self._extended_header: ExtendedHexpansionHeader = self._read_extended_hexpansion_header()
            if self._extended_header.flags & _EXTENDED_HEADER_FLAG_INITIALISED:
                print(f"D:{self.config.port}:Extended Header flags={self._extended_header.flags:08b}")
            else:
                print(f"D:{self.config.port}:Extended Header not initialised")
                self._extended_header.flags |= _EXTENDED_HEADER_FLAG_INITIALISED
                self._extended_header.flags |= _EXTENDED_HEADER_FLAG_RANGE_SENSOR
                self._extended_header.flags |= _EXTENDED_HEADER_FLAG_COLOUR_SENSOR
                self._extended_header.flags |= _EXTENDED_HEADER_FLAG_FLOOD_LEDS
                if self._detect_and_set_dist_pins_swapped_flag():
                    if self._write_extended_hexpansion_header(self._extended_header):
                        print(f"D:{self.config.port}:Extended Header Written, flags={self._extended_header.flags:08b}")
        else:
            self._extended_header = ExtendedHexpansionHeader() # create a dummy extended header for non-extended hexdrive types

        self._time_since_last_sensor_check: int = 0
        self._sensor_check_interval: int = _SENSOR_CHECK_INTERVAL_MS

        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
            self._range_sensor: VL53L0X | None = None
            self._range_period_ms: int = _DEFAULT_RANGE_PERIOD_MS  # inter-measurement period for continuous ranging (0 = back-to-back / as fast as the sensor allows)

        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR:
            self._colour_sensor: OPT4060 | None = None
            self._colour_period_ms: int = 0

        # LS Pins
        self._power_control: ePin = self.config.ls_pin[_ENABLE_PIN]
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_FLOOD_LEDS:
            self._led_control:   ePin = self.config.ls_pin[_LED_PIN]
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR:
            self._colour_int:    ePin = self.config.ls_pin[_COLOUR_INT_PIN]
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
            self._range_xshut:   ePin = self.config.ls_pin[_ALT_RANGE_XSHUT_PIN if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_PINS_SWAPPED else _RANGE_XSHUT_PIN]
            self._range_int:     ePin = self.config.ls_pin[_ALT_RANGE_INT_PIN if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_PINS_SWAPPED else _RANGE_INT_PIN]

        # Servo related
        if self._hexdrive_type.servos > 0:
            self._servo_pin_map: tuple[int, int, int, int] = self._hexdrive_type.servo_pin_map
            self._servo_centre: list[int] = [_SERVO_CENTRE] * self._hexdrive_type.servos

        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)

        if not self.initialise():
            print("HexDriveApp init failed")


#----------------------------------------------------------------
# PUBLIC methods
#----------------------------------------------------------------

    def initialise(self) -> bool:
        """Initialise the app - return True if successful, False if failed."""

        # Initialise HS Pins
        for _, hs_pin in enumerate(self.config.pin):
            # Set HexDrive Hexpansion HS pins to low level outputs
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)

        # Initialise LS Pins
        try:
            self._power_control.init(mode=Pin.OUT)
            if self._extended_header.flags & _EXTENDED_HEADER_FLAG_FLOOD_LEDS:
                self._led_control.init(mode=Pin.OUT)
            if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR:
                self._colour_int.init(mode=Pin.IN)
            if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
                self._range_xshut.init(mode=Pin.OUT)
                self._range_int.init(mode=Pin.IN)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:ls_pin setup failed {e}")
            return False

        # ensure SMPSU is turned off to start with
        self.set_power(False)

        # We delay the PWM initialisation until we actually need to set a servo position or motor speed
        # because there are a limited number of PWM resources and we want to leave them available for
        # other apps to use if the HexDrive is not actively being used.
        # So here we just initialise the internal frequency array to the default values for motors and servos
        for channel in range(self._hexdrive_type.motors):
            print(f"D:{self.config.port}:Motor {channel} on Physical channels {channel<<1} & {(channel<<1) + 1}")
            self._motor_output[channel]  = 0  # initialise motor output state to 0 (stopped)
            self._freq[channel<<1]       = _DEFAULT_PWM_FREQ
            self._freq[(channel<<1) + 1] = _DEFAULT_PWM_FREQ
        for channel in range(self._hexdrive_type.servos):
            physical_channel = self._servo_pin_map[channel]
            if physical_channel >= 0 and self._freq[physical_channel] == 0:
                # give priority to motor frequency if there is a conflict on the same physical channel, otherwise set to default servo frequency
                print(f"D:{self.config.port}:Servo {channel} on Physical channel {physical_channel}")
                self._freq[physical_channel] = _DEFAULT_SERVO_FREQ
        self._pwm_setup = True

        return True


    # Special function called by the BadgeOS to allow the app to clean up resources before it is removed from memory.
    # do not change the name of this function as it is called by the BadgeOS when the app is removed from memory.
    def deinit(self):
        """ De-initialise all PWM outputs and free up resources. """
        for _channel, _pwm in enumerate(self.PWMOutput):
            if _pwm is not None:
                try:
                    _pwm.deinit()
                except Exception:       # pylint: disable=broad-except
                    pass
                self.PWMOutput[_channel] = None
        for _channel in range(_MAX_NUM_CHANNELS):
            self._freq[_channel] = 0
        self._pwm_setup = False
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
            if self._range_sensor is not None:
                try:
                    self._range_int.irq(handler=None)   # detach the data-ready interrupt handler
                    self._range_sensor.stop()           # stop continuous ranging
                except Exception:       # pylint: disable=broad-except
                    pass
                self._range_sensor = None
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR:
            if self._colour_sensor is not None:
                try:
                    self._colour_int.irq(handler=None)   # detach the data-ready interrupt handler
                    self._colour_sensor.stop()           # stop continuous colour sensing
                except Exception:       # pylint: disable=broad-except
                    pass
                self._colour_sensor = None



    # Special function called by the BadgeOS to allow the app to do any background processing it needs to do.
    # do not change the name of this function as it is called by the BadgeOS in the main loop of the BadgeOS.
    def background_update(self, delta: int):
        """ This is called from the main loop of the BadgeOS to allow the app to do any background processing it needs to do. """
        self._time_since_last_sensor_check += delta
        if self._time_since_last_sensor_check > self._sensor_check_interval:  # check sensors every 10ms
            self._time_since_last_sensor_check = 0
            # Check for missed interrupts from the distance sensor and colour sensor and read them if they are ready
            # Range Sensor
            if (self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR) and self._range_sensor is not None and self._range_sensor.is_continuous:
                if 0 == self._range_int.value():
                    measurement = self._range_sensor.read()    # reads the measurement and clears the interrupt to re-arm the sensor
                    if isinstance(measurement, int):
                        distance = measurement
                        eventbus.emit(self.RangeEvent(distance))
            # Colour Sensor
            if (self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR) and self._colour_sensor is not None and self._colour_sensor.is_continuous:
                if 0 == self._led_control.value() or 0 == self._colour_int.value():
                    # Either:
                    #   LED is Off, so we need to poll the colour sensor to see if it has a new reading ready
                    #   or, the colour sensor interrupt is active (low) so we can read the colour sensor
                    measurement = self._colour_sensor.read()
                    if measurement is not None:
                        # we read the colour from the sensor class rather than using the return from read() to keep the linter quiet
                        colour = self._colour_sensor.colour or (0,0,0,0)
                        colour_name = self._colour_sensor.colour_name or "Unknown"
                        eventbus.emit(self.ColourEvent(colour, colour_name))

        # Keep Alive
        if not self._pwm_setup or not self._outputs_energised:
            # if we are not properly initialised then do not attempt to do anything
            return
        # Check keep alive period and turn off PWM outputs if exceeded
        self._time_since_last_update += delta
        if self._time_since_last_update > self._keep_alive_period:
            self._time_since_last_update = 0
            self._outputs_energised = False
            # First time the keep alive period has expired so report it
            if self._logging:
                print(f"D:{self.config.port}:Timeout")
            for channel,pwm in enumerate(self.PWMOutput):
                if pwm is not None:
                    try:
                        pwm.duty_u16(0)
                    except Exception as e:          # pylint: disable=broad-except
                        print(self._pwm_log_string(channel) + f"Off failed {e}")
                        self.PWMOutput[channel] = None  # Tidy Up


    def get_status(self) -> bool:
        """ Get the current status of the app - True if the app is running and able to respond to commands, False if not. """
        return self._pwm_setup


    @property
    def capabilities(self) -> int:
        """ Get the capabilities of the HexDrive Hexpansion module as a bitmask of flags."""
        return self._extended_header.flags


    def set_logging(self, state: bool):
        """ Set the logging state - True to enable logging, False to disable logging. """
        self._logging = state


    def set_power(self, state: bool) -> bool:
        """ Turn the SMPSU on or off. Returns success or failure. """
        if state == self._power_state:
            return True  # No change needed
        if self._logging:
            print(f"D:{self.config.port}:Power={'On' if state else 'Off'}")
        try:
            self._power_control.init(mode=Pin.OUT)
            self._power_control.value(state)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:power control failed {e}")
            return False
        self._power_state = state
        return True


    def set_keep_alive(self, period: int):
        """ Set the keep alive period in milliseconds:
            This is the period of time that can elapse without any commands being received before the app automatically
            turns off all outputs to prevent damage to motors or servos if something goes wrong. """
        self._keep_alive_period = period


    def set_freq(self, freq: int, channel: int | None = None, servo: bool = False) -> bool:
        """ Set the PWM frequency for a specific output, or all outputs if channel is None. Returns True if successful, False if failed.
            Use 50 to 200 for Servos and 5000 to 20000 for motors. """
        if freq < 0 or freq > 100000:
            return False
        physical_channel: int | None = None
        if channel is not None:
            _max_channel = self._hexdrive_type.servos if servo else self._hexdrive_type.motors
            if channel < 0 or channel >= _max_channel:
                return False
            # map from logical channel to physical channel(s) for servos and motors
            if servo and self._hexdrive_type.servos > 0:
                self._freq[channel] = freq
                physical_channel = self._servo_pin_map[channel]
            elif self._hexdrive_type.motors > 0:
                self._freq[channel << 1] = freq
                self._freq[(channel << 1) + 1] = freq
                physical_channel = 3- ((channel << 1) + (self._motor_output[channel] > 0)) # 3- to reverse pin order to match Hexpansion hardware
        else:
            if servo:
                for ch in range(self._hexdrive_type.servos):
                    self._freq[ch] = freq
            else:
                for ch in range(self._hexdrive_type.motors):
                    self._freq[ch<<1] = freq
                    self._freq[(ch<<1)+1] = freq
            physical_channel = None # All channels

        # Action new frequency immediately for any channels that are already setup
        for this_channel, pwm in enumerate(self.PWMOutput):
            if (physical_channel is None or (this_channel == physical_channel)) and pwm is not None:
                if freq == 0:
                    # If frequency is set to 0 then we deinit the PWM to free up resources as much as possible
                    pwm.deinit()
                    self.PWMOutput[this_channel] = None
                    self.config.pin[this_channel].init(mode=Pin.OUT)
                    self.config.pin[this_channel].value(0)
                    if self._logging:
                        print(self._pwm_log_string(this_channel) + " disabled")
                else:
                    try:
                        pwm.freq(freq)
                        if self._logging:
                            print(self._pwm_log_string(this_channel) + f"{freq}Hz set")
                    except Exception as e:  # pylint: disable=broad-except
                        print(self._pwm_log_string(this_channel) + f"set freq {freq} failed {e}")
                        return False
        return True


    def set_servoposition(self, channel: int | None = None, position: int | None = None) -> bool:
        """ Set the position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            The pulse width for a specific servo output is position + the centre offset (in us)
            Based on standard RC servos with centre at 1500us and range of 1000-2000us.
            The position is a signed value from -1000 to 1000 which is scaled to 500-2500us.
            This is a very wide range and may not be suitable for all servos, some will
            only be happy with 1000-2000us (i.e. position in the range -500 to 500). """
        if self._hexdrive_type.servos == 0:
            return False
        if position is None:
            # position == None -> Turn off PWM (some servos will then turn off, others will stay in last position)
            if channel is None:
                # channel == None -> Turn off all PWM outputs
                for ch, pwm in enumerate(self.PWMOutput):
                    if pwm is not None and ch in self._servo_pin_map:
                        try:
                            pwm.duty_ns(0)
                        except Exception as e:  # pylint: disable=broad-except
                            print(self._pwm_log_string(ch) + f"Off failed {e}")
                if self._logging:
                    print(self._pwm_log_string(None) + "Off")
                self._outputs_energised = False
                return True
            elif channel < 0 or channel >= self._hexdrive_type.servos:
                return False
            else:
                physical_channel = self._servo_pin_map[channel]
                pwm = self.PWMOutput[physical_channel]
                if pwm is None:
                    return False
                try:
                    pwm.duty_ns(0)
                    if self._logging:
                        print(self._pwm_log_string(physical_channel) + "Off")
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(physical_channel) + f"Off failed {e}")
                    return False
            # check if all channels are now off and set outputs_energised accordingly
            #self._check_outputs_energised()
        elif channel is not None:
            if channel < 0 or channel >= self._hexdrive_type.servos:
                return False
            if abs(position) > _MAX_SERVO_RANGE:
                return False
            physical_channel = self._servo_pin_map[channel]
            pulse_width_in_ns = (self._servo_centre[channel] + position) * 1000 # convert from us to ns
            if self.PWMOutput[physical_channel] is None:
                # Channel hasn't been setup yet so we need to initialise it from scratch
                self._freq[channel] = self._freq[channel] if (0 < self._freq[channel]) and (self._freq[channel] <= _MAX_SERVO_FREQ) else _DEFAULT_SERVO_FREQ
                try:
                    # Micropython v1.28 generates a spurious warning when we try to initialise a PWM on a pin that was previously used.
                    # "W (557771) ledc: GPIO 47 is not usable, maybe conflict with others"
                    # workaround is to set it to an input
                    pin = self.config.pin[physical_channel]
                    pin.init(mode=Pin.IN)
                    pwm = PWM(pin, freq = self._freq[channel])
                    pwm.duty_ns(pulse_width_in_ns)
                    self.PWMOutput[physical_channel] = pwm
                    if self._logging:
                        print(self._pwm_log_string(physical_channel) + f"{self.PWMOutput[physical_channel]} init")
                except Exception as e:      # pylint: disable=broad-except
                    # There are a finite number of PWM resources so it is possible that we run out
                    print(self._pwm_log_string(physical_channel) + f"PWM(init) failed {e}")
                    return False
            else:
                # Channel is already setup so we just need to change the duty cycle and possibly the frequency if it is too high for the servo
                pwm = self.PWMOutput[physical_channel]
                if pwm is None:
                    return False
                try:
                    if _MAX_SERVO_FREQ < pwm.freq():
                        # Ensure the frequency is suitable for use with Servos
                        # otherwise the pulse width will not be accepted
                        self._freq[channel] = _DEFAULT_SERVO_FREQ
                        pwm.freq(_DEFAULT_SERVO_FREQ)
                        if self._logging:
                            print(self._pwm_log_string(physical_channel) + f"{_DEFAULT_SERVO_FREQ}Hz for Servo")
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(physical_channel) + f"set freq failed {e}")
                    return False
                # Scale servo position to PWM duty cycle (500-2500us)
                try:
                    if 2000 < abs(pulse_width_in_ns - pwm.duty_ns()):    # allow tolerance of 2us to avoid unnecessary updates
                        #if self._logging:
                        #    print(self._pwm_log_string(physical_channel) + f"{pulse_width_in_ns}ns")
                        pwm.duty_ns(pulse_width_in_ns)
                        #if self._logging:
                        #    print(self._pwm_log_string(physical_channel) + f"{pwm} duty")
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(physical_channel) + f"set duty failed {e}")
                    return False

            self._outputs_energised = True
        self._time_since_last_update = 0
        return True


    def set_servocentre(self, centre: int, channel: int | None = None) -> bool:
        """ Set the centre position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            Note this does not change the current position of the servo.
            It will only affect the position next time it is set.
            You can use this to trim the centre position of the servo. """
        if self._hexdrive_type.servos == 0:
            return False
        if channel is not None and (channel < 0 or channel >= self._hexdrive_type.servos):
            return False
        if centre < (_SERVO_CENTRE - _SERVO_MAX_TRIM ) or centre > (_SERVO_CENTRE + _SERVO_MAX_TRIM):
            return False
        if channel is None:
            self._servo_centre = [centre] * self._hexdrive_type.servos
        else:
            self._servo_centre[channel] = centre
        return True


    # Set pairs of PWM duty cycles in one go using a signed value per motor channel (0-65535)
    def set_motors(self, outputs: tuple[int, ...]) -> bool:
        """ Set the motor outputs using a signed value for each motor channel. Returns True if successful, False if failed.
            The outputs are signed values in a tuple from -65535 to 65535 which are scaled to the PWM duty cycle range of 0-65535.
            A positive value will drive the motor in one direction, a negative value will drive it in the opposite direction,
            and a value of 0 will stop the motor. """
        if len(outputs) > self._hexdrive_type.motors:
            return False
        for motor, output in enumerate(outputs):
            if abs(output) > 65535:
                return False
            if output == self._motor_output[motor]:
                # no change in output for this motor so skip to the next one
                continue
            try:
                # if the output is changing direction then we need to switch which signal is being driven as the PWM output
                # rather than test for change of direction and also test that PWMOutput to be disabled exists we just do the latter check.
                output_to_enable  = 3- ((motor<<1) if output > 0 else ((motor<<1)+1))
                output_to_disable = 3- ((motor<<1)+1 if output > 0 else (motor<<1))
                # switch off the currently active output before switching the other one on to prevent both outputs being on at the same time
                pwm_to_disable = self.PWMOutput[output_to_disable]
                if pwm_to_disable is not None:
                    pwm_to_disable.deinit()
                    self.PWMOutput[output_to_disable] = None
                    print(f"D:{self.config.port}:pin{output_to_disable} = 0")
                    self.config.pin[output_to_disable].init(mode=Pin.OUT)
                    self.config.pin[output_to_disable].value(0)
                    if self._logging:
                        print(self._pwm_log_string(output_to_disable) + " disabled")
                if 0 != output or self.PWMOutput[output_to_enable] is not None:
                    # if output_to_enable is NOT already active and new output is 0 then we can leave it off for now.
                    # otherwise we need to set the new output value
                    self._set_pwmoutput(output_to_enable, abs(output))
            except Exception as e:          # pylint: disable=broad-except
                print(f"D:{self.config.port}:Motor{motor}:{output} set failed {e}")
                return False
            self._motor_output[motor] = output
            if output != 0:
                self._outputs_energised = True
        self._time_since_last_update = 0
        return True


#---------------------------------------------------------------------------------
# VL53L0X ToF distance sensor functions
#---------------------------------------------------------------------------------
# PUBLIC API
#---------------------------------------------------------------------------------

    def set_range_xshut(self, state: bool) -> None:
        """ Set the state of the distance sensor XSHUT pin to power cycle it for reset or power saving."""
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
            self._range_xshut.init(mode=Pin.OUT)
            self._range_xshut.value(state)
            if self._logging:
                print(f"D:{self.config.port}:Range Sensor XSHUT={'On' if state else 'Off'}")


    def get_range_int_state(self) -> bool:
        """ Return the current state of the range sensor interrupt pin. """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
            return bool(self._range_int.value())
        return False


    def set_range_period(self, period_ms: int) -> None:
        """ Set the inter-measurement period for continuous ranging in milliseconds.
            A value of 0 means back-to-back measurements as fast as the sensor allows. """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR:
            if period_ms < 0 or period_ms > 10000:
                # Invalid period, do nothing
                raise ValueError(f"D:{self.config.port}:Range Sensor period must be between 0 and 10000ms")
            if period_ms == self._range_period_ms:
                return  # No change needed
            self._range_period_ms = period_ms
            if self._range_sensor is not None:
                if self._range_sensor.start(period_ms):
                    if self._logging:
                        print(f"D:{self.config.port}:Range Sensor period set to {period_ms}ms")
                    return
        raise RuntimeError(f"D:{self.config.port}:Range Sensor period set failed")


    def range_enable(self, enable: bool) -> None:
        """Enable or disable interrupt-driven distance ranging.

        When enabled the VL53L0X runs in *continuous* mode: it measures repeatedly on its own and
        pulls its interrupt line low each time a new reading is ready. That falling edge is delivered
        to `_handle_range_interrupt`, which reads the distance and publishes it as a `RangeEvent`.
        The latest value is also cached and can be polled with `get_range`.

        Args:
            enable: True to start ranging, False to stop it and power the sensor down.
        """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR == 0:
            return
        if enable:
            if self._range_sensor is None:
                if self._i2c is None:
                    self._i2c = I2C(self.config.port)
                    if self._logging:
                        print(f"D:{self.config.port}:i2c init")
                self._range_sensor = VL53L0X(self._i2c, logging=self._logging)
                if self._logging:
                    print(f"D:{self.config.port}:Range Sensor created")
            sensor = self._range_sensor
            # Release the sensor from reset. It needs ~1.2ms to boot before it answers on I2C.
            # This is a one-off setup path (not the periodic update loop) so a short blocking wait
            # here is acceptable and keeps the steady-state ranging fully interrupt driven.
            self.set_range_xshut(True)
            time.sleep_ms(_RANGE_SENSOR_XSHUT_RESPONSE_TIME_MS)
            if not sensor.init(None):
                raise RuntimeError(f"D:{self.config.port}:Range Sensor init failed")
            # Register the data-ready interrupt BEFORE starting continuous ranging so that the very
            # first "measurement ready" falling edge cannot be missed.
            #
            # A *bound method* is passed as the handler. The badge delivers LS-pin interrupts with
            # mp_sched_schedule(handler, pin) - it passes the LS pin as the sole argument, and because
            # the handler is bound its 'self' already carries this app instance (and therefore the
            # sensor object and the flag we set) with it. That is the most efficient way for the
            # interrupt to know the context of the sensor that fired: no module globals, no lookup
            # tables, no searching - just a direct attribute access. NOTE the handler only sets a flag;
            # the actual I2C read and RangeEvent emission are deferred to background_update() because
            # this scheduler context must not touch asyncio (see _handle_range_interrupt).
            #
            # We MUST register for BOTH edges. The AW9523B expander interrupts on any change and the
            # badge firmware's C ISR shim (tildagon_pin_isr_handler) schedules whatever Python handler
            # is in the slot for the edge that occurred - with NO null check. When we clear the sensor's
            # interrupt (in read()) the line goes high, producing a rising edge; if only the falling
            # edge were registered, the rising-edge slot would be MP_OBJ_NULL and the VM would crash
            # (Guru Meditation LoadProhibited, EXCVADDR 0x0) trying to call NULL. Handling both edges is
            # harmless: the spurious rising-edge callback just sets the flag, and read() then finds no
            # measurement ready and returns without emitting.
            self._range_int.init(mode=Pin.IN)
            self._range_int.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._handle_range_interrupt)
            sensor.start(self._range_period_ms)
            if self._logging:
                print(f"D:{self.config.port}:Range Sensor Started")
        else:
            self._range_int.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=None)   # detach the interrupt handler first
            if self._range_sensor is not None:
                sensor = self._range_sensor
                sensor.stop()           # stop continuous ranging
                sensor.reset()          # force a full re-initialisation next time it is enabled
                if self._logging:
                    print(f"D:{self.config.port}:Range Sensor Stopped")
            self.set_range_xshut(False)         # power the sensor down (holds it in hardware reset)


    @property
    def range(self) -> int | None:
        """Return the most recent distance measurement in millimetres.

        Returns None until the first reading has been received. New readings arrive automatically
        (via `RangeEvent`) while continuous ranging is enabled with `range_enable`.
        """
        if (self._extended_header.flags & _EXTENDED_HEADER_FLAG_RANGE_SENSOR) and self._range_sensor is not None:
            return self._range_sensor.range
        return None


#----------------------------------------------------------------
# PUBLIC Colour Sensor (OPT406) methods
#---------------------------------------------------------------

    def get_colour_int_state(self) -> bool:
        """ Return the current state of the colour sensor interrupt pin. """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR:
            return bool(self._colour_int.value())
        return False


    def set_flood_led(self, state: bool) -> None:
        """ Set the state of the flood LED pin to turn on or off the LED to illuminate the area under the colour sensor. """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_FLOOD_LEDS:
            self._led_control.init(mode=Pin.OUT)
            self._led_control.value(state)
            if self._logging:
                print(f"D:{self.config.port}:Flood LEDs={'On' if state else 'Off'}")


    def colour_enable(self, enable: bool) -> None:
        """Enable or disable interrupt-driven colour sensing.

        When enabled the OPT4060 runs in *continuous* mode: it measures repeatedly on its own and
        pulls its interrupt line low each time a new reading is ready. That falling edge is delivered
        to `_handle_colour_interrupt`, which reads the colour and publishes it as a `ColourEvent`.

        Args:
            enable: True to start colour sensing, False to stop it and power the sensor down.
        """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR == 0:
            return
        if enable:
            if self._colour_sensor is None:
                if self._i2c is None:
                    self._i2c = I2C(self.config.port)
                    if self._logging:
                        print(f"D:{self.config.port}:i2c init")
                self._colour_sensor = OPT4060(self._i2c, logging=self._logging)
                if self._logging:
                    print(f"D:{self.config.port}:Colour Sensor created")
            sensor = self._colour_sensor

            # Register the data-ready interrupt BEFORE starting continuous sensing so that the very
            # first "measurement ready" falling edge cannot be missed.
            # We MUST register for BOTH edges. The AW9523B expander interrupts on any change and the
            # badge firmware's C ISR shim (tildagon_pin_isr_handler) schedules whatever Python handler
            # is in the slot for the edge that occurred - with NO null check. When we clear the sensor's
            # interrupt (in read()) the line goes high, producing a rising edge; if only the falling
            # edge were registered, the rising-edge slot would be MP_OBJ_NULL and the VM would crash
            # (Guru Meditation LoadProhibited, EXCVADDR 0x0) trying to call NULL. Handling both edges is
            # harmless: the spurious rising-edge callback just sets the flag, and read() then finds no
            # measurement ready and returns without emitting.
            self._colour_int.init(mode=Pin.IN)
            self._colour_int.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._handle_colour_interrupt)
            sensor.start(self._colour_period_ms)
            if self._logging:
                print(f"D:{self.config.port}:Colour Sensor Started")
        else:
            self._colour_int.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=None)   # detach the interrupt handler first
            if self._colour_sensor is not None:
                sensor = self._colour_sensor
                sensor.stop()           # stop continuous sensing
                sensor.reset()          # force a full re-initialisation next time it is enabled
                if self._logging:
                    print(f"D:{self.config.port}:Colour Sensor Stopped")


    def set_colour_period(self, period_ms: int) -> None:
        """ Set the inter-measurement period for continuous colour sensing in milliseconds.
            A value of 0 means back-to-back measurements as fast as the sensor allows. """
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR:
            if period_ms < 0 or period_ms > 10000:
                # Invalid period, do nothing
                raise ValueError(f"D:{self.config.port}:Colour Sensor period must be between 0 and 10000ms")
            if period_ms == self._colour_period_ms:
                return  # No change needed
            self._colour_period_ms = period_ms
            if self._colour_sensor is not None:
                if self._colour_sensor.start(period_ms):
                    if self._logging:
                        print(f"D:{self.config.port}:Colour Sensor period set to {period_ms}ms")
                    return
        raise RuntimeError(f"D:{self.config.port}:Colour Sensor period set failed")


    @property
    def colour(self) -> tuple[int, int, int, int] | None:
        """Return the most recent colour measurement as a tuple of (R, G, B) values.

        Returns None until the first reading has been received. New readings arrive automatically
        (via `ColourEvent`) while continuous sensing is enabled with `colour_enable`.
        """
        if (self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR) and self._colour_sensor is not None:
            return self._colour_sensor.colour
        return None


    @property
    def colour_name(self) -> str | None:
        """Return the most recent colour measurement as a human-readable name.

        Returns None until the first reading has been received. New readings arrive automatically
        (via `ColourEvent`) while continuous sensing is enabled with `colour_enable`.
        """
        if (self._extended_header.flags & _EXTENDED_HEADER_FLAG_COLOUR_SENSOR) and self._colour_sensor is not None:
            return self._colour_sensor.colour_name
        return None


#----------------------------------------------------------------
# PRIVATE ASYNC methods
#----------------------------------------------------------------
    async def _handle_stop_app(self, event):
        """ Handle the RequestStopAppEvent so that we can release resources """
        try:
            if event.app == self:
                if self._logging:
                    print(f"D:{self.config.port}:Stop")
                self.deinit()
                # The badge HexpansionManagerApp tidies up the LS and HS pins when a hexpansion app is removed
        except (AttributeError, TypeError):
            pass


# --------------------------------------------------
# PRIVATE methods for internal use only.
# --------------------------------------------------
    def _read_extended_hexpansion_header(self) -> ExtendedHexpansionHeader:
        # We use the spare bytes of the first EEPROM sector, after the header, to store a flags
        # which indicates if the distance sensor pins are swapped or not.
        if self._i2c is None:
            try:
                self._i2c = I2C(self.config.port)
            except Exception as e:          # pylint: disable=broad-exception-caught
                print(f"D:{self.config.port}:i2c setup failed {e}")
                return ExtendedHexpansionHeader(flags=0)  # return a default header with no flags set
        try:
            extended_header_bytes = self._i2c.readfrom_mem(_ADDR, _EXTENDED_HEADER_ADDR, _EXTENDED_HEADER_SIZE, addrsize=_ADDR_LEN * 8)
            self._extended_header = ExtendedHexpansionHeader.from_bytes(extended_header_bytes)
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"D:{self.config.port}:extended header read failed {e}")
            self._extended_header = ExtendedHexpansionHeader(flags=0)  # return a default header with no flags set
        return self._extended_header


    def _write_extended_hexpansion_header(self, header: ExtendedHexpansionHeader) -> bool:
        # we know that on our EEPROM the extended header is stored in the first sector after the main header, so we
        # can write it directly to that location and it all fits wihtin the page size of the EEPROM so we don't need to worry about chunking it up.
        # the bytes in this EEPROM space must be blank (0xFF) before we write to it, otherwise the write will fail.
        if self._i2c is None:
            try:
                self._i2c = I2C(self.config.port)
            except Exception as e:          # pylint: disable=broad-exception-caught
                print(f"D:{self.config.port}:i2c setup failed {e}")
                return False
        try:
            header_bytes = header.to_bytes()
            self._i2c.writeto_mem(_ADDR, _EXTENDED_HEADER_ADDR, header_bytes, addrsize=_ADDR_LEN * 8)
            return True
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"D:{self.config.port}:extended header write failed {e}")
            return False


    # Set a single PWM duty cycle (0-65535) for a specific MOTOR output
    # if the channel has not been setup yet then we initialise it from scratch, otherwise we just change the duty cycle
    def _set_pwmoutput(self, _channel: int, _duty_cycle: int) -> bool:
        if _duty_cycle < 0 or _duty_cycle > 65535:
            return False
        try:
            if self.PWMOutput[_channel] is None:
                # Channel hasn't been setup yet so we need to initialise it from scratch
                pin = self.config.pin[_channel]
                if self._logging:
                    print(self._pwm_log_string(_channel) + f"{self.PWMOutput[_channel]} init ... pin={pin}")
                # Micropython v1.28 generates a spurious warning when we try to initialise a PWM on a pin that was previously used.
                # "W (557771) ledc: GPIO 47 is not usable, maybe conflict with others"
                # workaround is to set it to an input
                pin.init(mode=Pin.IN)
                pwm = PWM(pin, freq = self._freq[_channel])
                pwm.duty_u16(_duty_cycle)
                self.PWMOutput[_channel] = pwm
                if self._logging:
                    print(self._pwm_log_string(_channel) + f"{self.PWMOutput[_channel]} init")
            pwm = self.PWMOutput[_channel]
            if pwm is None:
                return False
            if _duty_cycle != pwm.duty_u16():
                pwm.duty_u16(_duty_cycle)
                if self._logging:
                    print(self._pwm_log_string(_channel) + f"{_duty_cycle}")
        except Exception as e:              # pylint: disable=broad-except
            print(self._pwm_log_string(_channel) + f"set {_duty_cycle} failed {e}")
            return False
        return True


    def _pwm_log_string(self, channel: int | None) -> str:
        """ Helper method to generate a log string for a PWM output change. """
        return f"D:{self.config.port}:PWM[{channel if channel is not None else 'All'}]:"


    def _check_port_for_hexdrive(self, port: int) -> HexDriveType:
        if hexpansion_app is None:
            if self._logging:
                print(f"D:{port}:No hexpansion app found")
            return _DEFAULT_HEXDRIVE_TYPE
        if not hasattr(hexpansion_app, "_hexpansion_manager"):
            if self._logging:
                print(f"D:{port}:No _hexpansion_manager attribute found")
            return _DEFAULT_HEXDRIVE_TYPE
        manager = hexpansion_app._hexpansion_manager        # pylint: disable=protected-access
        if manager is None:
            if self._logging:
                print(f"D:{port}:_hexpansion_manager is None")
            return _DEFAULT_HEXDRIVE_TYPE
        headers = manager.hexpansion_headers
        if headers[port] is None:
            if self._logging:
                print(f"D:{port}:No hexpansion header found")
            return _DEFAULT_HEXDRIVE_TYPE
        pid = headers[port].pid
        print(f"D:{port}:PID={pid:#04x}")

        # check which type of HexDrive this is by scanning the HEXDRIVE_TYPES list
        for _, hexpansion_type in enumerate(_HEXDRIVE_TYPES):
            # we only use the LSByte of the PID to identify the type of HexDrive, as the MSByte is used for other things
            if pid & 0xFF == hexpansion_type.pid:
                return hexpansion_type
        # we are not interested in this type of hexpansion
        return _DEFAULT_HEXDRIVE_TYPE


    def _parse_version(self, version):
        """ Parse a version string, e.g. that of BadgeOS, into a list of components for comparison. Handles versions in the format v1.9.0-beta.1+build.123
            The version is split into components based on the delimiters '.' '-' and '+'."""
        #pre_components = ["final"]
        #build_components = ["0", "000000z"]
        #build = ""
        components = []
        if "+" in version:
            version, build = version.split("+", 1)          # pylint: disable=unused-variable
        #    build_components = build.split(".")
        if "-" in version:
            version, pre_release = version.split("-", 1)    # pylint: disable=unused-variable
        #    if pre_release.startswith("rc"):
        #        # Re-write rc as c, to support a1, b1, rc1, final ordering
        #        pre_release = pre_release[1:]
        #    pre_components = pre_release.split(".")
        version = version.strip("v").split(".")
        components = [int(item) if item.isdigit() else item for item in version]
        #components.append([int(item) if item.isdigit() else item for item in pre_components])
        #components.append([int(item) if item.isdigit() else item for item in build_components])
        return components


#---------------------------------------------------------------------------------
# VL53L0X ToF distance sensor functions
#---------------------------------------------------------------------------------
# PRIVATE methods
#---------------------------------------------------------------------------------
    def _detect_and_set_dist_pins_swapped_flag(self) -> bool:
        # Setup to ENABLE the distance sensor by setting the XSHUT pin high, then wait a short time for the sensor to power up
        # Check if the distance sensor is present by trying to read the ID register from the sensor.
        # If it fails then there is a genuine fault - return False
        # Then DISABLE the distance sensor by setting the XSHUT pin low, then wait a short time for the sensor to power down
        # Check if the distance sensor is present by trying to read the ID register from the sensor
        # If it fails then we know that the XSHUT pin is correct and has control over the sensor - leave flags as is and return True.
        # If it can still be read then try swapping the pins.
        # Then DISABLE the distance sensor by setting the alt XSHUT pin low, then wait a short time for the sensor to power down
        # Check if the distance sensor is present by trying to read the ID register from the sensor
        # If it fails then we know that the alt XSHUT pin is correct and has control over the sensor - set the swapped flag and return True.

        def _clean_up():
            # Clean up the pins to avoid leaving them in an inconsistent state
            try:
                self.config.ls_pin[_RANGE_XSHUT_PIN].init(mode=Pin.IN)
                self.config.ls_pin[_ALT_RANGE_XSHUT_PIN].init(mode=Pin.IN)
            except Exception as e:      # pylint: disable=broad-except
                print(f"D:{self.config.port}:Distance Sensor pin cleanup failed {e}")

        # Setup to ENABLE the distance sensor by setting the XSHUT pin high, then wait a short time for the sensor to power up
        self.config.ls_pin[_RANGE_XSHUT_PIN].init(mode=Pin.OUT)
        self.config.ls_pin[_RANGE_XSHUT_PIN].value(1)
        self.config.ls_pin[_ALT_RANGE_XSHUT_PIN].init(mode=Pin.OUT)
        self.config.ls_pin[_ALT_RANGE_XSHUT_PIN].value(1)
        time.sleep_ms(_RANGE_SENSOR_XSHUT_RESPONSE_TIME_MS)
        # Check if the distance sensor is present by trying to read the ID register from the sensor.
        try:
            if self._i2c is None:
                self._i2c = I2C(self.config.port)
                print(f"D:{self.config.port}:i2c init")
            range_sensor = VL53L0X(self._i2c, logging=False)
            #print(f"D:{self.config.port}:Distance Sensor created")
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:Distance Sensor create failed {e}")
            _clean_up()
            return False
        # If it fails then there is a genuine fault - return False
        if not range_sensor.check_id():
            #print(f"D:{self.config.port}:Distance Sensor check failed")
            _clean_up()
            return False
        # Then DISABLE the distance sensor by setting the XSHUT pin low, then wait a short time for the sensor to power down
        self.config.ls_pin[_RANGE_XSHUT_PIN].value(0)
        time.sleep_ms(_RANGE_SENSOR_XSHUT_RESPONSE_TIME_MS)
        # Check if the distance sensor is present by trying to read the ID register from the sensor
        # If it fails then we know that the XSHUT pin is correct and has control over the sensor - leave flags as is and return True.
        if not range_sensor.check_id():
            #print(f"D:{self.config.port}:Distance Sensor shutdown by XSHUT low")
            self._extended_header.flags &= ~_EXTENDED_HEADER_FLAG_RANGE_PINS_SWAPPED
            _clean_up()
            return True
        # If it can still be read then try swapping the pins.
        self.config.ls_pin[_RANGE_XSHUT_PIN].value(1)
        self.config.ls_pin[_ALT_RANGE_XSHUT_PIN].value(0)
        time.sleep_ms(_RANGE_SENSOR_XSHUT_RESPONSE_TIME_MS)
        # Then DISABLE the distance sensor by setting the alt XSHUT pin low, then wait a short time for the sensor to power down
        # Check if the distance sensor is present by trying to read the ID register from the sensor
        # If it fails then we know that the alt XSHUT pin is correct and has control over the sensor - set the swapped flag and return True.
        if not range_sensor.check_id():
            #print(f"D:{self.config.port}:Distance Sensor shutdown by ALT XSHUT low")
            self._extended_header.flags |= _EXTENDED_HEADER_FLAG_RANGE_PINS_SWAPPED
            _clean_up()
            return True
        #print(f"D:{self.config.port}:Distance Sensor not shutdown by either XSHUT pin")
        _clean_up()
        return False


    def _handle_range_interrupt(self, _pin):
        """Distance-sensor data-ready interrupt handler (a *bound* method - see `range_enable`).

        Invoked via mp_sched_schedule from the badge's LS-pin interrupt plumbing whenever the VL53L0X
        signals that a new continuous measurement is ready.

        Args:
            _pin: the LS pin object that fired (supplied by the scheduler, unused - the bound 'self'
                already identifies the sensor).
        """

        # check the actual state of the interrupt pin to avoid spurious rising-edge callbacks (see `range_enable`).
        if  0 != self._range_int.value() or self._range_sensor is None:
            return
        measurement = self._range_sensor.read()    # reads the measurement and clears the interrupt to re-arm the sensor
        if isinstance(measurement, int):
            distance = measurement
            eventbus.emit(self.RangeEvent(distance))



#---------------------------------------------------------------------------------
# OPT4060 Colour Sensor functions
#---------------------------------------------------------------------------------
# PRIVATE methods
#---------------------------------------------------------------------------------

    def _handle_colour_interrupt(self, _pin):
        """Colour-sensor data-ready interrupt handler (a *bound* method - see `colour_enable`).

        Invoked via mp_sched_schedule from the badge's LS-pin interrupt plumbing whenever the OPT4060
        signals that a new continuous measurement is ready.

        Args:
            _pin: the LS pin object that fired (supplied by the scheduler, unused - the bound 'self'
                already identifies the sensor).
        """

        # check the actual state of the interrupt pin to avoid spurious rising-edge callbacks (see `colour_enable`).
        if 0 != self._colour_int.value() or self._colour_sensor is None:
            return
        measurement = self._colour_sensor.read()
        if measurement is not None:
            # we read the colour from the sensor class rather than using the return from read() to keep the linter quiet
            colour = self._colour_sensor.colour or (0,0,0,0)
            colour_name = self._colour_sensor.colour_name or "Unknown"
            eventbus.emit(self.ColourEvent(colour, colour_name))



"""
SensorBase - Abstract base class for all BadgeBot I2C sensor drivers.

Each concrete driver must implement:
  - I2C_ADDR    : int  - default 7-bit I2C address
  - init(i2c)   - initialise the sensor; returns True on success
  - start(period_ms) - start continuous measurements at the given period (ms); returns True on success
  - stop()      - stop continuous measurements; returns True on success
  - read()      - take a measurement
  - reset()     - put sensor to a safe/low-power state (called on cleanup)
  - shutdown()  - optional power-down hook (called on cleanup)
"""

class SensorBase:
    """Abstract base class for BadgeBot I2C sensor drivers."""
    # Sub-classes must override these
    I2C_ADDR = 0x00
    READ_INTERVAL_MS = 250
    NAME = "Unknown"
    TYPE = "Unknown"


    def __init__(self, i2c: I2C, i2c_addr: int, logging: bool = False):
        self._i2c = i2c
        self._ready = False
        self._i2c_addr = i2c_addr
        self._logging = logging
        self._continuous = False
        self._period_ms = 0


    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def init(self, i2c: I2C | None) -> bool:
        """Initialise the sensor on the given I2C bus.

        Returns True if the sensor is found and configured successfully.
        Store the i2c object for later use in read().
        """
        if i2c is not None:
            self._i2c = i2c
        self._ready = False
        self._continuous = False
        try:
            self._ready = self._init()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"D:{self.NAME} init error: {e}")
            self._ready = False
        return self._ready


    def check_id(self) -> bool:
        """Check the sensor ID register to verify that the sensor is present.

        Returns True if the sensor ID matches the expected value.
        """
        try:
            return self._check_id()
        except Exception as e:          # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"D:{self.NAME} check_id error: {e}")
            return False


    def start(self, period_ms: int) -> bool:
        """Start continuous measurements at the given period (ms).

        Returns True if the sensor is ready to take measurements.
        """
        self._continuous = False
        self._period_ms = period_ms
        if not self._ready:
            if not self.init(None):
                return False
        try:
            if self._start():
                self._continuous = True
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"D:{self.NAME} start error: {e}")
        return self._continuous


    def stop(self) -> bool:
        """Stop continuous measurements.

        Returns True if has been stopped without error.
        """
        if not self._ready or not self._continuous:
            return False
        self._continuous = False
        try:
            return self._stop()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"D:{self.NAME} stop error: {e}")
            return False


    def read(self) -> tuple[int,int,int,int] | int | None:
        """Return the latest measurement.
        Returns None on failure.
        """
        if not self._ready:
            return None
        try:
            return self._read()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"D:{self.NAME} read error: {e}")
            return None


    def reset(self) -> None:
        """Mark the driver as un-initialised.

        Call this after the sensor has been hardware reset so that the next
        `start`/`init` re-runs the full initialisation sequence.
        """
        self._ready = False
        self._continuous = False


    @property
    def is_ready(self) -> bool:
        """True if the sensor is initialised and ready for measurements."""
        return self._ready


    @property
    def is_continuous(self) -> bool:
        """True if the sensor is running in continuous measurement mode."""
        return self._continuous


    @property
    def i2c_addr(self) -> int:
        """Return the I2C address of the sensor."""
        return self._i2c_addr


    @property
    def logging(self) -> bool:
        """ Get or set the logging flag for debug output. """
        return self._logging


    @logging.setter
    def logging(self, value: bool):
        self._logging = value


    # ------------------------------------------------------------------
    # Internal helpers - override in sub-classes
    # ------------------------------------------------------------------

    def _init(self) -> bool:
        """Hardware initialisation. Return True on success."""
        raise NotImplementedError

    def _check_id(self) -> bool:
        """Check the sensor ID register. Return True if the ID matches."""
        raise NotImplementedError

    def _start(self) -> bool:
        """Start continuous measurements at the given period (ms). Return True on success."""
        raise NotImplementedError

    def _stop(self) -> bool:
        """Stop continuous measurements. Return True on success."""
        raise NotImplementedError

    def _read(self) -> tuple | int | None:
        """Perform measurement. Return measurement in appropriate format."""
        raise NotImplementedError


    # ------------------------------------------------------------------
    # Utility helpers available to all drivers
    # ------------------------------------------------------------------

    def _read_u8(self, reg: int) -> int:
        return self._i2c.readfrom_mem(self._i2c_addr, reg, 1)[0]


    def _write_u8(self, reg: int, value: int) -> None:
        self._i2c.writeto_mem(self._i2c_addr, reg, bytes([value & 0xFF]))


    def _read_u16_be(self, reg: int) -> int:
        d = self._i2c.readfrom_mem(self._i2c_addr, reg, 2)
        return (d[0] << 8) | d[1]


    def _read_s16_be(self, reg: int) -> int:
        value = self._read_u16_be(reg)
        if value & 0x8000:
            value -= 0x10000
        return value


    def _write_u16_be(self, reg: int, value: int) -> None:
        self._i2c.writeto_mem(self._i2c_addr, reg, bytes([(value >> 8) & 0xFF, value & 0xFF]))


    def _write_u32_be(self, reg: int, value: int) -> None:
        self._i2c.writeto_mem(self._i2c_addr, reg, bytes([
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]))




"""
VL53L0X Time-of-Flight distance sensor driver.

Default I2C address: 0x29
Measurement: distance in mm (up to ~1200 mm in default mode).

This driver runs the sensor in interrupt-driven continuous mode: call init(), then start(period_ms),
and call read() each time the data-ready interrupt fires to fetch the latest distance.

Datasheet: https://www.st.com/resource/en/datasheet/vl53l0x.pdf
"""

_RANGE_I2C_ADDRESS = const(0x29)
_WHO_AM_I_REG    = const(0xC0)
_WHO_AM_I_EXPECT = const(0xEE)

# Key registers (abridged - sufficient for single-shot ranging)
_SYSRANGE_START                              = const(0x00)
_SYSTEM_SEQUENCE_CONFIG                      = const(0x01)
_SYSTEM_INTERRUPT_CONFIG                     = const(0x0A)
_SYSTEM_INTERRUPT_CLEAR                      = const(0x0B)
_RESULT_INTERRUPT_STATUS                     = const(0x13)
_RESULT_RANGE_STATUS                         = const(0x14)
_MSRC_CONFIG_CONTROL                         = const(0x60)
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = const(0x44)
_GPIO_HV_MUX_ACTIVE_HIGH                     = const(0x84)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = const(0xB0)
_GLOBAL_CONFIG_REF_EN_START_SELECT           = const(0xB6)
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = const(0x4E)
_DYNAMIC_SPAD_REF_EN_START_OFFSET            = const(0x4F)
_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = const(0x89)

_STOP_VARIABLE_REG = const(0x91)
_SPAD_INFO_REG = const(0x92)
_SPAD_POLL_REG = const(0x83)
_INTERRUPT_READY_MASK = const(0x07)

# Continuous-mode registers
_SYSTEM_INTERMEASUREMENT_PERIOD = const(0x04)  # 32-bit inter-measurement period (used only in timed continuous mode)
_OSC_CALIBRATE_VAL              = const(0xF8)  # 16-bit oscillator calibration value, used to scale the period into sensor ticks

_RANGE_TIMEOUT_MS = const(100)   # ms to wait for a measurement

_DEFAULT_TUNING_SETTINGS = (
    (const(0xFF), const(0x01)), (const(0x00), const(0x00)),
    (const(0xFF), const(0x00)), (const(0x09), const(0x00)), (const(0x10), const(0x00)), (const(0x11), const(0x00)),
    (const(0x24), const(0x01)), (const(0x25), const(0xFF)), (const(0x75), const(0x00)),
    (const(0xFF), const(0x01)), (const(0x4E), const(0x2C)), (const(0x48), const(0x00)), (const(0x30), const(0x20)),
    (const(0xFF), const(0x00)), (const(0x30), const(0x09)), (const(0x54), const(0x00)), (const(0x31), const(0x04)),
    (const(0x32), const(0x03)), (const(0x40), const(0x83)), (const(0x46), const(0x25)), (const(0x60), const(0x00)),
    (const(0x27), const(0x00)), (const(0x50), const(0x06)), (const(0x51), const(0x00)), (const(0x52), const(0x96)),
    (const(0x56), const(0x08)), (const(0x57), const(0x30)), (const(0x61), const(0x00)), (const(0x62), const(0x00)),
    (const(0x64), const(0x00)), (const(0x65), const(0x00)), (const(0x66), const(0xA0)),
    (const(0xFF), const(0x01)), (const(0x22), const(0x32)), (const(0x47), const(0x14)), (const(0x49), const(0xFF)),
    (const(0x4A), const(0x00)),
    (const(0xFF), const(0x00)), (const(0x7A), const(0x0A)), (const(0x7B), const(0x00)), (const(0x78), const(0x21)),
    (const(0xFF), const(0x01)), (const(0x23), const(0x34)), (const(0x42), const(0x00)), (const(0x44), const(0xFF)),
    (const(0x45), const(0x26)), (const(0x46), const(0x05)), (const(0x40), const(0x40)), (const(0x0E), const(0x06)),
    (const(0x20), const(0x1A)), (const(0x43), const(0x40)),
    (const(0xFF), const(0x00)), (const(0x34), const(0x03)), (const(0x35), const(0x44)),
    (const(0xFF), const(0x01)), (const(0x31), const(0x04)), (const(0x4B), const(0x09)), (const(0x4C), const(0x05)),
    (const(0x4D), const(0x04)),
    (const(0xFF), const(0x00)), (const(0x44), const(0x00)), (const(0x45), const(0x20)), (const(0x47), const(0x08)),
    (const(0x48), const(0x28)), (const(0x67), const(0x00)), (const(0x70), const(0x04)), (const(0x71), const(0x01)),
    (const(0x72), const(0xFE)), (const(0x76), const(0x00)), (const(0x77), const(0x00)),
    (const(0xFF), const(0x01)), (const(0x0D), const(0x01)),
    (const(0xFF), const(0x00)), (const(0x80), const(0x01)), (const(0x01), const(0xF8)),
    (const(0xFF), const(0x01)), (const(0x8E), const(0x01)), (const(0x00), const(0x01)),
    (const(0xFF), const(0x00)), (const(0x80), const(0x00)),
)

class VL53L0X(SensorBase):
    """VL53L0X Time-of-Flight distance sensor driver."""
    I2C_ADDR = _RANGE_I2C_ADDRESS
    NAME = "VL53L0X"
    TYPE = "Distance"
    READ_INTERVAL_MS = 100

    def __init__(self, i2c: I2C, logging: bool = False):
        super().__init__(i2c=i2c, i2c_addr=self.I2C_ADDR, logging=logging)
        self._continuous = False            # True while continuous (interrupt-driven) ranging is active
        self._stop_variable = 0             # used to store the stop variable value for the VL53L0X sensor
        self._last_range_mm = 0             # last range reading in mm


    @property
    def range(self) -> int:
        """Return the last range reading in mm."""
        return self._last_range_mm


    def _init(self) -> bool:
        """Initialise the sensor hardware.

        The one-off setup sequence (device ID check, SPAD calibration, reference calibration) is
        blocking because it polls the sensor, so it is deliberately kept out of the interrupt path.
        Safe to call repeatedly - it becomes a no-op once initialisation has succeeded.

        Returns:
            True if the sensor is initialised and ready, False on failure.
        """
        if not self._check_id():
            return False

        self._continuous = False

        # The VL53L0X needs a substantial startup sequence before single-shot
        # ranging becomes trustworthy;
        self._write_u8(
            _VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
            self._read_u8(_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01)
        self._write_u8(0x88, 0x00)
        self._open_stop_variable_window()
        self._stop_variable = self._read_u8(_STOP_VARIABLE_REG)
        self._close_stop_variable_window()
        self._write_u8(
            _MSRC_CONFIG_CONTROL,
            self._read_u8(_MSRC_CONFIG_CONTROL) | 0x12)
        self._set_signal_rate_limit(0.25)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        spad_info = self._get_spad_info()
        if spad_info is None:
            return False

        spad_count, spad_type_is_aperture = spad_info
        ref_spad_map = bytearray(self._i2c.readfrom_mem(self._i2c_addr, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6))
        self._write_u8(0xFF, 0x01)
        self._write_u8(_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self._write_u8(_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self._write_u8(0xFF, 0x00)
        self._write_u8(_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)

        first_spad_to_enable = 12 if spad_type_is_aperture else 0
        spads_enabled = 0
        for index in range(48):
            if index < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[index // 8] &= ~(1 << (index % 8))
                continue
            if (ref_spad_map[index // 8] >> (index % 8)) & 0x01:
                spads_enabled += 1
        self._i2c.writeto_mem(self._i2c_addr, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, bytes(ref_spad_map))

        print(f"D:VL53L0X SPAD count={spad_count}, enabled={spads_enabled}, type={'aperture' if spad_type_is_aperture else 'non-aperture'}")

        for reg, value in _DEFAULT_TUNING_SETTINGS:
            self._write_u8(reg, value)

        self._write_u8(_SYSTEM_INTERRUPT_CONFIG, 0x04)
        self._write_u8(
            _GPIO_HV_MUX_ACTIVE_HIGH,
            self._read_u8(_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10,
        )
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._perform_single_ref_calibration(0x40)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._perform_single_ref_calibration(0x00)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        return True


    def _check_id(self) -> bool:
        """Check the sensor's ID register to confirm that it is present and responding."""
        who = self._read_u8(_WHO_AM_I_REG)
        if who != _WHO_AM_I_EXPECT:
            if self._logging:
                print(f"D:VL53L0X unexpected ID 0x{who:02X} (expected 0x{_WHO_AM_I_EXPECT:02X})")
            return False
        return True


    def _start(self) -> bool:
        """Start continuous (interrupt-driven) ranging.

        In continuous mode the sensor measures repeatedly on its own and asserts its interrupt line
        each time a new reading is ready; call `read` from the interrupt handler to retrieve the value
        and re-arm the sensor.

        period_ms: inter-measurement period in milliseconds. 0 selects back-to-back mode (the
                sensor measures as fast as it can); a positive value selects timed mode with the
                requested gap between measurements.

        Returns:
            True on success, False on failure.
        """

        # Apply the per-device "stop variable" - required before (re)starting ranging.
        self._prepare_ranging()
        if self._period_ms > 0:
            # Timed continuous mode: the requested period must be scaled by the sensor's oscillator
            # calibration value before it is written to the inter-measurement period register.
            osc_calibrate_val = self._read_u16_be(_OSC_CALIBRATE_VAL)
            if osc_calibrate_val != 0:
                self._period_ms *= osc_calibrate_val
            self._write_u32_be(_SYSTEM_INTERMEASUREMENT_PERIOD, self._period_ms)
            mode = 0x04     # VL53L0X_REG_SYSRANGE_MODE_TIMED
        else:
            mode = 0x02     # VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
        self._write_u8(_SYSRANGE_START, mode)
        return True


    def _stop(self) -> bool:
        """Stop continuous ranging and return the sensor to idle. Returns success or failure."""
        self._write_u8(_SYSRANGE_START, 0x01)  # VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT (halts continuous)
        # Clear the stored stop variable window (matches the reference driver's stopContinuous()).
        self._write_u8(0xFF, 0x01)
        self._write_u8(0x00, 0x00)
        self._write_u8(_STOP_VARIABLE_REG, 0x00)
        self._write_u8(0x00, 0x01)
        self._write_u8(0xFF, 0x00)
        return True


    def reset(self) -> None:
        """Mark the driver as un-initialised.

        Call this after the sensor has been hardware reset via its XSHUT pin so that the next
        `start`/`init` re-runs the full initialisation sequence.
        """
        self._ready = False
        self._continuous = False
        self._last_range_mm = 0


    def _read(self) -> int | None:
        """Read the most recent range in millimetres and clear the data-ready interrupt.

        Clearing the interrupt re-arms the sensor for the next measurement.

        Returns:
            The measured distance in mm, or None if the sensor is not ready or no measurement is
            currently available.
        """

        # A single status read is sufficient: the data-ready interrupt bit confirms a fresh
        # measurement is waiting (we normally get here because the interrupt line already fired).
        if (self._read_u8(_RESULT_INTERRUPT_STATUS) & _INTERRUPT_READY_MASK) == 0:
            if self._logging:
                print("D:VL53L0X read called but no measurement ready")
            return None
        # The range value lives 10 bytes into the RESULT_RANGE_STATUS block in ST's register map;
        # this offset matches the reference driver.
        dist_mm = self._read_u16_be(_RESULT_RANGE_STATUS + 10)
        # Clear the interrupt so the sensor can complete the next continuous measurement.
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._last_range_mm = dist_mm
        return dist_mm


    def _open_stop_variable_window(self) -> None:
        self._write_u8(0x80, 0x01)
        self._write_u8(0xFF, 0x01)
        self._write_u8(0x00, 0x00)


    def _close_stop_variable_window(self) -> None:
        self._write_u8(0x00, 0x01)
        self._write_u8(0xFF, 0x00)
        self._write_u8(0x80, 0x00)


    def _prepare_ranging(self) -> None:
        # Write the per-device stop variable captured during initialisation. This must be done before
        # (re)starting ranging, in either single-shot or continuous mode.
        self._open_stop_variable_window()
        self._write_u8(_STOP_VARIABLE_REG, self._stop_variable)
        self._close_stop_variable_window()


    def _wait_for_interrupt_ready(self) -> bool:
        # Blocking poll used ONLY during the one-off reference calibration in _init(); steady-state
        # ranging is fully interrupt driven (see HexDriveApp._handle_range_interrupt).
        deadline = time.ticks_add(time.ticks_ms(), _RANGE_TIMEOUT_MS)
        while (self._read_u8(_RESULT_INTERRUPT_STATUS) & _INTERRUPT_READY_MASK) == 0:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return False
            time.sleep_ms(1)
        return True


    def _perform_single_ref_calibration(self, vhv_init_byte: int) -> None:
        self._write_u8(_SYSRANGE_START, 0x01 | vhv_init_byte)
        self._wait_for_interrupt_ready()
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_u8(_SYSRANGE_START, 0x00)


    def _set_signal_rate_limit(self, limit_mcps: float) -> None:
        int_limit = int(limit_mcps * (1 << 7))
        self._i2c.writeto_mem(self._i2c_addr, _FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, bytes([(int_limit >> 8) & 0xFF, int_limit & 0xFF]))


    def _get_spad_info(self) -> tuple[int, bool] | None:
        self._open_stop_variable_window()
        self._write_u8(0xFF, 0x06)
        self._write_u8(_SPAD_POLL_REG, self._read_u8(_SPAD_POLL_REG) | 0x04)
        self._write_u8(0xFF, 0x07)
        self._write_u8(0x81, 0x01)
        self._write_u8(0x80, 0x01)
        self._write_u8(0x94, 0x6B)
        self._write_u8(_SPAD_POLL_REG, 0x00)

        # Blocking poll: this runs only once, during _init(), so it is off the interrupt path.
        deadline = time.ticks_add(time.ticks_ms(), _RANGE_TIMEOUT_MS)
        while self._read_u8(_SPAD_POLL_REG) == 0x00:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return None
            time.sleep_ms(1)

        self._write_u8(_SPAD_POLL_REG, 0x01)
        spad_info = self._read_u8(_SPAD_INFO_REG)

        self._write_u8(0x81, 0x00)
        self._write_u8(0xFF, 0x06)
        self._write_u8(_SPAD_POLL_REG, self._read_u8(_SPAD_POLL_REG) & ~0x04)
        self._write_u8(0xFF, 0x01)
        self._close_stop_variable_window()

        return spad_info & 0x7F, ((spad_info >> 7) & 0x01) == 1


"""
OPT4060 RGBW Colour sensor driver.

Default I2C address: 0x44
Texas Instruments OPT4060 — high-speed, high-precision RGBW colour
sensor with four channels (Red, Green, Blue, Clear/White).

Supports up to four devices on a single I2C bus via address-select pin:
  0x44 — ADDR tied to GND - THIS IS WHAT WE ARE USING
  0x45 — ADDR tied to VCC
  0x46 — ADDR tied to SDA
  0x47 — ADDR tied to SCL

The OPT4060 shares its register map and the same DEVICE_ID value (0x821)
with the OPT4048 — the two devices have different channel
content (RGB vs CIE1931 XYZ) rather than a unique identifier register.

Measurements:
  - red   : Red channel (raw ADC code)
  - green : Green channel (raw ADC code)
  - blue  : Blue channel (raw ADC code)
  - w     : Clear / White channel (raw ADC code)
Datasheet: https://www.ti.com/lit/ds/symlink/opt4060.pdf
"""

_COLOUR_I2C_ADDRESS = const(0x44)


# ── Register addresses (16-bit big-endian) ──────────────────────────────────
_REG_RED_MSB        = 0x00   # Red channel MSB (exponent[15:12] | mantissa_hi[11:0])
_REG_RED_LSB        = 0x01   # Red channel LSB (mantissa_lo[15:8] | counter[7:4] | crc[3:0])
_REG_GREEN_MSB      = 0x02   # Green channel MSB
_REG_GREEN_LSB      = 0x03   # Green channel LSB
_REG_BLUE_MSB       = 0x04   # Blue channel MSB
_REG_BLUE_LSB       = 0x05   # Blue channel LSB
_REG_CLEAR_MSB      = 0x06   # Clear / White channel MSB
_REG_CLEAR_LSB      = 0x07   # Clear / White channel LSB
_REG_THRESH_LO      = 0x08   # Low threshold
_REG_THRESH_HI      = 0x09   # High threshold
_REG_CONFIG         = 0x0A   # Configuration register
_REG_INT_CTRL       = 0x0B   # Interrupt / threshold configuration
_REG_RES_CTRL       = 0x0C   # Result control / status flags
_REG_DEVICE_ID      = 0x11   # Device ID (expect 0x0821, lower 12 bits = 0x821)

# ── Device identification ────────────────────────────────────────────────────
_DEVICE_ID_MASK     = 0x0FFF   # Lower 12 bits contain device ID
_DEVICE_ID_EXPECT   = 0x0821   # Same value as OPT4048 — distinguish by channel content

# ── Data register bit masks (per 16-bit register) ───────────────────────────
# MSB register  [15:12] exponent, [11:0] mantissa_hi
# LSB register  [15:8]  mantissa_lo, [7:4] sample counter, [3:0] CRC
_DATA_EXPONENT_MASK  = 0xF000   # Bits 15:12 of MSB register
_DATA_MSB_MASK       = 0x0FFF   # Bits 11:0  of MSB register (mantissa high)
_DATA_LSB_MASK       = 0xFF00   # Bits 15:8  of LSB register (mantissa low)
_DATA_COUNTER_MASK   = 0x00F0   # Bits 7:4   of LSB register (sample counter)
_DATA_CRC_MASK       = 0x000F   # Bits 3:0   of LSB register (CRC)

# ── CONFIG register (0x0A) bit layout (16-bit big-endian) ────────────────────
# Bit 15      : QWAKE   — quick wake from standby
# Bits 14     : reserved
# Bits 13-10  : RANGE   — 4-bit full-scale range selector
# Bits 9-6    : CONVERSION_TIME — per-channel integration time selector
# Bits 5-4    : OPERATING_MODE  — power/conversion mode
# Bit 3       : INT_LATCH  — 1 = latch interrupt until status is read
# Bit 2       : INT_POL    — interrupt pin polarity (0 = active-low)
# Bits 1-0    : FAULT_COUNT — number of out-of-range results before interrupt
_CFG_QWAKE_MASK      = 0x8000   # Bit 15
_CFG_RANGE_MASK      = 0x3C00   # Bits 13:10
_CFG_CONV_TIME_MASK  = 0x03C0   # Bits 9:6
_CFG_OPER_MODE_MASK  = 0x0030   # Bits 5:4
_CFG_INT_LATCH_MASK  = 0x0008   # Bit 3
_CFG_INT_POL_MASK    = 0x0004   # Bit 2
_CFG_FAULT_CNT_MASK  = 0x0003   # Bits 1:0

# Range constants (RANGE field, bits 13:10)
RANGE_2K        = 0    # ~2.2 klux full scale
RANGE_4K        = 1    # ~4.5 klux
RANGE_9K        = 2    # ~9 klux
RANGE_18K       = 3    # ~18 klux
RANGE_36K       = 4    # ~36 klux
RANGE_72K       = 5    # ~72 klux
RANGE_144K      = 6    # ~144 klux
RANGE_AUTO      = 12   # Automatic range selection

# Conversion time constants (CONVERSION_TIME field, per channel)
CONV_600US      = 0    # 600 µs
CONV_1MS        = 1    # 1 ms
CONV_1_8MS      = 2    # 1.8 ms
CONV_3_4MS      = 3    # 3.4 ms
CONV_6_5MS      = 4    # 6.5 ms
CONV_12_7MS     = 5    # 12.7 ms
CONV_25MS       = 6    # 25 ms
CONV_50MS       = 7    # 50 ms
CONV_100MS      = 8    # 100 ms
CONV_200MS      = 9    # 200 ms
CONV_400MS      = 10   # 400 ms
CONV_800MS      = 11   # 800 ms

# Operating mode constants (OPERATING_MODE field, bits 5:4)
MODE_POWERDOWN  = 0    # Power-down
MODE_FORCED     = 1    # Forced (auto-range one-shot)
MODE_ONE_SHOT   = 2    # Single conversion then power-down
MODE_CONTINUOUS = 3    # Continuous conversion

# Interrupt polarity constants
INT_POL_ACTIVE_LOW  = 0
INT_POL_ACTIVE_HIGH = 1

# Fault count constants (number of faults before interrupt)
FAULT_COUNT_1   = 0
FAULT_COUNT_2   = 1
FAULT_COUNT_4   = 2
FAULT_COUNT_8   = 3

# ── INT_CTRL register (0x0B) bit layout ──────────────────────────────────────
# Bit 15-7    : reserved
# Bits 6-5    : THRESH_SEL  — threshold channel select (0=Red, 1=Green, 2=Blue, 3=Clear)
# Bit 4       : INT_DIR     — interrupt direction (1 = output, 0 = input)
# Bits 3-2    : INT_CFG     — interrupt configuration
# Bit 1-0     : reserved
_INT_CTRL_THRESH_SEL_MASK  = const(0x0060)   # Bits 6:5
_INT_CTRL_INT_DIR_MASK     = const(0x0010)   # Bit 4
_INT_CTRL_INT_CFG_MASK     = const(0x000C)   # Bits 3:2

# INT_CFG values
_INT_CFG_SMBUS    = const(0)    # SMBUS alert (threshold interrupt disabled for polling)
_INT_CFG_NEXT_CH  = const(1)    # Interrupt on next channel conversion complete
_INT_CFG_DISABLED = const(0)    # Alias: effectively disabled for polled usage
_INT_CFG_ALL_READY = const(3)   # Interrupt when all channels have converted

# INT_DIR values
_INT_DIR_INPUT  = const(0)   # INT pin is an input (disabled as output)
_INT_DIR_OUTPUT = const(1)   # INT pin is an output (driven by sensor)

# Threshold channel select values
_THRESH_CH_RED   = const(0)
_THRESH_CH_GREEN = const(1)
_THRESH_CH_BLUE  = const(2)
_THRESH_CH_CLEAR = const(3)

# ── RES_CTRL register (0x0C) status flags ────────────────────────────────────
# Bits 15-4   : reserved
# Bit 3       : OVERLOAD   — ADC saturation/overflow on any channel
# Bit 2       : CONV_READY — conversion-complete flag (all channels done)
# Bit 1       : FLAG_H     — measurement exceeds high threshold
# Bit 0       : FLAG_L     — measurement below low threshold
_RES_CTRL_OVERLOAD_MASK    = const(0x0008)   # Bit 3
_RES_CTRL_CONV_READY_MASK  = const(0x0004)   # Bit 2
_RES_CTRL_FLAG_H_MASK      = const(0x0002)   # Bit 1
_RES_CTRL_FLAG_L_MASK      = const(0x0001)   # Bit 0

_WHITE_CAL_SCALE = const(4096)


class OPT4060(SensorBase):
    """OPT4060 Colour Sensor driver.

    Returns four 20-bit ADC values:
      "red"   — Red channel
      "green" — Green channel
      "blue"  — Blue channel
      "w"     — Clear / White channel
    """

    I2C_ADDR = _COLOUR_I2C_ADDRESS
    NAME = "OPT4060"
    TYPE = "Colour"
    READ_INTERVAL_MS = 10


    def __init__(self, i2c: I2C, logging: bool = False):
        super().__init__(i2c=i2c, i2c_addr=self.I2C_ADDR, logging=logging)
        self._continuous = False            # True while continuous (interrupt-driven) sensing is active
        self._overload = False              # True if the last reading was saturated/overflowed
        self._last_colour: tuple[int, int, int, int] | None = None  # Last RGBC reading
        self._last_colour_name: str | None = None  # Last colour name (from lookup)
        self._white_gains: tuple[float, float, float, float] = (10.0, 7.0, 20.0, 2.0)  # white reference gains for RGBC channels, scaled by _WHITE_CAL_SCALE
        self._black_reference: tuple[int, int, int, int] | None = None  # Black reference RGBC values
        self._white_reference: tuple[int, int, int, int] | None = None  # White reference RGBC values

    @property
    def overload(self) -> bool:
        """True if the last reading was saturated/overflowed."""
        return self._overload


    @property
    def colour(self) -> tuple[int, int, int, int] | None:
        """Return the last RGBC reading as a tuple of (R, G, B, W), or None if no reading yet."""
        return self._last_colour


    @property
    def colour_name(self) -> str | None:
        """Return the last colour name (from lookup), or None if no reading yet."""
        return self._last_colour_name


    @property
    def black_reference(self) -> tuple[int, int, int, int] | None:
        """Return the current black reference RGBC values, or None if not set."""
        return self._black_reference


    @black_reference.setter
    def black_reference(self, colour: tuple[int, int, int, int] | None = None) -> None:
        """Capture the current RGBC reading as the black reference.
           If no colour is provided, use the last RGBC reading."""
        if colour is None:
            colour = self._last_colour
        if colour is None:
            return
        self._black_reference = colour
        print(f"D:Black reference: r={colour[0]}, g={colour[1]}, b={colour[2]}, w={colour[3]}")


    @property
    def white_reference(self) -> tuple[int, int, int, int] | None:
        """Return the current white reference RGBC values, or None if not set."""
        return self._white_reference


    @white_reference.setter
    def white_reference(self, colour: tuple[int, int, int, int] | None = None) -> bool:
        """Capture the current RGBC reading as the white reference and compute gains.
           If no colour is provided, use the last RGBC reading."""
        if colour is None:
            colour = self._last_colour
        if colour is None:
            return False

        self._white_reference = colour
        print(f"D:White reference: r={colour[0]}, g={colour[1]}, b={colour[2]}, w={colour[3]}")

        if self._black_reference is None:
            self._black_reference = (0, 0, 0, 0)

        self._white_gains = self._reference_to_gains(self._black_reference, self._white_reference)

        print(f"D:gains: {self._white_gains}")
        return True


    def apply_white_reference(self, colour: tuple[int, int, int, int] | None = None) -> tuple[int, int, int, int]:
        """Apply white reference gains to raw RGBC values and return adjusted RGBC tuple."""
        if colour is None:
            colour = self._last_colour
        if colour is None:
            return (0, 0, 0, 0)
        r, g, b, w = colour
        if self._white_gains is None:
            return (r, g, b, w)
        if self._black_reference is not None:
            r = max(0, r - self._black_reference[0])
            g = max(0, g - self._black_reference[1])
            b = max(0, b - self._black_reference[2])
            w = max(0, w - self._black_reference[3])
        return (
            max(0, int((r * self._white_gains[0]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE),
            max(0, int((g * self._white_gains[1]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE),
            max(0, int((b * self._white_gains[2]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE),
            max(0, int((w * self._white_gains[3]) + (_WHITE_CAL_SCALE // 2)) // _WHITE_CAL_SCALE) if w > 0 else 0,
        )


    @staticmethod
    def lookup_colour_RGB(colour: tuple[int, int, int, int]) -> str:
        """Identifies a color name from raw RGB channel readings using HSV colour space.

        HSV naturally separates chromatic colour (hue) from achromatic attributes
        (saturation and brightness), giving more accurate named-colour matching
        than projecting R/G/B onto the CIE xy diagram.

        Parameters:
            colour  : raw channel values (any consistent scale — need not be 0-255).
                      optional broadband clear-channel reading from the same sensor.
                      When provided it is used as a brightness reference to distinguish
                      Black from Gray from White in low-saturation scenes.
        """
        r, g, b, clear = colour

        max_c = max(r, g, b)
        if max_c == 0:
            return "Black"

        min_c = min(r, g, b)
        delta = max_c - min_c

        # Saturation (0.0 – 1.0): how far from grey the colour is
        s = delta / max_c

        # --- Achromatic branch (low saturation) ---
        if s < 0.20:
            # Use the clear channel as a brightness reference when available;
            # otherwise fall back to max_c compared to the channel ceiling.
            brightness_ref = clear if clear > 0 else max_c
            if brightness_ref > 0:
                reflectance = max_c / brightness_ref
            else:
                reflectance = 0.0
            if reflectance < 0.15:
                return "Black"
            if reflectance > 0.65:
                return "White"
            return "Gray"

        # --- Chromatic branch: compute hue (0 – 360°) ---
        if max_c == r:
            h = 60.0 * (((g - b) / delta) % 6)
        elif max_c == g:
            h = 60.0 * ((b - r) / delta + 2)
        else:
            h = 60.0 * ((r - g) / delta + 4)

        if h < 20 or h >= 340:
            return "Red"
        if h < 45:
            return "Orange"
        if h < 70:
            return "Yellow"
        if h < 150:
            return "Green"
        if h < 200:
            return "Cyan"
        if h < 260:
            return "Blue"
        return "Magenta"


    def _init(self) -> bool:
        if not self._check_id():
            return False

        self._continuous = False
        self._overload = False

        # Configure for fast continuous reads within ~10 ms budget:
        #   Range       : auto (best dynamic range)
        #   Conv time   : 1.8 ms per channel → 4 × 1.8 ms ≈ 7.2 ms total
        #   Mode        : continuous
        #   INT latch   : latched (bit 3 = 1)
        #   INT polarity: active-low (bit 2 = 0)
        #   Fault count : 1 (bits 1:0 = 0)
        cfg = (RANGE_AUTO << 10) | (CONV_1_8MS << 6) | (MODE_CONTINUOUS << 4) | _CFG_INT_LATCH_MASK
        self._write_u16_be(_REG_CONFIG, cfg)

        # Use latched interrupt mode so the CONV_READY flag stays set long enough
        # to be reliably sampled — the non-latched pulse is only ~1 µs wide.
        # the threshold values are arbitrary and set to be equal, so the interrupt will fire on every conversion.
        self._set_latched_interrupt(True, threshold_low=0x8400, threshold_high=0x8400)
        return True


    def _start(self) -> bool:
        """Start continuous (interrupt-driven) colour sensing.

        In continuous mode the sensor measures repeatedly on its own and asserts its interrupt line
        each time a new reading is ready; call `read` from the interrupt handler to retrieve the value
        and re-arm the sensor.

        TODO - take notice of the period_ms parameter and set the inter-measurement period register accordingly.

        Returns:
            True on success, False on failure.
        """
        # Enable continuous mode (see datasheet section 7.3.1)
        self._write_u8(0x00, 0x01)  # MODE = 1 (continuous)
        self._write_u8(0x01, 0x00)  # INT = 0 (interrupt on every sample)
        self._write_u8(0x02, 0x00)  # INTCLR = 0 (clear interrupt)
        return True


    def _stop(self) -> bool:
        """Stop continuous sensing and return the sensor to idle. Returns success or failure."""
        self._set_mode(MODE_POWERDOWN)
        return True


    def _read(self) -> tuple[int, int, int, int] | None:
        # is there a new reading available? (CONV_READY bit in RES_CTRL register)
        st = self._read_u16_be(_REG_RES_CTRL)
        if st & _RES_CTRL_CONV_READY_MASK:
            # Read the overload flag (saturation/overflow) and store it for later retrieval.
            self._overload = bool(st & _RES_CTRL_OVERLOAD_MASK)

            # Burst-read all 4 channels (8 registers × 2 bytes = 16 bytes)
            raw = self._i2c.readfrom_mem(self._i2c_addr, _REG_RED_MSB, 16)

            r = self._decode_channel(raw, 0)
            g = self._decode_channel(raw, 4)
            b = self._decode_channel(raw, 8)
            w = self._decode_channel(raw, 12)

            self._last_colour = (r, g, b, w)
            calibrated_colour = self.apply_white_reference(self._last_colour)
            self._last_colour_name = self.lookup_colour_RGB(calibrated_colour)
            return r, g, b, w
        return None


#---------------------------------------------------------------------------------
# PRIVATE methods
#---------------------------------------------------------------------------------

    def _check_id(self) -> bool:
        """Check the sensor's ID register to confirm that it is present and responding."""
        device_id = self._read_u16_be(_REG_DEVICE_ID) & _DEVICE_ID_MASK
        if device_id != _DEVICE_ID_EXPECT:
            if self._logging:
                print(f"D:OPT4060 unexpected ID 0x{device_id:04X} (expected 0x{_DEVICE_ID_EXPECT:04X})")
            return False
        return True


    def _set_latched_interrupt(self, threshold_ch: int = _THRESH_CH_CLEAR,
                              threshold_low: int = 0x0000, threshold_high: int = 0xFFFF):
        """Enable latched threshold interrupt.

        When enabled the INT pin is held asserted until the status register
        is read, which is more reliable than the 1 µs pulse of the
        non-latched interrupt mode.
        """
        self._write_u16_be(_REG_THRESH_LO, threshold_low)
        self._write_u16_be(_REG_THRESH_HI, threshold_high)

        cfg = self._read_u16_be(_REG_CONFIG)
        cfg |= _CFG_INT_LATCH_MASK      # INT_LATCH = 1
        self._write_u16_be(_REG_CONFIG, cfg)

        tcfg = self._read_u16_be(_REG_INT_CTRL)
        # Set threshold channel, INT as output, threshold interrupt config
        tcfg = (tcfg & 0x8001) | ((threshold_ch & 0x03) << 5) | (_INT_DIR_OUTPUT << 4) | (_INT_CFG_SMBUS << 2)
        self._write_u16_be(_REG_INT_CTRL, tcfg)


    def _set_mode(self, mode: int):
        """Set the operating mode (use MODE_* constants)."""
        cfg = self._read_u16_be(_REG_CONFIG)
        cfg = (cfg & ~_CFG_OPER_MODE_MASK) | ((mode & 0x03) << 4)
        self._write_u16_be(_REG_CONFIG, cfg)


    @staticmethod
    def _decode_channel(buf: bytes, offset: int) -> int:
        """Decode a single channel from a 4-byte (MSB+LSB register) slice.

        Each channel occupies two consecutive 16-bit big-endian registers:
          MSB register: exponent[15:12] | mantissa_hi[11:0]
          LSB register: mantissa_lo[15:8] | counter[7:4] | crc[3:0]

        ADC code = mantissa_20bit << exponent
        """
        msb_hi = buf[offset]
        msb_lo = buf[offset + 1]
        lsb_hi = buf[offset + 2]
        # lsb_lo contains counter + CRC — not needed for the value

        exp      = (msb_hi >> 4) & 0x0F
        mantissa = ((msb_hi & 0x0F) << 16) | (msb_lo << 8) | lsb_hi
        return mantissa << exp


    @staticmethod
    def _reference_to_gains(black: tuple[int, int, int, int], white: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
        ref_r = max(white[0] - black[0], 1)
        ref_g = max(white[1] - black[1], 1)
        ref_b = max(white[2] - black[2], 1)
        ref_w = max(white[3] - black[3], 1) if white[3] > 0 else _WHITE_CAL_SCALE
        gain_scale = _WHITE_CAL_SCALE * _WHITE_CAL_SCALE
        return (
            (gain_scale + (ref_r // 2)) / ref_r,
            (gain_scale + (ref_g // 2)) / ref_g,
            (gain_scale + (ref_b // 2)) / ref_b,
            (gain_scale + (ref_w // 2)) / ref_w,
        )

__app_export__ = HexDriveApp
