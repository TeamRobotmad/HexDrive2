"""HexDrive2 EEPROM app source shared between Team RobotMad apps."""

# This is the app to be installed from a HexDrive2 Hexpansion EEPROM.
# it is compiled and copied onto the EEPROM as app.mpy
# It is then run from the EEPROM by the BadgeOS.
import time
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
_ENABLE_PIN  = const(0)     # First LS pin used to enable the SMPSU
_COLOUR_INT_PIN = const(1)  # Second LS pin used to detect interrupts from the colour sensor to trigger readings without polling
_LED_PIN  = const(2)        # Third LS pin used to control an LED to illuminate the area under the colour sensor for better readings of reflected light from the surface below.
_RANGE_INT_PIN = const(3)    # Fourth LS pin used to detect interrupts from the distance sensor to trigger readings without polling
_RANGE_XSHUT_PIN = const(4)  # Fifth LS pin used to control the XSHUT pin of the distance sensor to allow it to be power cycled for reset or power saving
_ALT_RANGE_INT_PIN = const(4) # Some models of the VL52L0X sensor module have the interrupt and XSHUT pins swapped (alternative pin for hardware versions that have the distance sensor on a different LS pin)
_ALT_RANGE_XSHUT_PIN = const(3) # Some models of the VL52L0X sensor module have the interrupt and XSHUT pins swapped (alternative pin for hardware versions that have the distance sensor on a different LS pin)

# Hexpansion EEPROM constants
_ADDR_LEN = const(2)          # EEPROM I2C address length in bytes (1 or 2)
_ADDR = const(0x50)           # EEPROM I2C address (7-bit)


# EXTENDED Header constants
_EXTENDED_HEADER_ADDR = const(0x30)  # EEPROM address of the extended header
_EXTENDED_HEADER_SIZE = const(32)    # Size of the extended header in bytes
_EXTENDED_HEADER_MAGIC = b"HDR2"            # Magic bytes to identify the extended header
_EXTENDED_HEADER_VERSION = b"2026"          # Version of the extended header format

# EXTENDED header flags constants
_EXTENDED_HEADER_FLAG_INITIALISED = const(0x01)  # Flag indicating that the extended header has been initialised
_EXTENDED_HEADER_FLAG_DIST_PINS_SWAPPED = const(0x02)  # Flag indicating that the distance sensor pins are swapped

# Default values and limits:
_DEFAULT_PWM_FREQ = const(20000)           # 20kHz is a good default for motors as it is above the audible range for most people and works with most motors and ESCs
_DEFAULT_SERVO_FREQ = const(50)            # 50Hz = 20mS period
_DEFAULT_KEEP_ALIVE_PERIOD = const(1000)   # 1 second
_MAX_NUM_CHANNELS = const(4)               # Max number of PWM channels supported by any type of HexDrive (Hexpansion limitation, not BadgeBot limit)
_MAX_NUM_MOTORS = const(2)                 # Max number of motor channels supported by any type of HexDrive

# Servo Constants
_MAX_SERVO_FREQ = const(200)               # 200Hz = 5mS period (can work with some Servos but not all)
_SERVO_CENTRE    = const(1500)             # 1500us pulse width is the centre position for most RC servos (but some may be different, so we allow this to be trimmed)
_MAX_SERVO_RANGE = const(1400)             # 1400us either side of centre (VERY WIDE)
_SERVO_MAX_TRIM  = const(1000)             # 1000us either side of centre for trimming the centre position


class HexDriveType:
    """Represents a sub-type of HexDrive Hexpansion module."""
    __slots__ = ("pid", "name", "motors", "servos", "servo_pin_map")

    def __init__(self, pid_byte: int, motors: int = 0, servos: int = 0, name: str = "Uncommitted", servo_pins: tuple[int, int, int, int] = (-1, -1, -1, -1)):
        self.pid: int = pid_byte         # Product ID byte read from the EEPROM to identify the type of HexDrive
        self.name: str = name            # A friendly name for the type of HexDrive
        self.motors: int = motors        # Number of motor channels supported by this type of HexDrive (0, 1 or 2)
        self.servos: int = servos        # Number of servo channels supported by this type of HexDrive (0, 2 or 4)
        self.servo_pin_map: tuple[int, int, int, int] = servo_pins # Map the logical servo channels to the physical pin index according to hardware version

_HEXDRIVE_TYPES = (
    HexDriveType(0xC8, motors=2, servos=2, servo_pins=(3, 1, -1, -1)),  # uncommitted version can be used for anything
    HexDriveType(0xC9, servos=2, name="2 Servo", servo_pins=(3, 1, -1, -1)),
    HexDriveType(0xCA, motors=2, name="2 Motor"),
    HexDriveType(0xCE, motors=1, name="1 Motor"),
    HexDriveType(0xCF, motors=1, servos=1, name="1 Mot 1 Srvo", servo_pins=(1, -1, -1, -1)),
)


_DEFAULT_HEXDRIVE_TYPE = _HEXDRIVE_TYPES[0]  # default to the uncommitted version if we can't read the EEPROM for some reason


# --------------------------------------------------
# Extended Hexpansion Header class for reading and writing the extended header of the hexpansion EEPROM.
# This uses the fact that the standard Hexpansion header is 32 bytes long, and the extended header is
# stored in the spare bytes of the first sector of the EEPROM. As we know that our EEPROMS use 64-byte pages.
# --------------------------------------------------
class ExtendedHexpansionHeader:
    _header_format = "<4s4sI19s"
    _magic = _EXTENDED_HEADER_MAGIC

    def __init__(
        self,
        manifest_version: str = _EXTENDED_HEADER_VERSION.decode(),
        flags: int = 0,
        spare: str = "\x00" * 19
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
        checksum = 0x55
        for byte in b:
            checksum ^= byte
        return checksum

    def to_bytes(self, include_checksum=True):
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


def _handle_range_interrupt(epin):
    # Get range from sensor and emit event
    # TODO - how do we access the HexDriveApp instance to get the range sensor object? We need to store a reference to the HexDriveApp instance somewhere so we can access it here.
    # perhaps just call into a new method in the sensor class that can know how things are setup and hence what the interrupt was for - as may not always be a new range to be sent as an event?
    sensor = ???
    if sensor is None:
        print("D:VL53L0X interrupt received but sensor is not initialised")
        return
    range = sensor.get_range()  # Get the range from the sensor
    if range is not None:
        eventbus.emit(HexDriveApp.RangeEvent(range))
        print(f"D:VL53L0X interrupt received, range={range}mm")


class HexDriveApp(app.App):         # pylint: disable=no-member
    """ HexDrive Hexpansion App for BadgeBot."""
    VERSION = 1         # Increment this when making changes to the app that require the hexpansion EEPROM app to be re-flashed with the new code.

    class RangeEvent(Event):
        """Emitted when a new ToF distance measurement is obtained, providing the distance to target in mm."""
        def __init__(self, range: int):
            self.range = range

        def __str__(self):
            return f"Range: {self.range}mm"


    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()
        if config is None:
            raise TypeError("HexDriveApp requires a HexpansionConfig on initialisation")

        self.config: HexpansionConfig = config
        self._logging: bool = True

        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            if ver >= _MIN_BADGEOS_VERSION:
                pass
            else:
                raise TypeError("BadgeOS version is too old for HexDriveApp")
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:Ver check failed {e}!")

        # What flavour of HexDrive Hexpansion module do we have plugged in?
        _hexdrive_type = self._check_port_for_hexdrive(self.config.port)

        # report app starting and which port it is running on
        print(f"D:HexDrive2 Type:'{_hexdrive_type.name}' App V{self.VERSION} by RobotMad on port {self.config.port}")

        self._hexdrive_type: HexDriveType = _hexdrive_type
        self._servo_pin_map: tuple[int, int, int, int] = self._hexdrive_type.servo_pin_map
        self._keep_alive_period: int = _DEFAULT_KEEP_ALIVE_PERIOD
        self._power_state: bool = False
        self._pwm_setup: bool = False
        self._time_since_last_update: int = 0
        self._outputs_energised: bool = False
        self.PWMOutput: list[PWM | None] = [None] * _MAX_NUM_CHANNELS
        self._freq: list[int] = [0] * _MAX_NUM_CHANNELS
        self._motor_output: list[int] = [0] * self._hexdrive_type.motors
        self._i2c: I2C | None = None
        self._extended_header: ExtendedHexpansionHeader = self._read_extended_hexpansion_header()
        if self._extended_header.flags & _EXTENDED_HEADER_FLAG_INITIALISED:
            print(f"D:{self.config.port}:Extended Header Initialised, flags={self._extended_header.flags:08b}")
        else:
            print(f"D:{self.config.port}:Extended Header Not Initialised")
            self._extended_header.flags |= _EXTENDED_HEADER_FLAG_INITIALISED
            if self._write_extended_hexpansion_header(self._extended_header):
                print(f"D:{self.config.port}:Extended Header Written, flags={self._extended_header.flags:08b}")
        self._range_sensor: VL53L0X | None = None
        #self._colour_sensor: OPT4060 | None = None

        # LS Pins
        self._ALT_RANGE_pins: bool = False
        self._power_control: ePin = self.config.ls_pin[_ENABLE_PIN]
        self._led_control:   ePin = self.config.ls_pin[_LED_PIN]
        self._colour_int:    ePin = self.config.ls_pin[_COLOUR_INT_PIN]
        self._range_xshut:    ePin = self.config.ls_pin[_ALT_RANGE_XSHUT_PIN if self._extended_header.flags & _EXTENDED_HEADER_FLAG_DIST_PINS_SWAPPED else _RANGE_XSHUT_PIN]
        self._range_int:      ePin = self.config.ls_pin[_ALT_RANGE_INT_PIN if self._extended_header.flags & _EXTENDED_HEADER_FLAG_DIST_PINS_SWAPPED else _RANGE_INT_PIN]

        # Servo related
        self._servo_pin_map: tuple[int, int, int, int] = self._hexdrive_type.servo_pin_map
        self._servo_centre: list[int] = [_SERVO_CENTRE] * self._hexdrive_type.servos

        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)

        if not self.initialise():
            print("HexDriveApp init failed")


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
            self._led_control.init(mode=Pin.OUT)
            self._range_xshut.init(mode=Pin.OUT)
            self._colour_int.init(mode=Pin.IN)
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
            self._motor_output[channel] = 0  # initialise motor output state to 0 (stopped)
            self._freq[channel<<1]     = _DEFAULT_PWM_FREQ
            self._freq[(channel<<1) + 1] = _DEFAULT_PWM_FREQ
        for channel in range(self._hexdrive_type.servos):
            physical_channel = self._servo_pin_map[channel]
            if physical_channel >= 0 and self._freq[physical_channel] == 0:
                # give priority to motor frequency if there is a conflict on the same physical channel, otherwise set to default servo frequency
                print(f"D:{self.config.port}:Servo {channel} on Physical channel {physical_channel}")
                self._freq[physical_channel] = _DEFAULT_SERVO_FREQ
        self._pwm_setup = True

        return True


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


    # Special function called by the BadgeOS to allow the app to clean up resources before it is removed from memory.
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
        if self._range_sensor is not None:
            try:
                self._range_sensor.stop()
                # remove irq callback TODO is this right?
                self._range_int.irq(handler=None)
            except Exception:       # pylint: disable=broad-except
                pass
            self._range_sensor = None


    def background_update(self, delta: int):
        """ This is called from the main loop of the BadgeOS to allow the app to do any background processing it needs to do. """
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


    def set_range_xshut(self, state: bool) -> bool:
        """ Set the state of the distance sensor XSHUT pin to power cycle it for reset or power saving. Returns success or failure. """
        try:
            self._range_xshut.init(mode=Pin.OUT)
            self._range_xshut.value(state)
            if self._logging:
                print(f"D:{self.config.port}:Distance Sensor XSHUT={'On' if state else 'Off'}")
            return True
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:Distance Sensor XSHUT control failed {e}")
            return False


    def set_sensor_led(self, state: bool) -> bool:
        """ Set the state of the colour sensor LED pin to turn on or off the LED to illuminate the area under the colour sensor. Returns success or failure. """
        try:
            self._led_control.init(mode=Pin.OUT)
            self._led_control.value(state)
            if self._logging:
                print(f"D:{self.config.port}:Colour Sensor LED={'On' if state else 'Off'}")
            return True
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:Colour Sensor LED control failed {e}")
            return False


    def range_enable(self, enable: bool) -> bool:
        """ Enable or disable the distance sensor. Returns success or failure. """
        if self._range_sensor is None and enable:
            try:
                if self._i2c is None:
                    self._i2c = I2C(self.config.port)
                self._range_sensor = VL53L0X(self._i2c)
                if self._range_sensor is not None and self._logging:
                    print(f"D:{self.config.port}:Distance Sensor Initialised")
            except Exception as e:      # pylint: disable=broad-except
                print(f"D:{self.config.port}:Distance Sensor Initialisation failed {e}")
                return False
        if self._range_sensor is not None:
            try:
                if enable:
                    self.set_range_xshut(True)
                    # configure interrupt pin to trigger on falling edge when a new range measurement is ready
                    self._range_int.init(mode=Pin.IN)
                    self._range_int.irq(trigger=Pin.IRQ_FALLING, handler=_handle_range_interrupt())
                    self._range_sensor.start()
                    if self._logging:
                        print(f"D:{self.config.port}:Distance Sensor Started")
                else:
                    self._range_sensor.stop()
                    self.set_range_xshut(False)
                    if self._logging:
                        print(f"D:{self.config.port}:Distance Sensor Stopped")
                return True
            except Exception as e:      # pylint: disable=broad-except
                print(f"D:{self.config.port}:Distance Sensor control failed {e}")
                return False
        elif not enable:
            # sensor is already disabled so nothing to do
            return True
        return False


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
        if channel is not None:
            _max_channel = self._hexdrive_type.servos if servo else self._hexdrive_type.motors
            if channel < 0 or channel >= _max_channel:
                return False
            # map from logical channel to physical channel(s) for servos and motors
            if servo:
                self._freq[channel] = freq
                physical_channel = self._servo_pin_map[channel]
            else:
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


    def _pwm_log_string(self, channel: int | None) -> str:
        """ Helper method to generate a log string for a PWM output change. """
        return f"D:{self.config.port}:PWM[{channel if channel is not None else 'All'}]:"


    def set_servoposition(self, channel: int | None = None, position: int | None = None) -> bool:
        """ Set the position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            The pulse width for a specific servo output is position + the centre offset (in us)
            Based on standard RC servos with centre at 1500us and range of 1000-2000us.
            The position is a signed value from -1000 to 1000 which is scaled to 500-2500us.
            This is a very wide range and may not be suitable for all servos, some will
            only be happy with 1000-2000us (i.e. position in the range -500 to 500). """
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


# --------------------------------------------------
# Private methods for internal use only.
# --------------------------------------------------




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




"""
VL53L0X Time-of-Flight distance sensor driver.

Default I2C address: 0x29
Measurement: distance in mm (up to ~1200 mm in default mode).

This driver uses single-shot ranging.

Datasheet: https://www.st.com/resource/en/datasheet/vl53l0x.pdf
"""

_I2C_ADDRESS = const(0x29)
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

class VL53L0X():
    """VL53L0X Time-of-Flight distance sensor driver."""
    I2C_ADDR = _I2C_ADDRESS
    READ_INTERVAL_MS = 100

    def __init__(self, i2c: I2C, logging: bool = False):
        self._i2c = i2c
        self._ready = False
        self._i2c_addr = self.I2C_ADDR
        self._stop_variable = 0             # used to store the stop variable value for the VL53L0X sensor
        self._logging = logging


    @property
    def logging(self) -> bool:
        return self._logging


    @logging.setter
    def logging(self, value: bool):
        self._logging = value


    def _read_u8(self, reg: int) -> int:
        try:
            return self._i2c.readfrom_mem(self._i2c_addr, reg, 1)[0]
        except Exception as e:      # pylint: disable=broad-exception-caught
            self._ready = False
            if self._logging:
                print(f"D:VL53L0X read error: {e}")
            return 0


    def _write_u8(self, reg: int, value: int) -> bool:
        try:
            self._i2c.writeto_mem(self._i2c_addr, reg, bytes([value & 0xFF]))
            return True
        except Exception as e:      # pylint: disable=broad-exception-caught
            self._ready = False
            if self._logging:
                print(f"D:VL53L0X write error: {e}")
            return False


    def _read_u16_be(self, reg: int) -> int:
        try:
            d = self._i2c.readfrom_mem(self._i2c_addr, reg, 2)
            return (d[0] << 8) | d[1]
        except Exception as e:      # pylint: disable=broad-exception-caught
            self._ready = False
            if self._logging:
                print(f"D:VL53L0X read error: {e}")
            return 0


    def _init(self) -> bool:
        try:
            who = self._read_u8(_WHO_AM_I_REG)
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"D:Cannot read VL53L0X ID: {e}")
            return False
        if who != _WHO_AM_I_EXPECT:
            if self._logging:
                print(f"D:VL53L0X unexpected ID 0x{who:02X} (expected 0x{_WHO_AM_I_EXPECT:02X})")
            return False

        # The VL53L0X needs a substantial startup sequence before single-shot
        # ranging becomes trustworthy;
        if not self._write_u8(
            _VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
            self._read_u8(_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01):
            return False
        if not self._write_u8(0x88, 0x00):
            return False
        if not self._open_stop_variable_window():
            return False
        self._stop_variable = self._read_u8(_STOP_VARIABLE_REG)
        if not self._close_stop_variable_window():
            return False

        if not self._write_u8(
            _MSRC_CONFIG_CONTROL,
            self._read_u8(_MSRC_CONFIG_CONTROL) | 0x12):
            return False
        if not self._set_signal_rate_limit(0.25):
            return False
        if not self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xFF):
            return False

        spad_info = self._get_spad_info()
        if spad_info is None:
            return False

        spad_count, spad_type_is_aperture = spad_info
        ref_spad_map = bytearray(self._i2c.readfrom_mem(self._i2c_addr, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6))
        if not self._write_u8(0xFF, 0x01):
            return False
        if not self._write_u8(_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00):
            return False
        if not self._write_u8(_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C):
            return False
        if not self._write_u8(0xFF, 0x00):
            return False
        if not self._write_u8(_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4):
            return False

        first_spad_to_enable = 12 if spad_type_is_aperture else 0
        spads_enabled = 0
        for index in range(48):
            if index < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[index // 8] &= ~(1 << (index % 8))
                continue
            if (ref_spad_map[index // 8] >> (index % 8)) & 0x01:
                spads_enabled += 1
        if not self._i2c.writeto_mem(self._i2c_addr, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, bytes(ref_spad_map)):
            return False

        for reg, value in _DEFAULT_TUNING_SETTINGS:
            if not self._write_u8(reg, value):
                return False

        if not self._write_u8(_SYSTEM_INTERRUPT_CONFIG, 0x04):
            return False
        if not self._write_u8(
            _GPIO_HV_MUX_ACTIVE_HIGH,
            self._read_u8(_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10,
        ):
            return False
        if not self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01):
            return False

        if not self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8):
            return False
        if not self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x01):
            return False
        if not self._perform_single_ref_calibration(0x40):
            return False
        if not self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x02):
            return False
        if not self._perform_single_ref_calibration(0x00):
            return False
        if not self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8):
            return False
        self._ready = True
        return True


    def start(self) -> bool:
        if not self._ready:
            if not self._init():
                return False
        if not self._prepare_single_shot():
            return False
        if not self._write_u8(_SYSRANGE_START, 0x01):
            return False
        return True


    def stop(self) -> bool:
        if not self._write_u8(_SYSRANGE_START, 0x00):
            return False
        return True


    def get_range(self) -> int:
        """ Get a single range measurement in mm. Returns distance in mm, or 0 on error. """
        if not self._ready:
            # Sensor not configured/available
            return 0

        # TODO - do we need to check both _SYSRANGE_START and _RESULT_INTERRUPT_STATUS to see if the sensor is ready for a new measurement? or can we be more efficient by just checking one of them?
        if  self._read_u8(_SYSRANGE_START) & 0x01:
            # Sensor is still performing a measurement, so we will return a timeout error.
            return 0

        # Check that the sensor is ready to give us back a range measurement. If not, we will return a timeout error.
        if (self._read_u8(_RESULT_INTERRUPT_STATUS) & _INTERRUPT_READY_MASK) == 0:
            # Sensor does not have a range measurement ready yet, so we will return a timeout error.
            return 0

        # The range value lives 10 bytes into the RESULT_RANGE_STATUS block in ST's register map; this offset matches the reference driver.
        dist_mm = self._read_u16_be(_RESULT_RANGE_STATUS + 10)
        if self._logging:
            print(f"D:VL53L0X measured {dist_mm} mm")

        # Clear the interrupt so that the sensor is ready for the next measurement.
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        return dist_mm


    def _open_stop_variable_window(self) -> bool:
        if not self._write_u8(0x80, 0x01):
            return False
        if not self._write_u8(0xFF, 0x01):
            return False
        if not self._write_u8(0x00, 0x00):
            return False
        return True


    def _close_stop_variable_window(self) -> bool:
        if not self._write_u8(0x00, 0x01):
            return False
        if not self._write_u8(0xFF, 0x00):
            return False
        if not self._write_u8(0x80, 0x00):
            return False
        return True


    def _prepare_single_shot(self) -> bool:
        if not self._open_stop_variable_window():
            return False
        if not self._write_u8(_STOP_VARIABLE_REG, self._stop_variable):
            return False
        if not self._close_stop_variable_window():
            return False
        return True


    def _wait_for_interrupt_ready(self) -> bool:
        #TODO use the GPIO interrupt pin to trigger a callback when the measurement is ready isntead of waiting here...
        deadline = time.ticks_add(time.ticks_ms(), _RANGE_TIMEOUT_MS)
        while (self._read_u8(_RESULT_INTERRUPT_STATUS) & _INTERRUPT_READY_MASK) == 0:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return False
            time.sleep_ms(1)
        return True


    def _perform_single_ref_calibration(self, vhv_init_byte: int) -> bool:
        if not self._write_u8(_SYSRANGE_START, 0x01 | vhv_init_byte):
            return False
        if not self._wait_for_interrupt_ready():
            return False
        if not self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01):
            return False
        if not self._write_u8(_SYSRANGE_START, 0x00):
            return False
        return True


    def _set_signal_rate_limit(self, limit_mcps: float) -> bool:
        int_limit = int(limit_mcps * (1 << 7))
        if not self._i2c.writeto_mem(self._i2c_addr, _FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, bytes([(int_limit >> 8) & 0xFF, int_limit & 0xFF])):
            return False
        return True


    def _get_spad_info(self) -> tuple[int, bool] | None:
        if not self._open_stop_variable_window():
            return None
        if not self._write_u8(0xFF, 0x06):
            return None
        if not self._write_u8(_SPAD_POLL_REG, self._read_u8(_SPAD_POLL_REG) | 0x04):
            return None
        if not self._write_u8(0xFF, 0x07):
            return None
        if not self._write_u8(0x81, 0x01):
            return None
        if not self._write_u8(0x80, 0x01):
            return None
        if not self._write_u8(0x94, 0x6B):
            return None
        if not self._write_u8(_SPAD_POLL_REG, 0x00):
            return None


        # TODO - try to avoid use of time.sleep_ms() in this driver as it blocks the main loop and prevents other tasks from running
        deadline = time.ticks_add(time.ticks_ms(), _RANGE_TIMEOUT_MS)
        while self._read_u8(_SPAD_POLL_REG) == 0x00:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return None
            time.sleep_ms(1)

        if not self._write_u8(_SPAD_POLL_REG, 0x01):
            return None
        spad_info = self._read_u8(_SPAD_INFO_REG)

        if not self._write_u8(0x81, 0x00):
            return None
        if not self._write_u8(0xFF, 0x06):
            return None
        if not self._write_u8(_SPAD_POLL_REG, self._read_u8(_SPAD_POLL_REG) & ~0x04):
            return None
        if not self._write_u8(0xFF, 0x01):
            return None
        if not self._close_stop_variable_window():
            return None

        return spad_info & 0x7F, ((spad_info >> 7) & 0x01) == 1



__app_export__ = HexDriveApp
