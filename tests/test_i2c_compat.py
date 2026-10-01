import builtins
import importlib.util
import sys
import types
import unittest
from pathlib import Path

# pylint: disable=protected-access

SOURCE = Path(__file__).resolve().parents[1] / "hexdrive2.py"
RUNTIME_MODULES = (
    "app",
    "events",
    "i2c_mgr",
    "machine",
    "micropython",
    "ota",
    "system",
    "system.eventbus",
    "system.hexpansion",
    "system.hexpansion.app",
    "system.hexpansion.config",
    "system.hexpansion.events",
    "system.hexpansion.util",
    "system.scheduler",
    "system.scheduler.events",
    "tildagon",
)


class Dummy:
    def __init__(self, *args, **kwargs):
        pass


class DummyApp:
    pass


class DummyEvent:
    pass


def _module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    return module


def _runtime_stubs(with_manager):
    stubs = {
        "app": _module("app", App=DummyApp),
        "events": _module("events", Event=DummyEvent),
        "machine": _module("machine", PWM=Dummy, Pin=Dummy, I2C=Dummy),
        "micropython": _module(
            "micropython",
            const=lambda value: value,
            native=lambda function: function,
            viper=lambda function: function,
        ),
        "ota": _module("ota"),
        "system": _module("system"),
        "system.eventbus": _module("system.eventbus", eventbus=Dummy()),
        "system.hexpansion": _module("system.hexpansion"),
        "system.hexpansion.app": _module("system.hexpansion.app"),
        "system.hexpansion.config": _module(
            "system.hexpansion.config", HexpansionConfig=Dummy
        ),
        "system.hexpansion.events": _module(
            "system.hexpansion.events",
            HexpansionInsertionEvent=DummyEvent,
            HexpansionRemovalEvent=DummyEvent,
        ),
        "system.hexpansion.util": _module(
            "system.hexpansion.util", get_slots_by_vid_pid=lambda *args: []
        ),
        "system.scheduler": _module("system.scheduler"),
        "system.scheduler.events": _module(
            "system.scheduler.events", RequestStopAppEvent=DummyEvent
        ),
        "tildagon": _module("tildagon", Pin=Dummy),
    }
    if with_manager:
        stubs["i2c_mgr"] = _module(
            "i2c_mgr",
            READ=1,
            WRITE=2,
            CHECK=3,
            MIN_PERIOD_MS=10,
            add_job=lambda *args, **kwargs: Dummy(),
        )
    return stubs


def load_hexdrive2(with_manager):
    saved_modules = {name: sys.modules.get(name) for name in RUNTIME_MODULES}
    module_name = f"hexdrive2_test_{'manager' if with_manager else 'legacy'}"
    original_import = builtins.__import__

    try:
        for name in RUNTIME_MODULES:
            sys.modules.pop(name, None)
        sys.modules.update(_runtime_stubs(with_manager))

        if not with_manager:
            def legacy_import(name, *args, **kwargs):
                if name == "i2c_mgr":
                    raise ImportError("legacy BadgeOS")
                return original_import(name, *args, **kwargs)

            builtins.__import__ = legacy_import

        spec = importlib.util.spec_from_file_location(module_name, SOURCE)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        builtins.__import__ = original_import
        for name in RUNTIME_MODULES:
            sys.modules.pop(name, None)
            if saved_modules[name] is not None:
                sys.modules[name] = saved_modules[name]
        sys.modules.pop(module_name, None)


class FakeI2C:
    def __init__(self, responses):
        self.responses = list(responses)
        self.reads = []
        self.writes = []

    def readfrom_mem_into(self, address, register, destination):
        self.reads.append((address, register, len(destination)))
        destination[:] = self.responses.pop(0)

    def writeto_mem(self, address, register, data):
        self.writes.append((address, register, bytes(data)))


class FakeJob:
    def __init__(self, sequence, data):
        self.sequence = sequence
        self.data = data

    def read_into(self, destination):
        destination[:] = self.data
        return self.sequence


def encode_channel(value, counter=0):
    return bytes(
        (
            (value >> 16) & 0x0F,
            (value >> 8) & 0xFF,
            value & 0xFF,
            (counter & 0x0F) << 4,
        )
    )


class LegacyI2CTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hexdrive2 = load_hexdrive2(with_manager=False)

    def test_import_uses_empty_job_steps(self):
        self.assertIsNone(self.hexdrive2.i2c_mgr)
        self.assertEqual(self.hexdrive2.VL53L0X._JOB_STEPS, ())
        self.assertEqual(self.hexdrive2.OPT4060._JOB_STEPS, ())

    def _motor_app(self, operations):
        class MotorPin:
            def init(self, **kwargs):
                operations.append("pin_init")

            def value(self, value):
                operations.append(("pin", value))

        class MotorPWM:
            def __init__(self, channel):
                self.channel = channel
                self.duty = 0
                self.fail = False

            def duty_u16(self, value=None):
                if value is None:
                    return self.duty
                if self.fail:
                    raise OSError("write failed")
                self.duty = value
                operations.append(("duty", self.channel, value))

            def deinit(self):
                operations.append(("deinit", self.channel))

            def init(self, **kwargs):
                operations.append(("init", self.channel))

        self.hexdrive2.Pin.OUT = 1
        app = object.__new__(self.hexdrive2.HexDriveApp)
        app.config = types.SimpleNamespace(port=4, pin=[MotorPin() for _ in range(4)])
        app._hexdrive_type = types.SimpleNamespace(motors=2)
        app._logging = False
        app._motor_output = [0, 0]
        app._motor_requests = [0, 0]
        app._pwm_requests = [0, 0, 0, 0]
        app.pwm_outputs = [MotorPWM(channel) for channel in range(4)]
        app._pwm_pin_index = [3, -1, 1, -1]
        app._freq = [1000] * 4
        app._outputs_energised = False
        app._time_since_last_update = 100
        return app

    def test_motor_reversal_disables_old_channel_before_enabling_new(self):
        operations = []
        app = self._motor_app(operations)
        self.assertTrue(app.set_motors([100, 0]))
        operations.clear()
        self.assertTrue(app.set_motors([-200, 0]))
        self.assertEqual(operations, [
            ("deinit", 0), "pin_init", ("pin", 0), ("init", 1), ("duty", 1, 200),
        ])
        operations.clear()
        app._time_since_last_update = 100
        self.assertTrue(app.set_motors([-200, 0]))
        self.assertEqual(operations, [])
        self.assertEqual(app._time_since_last_update, 0)
        self.assertTrue(app.set_motors([0, 0]))
        self.assertEqual(app.pwm_outputs[1].duty, 0)

    def test_failed_motor_write_does_not_commit_requested_output(self):
        app = self._motor_app([])
        app.pwm_outputs[0].fail = True
        self.assertFalse(app.set_motors([100, 0]))
        self.assertEqual(app._motor_output, [0, 0])
        self.assertEqual(app._time_since_last_update, 100)

    def test_keep_alive_timeout_turns_off_all_active_outputs(self):
        app = self._motor_app([])
        app._pwm_setup = True
        app._keep_alive_period = 50
        self.assertTrue(app.set_motors([100, 200]))
        app._update_keep_alive(50)
        self.assertEqual(app.pwm_outputs[0].duty, 100)
        app._update_keep_alive(1)
        self.assertEqual(app.pwm_outputs[0].duty, 0)
        self.assertEqual(app.pwm_outputs[2].duty, 0)
        self.assertFalse(app._outputs_energised)
        self.assertEqual(app._time_since_last_update, 0)

    def test_split_white_scaling_matches_full_integer_product(self):
        for value in (-1000, -1, 0, 1, 8191, 8192, 16383, 16384, 1048575):
            for gain in (0, 5, 80, 16383, 16384, 67109, 268435):
                self.assertEqual(
                    self.hexdrive2._scaled_white_value(value, gain),
                    (value * gain + 8192) // 16384,
                )

    def test_colour_lookup_buffer_matches_tuple_api(self):
        lookup = self.hexdrive2.ColourLookup
        samples = (
            ((0, 0, 0, 0), 0, "Black", 1200, 0),
            ((10, 10, 10, 100), 0, "Black", 0, 0),
            ((40, 40, 40, 100), 2, "Gray", 0, 0),
            ((100, 100, 100, 100), 1, "White", 0, 0),
            ((200, 20, 10, 100), 3, "Red", 30, 95),
            ((200, 80, 0, 100), 4, "Orange", 240, 100),
            ((200, 180, 0, 100), 5, "Yellow", 540, 100),
            ((20, 180, 20, 100), 6, "Green", 1200, 88),
            ((20, 180, 180, 100), 7, "Cyan", 1800, 88),
            ((20, 20, 200, 100), 8, "Blue", 2400, 90),
            ((180, 20, 180, 100), 9, "Magenta", 3000, 88),
        )
        result = [0, 0, 0]
        for sample, expected_id, expected_name, expected_hue, expected_saturation in samples:
            self.assertEqual(lookup.rgbw_to_str(sample), (expected_name, expected_hue, expected_saturation))
            actual_name = lookup.rgbw_to_str(sample, result)
            self.assertEqual(actual_name, expected_name)
            self.assertEqual(result, [expected_id, expected_hue, expected_saturation])

    def test_colour_name_reuses_white_balance_buffer(self):
        sensor = self.hexdrive2.OPT4060(Dummy(), 1)
        colour = (1000, 2000, 3000, 4000)
        expected = self.hexdrive2.ColourLookup.rgbw_to_str(
            sensor.apply_white_reference(colour)
        )
        work_buffer = sensor._colour_work_buffer
        calibrated = sensor.apply_white_reference(colour, work_buffer)
        self.assertIs(calibrated, work_buffer)
        self.assertEqual(calibrated, list(sensor.apply_white_reference(colour)))
        sensor._last_colour_buffer[:] = colour
        sensor._has_colour = True

        self.assertEqual(sensor.colour_name, expected[0])
        self.assertEqual(sensor.colour_hue, expected[1])
        self.assertEqual(sensor.colour_saturation, expected[2])
        self.assertIs(sensor._colour_work_buffer, work_buffer)

    def test_calibration_references_snapshot_mutable_inputs(self):
        sensor = self.hexdrive2.OPT4060(Dummy(), 1)
        black = [10, 20, 30, 40]
        white = [1010, 2020, 3030, 4040]

        sensor.black_reference = black
        sensor.white_reference = white
        black[0] = 0
        white[0] = 0

        self.assertEqual(sensor.black_reference, (10, 20, 30, 40))
        self.assertEqual(sensor.white_reference, (1010, 2020, 3030, 4040))

    def test_range_read_waits_for_ready_then_reads_and_increments(self):
        i2c = FakeI2C([b"\x00"])
        sensor = self.hexdrive2.VL53L0X(i2c, 1)
        sensor._ready = True

        self.assertIsNone(sensor.read())
        self.assertEqual(sensor.sequence, 0)
        self.assertEqual(i2c.writes, [])

        i2c.responses.extend([b"\x07", b"\x01\x23"])
        self.assertEqual(sensor.read(), 0x123)
        self.assertEqual(sensor.sequence, 1)
        self.assertEqual(
            i2c.writes[-1][1:],
            (self.hexdrive2._SYSTEM_INTERRUPT_CLEAR, b"\x01"),
        )

    def test_colour_read_waits_for_ready_then_reads_and_increments(self):
        i2c = FakeI2C([b"\x00\x00"])
        sensor = self.hexdrive2.OPT4060(i2c, 1)
        sensor._ready = True

        self.assertIsNone(sensor.read())
        self.assertEqual(sensor.sequence, 0)

        values = (0x12345, 0x23456, 0x34567, 0x45678)
        i2c.responses.extend(
            [
                bytes((0, self.hexdrive2._RES_CTRL_CONV_READY_MASK)),
                b"".join(
                    encode_channel(value, index)
                    for index, value in enumerate(values)
                ),
            ]
        )

        measurement = sensor.read()
        self.assertEqual(measurement, values)
        self.assertIs(sensor.colour, measurement)
        self.assertEqual(sensor.sequence, 1)


class ManagerI2CTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hexdrive2 = load_hexdrive2(with_manager=True)

    def test_import_builds_manager_job_steps(self):
        self.assertIsNotNone(self.hexdrive2.i2c_mgr)
        self.assertEqual(len(self.hexdrive2.VL53L0X._JOB_STEPS), 4)
        self.assertEqual(len(self.hexdrive2.OPT4060._JOB_STEPS), 3)

    def test_colour_job_callback_updates_buffers_without_tuple(self):
        values = (0x12345, 0x23456, 0x34567, 0x45678)
        sensor = self.hexdrive2.OPT4060(Dummy(), 1)
        sensor._ready = True
        sensor._job = FakeJob(
            31,
            bytes((0, self.hexdrive2._RES_CTRL_CONV_READY_MASK))
            + b"".join(encode_channel(value) for value in values),
        )

        sensor._on_job_data(None)
        self.assertEqual(sensor.sequence, 31)
        self.assertIsNone(sensor._last_colour)

        result = [0, 0, 0, 0]
        self.assertIs(sensor.colour_into(result), result)
        self.assertEqual(result, list(values))
        self.assertIsNone(sensor._last_colour)
        self.assertEqual(sensor.colour, values)
        self.assertIs(sensor.colour, sensor._last_colour)

    def test_cached_reads_retain_manager_sequence_and_suppress_duplicates(self):
        range_sensor = self.hexdrive2.VL53L0X(Dummy(), 1)
        range_sensor._ready = True
        range_sensor._job = FakeJob(17, b"\x07\x01\x23")

        self.assertEqual(range_sensor.read(), 0x123)
        self.assertEqual(range_sensor.sequence, 17)
        self.assertIsNone(range_sensor.read())
        self.assertEqual(range_sensor.sequence, 17)

        values = (0x12345, 0x23456, 0x34567, 0x45678)
        colour_sensor = self.hexdrive2.OPT4060(Dummy(), 1)
        colour_sensor._ready = True
        colour_sensor._job = FakeJob(
            29,
            bytes((0, self.hexdrive2._RES_CTRL_CONV_READY_MASK))
            + b"".join(encode_channel(value) for value in values),
        )

        measurement = colour_sensor.read()
        self.assertEqual(measurement, values)
        self.assertIs(colour_sensor.colour, measurement)
        self.assertEqual(colour_sensor.sequence, 29)
        self.assertIsNone(colour_sensor.read())
        self.assertEqual(colour_sensor.sequence, 29)


if __name__ == "__main__":
    unittest.main()
