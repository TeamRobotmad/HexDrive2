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

        self.assertEqual(sensor.read(), values)
        self.assertEqual(sensor.sequence, 1)


class ManagerI2CTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hexdrive2 = load_hexdrive2(with_manager=True)

    def test_import_builds_manager_job_steps(self):
        self.assertIsNotNone(self.hexdrive2.i2c_mgr)
        self.assertEqual(len(self.hexdrive2.VL53L0X._JOB_STEPS), 4)
        self.assertEqual(len(self.hexdrive2.OPT4060._JOB_STEPS), 3)

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

        self.assertEqual(colour_sensor.read(), values)
        self.assertEqual(colour_sensor.sequence, 29)
        self.assertIsNone(colour_sensor.read())
        self.assertEqual(colour_sensor.sequence, 29)


if __name__ == "__main__":
    unittest.main()
