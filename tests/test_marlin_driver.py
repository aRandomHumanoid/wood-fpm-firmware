import threading
import queue

import pytest

import src.drivers.marlin as marlin_module
from src.drivers.marlin import MarlinDriver, MarlinError, parse_probe_position


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def advance(self, delta):
        self.now += delta


class ScriptedSerial:
    def __init__(self, clock, scripted_reads):
        self._clock = clock
        self._reads = list(scripted_reads)

    def readline(self):
        if not self._reads:
            self._clock.advance(0.25)
            return b""
        delay, line = self._reads.pop(0)
        self._clock.advance(delay)
        return line


def test_read_until_ok_treats_busy_lines_as_activity(monkeypatch):
    clock = FakeClock()
    driver = MarlinDriver.__new__(MarlinDriver)
    driver._serial = ScriptedSerial(
        clock,
        [
            (0.6, b"echo:busy: processing\n"),
            (0.6, b"echo:busy: processing\n"),
            (0.6, b"ok\n"),
        ],
    )
    driver._transcript_hook = None

    monkeypatch.setattr(marlin_module.time, "monotonic", clock.monotonic)

    lines = driver._read_until_ok(overall_timeout_s=2.0)

    assert lines == [
        "echo:busy: processing",
        "echo:busy: processing",
        "ok",
    ]


def test_read_until_ok_busy_lines_still_respect_total_timeout(monkeypatch):
    clock = FakeClock()
    driver = MarlinDriver.__new__(MarlinDriver)
    driver._serial = ScriptedSerial(
        clock,
        [
            (0.6, b"echo:busy: processing\n"),
            (0.6, b"echo:busy: processing\n"),
            (0.6, b"echo:busy: processing\n"),
        ],
    )
    driver._transcript_hook = None

    monkeypatch.setattr(marlin_module.time, "monotonic", clock.monotonic)

    with pytest.raises(MarlinError, match="timeout"):
        driver._read_until_ok(overall_timeout_s=1.5)


def test_send_passes_timeout_to_io_loop():
    driver = MarlinDriver.__new__(MarlinDriver)
    driver._cmd_q = queue.Queue()
    driver._running = True
    driver._transcript_hook = None

    class FakeSerial:
        def __init__(self):
            self.writes = []

        def write(self, data):
            self.writes.append(data)

        def flush(self):
            pass

    driver._serial = FakeSerial()

    captured = {}

    def fake_read_until_ok(timeout_s=30.0):
        captured["timeout_s"] = timeout_s
        driver._running = False
        return ["ok"]

    driver._read_until_ok = fake_read_until_ok
    driver._io_thread = threading.Thread(target=driver._io_loop, daemon=True)
    driver._io_thread.start()

    try:
        result = driver.send("M115", timeout_s=123.0)
    finally:
        driver._running = False
        driver._io_thread.join(timeout=1.0)

    assert result == ["ok"]
    assert captured["timeout_s"] == 123.0
    assert driver._serial.writes == [b"M115\n"]


def test_parse_probe_position_reads_probe_hit_coordinates():
    lines = [
        "echo:endstops hit: X:2.900 Y:4.000 Z:0.000",
        "ok",
    ]

    parsed = parse_probe_position(lines)

    assert parsed == {"X": 2.9, "Y": 4.0, "Z": 0.0}