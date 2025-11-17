#!/usr/bin/env python3
# pylint: disable=import-error, too-few-public-methods
"""mocking functions"""

import sys
import threading
import time as _time

import pytest
import src.main
from gpiozero import Device, DigitalInputDevice
from gpiozero.pins.mock import MockFactory


class FakeSerial:
    """ for mocking serial.Serial """
    registry = {}

    def __init__(self, dev, baudrate=115200, timeout=0.1, *_, **__):
        self.dev = dev
        self.baudrate = baudrate
        self.timeout = timeout
        self._rx = bytearray()
        self._tx = bytearray()
        self._closed = False
        self._cv = threading.Condition()
        FakeSerial.registry[dev] = self

    def inject_rx(self, data: bytes):
        """ inject data to be received """
        with self._cv:
            self._rx.extend(data)
            self._cv.notify_all()

    @property
    def tx_data(self) -> bytes:
        """ get transmitted data """
        return bytes(self._tx)

    def write(self, data: bytes) -> int:
        """ wrapper pyserial API """
        self._tx.extend(data)
        return len(data)

    def read(self, n: int = 1) -> bytes:
        """ wrapper pyserial API """
        end = _time.monotonic() + (self.timeout or 0)
        with self._cv:
            while not self._rx and not self._closed:
                remaining = end - _time.monotonic()
                if remaining <= 0:
                    return b""
                self._cv.wait(remaining)
            if self._closed:
                return b""
            out = self._rx[:n]
            del self._rx[:n]
            return bytes(out)

    def close(self):
        """ wrapper pyserial API """
        with self._cv:
            self._closed = True
            self._cv.notify_all()


@pytest.fixture(autouse=True)
def _patch_pyserial(monkeypatch):
    """ change serial.Serial to FakeSerial """
    monkeypatch.setattr("serial.Serial", FakeSerial)
    monkeypatch.setattr("src.main.Serial", FakeSerial, raising=False)
    yield


@pytest.fixture(autouse=True)
def use_gpiozero_mock_factory():
    """set to use the mocking function instead of real GPIO"""
    Device.pin_factory = MockFactory()
    yield


@pytest.fixture(autouse=True)
def fast_sleep(monkeypatch):
    """speed up sleep calls inside both global time and src.main.time"""
    real_sleep = _time.sleep

    def _scaled_sleep(seconds: float):
        # 1.0s -> 0.002s, 0.5s -> 0.001s
        real_sleep(max(0.0005, seconds * 0.002))

    monkeypatch.setattr("time.sleep", _scaled_sleep)
    try:
        monkeypatch.setattr("src.main.time.sleep", _scaled_sleep, raising=False)
    except ImportError:
        pass

    return _scaled_sleep


@pytest.fixture(autouse=True)
def _fast_thread_switch():
    try:
        sys.setswitchinterval(1e-5)  # 5ms -> 10us
    except (AttributeError, ValueError, RuntimeError):
        pass
    yield


@pytest.fixture(autouse=True)
def _patch_button_no_hold(monkeypatch):

    class _PatchedButton(DigitalInputDevice):
        def __init__(self, pin, pull_up=True, **kwargs):
            kwargs.pop("hold_time", None)
            super().__init__(pin, pull_up=pull_up, **kwargs)

        @property
        def is_pressed(self):
            """override to avoid hold_time"""
            return self.is_active

    monkeypatch.setattr(src.main, "Button", _PatchedButton)
    yield
