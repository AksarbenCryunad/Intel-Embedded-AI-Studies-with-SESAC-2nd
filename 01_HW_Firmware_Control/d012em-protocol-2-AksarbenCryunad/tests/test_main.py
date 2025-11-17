#!/usr/bin/env python3
# pylint: disable=too-few-public-methods
"""for test the src/main.py"""

import importlib
import subprocess


def test_read_imu():
    """check the return value of read_imu()"""
    main = importlib.import_module("src.main")
    data = main.read_imu()
    assert data == {
        "ax": 1000,
        "ay": -1000,
        "az": 500,
        "gx": 200,
        "gy": -200,
        "gz": 0,
    }


def test_wake_device():
    """check the return value of wake_device()"""
    main = importlib.import_module("src.main")
    before, after = main.wake_device()
    sleep_before = (before >> 6) & 1
    sleep_after = (after >> 6) & 1
    assert sleep_after == (sleep_before ^ 1)


def test_rfid_poll_once():
    """check the return value of rfid_poll_once()"""
    main = importlib.import_module("src.main")
    present, atqa = main.rfid_poll_once()
    assert present is True
    assert atqa == bytes([0x04, 0x00])


def test_rfid_antenna():
    """check the return value of rfid_set_antenna()"""
    main = importlib.import_module("src.main")

    v_on = main.rfid_set_antenna(True)
    assert (v_on & 0x03) == 0x03

    v_off = main.rfid_set_antenna(False)
    assert (v_off & 0x03) == 0x00


def test_ssh_get_arch_arm64(monkeypatch):
    """check the return value of ssh_get_arch() for arm64"""

    class Dummy:
        """Dummy class"""

        stdout = "aarch64\n"
        returncode = 0
        stderr = ""

    def fake_run(argv, *_a, **_kw):
        assert argv[2] == "uname -m", f"Fault cmd: {argv[2]}"
        return Dummy()

    monkeypatch.setattr(subprocess, "run", fake_run)

    main = importlib.import_module("src.main")
    assert main.ssh_get_arch() == "aarch64"
