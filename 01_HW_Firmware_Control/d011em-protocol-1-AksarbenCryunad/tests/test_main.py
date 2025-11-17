#!/usr/bin/env python3
# pylint: disable=import-error
"""unit test for main.py"""

import threading
import time
from collections import Counter

import serial
from gpiozero import Device
from gpiozero.pins.mock import MockFactory, MockPin
from src.main import (blink_led, blink_led_through_button,
                      check_to_input_button, receive_msg, transmit_msg)


def _pin(n: int) -> MockPin:
    factory: MockFactory = Device.pin_factory  # type: ignore
    return factory.pin(n)  # type: ignore


def test_blink_led():
    """test for blink_led()"""
    p18 = _pin(18)
    blink_led()
    assert p18.state == 0


def test_check_to_input_button(capsys):
    """using the thread to simulate the button press/release"""
    p18 = _pin(18)

    t_run = threading.Thread(target=check_to_input_button, daemon=True)
    t_run.start()
    time.sleep(0.01)

    for _ in range(12):
        p18.drive_low()
        time.sleep(0.01)
        p18.drive_high()
        time.sleep(0.01)

    t_run.join(timeout=2.0)
    assert not t_run.is_alive(), "Doesn't exit the check_to_input_button yet!"

    lines = [ln for ln in capsys.readouterr().out.strip().splitlines() if ln]
    counts = Counter(lines)
    assert counts["pressed"] == 10
    assert counts["released"] in (9, 10)
    assert len(lines) in (19, 20)
    assert lines[0] == "pressed"
    for i in range(len(lines) - 1):
        if lines[i] == "pressed":
            assert lines[i + 1] == "released"
        else:
            assert lines[i + 1] == "pressed"


def test_blink_led_through_button():
    """test for blink_led_through_button()"""
    led12 = _pin(12)
    btn13 = _pin(13)

    t = threading.Thread(target=blink_led_through_button, daemon=True)
    t.start()
    time.sleep(0.005)

    btn13.drive_low()

    t.join(timeout=2.0)
    assert not t.is_alive(), "Doesn't exit the blink_led_through_button yet!"
    assert led12.state == 0


def test_transmit_msg():
    """ test for transmit_msg() """
    transmit_msg()

    fake = serial.Serial.registry["/dev/ttyAMA3"]
    tx = fake.tx_data.decode().splitlines()
    assert len(tx) == 10
    assert tx[0] == "Hello World! 0"
    assert tx[-1] == "Hello World! 9"


def test_receive_msg(capsys):
    """ test for receive_msg() """
    t = threading.Thread(target=receive_msg, daemon=True)
    t.start()
    time.sleep(0.005)

    # input the data to be received
    fake = serial.Serial.registry["/dev/ttyAMA3"]
    fake.inject_rx(b"foo\nbar\nexit\n")

    # wait the thread to finish
    t.join(timeout=2.0)
    assert not t.is_alive(), "Doesn't exit the receive_msg yet!"

    # check the output
    out = capsys.readouterr().out.strip().splitlines()
    assert out == ["foo", "bar", "exit"]
