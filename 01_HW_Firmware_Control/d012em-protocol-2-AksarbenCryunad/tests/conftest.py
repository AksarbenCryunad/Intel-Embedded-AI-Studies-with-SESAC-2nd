#!/usr/bin/env python3
# pylint: disable=import-error, too-few-public-methods
"""for mocking"""

import builtins
import sys
from enum import IntEnum

import pytest


class Mpu6050Reg(IntEnum):
    """MPU6050 registers"""

    ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43


class Rc522Reg(IntEnum):
    """RC522 registers"""

    COMMAND = 0x01
    COMM_IEN = 0x02
    COMM_IRQ = 0x04
    ERROR = 0x06
    STATUS1 = 0x07
    STATUS2 = 0x08
    FIFO_DATA = 0x09
    FIFO_LEVEL = 0x0A
    CONTROL = 0x0C
    BIT_FRAMING = 0x0D
    MODE = 0x11
    TX_CONTROL = 0x14
    VERSION = 0x37


class Rc522Cmd(IntEnum):
    """RC522 commands"""

    IDLE = 0x00
    TRANSCEIVE = 0x0C
    SOFT_RESET = 0x0F


class FakeSMBus:
    """
    fake SMBus for operating without the /dev/i2c-1
     - bytearray for the register as 256byte
     - implemented the wrapper API for read/write_byte_data
    """

    def __init__(self, bus_id=1):
        self.bus_id = bus_id
        self.regs = [0] * 256

        # wake up
        self.regs[int(Mpu6050Reg.PWR_MGMT_1)] = 1 << 6

        def put_word(reg_h, val):
            reg_h = int(reg_h)
            if val < 0:
                val = (1 << 16) + val
            self.regs[reg_h] = (val >> 8) & 0xFF
            self.regs[reg_h + 1] = val & 0xFF

        # ACCEL: 1000, -1000, 500
        put_word(Mpu6050Reg.ACCEL_XOUT_H, 1000)
        put_word(Mpu6050Reg.ACCEL_XOUT_H + 2, -1000)
        put_word(Mpu6050Reg.ACCEL_XOUT_H + 4, 500)

        # GYRO : 200, -200, 0
        put_word(Mpu6050Reg.GYRO_XOUT_H, 200)
        put_word(Mpu6050Reg.GYRO_XOUT_H + 2, -200)
        put_word(Mpu6050Reg.GYRO_XOUT_H + 4, 0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def write_byte_data(self, addr, reg, value):
        """wrapper API"""
        addr_i = int(addr)
        reg_i = int(reg)
        val_i = int(value) & 0xFF
        assert addr_i == int(Mpu6050Reg.ADDR)
        self.regs[reg_i] = val_i

    def read_byte_data(self, addr, reg):
        """wrapper API"""
        addr_i = int(addr)
        reg_i = int(reg)
        assert addr_i == int(Mpu6050Reg.ADDR)
        return self.regs[reg_i]


class FakeSpiDev:
    """
    RC522용 간단 SPI 모킹:
     - regs[0x00..0x7F]
     - xfer2([addr,val]) write, xfer2([addr,0]) read
     - REQA 시나리오를 최소 구현: FIFO에 0x26 들어오고 Transceive 및 StartSend 시
       CommIrqReg의 RxIRq 세우고, FIFOLevel=2, FIFOData에 ATQA [0x04, 0x00] 적재
    """

    def __init__(self):
        self.max_speed_hz = 1_000_000
        self.mode = 0
        self.regs = [0] * 0x80
        self.fifo = []  # 수신 응답버퍼
        # 기본: 안테나 OFF
        self.regs[int(Rc522Reg.TX_CONTROL)] = 0x00

    def open(self, bus, dev):
        """wrapper API"""

    def close(self):
        """wrapper API"""

    def xfer2(self, data):
        """wrapper API"""
        assert len(data) == 2
        header, payload = data[0], data[1]
        read = bool(header & 0x80)
        addr = (header & 0x7E) >> 1

        if read:
            if addr == int(Rc522Reg.FIFO_DATA):
                if self.fifo:
                    byte = self.fifo.pop(0)
                else:
                    byte = 0x00
                return [header, byte & 0xFF]

            return [header, self.regs[addr] & 0xFF]

        # write
        if addr == int(Rc522Reg.FIFO_DATA):
            self.regs[int(Rc522Reg.FIFO_LEVEL)] = min(
                self.regs[int(Rc522Reg.FIFO_LEVEL)] + 1, 64
            )
            self.regs[addr] = payload & 0xFF
        else:
            self.regs[addr] = payload & 0xFF

        if addr == int(Rc522Reg.BIT_FRAMING):
            if (payload & 0x80) and (
                self.regs[int(Rc522Reg.COMMAND)] == int(Rc522Cmd.TRANSCEIVE)
            ):
                if self.regs[int(Rc522Reg.FIFO_DATA)] == 0x26:
                    self.fifo = [0x04, 0x00]
                    self.regs[int(Rc522Reg.FIFO_LEVEL)] = 2
                    self.regs[int(Rc522Reg.COMM_IRQ)] |= 0x30
                    self.regs[int(Rc522Reg.ERROR)] = 0x00

        if addr == int(Rc522Reg.FIFO_LEVEL) and (payload & 0x80):
            self.regs[int(Rc522Reg.FIFO_LEVEL)] = 0
            self.fifo = []

        return [header, 0x00]


@pytest.fixture(scope="session", autouse=True)
def patch_dependencies():
    """change from smbus2, spidev to FakeSMBus, FakeSpiDev"""

    class DummySMBusModule:
        """dummy for smbus2"""

        SMBus = FakeSMBus

    sys.modules.pop("smbus2", None)

    class DummySpiDevModule:
        """dummy for spidev"""

        SpiDev = FakeSpiDev

    sys.modules.pop("spidev", None)

    original_import = builtins.__import__

    def fake_import(name, _globals=None, _locals=None, fromlist=(), level=0):
        if name == "smbus2":
            return DummySMBusModule()
        if name == "spidev":
            return DummySpiDevModule()
        return original_import(name, _globals, _locals, fromlist, level)

    builtins.__import__ = fake_import
