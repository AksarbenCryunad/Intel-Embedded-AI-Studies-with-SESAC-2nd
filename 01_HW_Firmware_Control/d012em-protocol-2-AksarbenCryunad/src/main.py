#!/usr/bin/env python3
"""
i2c 과제
 - mpu6050에서 6축 가속도/자이로 값 읽는 코드 작성
 - mpu6050 레지스터 중, PWR_MGMT_1에 있는 sleep bit를 설정하여
   장치를 sleep/wake 상태를 toggle 하는 코드 작성

spi 과제
 - MRC522 모듈로 REQA(7비트 프레임) 전송 후 ATQA(2바이트) 응답 수신하는 코드
   작성
 - MRC522 모듈의 TX_CONTROL 레지스터에 안테나 드라이브 on/off하는 코드 작성

ssh/sftp 과제
 - SSH로 원격에서 architercure 문자열 받아오기
"""

import time
import subprocess
from enum import IntEnum
from typing import Dict, Optional, Tuple

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = object

try:
    import spidev
except ImportError:
    spidev = None


class Mpu6050Reg(IntEnum):
    """MPU6050 레지스터 주소 (필요시 추가)"""

    ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    WHO_AM_I = 0x75


class Rc522Reg(IntEnum):
    """RC522 레지스터 주소 (필요시 추가)"""

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
    """RC522 명령어 (필요시 추가)"""

    IDLE = 0x00
    TRANSCEIVE = 0x0C
    SOFT_RESET = 0x0F


class Rc522SPI:
    """아주 작은 RC522 드라이버"""

    def __init__(self):
        if spidev is None:
            raise RuntimeError("spidev 미지원 환경입니다.")

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1_000_000
        self.spi.mode = 0 # mode가 4가지라고 함 

    def close(self):
        """RC522 SPI 종료"""
        self.spi.close()

    def write_reg(self, reg: Rc522Reg, val: int):
        """write"""
        self.spi.xfer2(
            [_rc522_addr_byte(int(reg), read=False), int(val) & 0xFF]
        )

    def read_reg(self, reg: Rc522Reg) -> int:
        """read"""
        resp = self.spi.xfer2([_rc522_addr_byte(int(reg), read=True), 0x00])
        return resp[1] & 0xFF

    def set_bits(self, reg: Rc522Reg, mask: int):
        """reg의 mask 비트들을 1로 설정"""
        self.write_reg(reg, self.read_reg(reg) | mask)

    def clear_bits(self, reg: Rc522Reg, mask: int):
        """reg의 mask 비트들을 0으로 설정"""
        self.write_reg(reg, self.read_reg(reg) & (~mask & 0xFF))

    def antenna_on(self, on: bool = True):
        """안테나 드라이브 on/off"""
        if on:
            self.set_bits(Rc522Reg.TX_CONTROL, 0x03)
        else:
            self.clear_bits(Rc522Reg.TX_CONTROL, 0x03)

    def transceive_7bit(self, tx_byte: int, timeout_loop: int = 50) -> bytes:
        """REQA(7비트 프레임) 전송 → FIFO 응답 수신 (간단 폴링 구현)"""
        self.write_reg(Rc522Reg.COMMAND, Rc522Cmd.IDLE)  # 명령어 IDLE
        self.write_reg(Rc522Reg.COMM_IRQ, 0x7F)  # 모든 IRQ 클리어
        self.write_reg(Rc522Reg.FIFO_LEVEL, 0x80)  # FIFO Flush
        self.write_reg(Rc522Reg.FIFO_DATA, tx_byte)  # FIFO에 1바이트 적재
        self.write_reg(Rc522Reg.BIT_FRAMING, 0x07)  # 7비트 프레임
        self.write_reg(Rc522Reg.COMMAND, Rc522Cmd.TRANSCEIVE)
        self.set_bits(Rc522Reg.BIT_FRAMING, 0x80)  # StartSend

        # IRQ 폴링 (RxIRq|IdleIRq)
        for _ in range(timeout_loop):
            irq = self.read_reg(Rc522Reg.COMM_IRQ)
            if irq & 0x30:
                break

        # 에러 체크, BufferOvfl, ParityErr, ProtocolErr
        if self.read_reg(Rc522Reg.ERROR) & 0x13:
            return b""

        # 수신 길이
        level = self.read_reg(Rc522Reg.FIFO_LEVEL)
        out = []
        for _ in range(level):
            out.append(self.read_reg(Rc522Reg.FIFO_DATA))
        return bytes(out)


def _read_word(bus: SMBus, addr: int, reg_h: int) -> int:
    """i2c, 레지스터 2개(High, Low) 읽어서 16bit 정수로 반환"""
    hi = bus.read_byte_data(addr, reg_h)
    lo = bus.read_byte_data(addr, reg_h + 1)
    val = (hi << 8) | lo
    if val & 0x8000:
        val = -((0xFFFF - val) + 1)
    return val


def _rc522_addr_byte(addr: int, read: bool) -> int:
    """RC522 SPI 어드레스 포맷: [addr<<1 | RW | 0] (RW=1이면 Read)"""
    a = (int(addr) << 1) & 0x7E
    if read:
        a |= 0x80
    return a


def read_imu() -> Dict[str, int]:
    """
    과제 1)
    - MPU6050에서 가속도/자이로 6축 값을 읽어서 dict로 반환
    - {'ax':..., 'ay':..., 'az':..., 'gx':..., 'gy':..., 'gz':...}
    """
    ax, ay, az = 0, 0, 0
    gx, gy, gz = 0, 0, 0

    with SMBus(1) as bus:
        bus.write_byte_data(Mpu6050Reg.ADDR, Mpu6050Reg.PWR_MGMT_1,0x0)
        # TODO: I2C로 MPU6050에서 6축 값 읽기
        ax = _read_word(bus, Mpu6050Reg.ADDR, Mpu6050Reg.ACCEL_XOUT_H)
        ay = _read_word(bus, Mpu6050Reg.ADDR, Mpu6050Reg.ACCEL_XOUT_H + 2)
        az = _read_word(bus, Mpu6050Reg.ADDR, Mpu6050Reg.ACCEL_XOUT_H + 4)
        gx = _read_word(bus, Mpu6050Reg.ADDR, Mpu6050Reg.GYRO_XOUT_H)
        gy = _read_word(bus, Mpu6050Reg.ADDR, Mpu6050Reg.GYRO_XOUT_H + 2)
        gz = _read_word(bus, Mpu6050Reg.ADDR, Mpu6050Reg.GYRO_XOUT_H + 4)

    return {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}


def wake_device() -> Tuple[int, int]:
    """
    과제 2): SLEEP <-> WAKE
    - PWM_MGMT_1 전체 register value (before, after) 반환
    """
    with SMBus(1) as bus:
        # TODO: PWR_MGMT_1 레지스터 읽고, sleep bit 토글
        before = bus.read_byte_data(Mpu6050Reg.ADDR, Mpu6050Reg.PWR_MGMT_1)
        after = before ^ (1 << 6)
        bus.write_byte_data(Mpu6050Reg.ADDR, Mpu6050Reg.PWR_MGMT_1, after)
        verify = bus.read_byte_data(Mpu6050Reg.ADDR, Mpu6050Reg.PWR_MGMT_1)

    return before, verify


def rfid_poll_once() -> Tuple[bool, Optional[bytes]]:
    """
    과제 3)
      - RC522로 REQA 전송 → ATQA(2바이트) 응답이면 태그 존재
      - (present, atqa_bytes) 반환
    """
    r = Rc522SPI()
    try:
        # TODO: REQA 전송 후 ATQA 수신
        # 0x26은 'PICC'에서 'REQA' 명령을 나타내는 값
        atqa = r.transceive_7bit(0x26)
        present = len(atqa) == 2 and atqa == b"\x04\x00"
        return present, atqa
    finally:
        r.close()


def rfid_set_antenna(on: bool) -> int:
    """
    과제 4)
      - 모듈 레지스터 쓰기 예: 안테나 ON/OFF
      - 쓰고 나서 읽어서 상태(int) 반환 (TX_CONTROL 레지스터)
    """
    r = Rc522SPI()
    try:
        # TODO: 안테나 on/off 설정
        r.antenna_on(on)
        return r.read_reg(Rc522Reg.TX_CONTROL)
    finally:
        r.close()


def ssh_get_arch() -> str:
    """
    SSH로 원격에서 architecture 문자열을 받아와 반환.
    - 반드시 user_host / cmd를 채워야 함 (비워두면 ValueError)
    - SSH 실패(returncode!=0)면 RuntimeError
    - 출력이 비정상이거나 arm64 계열이 아니면 AssertionError
    """
    archs = ("aarch64", "arm64")

    # TODO: user_host, cmd 채우기
    user_host = "aksarbencryunad@raspberrypi"
    cmd = "uname -m"

    if not user_host or not user_host.strip():
        raise ValueError("user_host를 반드시 채우세요.")
    if not cmd or not cmd.strip():
        raise ValueError("cmd를 반드시 채우세요.")

    result = subprocess.run(
        ["ssh", user_host, cmd],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if result.returncode != 0:
        raise RuntimeError(f"SSH failed: {result.stderr.strip()}")

    arch = (result.stdout or "").strip()
    if not arch:
        raise ValueError("원격 arch 문자열이 비어 있습니다.")

    if arch not in archs:
        raise AssertionError(f"arm64가 아닙니다: got {arch!r}")

    return arch


if __name__ == "__main__":
    imu_data = read_imu()
    print("IMU Data:", imu_data)

    before_reg, after_reg = wake_device()
    print(f"Before: {before_reg:#04x}, After: {after_reg:#04x}")
    print("Device is now", "awake" if not (after_reg & (1 << 6)) else "asleep")

    tag_present, atqa_val = rfid_poll_once()
    print("Tag present:", tag_present, "ATQA:", atqa_val)
    tx_control = rfid_set_antenna(True)
    print(f"TX_CONTROL after antenna on: {tx_control:#04x}")

    ret_arch = ssh_get_arch()
    print("Remote architecture:", ret_arch)
