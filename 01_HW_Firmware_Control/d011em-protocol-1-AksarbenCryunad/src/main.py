#!/usr/bin/env python3
# pylint: disable=import-error

"""
GPIO 과제
- gpiozero 라이브러리 사용

UART 과제
- pyserial 라이브러리 사용
- UART 통신: 115200 bps, 8N1, 플로우 제어 없음
- TXD3/RXD3을 활용
"""

import sys
import time

from gpiozero import LED, Button
from serial import Serial


def blink_led() -> None:
    """
    [문제 1] 18번 핀에 연결된 LED를 1초 간격으로 ON/OFF를 10번 반복
    - gpiozero.LED 사용
    - 종료시 LED는 OFF 상태
    """
    # TODO: blink_led 구현
    led = LED(18)
    for _ in range(10):
        led.on()
        time.sleep(1)
        led.off()
        time.sleep(1)
    led.close()


def check_to_input_button() -> None:
    """
    [문제 2] 18번 핀 버튼 입력을 받아서
      - 눌렸을 때: 'pressed'
      - 뗐을 때:   'released'
    를 출력한다.
    - polling 방식으로 구현할 것.
    - 버튼 입력을 10번 받았으면 종료.
    """
    # TODO: check_to_input_button 구현
    button = Button(18, pull_up=True)
    pressed_count = 0
    is_pressed = False
    
    while pressed_count < 10:
        if button.is_pressed and not is_pressed:
            print("pressed")
            pressed_count += 1
            is_pressed = True
        elif not button.is_pressed and is_pressed:
            print("released")
            is_pressed = False
        
        time.sleep(0.01)

    button.close()


def blink_led_through_button() -> None:
    """
    [문제 3]
    - 12번: LED 출력
    - 13번: Button 입력
    - 버튼이 눌려 있는 동안에만 LED가 0.5초 간격으로 깜빡인다.
    - 버튼이 10번 눌려졌으면 종료.
    - 종료시 LED는 OFF 상태
    """
    led = LED(12)
    button = Button(13)
    press_count = 0

    def start_blink():
        nonlocal press_count
        press_count += 1
        if press_count > 10:
            sys.exit(0)
        led.blink(on_time=0.5, off_time=0.5)

    def stop_blink():
        led.off()

    button.when_pressed = start_blink
    button.when_released = stop_blink
    
    try:
        while press_count <= 10:
            time.sleep(0.01)
    except SystemExit:
        pass
    finally:
        led.close()
        button.close()


def transmit_msg() -> None:
    """
    [문제 1] UART3로 "Hello World! {i}" 문자열을 1초마다 전송
    - 총 10번 전송 후 종료
    - 개행을 붙여 전송 (수신/테스트 편의)
    """
    # TODO: transmit_msg 구현
    ser = Serial("/dev/ttyAMA3", baudrate=115200, timeout=1)
    for i in range(10):
        message = f"Hello World! {i}\n"
        ser.write(message.encode("utf-8"))
        time.sleep(1)
    ser.close()


def receive_msg() -> None:
    """
    [문제 2] UART3에서 줄 단위로 읽어 화면에 출력.
    - 'exit' (대소문자 무시) 라인을 수신하면 함수 종료
    """
    # TODO: receive_msg 구현
    ser = Serial("/dev/ttyAMA3", baudrate=115200, timeout=1)

    while True:
        line = ""
        # readline 메서드가 없으므로 한 글자씩 읽어서 줄을 만듭니다.
        while True:
            char_byte = ser.read(1)
            if not char_byte:
                break
            char = char_byte.decode("utf-8")
            if char == '\n':
                break
            line += char
        
        if not line:
            time.sleep(0.01)
            continue

        print(line)

        if line.lower() == "exit":
            break
    ser.close()


if __name__ == "__main__":
    blink_led()
    check_to_input_button()
    blink_led_through_button()

    transmit_msg()
    receive_msg()
