#!/usr/bin/env python3
"""
과제
 - MAC / CPU 아키텍처 / 현재 디렉토리(PWD)를 명령어로 얻어라.
 - U-Boot 소스 디렉토리에서 빌드 후, 산출물 'u-boot'가 ARM aarch64인지
   판정하라.
 -

요구:
- subprocess.run(capture_output=True, text=True, check=False) 사용
- 빈 출력/오류 시 ValueError 발생
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from typing import Dict, Optional


def _run(cmd: list[str], cwd: Optional[str] = None):
    return subprocess.run(
        cmd, cwd=cwd, capture_output=True, text=True, check=False
    )
    

def get_mac_address() -> str:
    """
    ifconfig -a 출력에서 루프백(lo) 제외 첫 번째 MAC 주소를 소문자 형태로 반환.
    예: 'b8:27:eb:12:34:56'
    """
    if not shutil.which("ifconfig"):
        raise EnvironmentError("Failed to find 'ifconfig' command.")
    
    result = _run(["ifconfig", "-a"])

    # TODO: output, blocks 구현
    output = result.stdout
    blocks = re.split(r'\n(?=\S)', output)
    for block in blocks:
        if block.startswith("lo"):  # 루프백 제외
            continue

        m = re.search(r"ether\s+([0-9a-fA-F:]{17})", block)
        if m:
            return m.group(1).lower()

    raise ValueError("Failed to get MAC address.")


def get_cpu_arch() -> str:
    """
    `uname -m` 한 줄 결과를 반환. (예: 'x86_64', 'aarch64', 'armv7l')
    """
    result = _run(["uname", "-m"])
    return result.stdout.strip()


def get_pwd() -> str:
    """
    `pwd` 한 줄 결과(절대경로)를 반환.
    """
    # 1. _run 함수를 사용하여 ['pwd'] 명령어를 실행합니다.
    result = _run(['pwd'])
    
    # 2. 실행 결과 객체(result)의 .stdout 속성에 접근하여 출력을 가져옵니다.
    # 3. .strip() 메서드를 사용하여 줄바꿈을 제거합니다.
    return result.stdout.strip()


def check_uboot_aarch64(uboot_src: str) -> Dict[str, object]:
    """
    지정된 U-Boot 소스 디렉토리(uboot_src)에서 빌드 후,
    산출물 'u-boot'에 대해 `file u-boot`를 실행하여 ARM aarch64 여부를 판정한다.

    반환 예:
      {
        'ok': True/False,                 # ARM aarch64 여부
        'arch': 'ARM aarch64' or None,    # 매칭되면 'ARM aarch64', 아니면 None
        'file_output': '<file 출력 전문>', # file 결과 원문
      }
    실패 상황(경로 오류/빌드 실패/아티팩트 없음)은 ValueError로 던진다.
    """
    if not os.path.isdir(uboot_src):
        raise ValueError(
            "Failed to read the u-boot directory path" f" {uboot_src}"
        )

    # defconfig: 'make rpi_arm64_defconfig'
    _run(["make", "rpi_arm64_defconfig"], uboot_src)

    # build: 'make CROSS_COMPILE=aarch64-linux-gnu- -j$(nproc) u-boot'
    # nproc 값을 먼저 얻어서 make 명령어에 전달합니다.
    nproc_result = _run(["nproc"])
    nproc_count = nproc_result.stdout.strip()
    build = _run(
        ["make", "CROSS_COMPILE=aarch64-linux-gnu-", f"-j{nproc_count}", "u-boot"],
        uboot_src,
    )
    if build.returncode != 0:
        raise ValueError(f"Failed to build\n{build.stderr}")

    # check the u-boot file
    uboot_path = os.path.join(uboot_src, "u-boot")
    if not os.path.exists(uboot_path):
        raise ValueError("Failed to check the u-boot file!")

     # check the architecture
    f = _run(["file", uboot_path])
    file_out = (
        (f.stdout or "") + ("\n" + f.stderr if f.stderr else "")
    ).strip()
    is_aarch64 = re.search(r"\bARM aarch64\b", file_out) is not None

    return {
        "ok": is_aarch64,
        "arch": "ARM aarch64" if is_aarch64 else None,
        "file_output": file_out,
    }


def check_rpi_config(boot_mount: str) -> Dict[str, object]:
    """
    지정한 boot 마운트 경로에 대해
    'boot/firmware/config.txt' 파일이 존재하는지 확인한다.

    반환:
      {
        'ok': True/False,
        'config_path': '<경로>',
      }

    - 파일이 없으면 ok=False
    """
    if not isinstance(boot_mount, str) or not boot_mount:
        raise ValueError("boot_mount 인자가 비었습니다.")

    #TODO: config_path, exists 구현
    config_path = os.path.join(boot_mount, "firmware", "config.txt")
    exists = os.path.isfile(config_path)

    return {"ok": exists, "config_path": config_path}

if __name__ == "__main__":
    # example usage
    print("MAC:", get_mac_address())
    print("CPU:", get_cpu_arch())
    print("PWD:", get_pwd())

    print(check_uboot_aarch64("/home/intel/u-boot"))
    print(check_rpi_config("/media/pi/boot"))
