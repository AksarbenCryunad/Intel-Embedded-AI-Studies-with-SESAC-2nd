#!/usr/bin/env python3
# pylint: disable=import-error

"""Test main"""

import os
import re
import shutil
import types

import pytest
import src.main as m
from src.main import (
    check_rpi_config,
    check_uboot_aarch64,
    get_cpu_arch,
    get_mac_address,
    get_pwd,
)


def _cp(stdout="", stderr="", returncode=0):
    """subprocess.CompletedProcess"""
    return types.SimpleNamespace(
        returncode=returncode, stdout=stdout, stderr=stderr
    )


def test_get_mac_address_from_ifconfig(monkeypatch):
    """ifconfig -a"""
    out = """
lo: flags=...
    inet 127.0.0.1
    ether 00:00:00:00:00:00

eth0: flags=...
    inet 192.168.0.10
    ether b8:27:eb:12:34:56  txqueuelen 1000
"""
    monkeypatch.setattr(m, "_run", lambda cmd: _cp(stdout=out))
    monkeypatch.setattr(
        shutil,
        "which",
        lambda name: "/sbin/ifconfig" if name == "ifconfig" else None,
    )

    mac = get_mac_address()
    assert re.fullmatch(
        r"[0-9a-f]{2}(:[0-9a-f]{2}){5}", mac
    ), f"MAC 형식 불일치: {mac}"
    assert mac != "00:00:00:00:00:00"


def test_get_cpu_arch(monkeypatch):
    """uname -m"""
    monkeypatch.setattr(m, "_run", lambda cmd: _cp(stdout="x86_64\n"))
    assert get_cpu_arch() == "x86_64"


def test_get_pwd(monkeypatch):
    """pwd"""
    monkeypatch.setattr(
        m, "_run", lambda cmd: _cp(stdout="/home/intel/2nd-embedded\n")
    )
    assert "2nd-embedded" in get_pwd()


def test_check_uboot_aarch64(monkeypatch, tmp_path):
    """
    command check, build check, file check
    """

    src = tmp_path / "u-boot-src"
    src.mkdir()

    calls = []

    def fake_run_ok(cmd, cwd=None):
        calls.append((cmd, cwd))
        cmd_str = " ".join(cmd)
        if cmd[:2] == ["make", "rpi_arm64_defconfig"]:
            return _cp(stdout="defconfig ok\n")
        if (
            "CROSS_COMPILE=aarch64-linux-gnu-" in cmd_str
            and "u-boot" in cmd_str
        ):
            return _cp(stdout="build ok\n")
        if cmd[:1] == ["file"]:
            return _cp(
                stdout="u-boot: ELF 64-bit LSB executable,"
                "ARM aarch64, version 1 (SYSV)\n"
            )
        return _cp(stdout="")

    monkeypatch.setattr(os.path, "exists", lambda p: p.endswith("u-boot"))
    monkeypatch.setattr(m, "_run", fake_run_ok)

    info = check_uboot_aarch64(str(src))
    assert info["ok"] is True
    assert info["arch"] == "ARM aarch64"
    assert "aarch64" in info["file_output"]
    assert all(cwd == str(src) for _, cwd in calls if cwd is not None)

    def fake_run_wrong_arch(cmd, _cwd=None):
        if cmd[:1] == ["file"]:
            return _cp(
                stdout="u-boot: ELF 32-bit LSB executable,"
                "ARM, EABI5 version 1 (SYSV)\n"
            )
        return _cp(stdout="ok\n")

    monkeypatch.setattr(m, "_run", fake_run_wrong_arch)
    monkeypatch.setattr(os.path, "exists", lambda p: p.endswith("u-boot"))

    info = check_uboot_aarch64(str(src))
    assert info["ok"] is False
    assert info["arch"] is None
    assert "ELF 32-bit" in info["file_output"]

    monkeypatch.setattr(m, "_run", lambda *a, **k: _cp(stdout="ok\n"))
    monkeypatch.setattr(os.path, "exists", lambda p: False)
    with pytest.raises(ValueError):
        check_uboot_aarch64(str(src))

    with pytest.raises(ValueError):
        check_uboot_aarch64("/definitely/not/exist")


def test_check_rpi_config_single(tmp_path):
    """check the config.txt file"""
    boot = tmp_path / "boot"
    fw = boot / "firmware"
    fw.mkdir(parents=True)

    cfg = fw / "config.txt"

    cfg.write_text("dummy")
    info = check_rpi_config(str(boot))
    assert info["ok"] is True
    assert info["config_path"].endswith("boot/firmware/config.txt")

    cfg.unlink()
    info2 = check_rpi_config(str(boot))
    assert info2["ok"] is False
