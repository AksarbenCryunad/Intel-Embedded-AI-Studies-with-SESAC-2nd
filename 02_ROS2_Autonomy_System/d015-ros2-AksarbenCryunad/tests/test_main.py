# pylint: disable=import-error, redefined-outer-name
"""test"""

import importlib.util
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

# check the status
PACKAGE_NAME = os.environ.get("ROS_PKG_NAME", "ros_pkg")
WORKSPACE = Path.home() / "ws_ros2"


# check the envrionment
def _has_ros_jazzy() -> bool:
    return Path("/opt/ros/jazzy/setup.bash").exists()


def _has_xhost_env() -> bool:
    return (
        shutil.which("xhost") is not None
        and bool(os.environ.get("DISPLAY"))
        and Path("/tmp/.X11-unix").exists()
    )


def _has_ros2_cli() -> bool:
    return shutil.which("ros2") is not None


def _has_colcon() -> bool:
    return shutil.which("colcon") is not None


# src/main.py
def _load_student_main():
    project_root = Path(__file__).resolve().parents[1]
    student_main = project_root / "src" / "main.py"
    if not student_main.exists():
        raise ImportError(f"학생 파일을 찾을 수 없습니다: {student_main}")
    spec = importlib.util.spec_from_file_location("student_main", student_main)
    mod = importlib.util.module_from_spec(spec)  # type: ignore
    assert spec and spec.loader
    spec.loader.exec_module(mod)  # type: ignore
    return mod


# for delete the test folder
@pytest.fixture(autouse=True, scope="function")
def clean_ws():
    """clean workspace before and after each test"""
    if WORKSPACE.exists():
        shutil.rmtree(WORKSPACE)
    yield
    if WORKSPACE.exists():
        shutil.rmtree(WORKSPACE)


@pytest.fixture
def main_mod():
    """load student main"""
    return _load_student_main()


def test_source_ros_env_ok(main_mod):
    """ros environments check"""
    rc, out, err = main_mod.task_source_ros()
    assert rc == 0, f"source 실패: {err}"
    env = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
    assert env.get("ROS_DISTRO") == "jazzy", "ROS_DISTRO가 jazzy가 아님"
    assert "AMENT_PREFIX_PATH" in env, "AMENT_PREFIX_PATH 누락"


def test_pkg_create_and_package_xml(main_mod):
    """pkg create check"""
    rc, _out, err = main_mod.task_pkg_create_cmd()
    assert rc == 0, f"ros2 pkg create 실패: {err}"

    pkg_xml = WORKSPACE / "src" / PACKAGE_NAME / "package.xml"
    assert pkg_xml.exists(), f"package.xml 미생성: {pkg_xml}"

    root = ET.parse(pkg_xml).getroot()
    n = root.find("name")
    assert n is not None and n.text and n.text.strip() == PACKAGE_NAME
    d = root.find("description")
    assert (
        d is not None and d.text and "D015 homework ROS2 workspace" in d.text
    )
    m = root.find("maintainer")
    assert m is not None and m.text and m.text.strip()
    assert "email" in m.attrib and m.attrib["email"]
    lic = root.find("license")
    assert lic is not None and lic.text and lic.text.strip().upper() == "MIT"
    deps = [e.text.strip() for e in root.findall("depend") if e.text]
    assert "rclpy" in deps and "std_msgs" in deps


def test_colcon_build_creates_install(main_mod):
    """colcon build check"""
    if not (WORKSPACE / "src" / PACKAGE_NAME / "package.xml").exists():
        rc, _out, err = main_mod.task_pkg_create_cmd()
        assert rc == 0, f"ros2 pkg create 실패: {err}"

    rc, _out, err = main_mod.task_colcon_build_cmd()
    assert rc == 0, f"colcon build 실패: {err}"
    assert (
        WORKSPACE / "install" / "setup.bash"
    ).exists(), "install/setup.bash 미생성"


def test_xhost_cmd_runs(main_mod):
    """xhost check"""
    rc, out, err = main_mod.task_xhost_cmd()
    assert rc == 0, f"xhost 실행 실패: {err}"
    low = (out or "").lower()
    assert any(
        k in low for k in ("added", "enabled", "access control")
    ), f"xhost 출력 비정상: {out}"


def test_docker_run_cmd_string_only(main_mod):
    """docker command check"""
    cmd = main_mod.task_docker_run_cmd().strip()
    assert cmd.startswith("docker run -it --rm --name humble-gui")
    assert "--network=host" in cmd
    assert "-e DISPLAY=$DISPLAY" in cmd
    assert "-v /tmp/.X11-unix:/tmp/.X11-unix:rw" in cmd
    assert "-e ROS_DOMAIN_ID=" in cmd
    assert "-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in cmd
    assert cmd.endswith("arm64v8/ros:humble bash")
