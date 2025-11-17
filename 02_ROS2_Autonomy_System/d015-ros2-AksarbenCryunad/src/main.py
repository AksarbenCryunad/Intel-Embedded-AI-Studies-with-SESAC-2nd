# pylint: disable=import-error

"""
ROS2 실습 과제
1) ROS2 Jazzy 환경 적용 명령어 반환
2) 주어진 workspace밑에 ros_pkg 생성 명령 반환
3) 주어진 workspace밑에서 ros2 빌드 실행 명령 반환
4) root 유저에게 X권한을 열어주는 명령어 반환
5) humble-gui이름의 컨테이너 실행하는 docker 명령어 작성
"""

import os
import subprocess
from pathlib import Path


def _run(cmd: str, cwd: str | None = None) -> tuple[int, str, str]:
    """명령어 실행 wrapper: returncode, stdout, stderr 반환"""
    result = subprocess.run(
        cmd, check=False, shell=True, cwd=cwd, capture_output=True, text=True
    )
    return result.returncode, result.stdout, result.stderr


def task_source_ros() -> tuple[int, str, str]:
    """
    문제 1:
    ROS2 Jazzy 환경을 불러오는 명령어를 반환하시오.
    """
    # TODO: 여기에 코드를 작성하시오
    cmd = "source /opt/ros/jazzy/setup.bash && env"
    return _run(f"bash -lc '{cmd}'")


def task_pkg_create_cmd() -> tuple[int, str, str]:
    """
    문제 2:
    주어진 workspace/src 밑에 ros_pkg를 생성하는 명령어를 반환하시오.
    조건:
      - --build-type ament_python
      - workspace는 ~/ws_ros2로 지정한다
      - 빌드 타입은 ament_python 사용
      - 라이센스는 MIT 사용
      - 패키지 디펜던시는 rclpy와 std_msgs가 있음
      - maintainer이름은 kcci로 지정
      - maintainer email은 kcci@kcci.com으로 지정
      - 코드 설명은 D015 homework ROS2 workspace로 지정
    """
    # TODO: 여기에 코드를 작성하시오
    workspace = Path.home() /"ws_ros2"
    src_dir = workspace / "src"
    src_dir.mkdir(parents=True, exist_ok=True)

    cmd = "ros2 pkg create ros_pkg --build-type ament_python --license MIT --maintainer-name kcci --maintainer-email kcci@kcci.com --description \"D015 homework ROS2 workspace\" --dependencies rclpy std_msgs"
    return _run(cmd, cwd=os.path.join(workspace, "src"))


def task_colcon_build_cmd() -> tuple[int, str, str]:
    """
    문제 3:
    workspace(~/ws_ros2) 경로에서 ros2 빌드하는 명령어를 반환하시오.
    """
    # TODO: 여기에 코드를 작성하시오
    workspace = Path.home() / "ws_ros2"
    workspace.mkdir(parents=True, exist_ok=True)
    cmd = "colcon build --symlink-install"
    return _run(cmd, cwd=workspace)


def task_xhost_cmd() -> tuple[int, str, str]:
    """
    문제 4:
    root 유저에 X 권한을 여는 xhost 명령어를 반환하시오.
    """
    # TODO: 여기에 코드를 작성하시오
    cmd = "xhost +local:root" # xhost는 gui의 권한설정
    return _run(cmd)


def task_docker_run_cmd() -> str:
    """
    문제 5:
    humble-gui 컨테이너를 실행하는 docker run 명령어를 반환하시오.
    조건:
      - network는 host를 그대로 사용
      - display 환경변수를 host 값 그대로 사용
      - domain id값은 24로 고정
      - rmw_cyclondds_cpp를 RMW 엔진으로 사용
      - docker image는 arm64v8/ros:humble을 사용
      - 컨테이너 이름은 humble-gui
      - 실행 명령은 bash 실행
      - -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    """
    # TDO: 여기에 코드를 작성하시오
    return (
        "docker run -it --rm --name humble-gui --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e ROS_DOMAIN_ID=24 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp arm64v8/ros:humble bash"
    ) # -it의 t는 터미널을 열고 i는 대화를 할 수 있는 명령어로 둘다 사용하면 터미널을 열면서 대화까지 가능하게 만드는 것으로 생각됨


if __name__ == "__main__":
    print("1) ROS2 Jazzy 환경 적용 명령어 반환")
    code, out, err = task_source_ros()
    print(f"returncode: {code}\nstdout: {out}\nstderr: {err}")

    print("\n2) 주어진 workspace밑에 ros_pkg 생성 명령 반환")
    code, out, err = task_pkg_create_cmd()
    print(f"returncode: {code}\nstdout: {out}\nstderr: {err}")

    print("\n3) 주어진 workspace밑에서 ros2 빌드 실행 명령 반환")
    code, out, err = task_colcon_build_cmd()
    print(f"returncode: {code}\nstdout: {out}\nstderr: {err}")

    print("\n4) root 유저에게 X권한을 열어주는 명령어 반환")
    code, out, err = task_xhost_cmd()
    print(f"returncode: {code}\nstdout: {out}\nstderr: {err}")

    print("\n5) humble-gui이름의 컨테이너 실행하는 docker 명령어 작성")
    RET = task_docker_run_cmd()
    print(f"docker run command:\n{RET}")
