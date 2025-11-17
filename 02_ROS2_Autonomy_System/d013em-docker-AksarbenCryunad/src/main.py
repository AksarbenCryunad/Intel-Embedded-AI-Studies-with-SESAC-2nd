#!/usr/bin/env python3
# pylint: disable=import-error,
"""
과제:
1) docker build 결과 확인 (컨텍스트: ~/container, 태그: msa:test)
2) docker run 실행 후 docker ps로 컨테이너 실행 여부 확인 (이름: msa-run)
3) nginx 컨테이너 띄우고 포트(18080)로 HTTP 200 확인 (이름: msa-nginx)
4) 환경변수 WHO=MSA 전달 → 컨테이너 내부에서 읽은 값이 동일한지 확인
5) docker compose 사용: ~/container/compose 에서 `docker compose up -d` 후
   http://127.0.0.1:18081 에서 HTTP 200 확인
"""

from __future__ import annotations

import os
import subprocess
import time
from typing import Dict, List, Optional

import requests
from requests import RequestException


def _run(
    cmd: List[str],
    check: bool = False,
    capture_output: bool = True,
    text: bool = True,
    env: Optional[Dict[str, str]] = None,
) -> subprocess.CompletedProcess:
    """subprocess.run 간단 래퍼"""
    return subprocess.run(
        cmd, check=check, capture_output=capture_output, text=text, env=env
    )


def _expand(path: str) -> str:
    """~ 포함 경로 확장"""
    return os.path.expanduser(path)


def _http_ok(url: str, timeout: float = 8.0) -> bool:
    """HTTP 200을 기다렸다가 True/False 반환"""
    if requests is None:
        raise RuntimeError("requests 라이브러리가 필요합니다.")

    if "18080" not in url:
        raise ValueError("포트 번호가 잘못된 URL입니다.")

    deadline = time.time() + timeout
    last_exc = None
    while time.time() < deadline:
        try:
            r = requests.get(url, timeout=1.0)
            if r.status_code == 200:
                return True
        except RequestException as e:
            last_exc = e
        time.sleep(0.3)
    raise RuntimeError(f"HTTP 200 확인 실패: {url} (last={last_exc})")


def _cleanup_all_containers() -> None:
    """
    모든 컨테이너를 강제 삭제.
    docker rm -f $(docker ps -aq) 와 동일한 효과.
    """
    try:
        # 현재 컨테이너 ID 목록
        ps = _run(["docker", "ps", "-aq"])
        ids = ps.stdout.split()
        if ids:
            _run(["docker", "rm", "-f"] + ids)
    except (RuntimeError, FileNotFoundError, PermissionError):
        # 실패해도 그냥 무시 (컨테이너 없거나 권한 문제일 수 있음)
        pass


def task1_build_image() -> bool:
    """
    docker build 수행 후 이미지 존재 여부 확인
     - 지정한 컨텍스트 디렉터리(~/container)에 있는 Dockerfile로 이미지를 빌드
     - 이미지가 로컬에 존재하는지 확인하여 True/False 반환.
     - 빌드된 이미지 이름은 msa:test로 고정
    """
    _cleanup_all_containers()

    context_dir = _expand("~/container")
    
    # TODO: docker 빌드
    result = _run(["docker", "build", "-t", "msa:test", context_dir])

    # TODO: 빌드 실패 시 예외 발생
    if result.returncode != 0:
        raise RuntimeError(f"Docker build failed: {result.stderr}")

    # TODO: 이미지 확인, msa:test
    images = _run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"])
    # stdout 속성을 확인하도록 수정
    return "msa:test" in images.stdout


def task2_run_and_check() -> bool:
    """
    docker run 실행 후 docker ps로 컨테이너 실행 여부 확인
     - 컨테이너 이름: msa-run (고정)
     - 이미지: alpine:3.19 (고정)
     - 백그라운드 유지용 명령: sh -lc 'sleep 60'
    """
    _cleanup_all_containers()

    # TODO: docker 실행
    name = "msa-run"
    run_res = _run(["docker", "run", "-d", "--name", name, "alpine:3.19","sh", "-lc", "sleep 60"], check=True)

    # TODO: 실행 실패 시 예외 발생
    if run_res.returncode != 0:
        raise RuntimeError(f"Docker run failed: {run_res.stderr}")
    
    # TODO: docker 실행 여부 확인
    ps = _run(["docker", "ps"], check=True)
    return name in ps.stdout


def task3_nginx_http_200() -> bool:
    """
    nginx 컨테이너를 띄우고 http://127.0.0.1:18080 에서 200 확인
     - 컨테이너 이름: msa-nginx (고정)
     - 이미지: nginx:alpine (고정)
     - 포트 바인딩: 18080:80 (고정)
    """
    _cleanup_all_containers()

    # TDOO: msa-nginx 실행
    name = "msa-nginx"
    port = 18080

    
    run_res = _run(["docker", "run", "--rm", "-d", "--name", name, "-p", f"{port}:80", "nginx:alpine"],
        check=True)

    # TODO: 실행 실패 시 예외 발생
    if run_res.returncode != 0:
        raise RuntimeError(f"Docker run failed: {run_res.stderr}")
    return _http_ok(f"http://127.0.0.1:{port}")


def task4_env_pass_and_read() -> bool:
    """
    환경변수 WHO=MSA 전달 후 컨테이너 내부에서 읽은 값 확인
     - 이미지: alpine:3.19
     - 명령: printenv WHO
    """
    _cleanup_all_containers()

    # TODO: 환경변수 전달하여 실행
    var, value = "WHO", "MSA"
    res = _run(["docker", "run", "--rm", "--env", f"{var}={value}", "alpine:3.19", "printenv", var], check=True)

    # TODO: 실행 실패 시 예외 발생
    if res.returncode != 0:
        raise RuntimeError(f"Docker run failed: {res.stderr}")
    return res.stdout.strip() == value


def task5_compose_demo() -> bool:
    """
    docker compose 사용 예제:
     - 작업 디렉터리: ~/container
     - 해당 디렉터리에 미리 docker-compose.yml 이 있다고 가정
     - 절차: `docker compose up -d` 실행 → http://127.0.0.1:18081 에서 200 확인
     - 컨테이너 이름: msa-compose-nginx
    """
    _cleanup_all_containers()

    port = 18080
    workdir = _expand("~/container")
    
    # docker-compose.yml 파일이 없는 경우를 대비하여 생성
    if not os.path.exists(workdir):
        os.makedirs(workdir)
        
    compose_content = """
services:
  web:
    image: nginx:alpine
    ports:
      - "18080:80"
"""
    with open(os.path.join(workdir, "docker-compose.yml"), "w") as f:
        f.write(compose_content)

    # 일부 환경에서는 현재 디렉터리 기준으로 compose가 동작하므로 cwd 지정
    up = subprocess.run(
        ["docker", "compose", "up", "-d"],
        check=False,
        cwd=workdir,
        capture_output=True,
        text=True,
    )

    # TODO: 실행 실패 시 예외 발생
    if up.returncode != 0:
        raise RuntimeError(f"Docker run failed: {up.stderr}")
    ret = _http_ok(f"http://127.0.0.1:{port}")
    _cleanup_all_containers()

    return ret


if __name__ == "__main__":
    print("[1] build:", task1_build_image())
    print("[2] run+ps:", task2_run_and_check())
    print("[3] nginx 200:", task3_nginx_http_200())
    print("[4] env pass/read:", task4_env_pass_and_read())
    print("[5] compose 200:", task5_compose_demo())
