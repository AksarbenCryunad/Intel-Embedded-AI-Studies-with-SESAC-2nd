#!/usr/bin/env python3
# pylint: disable=import-error
"""for pytest"""

from src.main import (
    task1_build_image,
    task2_run_and_check,
    task3_nginx_http_200,
    task4_env_pass_and_read,
    task5_compose_demo,
)


def test_task1_build_image():
    """check if docker image from Dockerfile is built"""
    assert task1_build_image() is True


def test_task2_run_and_check():
    """check if container runs and exits with code 0"""
    assert task2_run_and_check() is True


def test_task3_nginx_http_200():
    """check if nginx container returns HTTP 200 on port 18080"""
    assert task3_nginx_http_200() is True


def test_task4_env_pass_and_read():
    """check if env variable is passed and read correctly in container"""
    assert task4_env_pass_and_read() is True


def test_task5_compose_demo():
    """check if docker compose setup returns HTTP 200 on port 18081"""
    assert task5_compose_demo() is True
