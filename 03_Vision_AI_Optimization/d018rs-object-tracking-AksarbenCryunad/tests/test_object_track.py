import json

import os
import numpy as np
import cv2
from src import object_track


def test_lab_config():
    # 최종 설정할 LAB-cal.json 파일을 프로젝트 루트에 넣으세요.
    root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    src_path = os.path.join(root_path, 'src')
    config_name = 'LAB-cal.json'
    config_path = None
    for path in [root_path, src_path]:
        candidate = os.path.join(path, config_name)
        if os.path.exists(candidate):
            config_path = candidate
            break
    assert config_path is not None, f"{config_name}이 없습니다."
    with open(config_path, 'r') as f:
        data = json.load(f)
    required_keys = {'l_min', 'l_max', 'a_min', 'a_max', 'b_min', 'b_max'}
    assert required_keys.issubset(
        data.keys()), f"{config_name} 누락된 정보가 있습니다: {required_keys - set(data.keys())}"


def test_find_biggest_contour_none():
    mask = np.zeros((100, 100), dtype=np.uint8)
    result = object_track.find_biggest_contour(mask)
    assert result is None


def test_find_biggest_contour_simple():
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2.rectangle(mask, (10, 10), (30, 30), 255, -1)
    cv2.rectangle(mask, (50, 50), (90, 90), 255, -1)
    contour = object_track.find_biggest_contour(mask)
    area = cv2.contourArea(contour)
    assert area > 1000


def test_draw_boundingbox():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2.rectangle(mask, (10, 10), (30, 30), 255, -1)
    contour = object_track.find_biggest_contour(mask)
    object_track.draw_boundingbox(img, contour)
    # Check if rectangle was drawn (look for green pixel)
    green_pixel = np.any(np.all(img == [0, 255, 0], axis=-1))
    assert green_pixel


def test_save_and_load_config(tmp_path):
    # Set globals
    object_track.l_min = 10
    object_track.l_max = 20
    object_track.a_min = 30
    object_track.a_max = 40
    object_track.b_min = 50
    object_track.b_max = 60
    path = tmp_path / 'test_config.json'
    object_track.save_config(str(path))
    # Reset
    object_track.l_min = 0
    object_track.l_max = 0
    object_track.a_min = 0
    object_track.a_max = 0
    object_track.b_min = 0
    object_track.b_max = 0
    object_track.load_config(str(path))
    assert object_track.l_min == 10
    assert object_track.l_max == 20
    assert object_track.a_min == 30
    assert object_track.a_max == 40
    assert object_track.b_min == 50
    assert object_track.b_max == 60
