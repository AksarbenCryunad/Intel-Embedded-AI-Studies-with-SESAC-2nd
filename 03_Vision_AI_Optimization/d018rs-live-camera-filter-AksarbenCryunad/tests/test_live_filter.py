import cv2
import numpy as np
from src.filters import Filters


def test_kernel_implementation():
    """
    Filters class에 kernels를 구현했는지 여부를 검사.
    """
    f = Filters()
    assert 8 <= len(f.kernels), "Filters class에 kernels를 구현하세요."


def test_apply_filter():
    """
    Filters class에 apply_filter()가 구현 되어 있는지 확인.
    """
    f = Filters()

    # Test kernel
    f.kernels["test_filter"] = np.array(
        [
            [0, 0, 0],
            [0, 2, 0],
            [0, 0, 0]], dtype=np.float32)

    # Test frame image
    fake_frame = np.array(
        [
            [1, 2, 3],
            [4, 5, 6],
            [7, 8, 9]], dtype=np.float32)

    # apply_filter() 구현사항 확인
    expected_frame = cv2.filter2D(fake_frame, -1, f.kernels["test_filter"])
    filtered = f.apply_filter(fake_frame, "test_filter")
    assert np.array_equal(filtered, expected_frame), "apply_filter()를 구현하세요."


def test_switch_next():
    """
    UP key를 누르면 다음 kernel 정보로 변경되어야 함(circular)
    """
    new_kernels = {
        "first": 1,
        "second": 2,
        "third": 3
    }

    f = Filters(new_kernels)

    # 초기 커널 설정 정보 확인
    assert "first" == f.get_current_filter_name()

    # 다음 커널 정보 ("second")
    f.switch_next_filter()
    assert "second" == f.get_current_filter_name()

    # 다음 커널 정보 ("third")
    f.switch_next_filter()
    assert "third" == f.get_current_filter_name()

    # 다음 커널 정보 ("first", circular)
    f.switch_next_filter()
    assert "first" == f.get_current_filter_name()


def test_switch_previous():
    """
    DOWN key를 누르면 이전 kernel 정보로 변경되어야 함(circular)
    """
    new_kernels = {
        "first": 1,
        "second": 2,
        "third": 3
    }

    f = Filters(new_kernels)

    # 초기 커널 설정 정보 확인
    assert "first" == f.get_current_filter_name()

    # 이전 커널 정보 ("third")
    f.switch_previous_filter()
    assert "third" == f.get_current_filter_name()

    # 이전 커널 정보 ("second")
    f.switch_previous_filter()
    assert "second" == f.get_current_filter_name()

    # 이전 커널 정보 ("first")
    f.switch_previous_filter()
    assert "first" == f.get_current_filter_name()

    # 이전 커널 정보 ("third", circular)
    f.switch_previous_filter()
    assert "third" == f.get_current_filter_name()
