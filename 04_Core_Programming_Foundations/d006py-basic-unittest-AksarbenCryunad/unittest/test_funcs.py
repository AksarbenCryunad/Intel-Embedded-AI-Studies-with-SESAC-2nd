# TODO: 사용자 모듈 import
import pytest
from my_functions import is_even, calculate_average, find_max, find_min

# TODO: 아래의 코드를 삭제하고 unittest를 작성하세요.

# is_even 함수를 위한 테스트 케이스
def test_is_even():
    assert is_even(4) == True
    assert is_even(7) == False
    assert is_even(0) == True
    assert is_even(-2) == True

# calculate_average 함수를 위한 테스트 케이스
def test_calculate_average():
    assert calculate_average([1, 2, 3, 4, 5]) == 3.0
    assert calculate_average([10, 20, 30]) == 20.0
    assert calculate_average([]) == 0
    assert calculate_average([5]) == 5.0

# find_max 함수를 위한 테스트 케이스
def test_find_max():
    assert find_max([1, 5, 2, 8, 3]) == 8
    assert find_max([-1, -5, -2]) == -1
    assert find_max([10]) == 10
    assert find_max([]) is None

# find_min 함수를 위한 테스트 케이스
def test_find_min():
    assert find_min([1, 5, 2, 8, 3]) == 1
    assert find_min([-1, -5, -2]) == -5
    assert find_min([10]) == 10
    assert find_min([]) is None
