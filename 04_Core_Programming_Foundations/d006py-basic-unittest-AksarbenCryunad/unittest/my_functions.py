def is_even(number):
    """주어진 값이 짝수이면 True, 홀수이면 False를 반환합니다."""
    return number % 2 == 0

def calculate_average(numbers):
    """주어진 정수 list의 평균을 반환합니다."""
    if not numbers:
        return 0
    return sum(numbers) / len(numbers)

def find_max(numbers):
    """주어진 정수 list의 최댓값을 반환합니다."""
    if not numbers:
        return None
    return max(numbers)

def find_min(numbers):
    """주어진 정수 list의 최솟값을 반환합니다."""
    if not numbers:
        return None
    return min(numbers)