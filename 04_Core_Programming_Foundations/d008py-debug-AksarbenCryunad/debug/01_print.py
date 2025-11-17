import random

def find_max(numbers:list)->int:
    max_num = 0
    for num in numbers:
        if num > max_num:
            print(f"{max_num}이 새로운 값 {num} 보다 작아서 업데이트 합니다")
            max_num = num
    return max_num

num_list = [random.randint(-50, 3) for _ in range(5)]
print("Numbers: ", num_list)
m = find_max(num_list)
print("Maximum value: ", m)