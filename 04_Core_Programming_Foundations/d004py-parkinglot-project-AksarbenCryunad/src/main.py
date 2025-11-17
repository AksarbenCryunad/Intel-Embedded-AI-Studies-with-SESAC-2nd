"""
Implement User Management System
"""

import re
import json

from enum import Enum

rows = int(
    input("[Admin] Enter your row: ")
)

cols = 10
park_info = [[0 for _ in range(cols)] for _ in range(rows)]

class ParkRole(Enum):
    PR_None     = 0x0001
    PR_Pregnant = 0x0002
    PR_Disabled = 0x0004

    def __str__(self):
        if (self.name == ParkRole.PR_None):
            return "일반 사용자"
        
        if (self.name == ParkRole.PR_Pregnant):
            return "임산부 사용자"
        
        if (self.name == ParkRole.PR_Disabled):
            return "장애인 사용자"
        
car_info = {
    9854: {'가입기간': '2025-08-25', "차량종류": "oil",      '할인율': 1.0},
    1524: {'가입기간': '2025-08-25', "차량종류": "oil",      '할인율': 0.1},
    8613: {'가입기간': '2025-07-19', "차량종류": "electric", '할인율': 0.2},
    9831: {'가입기간': '2024-04-18', "차량종류": "oil",      '할인율': 0.15},
    6942: {'가입기간': '2025-09-08', "차량종류": "oil",      '할인율': 0.15},
    1307: {'가입기간': '2025-05-02', "차량종류": "hybrid",   '할인율': 0.3},
    5278: {'가입기간': '2024-12-07', "차량종류": "oil",      '할인율': 0.1},
    7190: {'가입기간': '2024-03-29', "차량종류": "electric", '할인율': 0.25},
    3465: {'가입기간': '2024-09-12', "차량종류": "oil",      '할인율': 0.25},
    2801: {'가입기간': '2024-02-09', "차량종류": "oil",      '할인율': 0.2},
    4589: {'가입기간': '2024-11-23', "차량종류": "hybrid",   '할인율': 0.1},
    6210: {'가입기간': '2024-07-28', "차량종류": "electric", '할인율': 0.2}
}

park_car_info = {
    # 1669: {'차량종류': 'oil',       '입차시각': '09/01 10:54', "규칙": ParkRole.PR_None},
    # 7190: {'차량종류': 'hybrid',    '입차시각': '09/01 10:45', "규칙": ParkRole.PR_Disabled},
    # 3465: {'차량종류': 'hybrid',    '입차시각': '09/01 09:48', "규칙": ParkRole.PR_None},
    # 6210: {'차량종류': 'electric',  '입차시각': '09/01 09:20', "규칙": ParkRole.PR_Pregnant},
    # 5278: {'차량종류': 'oil',       '입차시각': '09/01 10:05', "규칙": ParkRole.PR_None},
    # 2801: {'차량종류': 'hybrid',    '입차시각': '09/01 10:59', "규칙": ParkRole.PR_None},
    # 8613: {'차량종류': 'oil',       '입차시각': '09/01 10:21', "규칙": ParkRole.PR_None},
    # 1307: {'차량종류': 'electric',  '입차시각': '09/01 09:08', "규칙": ParkRole.PR_None},
    # 9831: {'차량종류': 'hybrid',    '입차시각': '09/01 09:15', "규칙": ParkRole.PR_None},
    # 6942: {'차량종류': 'oil',       '입차시각': '09/01 09:57', "규칙": ParkRole.PR_Disabled},
    # 4589: {'차량종류': 'electric',  '입차시각': '09/01 10:33', "규칙": ParkRole.PR_None}
}

def is_empty(row, col):
    return bool(park_info[row][col] == 0)

def is_parked(car_num):
    return bool(car_num in park_car_info.keys())

def agent_sale_rate(car_num):    
    info = car_info.get(car_num)
    _car_info = park_car_info[car_num]
    car_category = _car_info["차량종류"]

    out_sale = info["할인율"] if info is not None else 0
    match car_category:
        case "oil":      out_sale += 0
        case "hybird":   out_sale += 0.05
        case "electric": out_sale += 0.1
        case _:          pass
        
    out_sale = 1 if out_sale > 1 else out_sale
    return out_sale

def check_date(date):
    pattern = re.compile(r'^(0[1-9]|1[0-2])/(0[1-9]|[12][0-9]|3[01]) ([01][0-9]|2[0-3]):([0-5][0-9])$')
    return pattern.match(date)

def check_role(car_num, row, col, user_role=None):
    # 장애인 지정 번호: 1, 2
    # 임산부 지정 번호: 9, 10
    user_role = park_car_info[car_num]["규칙"] if has_car(car_num) else user_role
    if (user_role == ParkRole.PR_None):
        if (col == 9 or col == 10):
            return False

        if (col == 1 or col == 2):
            return False
        
        return True
    elif (user_role == ParkRole.PR_Disabled):
        if (col == 9 or col == 10):
            return False
        
        return True
    elif (user_role == ParkRole.PR_Pregnant):
        if (col == 1 or col == 2):
            return False
        
        return True
    else:
        return True
    
def has_car(car_num):
    return car_num in park_car_info

def print_park_tower():
    print(" ", end="")
    for col in range(cols):
        print(f"   {col}", end="")
    print()
    
    for row in range(rows):
        print(f"{row + 1}F", end=" ")
        for col in range(cols):
            sign = " " if is_empty(row, col) else "X"
            p = '(' if col in [0, 1] else "{" if col in [8, 9] else "["
            e = ')' if col in [0, 1] else "}" if col in [8, 9] else "]"
            print(f"{p}{sign}{e}", end=" ")
        print()

def recommend_park_pos():
    for row in range(rows):
        for col in range(cols):
            if (is_empty(row, col)):
                return row, col

    return None, None

def cal_date(lhs_date: str, rhs_date: str):
    lhs_md, lhs_hm = lhs_date.split(" ")
    lhs_month, lhs_day = [int(lhs) for lhs in lhs_md.split("/")]
    lhs_hour, lhs_minute = [int(lhs) for lhs in lhs_hm.split(":")]

    rhs_md, rhs_hm = rhs_date.split(" ")
    rhs_month, rhs_day = [int(rhs) for rhs in rhs_md.split("/")]
    rhs_hour, rhs_minute = [int(rhs) for rhs in rhs_hm.split(":")]

    date_per_month = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    lhs_all_minitue = (lhs_minute) + (lhs_hour * 60) + (lhs_day * 24 * 60) + (sum([date_per_month[month_idx] for month_idx in range(lhs_month)]))
    rhs_all_minitue = (rhs_minute) + (rhs_hour * 60) + (rhs_day * 24 * 60) + (sum([date_per_month[month_idx] for month_idx in range(rhs_month)]))
    
    return (rhs_all_minitue - lhs_all_minitue)
    

def cmd_enter():
    car_num = int(input("4자리 차량번호를 입력해주세요: "))
    if not (car_num > 0 and car_num < 10000):
        return
    
    if is_parked(car_num):
        print(f"이미 차량이 주차되어 있습니다.")
        return
    
    is_pregnent = input("임산부는 할인 정책이 있습니다. Y/N")
    is_pregnent = True if is_pregnent == "Y" else False

    is_disabled = input("장애인은 할인 정책이 있습니다. Y/N")
    is_disabled = True if is_disabled == "Y" else False
    user_role = ParkRole.PR_Pregnant if is_pregnent else ParkRole.PR_Disabled if is_disabled else ParkRole.PR_None

    print(f"입차 시각을 입력해주세요. ex) 09/01 10:25")
    while True:
        car_date = input("입차 시각을 입력해주세요: ")
        if not (check_date(car_date)):
            print(f"입력 형식이 잘못되었습니다. ex) 09/01 10:25")
            continue

        break

    car_cate = input("차량 종류를 입력해주세요: ")

    while True:
        park_row = int(input(f"주차할 층을 입력해주세요: ex) 1~{rows}"))
        if not (park_row > 0 and park_row < (rows + 1)):
            print("유효하지 않은 층을 입력하였습니다.")
            continue

        park_col = int(input(f"주차할 번호를 입력해주세요: ex) 1~10"))
        if not (park_col > 0 and park_col < (10 + 1)):
            print("유효하지 않은 번호를 입력하였습니다.")
            continue
        
        if not (is_empty(park_row - 1, park_col - 1)):
            print("해당 층/번호에 이미 차량이 주차되어 있습니다.")
            recmd_row, recmd_col = recommend_park_pos()

            user_cmd = input(f"{recmd_row}층 {recmd_col}번호가 비어있습니다. 해당 위치에 주차하시겠습니까? Y/N: ")
            if (user_cmd == "Y"):
                park_info[recmd_row - 1][recmd_col - 1] = car_num
                park_car_info[car_num] = {
                    "차량종류": car_cate,
                    "입차시각": car_date,
                    "규칙": user_role,
                    "위치": [park_row, park_col]
                }

                return
        elif not (check_role(car_num, park_row - 1, park_col - 1)):
            print(f"{str(user_role)}는 해당 좌석을 사용할 수 없습니다.")
        else:
            park_info[park_row - 1][park_col - 1] = car_num
            park_car_info[car_num] = {
                "차량종류": car_cate,
                "입차시각": car_date,
                "규칙": user_role,
                "위치": [park_row - 1, park_col - 1]
            }

            print(f"{park_row}층 {park_col}번호에 주차가 완료되었습니다.")
            return

def cmd_exit():
    while True:
        car_num = int(input("출차할 차량 번호를 입력해주세요"))
        if not (has_car(car_num)):
            print(f"{car_num} 번호는 주차되어 있지 않습니다. 다시 선택해주세요.")
            continue

        while True:
            car_date = input("출차 시각을 입력해주세요: ")
            if not (check_date(car_date)):
                print(f"입력 형식이 잘못되었습니다. ex) 09/01 10:25")
                continue

            break
        
        park_all_min = cal_date(park_car_info[car_num]["입차시각"], car_date)
        park_hour = park_all_min // 60
        park_minite = park_all_min % 60

        agent_cost = 1 - agent_sale_rate(car_num)
        considering_agent_cost = park_hour + 1 if (park_all_min % 60) else park_hour
        considering_agent_cost = considering_agent_cost * 3000 * agent_cost
        
        pop_info = park_car_info.pop(car_num)
        row, col = pop_info["위치"]
        park_info[row][col] = 0
        
        print(f"총 주차 시간: {park_hour}시간 {park_minite}분 - 총 금액: {considering_agent_cost}")
        print(f"출차가 완료되었습니다.")

        return