"""
Implement User Management System
"""
import re  
from datetime import datetime  

# 초기 사용자 목록 생성
users = {
    '3': {
        'name': '김뷰어',
        'date_of_birth': '2000-01-01',
        'password': 'password_123!',
        'role': 'viewer'
    },
    '2': {
        'name': '박에디터',
        'date_of_birth': '1995-05-15',
        'password': 'password_456@',
        'role': 'editor'
    },
    '1': {
        'name': '이관리자',
        'date_of_birth': '1980-11-20',
        'password': 'password_789#',
        'role': 'admin'
    }
}

# 프로그램 실행 시 전체 사용자 목록 출력
print("\n### 현재 사용자 목록 ###")
for user_id, info in users.items():
    print(f"ID: {user_id}, 이름: {info['name']}, 역할: {info['role']}")
print("-" * 25)

logged_in_id = None
logged_in_role = None

while True:
    print("\n--- 메뉴 ---")
    print("1. 회원가입")
    print("2. 로그인")
    print("3. 종료")
    choice = input("선택하세요:(1: 회원가입, 2: 로그인, 3: 종료) ")

     # 1. 회원가입
    if choice == '1':
        print("### 회원가입 ###")
        while True:
            user_id = input("새로운 아이디를 입력하세요: ")
            if user_id in users:
                print("이미 존재하는 아이디입니다. 다른 아이디를 선택해주세요.")
            else:
                break

        name = input("이름을 입력하세요: ")
        while True:
            date_of_birth = input("생년월일(YYYY-MM-DD)을 입력하세요: ")
            try:
                datetime.strptime(date_of_birth, '%Y-%m-%d')
                break
            except ValueError:
                print("유효하지 않은 날짜 형식입니다. YYYY-MM-DD 형식으로 입력해주세요.")
        
        while True:
            password = input("비밀번호를 입력하세요(비밀번호는 10자 이상이며, 숫자, 대문자, 소문자, 특수문자를 모두 포함해야 합니다): ")
            if (len(password) >= 10 and 
                re.search("[a-z]", password) and 
                re.search("[A-Z]", password) and 
                re.search("[0-9]", password) and 
                re.search("[!@#$%^&*(),.?\":{}|<>]", password)):
                break
            else:
                print("다시 입력하세요. 비밀번호는 10자 이상이며, 숫자, 대문자, 소문자, 특수문자를 모두 포함해야 합니다.")
        
        # 새롭게 추가된 역할 선택 로직
        while True:
            print("\n어떤 역할로 가입하시겠습니까?")
            print("1. viewer")
            print("2. editor")
            role_choice = input("선택 (1. viewer 2. editor): ")
            if role_choice == '1':
                new_role = 'viewer'
                break
            elif role_choice == '2':
                new_role = 'editor'
                break
            else:
                print("잘못된 선택입니다. 다시 선택해주세요.")

        new_user = {
            'name': name,
            'date_of_birth': date_of_birth,
            'password': password,
            'role': new_role # 사용자가 선택한 역할로 할당
        }
        users[user_id] = new_user
        print("회원가입이 완료되었습니다! 🎉")
        
        print("\n### 현재 사용자 목록 ###")
        for id, info in users.items():
            print(f"ID: {id}, 이름: {info['name']}, 역할: {info['role']}")
        print("-" * 25)

        # 2. 로그인
    elif choice == '2':
        print("\n### 로그인 ###")
        user_id = input("아이디를 입력하세요: ")
        password = input("비밀번호를 입력하세요: ")
        
        # 로그인 실패 시 메시지를 출력하고, while True 루프의 시작으로 돌아갑니다.
        if user_id in users and users[user_id]['password'] == password:
            logged_in_id = user_id
            logged_in_role = users[user_id]['role']
            print(f"로그인 성공! 환영합니다, {users[user_id]['name']}님!")
            
            # 로그인 후 메뉴
            while logged_in_id:
                print("\n--- 로그인 후 메뉴 ---")
                if logged_in_role == 'viewer':
                    print("1. 정보 수정")
                    print("2. 계정 삭제")
                    print("3. 로그아웃")
                    action = input("선택하세요(1. 정보 수정 2. 계정 삭제 3. 로그아웃): ")
                else: # editor, admin
                    print("1. 사용자 정보 수정")
                    print("2. 사용자 계정 삭제")
                    print("3. 로그아웃")
                    action = input("선택하세요(1.사용자 정보 수정 2.사용자 계정 삭제 3.로그아웃): ")

                # 1. 정보 수정/사용자 정보 수정
                if action == '1':
                    if logged_in_role == 'viewer':
                        target_id = logged_in_id
                        print(f"당신은 본인의 정보({target_id})만 수정할 수 있습니다.")
                    else: # editor, admin
                        target_id = input("정보를 수정할 사용자의 ID를 입력하세요: ")
                        if target_id not in users:
                            print("사용자를 찾을 수 없습니다.")
                            continue

                    new_name = input("새로운 이름을 입력하세요 (변경하지 않으려면 Enter): ")
                    if new_name:
                        users[target_id]['name'] = new_name
                    
                    print("사용자 정보가 수정되었습니다.")
                    print("\n### 현재 사용자 목록 ###")
                    for id, info in users.items():
                        print(f"ID: {id}, 이름: {info['name']}, 역할: {info['role']}")
                    print("-" * 25)

                # 2. 계정 삭제/사용자 계정 삭제
                elif action == '2':
                    if logged_in_role in ['viewer', 'editor']:
                        target_id = logged_in_id
                        confirm = input(f"정말로 본인의 계정({target_id})을 삭제하시겠습니까? (yes/no): ")
                        if confirm.lower() == 'yes':
                            del users[target_id]
                            print("계정이 삭제되었습니다.")
                            logged_in_id = None # 로그아웃 처리
                        else:
                            print("계정 삭제가 취소되었습니다.")
                    elif logged_in_role == 'admin':
                        target_id = input("삭제할 사용자의 ID를 입력하세요: ")
                        if target_id not in users:
                            print("사용자를 찾을 수 없습니다.")
                            continue
                        
                        if target_id == logged_in_id:
                            print("본인(관리자) 계정은 삭제할 수 없습니다.")
                            continue

                        confirm = input(f"{target_id}의 계정을 정말로 삭제하시겠습니까? (yes/no): ")
                        if confirm.lower() == 'yes':
                            del users[target_id]
                            print("사용자 계정이 삭제되었습니다.")
                        else:
                            print("계정 삭제가 취소되었습니다.")
                    
                    print("\n### 현재 사용자 목록 ###")
                    for id, info in users.items():
                        print(f"ID: {id}, 이름: {info['name']}, 역할: {info['role']}")
                    print("-" * 25)

                # 3. 로그아웃
                elif action == '3':
                    print("로그아웃되었습니다.")
                    logged_in_id = None
                
                else:
                    print("잘못된 선택입니다.")
        else: # 로그인 실패 시 이 부분이 실행됩니다.
            print("아이디 또는 비밀번호가 올바르지 않습니다.")


    # 3. 종료
    elif choice == '3':
        print("프로그램을 종료합니다.")
        break
    
    else:
        print("잘못된 선택입니다. 다시 시도해주세요.")