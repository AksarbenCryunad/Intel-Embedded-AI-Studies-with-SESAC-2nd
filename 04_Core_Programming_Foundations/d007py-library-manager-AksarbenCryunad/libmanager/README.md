
# Library Manager

과제 목표:
- 모듈/패키지 구조 설계
- OOP 원칙 적용 (추상화/상속/캡슐화/다형성)
- `@staticmethod`/`@classmethod`
- VS Code 디버깅, 환경변수(`os.environ`), 시간 측정(`datetime`) 실습

## 프로젝트 구조
```
project/
├── library/
│   ├── __init__.py
│   ├── models/
│   │   ├── __init__.py
│   │   └── book.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── base_service.py
│   │   └── library_service.py
│   └── utils/
│       ├── __init__.py
│       └── timing.py
├── tests/
│   ├── __init__.py
│   ├── test_book.py
│   └── test_library.py
└── main.py
```

## 시작하기
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pytest
```

## 과제
- `library/models/book.py`, `library/services/base_service.py`, `library/services/library_service.py`를 TODO에 맞게 구현하세요.
- 모든 테스트가 통과해야 합니다.
- tests_book.py test_library.py를 수정하지 마세요.

## 환경 변수
- `LIBRARY_MODE=dev` 인 경우 디버그 로그가 출력됩니다.
