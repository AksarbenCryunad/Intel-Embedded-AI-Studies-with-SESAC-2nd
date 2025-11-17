
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class Book:
    """책 정보를 표현하는 데이터 클래스.
    TODO: 아래 요구사항을 만족하도록 구현을 보완하세요.
    - 인스턴스 변수: title(str), author(str), year(int)
    - 클래스 변수: book_count(int) — 생성될 때마다 +1
    - __str__는 "{title} by {author} ({year})" 형식 반환
    - @classmethod from_dict(cls, data: Dict[str, Any]) -> Book 구현
    """
    title: str
    author: str
    year: int

    book_count: int = 0
    

    # TODO: 클래스 변수 book_count 선언 및 증가 로직 추가
    # TODO: 생성 시 book_count 증가
    # book_count: int = 0  # 힌트: dataclass의 필드가 아닌 클래스 속성으로 선언
    def __init__(self, title: str, author: str, year: int):
        self.title = title
        self.author = author
        self.year = year
        type(self).book_count += 1

    def __str__(self) -> str:
        # TODO: 지정된 문자열 포맷 반환
        # e.g. 책이름 by 지은이 (2001)
        return f"{self.title} by {self.author} ({self.year})"

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Book":
        # TODO: dict에서 title/author/year를 읽어 Book 생성
        return cls(
            title=data['title'],
            author=data['author'],
            year=data['year']
        )
