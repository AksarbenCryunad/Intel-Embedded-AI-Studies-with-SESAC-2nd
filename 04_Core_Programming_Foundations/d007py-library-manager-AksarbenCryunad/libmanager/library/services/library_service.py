
from __future__ import annotations
from typing import Iterable, List
from library.models.book import Book
from library.services.base_service import BaseService


class LibraryService(BaseService):
    """도서 목록을 메모리에서 관리하는 서비스.
    TODO:
      - 내부 상태를 캡슐화하기 위해 _books(list[Book])를 사용
      - add_book/remove_book/list_books/find_book 구현
      - 존재하지 않는 책 삭제/검색 시 ValueError 발생
    """

    def __init__(self) -> None:
        self._books: List[Book] = []

    def add_book(self, book: Book) -> None:
        self._books.append(book)

    def remove_book(self, title: str) -> None:
        for i, book in enumerate(self._books):
            if book.title == title:
                del self._books[i]
                return
        raise ValueError(f"책 '{title}'을(를) 찾을 수 없습니다.")

    def list_books(self) -> Iterable[Book]:
        return list(self._books)

    def find_book(self, title: str) -> Book:
        for book in self._books:
            if book.title == title:
                return book
        raise ValueError(f"책 '{title}'을(를) 찾을 수 없습니다.")
