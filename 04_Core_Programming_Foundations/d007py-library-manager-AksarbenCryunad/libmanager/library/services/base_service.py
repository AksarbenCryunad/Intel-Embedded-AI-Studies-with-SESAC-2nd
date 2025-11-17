
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Iterable
from library.models.book import Book

class BaseService(ABC):
    """서비스 공통 인터페이스.
    TODO: 아래 메서드를 하위 클래스에서 구현하도록 하세요.
    """

    @abstractmethod
    def add_book(self, book: Book) -> None:
        ...

    @abstractmethod
    def remove_book(self, title: str) -> None:
        ...

    @abstractmethod
    def list_books(self) -> Iterable[Book]:
        ...

    @abstractmethod
    def find_book(self, title: str) -> Book:
        ...
