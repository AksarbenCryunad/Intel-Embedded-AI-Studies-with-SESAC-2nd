
import builtins
import importlib
import types
import pytest

from library.models.book import Book

def test_book_count_increments(monkeypatch):
    # 새로 import하여 클래스 변수 초기화를 보장
    module = importlib.import_module('library.models.book')
    importlib.reload(module)
    BookLocal = module.Book

    # 초기 카운트가 없거나 0이라고 가정
    assert getattr(BookLocal, 'book_count', 0) in (0,)

    b1 = BookLocal(title='A', author='B', year=2024)
    b2 = BookLocal(title='C', author='D', year=2020)
    assert BookLocal.book_count == 2

def test_from_dict_and_str():
    b = Book.from_dict({'title': '책이름', 'author': '지은이', 'year': 2025})
    assert str(b) == '책이름 by 지은이 (2025)'
