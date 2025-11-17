
import pytest
from library.models.book import Book
from library.services.library_service import LibraryService

def make_books():
    return [
        Book.from_dict({'title': '책이름', 'author': '지은이', 'year': 2000}),
        Book.from_dict({'title': '다른 책이름', 'author': '김지은', 'year': 2001}),
    ]

def test_add_and_list_books():
    svc = LibraryService()
    for b in make_books():
        svc.add_book(b)
    books = list(svc.list_books())
    assert len(books) == 2
    assert books[0].title == '책이름'

def test_find_book_success_and_fail():
    svc = LibraryService()
    svc.add_book(Book.from_dict({'title': 'X', 'author': 'Y', 'year': 2000}))
    found = svc.find_book('X')
    assert found.author == 'Y'
    with pytest.raises(ValueError):
        svc.find_book('NOPE')

def test_remove_book():
    svc = LibraryService()
    b1, b2 = make_books()
    svc.add_book(b1)
    svc.add_book(b2)
    svc.remove_book('책이름')
    books = list(svc.list_books())
    assert len(books) == 1
    assert books[0].title == '다른 책이름'
    with pytest.raises(ValueError):
        svc.remove_book('Not Exists')
