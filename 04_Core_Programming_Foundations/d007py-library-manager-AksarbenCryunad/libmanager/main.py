
import os
from library.services.library_service import LibraryService
from library.models.book import Book

def log(msg: str):
    if os.environ.get("LIBRARY_MODE", "prod") == "dev":
        print(f"[DEV] {msg}")

def run_demo():
    svc = LibraryService()
    log("Library Manager started.")
    b1 = Book.from_dict({"title": "Python OOP", "author": "Alice", "year": 2023})
    b2 = Book.from_dict({"title": "Clean Code", "author": "Robert C. Martin", "year": 2008})
    svc.add_book(b1)
    svc.add_book(b2)
    found = svc.find_book("Python OOP")
    print(found)

if __name__ == "__main__":
    run_demo()
