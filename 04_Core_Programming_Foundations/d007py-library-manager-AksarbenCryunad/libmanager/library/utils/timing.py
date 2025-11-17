
from __future__ import annotations
from datetime import datetime
from functools import wraps
from typing import Callable, Any

def timeit(func: Callable[..., Any]) -> Callable[..., Any]:
    """함수 실행 시간을 출력하는 데코레이터."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        start = datetime.now()
        result = func(*args, **kwargs)
        duration = (datetime.now() - start).total_seconds()
        print(f"{func.__name__} took {duration:.6f}s")
        return result
    return wrapper
