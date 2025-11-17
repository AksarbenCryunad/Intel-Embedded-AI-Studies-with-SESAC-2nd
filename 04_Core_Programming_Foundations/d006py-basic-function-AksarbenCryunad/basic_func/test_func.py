from basic_func import add, sub, mul, div, power, square, greet


def test_add():
    assert 12 == add(10, 2)
    assert -8 == add(-10, 2)
    assert 10 == add(10, 0)
    assert 2 == add(0, 2)


def test_sub():
    assert 8 == sub(10, 2)
    assert -12 == sub(-10, 2)
    assert 10 == sub(10, 0)
    assert -2 == sub(0, 2)


def test_mul():
    assert 20 == mul(10, 2)
    assert -20 == mul(-10, 2)
    assert 0 == mul(10, 0)
    assert 0 == mul(0, 2)


def test_div():
    assert 5.0 == div(10, 2)
    assert -5.0 == div(-10, 2)
    # ZeroDivisionError
    # assert 0 == div(10, 0)
    assert 0 == div(0, 2)


def test_power():
    assert 100 == power(10, 2)
    assert 100 == power(-10, 2)
    assert 1 == power(10, 0)
    assert 0 == power(0, 2)


def test_square():
    assert 100 == square(10)
    assert 100 == square(-10)
    assert 1 == square(1)
    assert 0 == square(0)


def test_greet():
    assert "안녕하신가 낯선자!" == greet()
    assert "안녕하십니까 마법사!" == greet(이름="마법사", 나이=50)
    assert "안녕 낯선자!" == greet(나이=4)
