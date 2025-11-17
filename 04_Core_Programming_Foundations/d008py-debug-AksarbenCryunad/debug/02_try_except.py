def square(value:int) -> int:
    s = 0
    try:
        s = value ** 2
    except TypeError as e:
        print("Type error 발생!", end="")
        print(e)
        s = 0
    return s

print(square("20"))

def divide(a:int, b:int) -> float:
    f = 0.0
    try:
        f = a / b
    except ZeroDivisionError:
        print("Exception, setting zero.")
        f = 0.0
    return f

print(divide(10, 0))