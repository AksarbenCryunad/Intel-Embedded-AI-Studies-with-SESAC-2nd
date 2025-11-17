import assignment

def test_name_type():
    assert isinstance(assignment.name, str)

def test_age_type():
    assert isinstance(assignment.age, int)

def test_numbers_content():
    assert 2 in assignment.numbers
    assert len(assignment.numbers) == 3

def test_is_student():
    assert assignment.is_student is True