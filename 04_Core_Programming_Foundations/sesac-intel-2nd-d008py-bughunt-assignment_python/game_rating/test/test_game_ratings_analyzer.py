import pytest
import game_ratings_analyzer



def test_normalize_path():
    assert "path/path2" == game_ratings_analyzer.normalize_path("path/path2")
    assert "path/path2" == game_ratings_analyzer.normalize_path("path\\path2")

def test_is_valid_rating():
    assert True == game_ratings_analyzer.is_valid_rating(8)
    assert True == game_ratings_analyzer.is_valid_rating(0)
    assert False == game_ratings_analyzer.is_valid_rating(True)
    assert False == game_ratings_analyzer.is_valid_rating("8")

def test_is_tie():
    test_data = [["title1", 10], ["title2", 10], ["title3", 10]]
    test_data2 = [["title1", 10], ["title2", 0], ["title3", 10]]
    assert True == game_ratings_analyzer.is_tie(test_data)
    assert False == game_ratings_analyzer.is_tie(test_data2)

def test_read_ratings():
    


def test_generate_report():
    data = {'Elden Ring': [10, 3],
            "Baldur's Gate 3": [9, 4],
            'League of Legends': [8, 5],
            'Cyberpunk 2077': [7, 6],
            'Age of Empires IV': [8, 7],
            'Valorant': [9, 8],
            'Palworld': [8, 9],
            'The Sims 4': [9, 1],
            'Counter-Strike 2': [9],
            'Stardew Valley': [10]}
    assert True == game_ratings_analyzer.generate_report(data, 10)
    assert False == game_ratings_analyzer.generate_report(data, 11)