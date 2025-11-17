import pytest
from unittest.mock import mock_open, patch
from score import load_from_csv, subject_average, student_average


mock_csv_content = \
    """이름,국어,수학,영어,과학,사회
김철수,85,90,78,92,88
이영희,92,85,95,87,90
박민수,78,88,82,85,91"""


def test_load_from_csv():
    # Feed mocking data to the load_from_csv()
    with patch("builtins.open", mock_open(read_data=mock_csv_content)):
        scores, subjects = load_from_csv("mock_score_data.csv")
        assert len(scores) == 3
        assert scores["김철수"] == ["85", "90", "78", "92", "88"]
        assert scores["이영희"] == ["92", "85", "95", "87", "90"]
        assert scores["박민수"] == ["78", "88", "82", "85", "91"]

        assert len(subjects) == 5
        assert subjects[0] == "국어"
        assert subjects[1] == "수학"
        assert subjects[2] == "영어"
        assert subjects[3] == "과학"
        assert subjects[4] == "사회"


def test_student_average():
    # Feed mocking data to the load_from_csv()
    with patch("builtins.open", mock_open(read_data=mock_csv_content)):
        scores, _ = load_from_csv("mock_score_data.csv")
        avg = student_average(scores)
        assert len(avg) == 3
        assert avg[0] == ("이영희", 89.8)
        assert avg[1] == ("김철수", 86.6)
        assert avg[2] == ("박민수", 84.8)


def test_subject_average():
    # Feed mocking data to the load_from_csv()
    with patch("builtins.open", mock_open(read_data=mock_csv_content)):
        scores, subjects = load_from_csv("mock_score_data.csv")
        subject_avg = subject_average(scores, subjects)
        assert len(subject_avg) == 5
        # 소숫점 근사값 비교, +/- 0.1
        assert subject_avg["국어"] == pytest.approx(85.0, abs=0.1)
        assert subject_avg["수학"] == pytest.approx(87.6, abs=0.1)
        assert subject_avg["영어"] == pytest.approx(85.0, abs=0.1)
        assert subject_avg["과학"] == pytest.approx(88.0, abs=0.1)
        assert subject_avg["사회"] == pytest.approx(89.67, abs=0.1)
