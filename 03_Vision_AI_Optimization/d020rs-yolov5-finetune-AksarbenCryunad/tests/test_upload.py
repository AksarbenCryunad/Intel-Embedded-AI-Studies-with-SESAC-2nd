import os


def test_pdf_submitted():
    report_path = os.path.abspath("report.pdf")
    assert os.path.exists(report_path), "report.pdf  is missing"
