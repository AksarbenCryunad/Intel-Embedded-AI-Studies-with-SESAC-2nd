import ast
import io
import os
import runpy
import types
import builtins
import re
import pytest


TARGET_FILE = "assignment_drama_template.py"
SCREENSHOT_FILE = "screenshot.png"


class InputFeeder:
    def __init__(self, answers):
        self.answers = list(answers)
        self.last = answers[-1] if answers else ""

    def __call__(self, prompt=""):
        if self.answers:
            return self.answers.pop(0)
        return self.last  # 추가로 호출되면 마지막 값을 반복 반환


def load_source():
    with open(TARGET_FILE, "r", encoding="utf-8") as f:
        return f.read()


def exec_module_with_inputs(inputs):
    feeder = InputFeeder(inputs)
    original_input = builtins.input
    try:
        builtins.input = feeder
        # 출력 캡처 (형식 검사는 선택)
        buf = io.StringIO()
        original_stdout = os.sys.stdout
        os.sys.stdout = buf
        try:
            g = runpy.run_path(TARGET_FILE, run_name="__main__")
        finally:
            os.sys.stdout = original_stdout
        return g, buf.getvalue()
    finally:
        builtins.input = original_input


def test_no_control_flow_or_defs():
    src = load_source()
    tree = ast.parse(src)

    banned_nodes = (
        ast.If, ast.For, ast.While, ast.Try, ast.With,
        ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef,
        ast.Match,  # Python 3.10 패턴매칭
        ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp,
    )

    offenders = []

    class Visitor(ast.NodeVisitor):
        def generic_visit(self, node):
            if isinstance(node, banned_nodes):
                offenders.append(type(node).__name__)
            super().generic_visit(node)

    Visitor().visit(tree)

    assert not offenders, f"금지 문법 사용 발견: {', '.join(offenders)}"


def test_remove_placeholders_in_source():
    src = load_source()
    assert "IMPLEMENT ME" not in src, "소스에 'IMPLEMENT ME' 플레이스홀더가 남아 있습니다. 모두 제거하십시오."


def test_minimum_inputs_required_in_source():
    src = load_source()
    tree = ast.parse(src)

    input_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "input"
    ]

    count = len(input_calls)
    assert count >= 4, f"실제 input() 호출이 {count}회입니다. 최소 4회 이상이어야 합니다."


def is_quoted_nonempty(s: str) -> bool:
    if not isinstance(s, str):
        return False
    s2 = s.strip()
    return len(s2) >= 2 and s2[0] == '"' and s2[-1] == '"' and s2 not in ('""', '" "')


@pytest.mark.parametrize("answers", [["새제목", "수정제목", "장르", "주제"]])
def test_run_and_validate_values(answers):
    g, _out = exec_module_with_inputs(answers)

    # 존재
    for name in ("drama1", "drama2", "drama3"):
        assert name in g, f"{name} 변수가 정의되어 있지 않습니다."

    # 구조/값
    for dname in ("drama1", "drama2", "drama3"):
        d = g[dname]
        assert isinstance(d, dict), f"{dname}은 dict여야 합니다."

        required = ["제목", "장르", "주제", "방영기간", "배우", "명대사"]
        for k in required:
            assert k in d, f"{dname}에 키 '{k}'가 없습니다."
            v = d[k]
            assert v not in ("", None, []), f"{dname}['{k}'] 값이 비어 있습니다."

        # 배우: 리스트 + 모든 요소 문자열
        assert isinstance(d["배우"], list), f"{dname}['배우']는 list여야 합니다."
        assert all(isinstance(x, str) and x.strip() != "" for x in d["배우"]), \
            f"{dname}['배우']의 모든 요소는 빈 문자열이 아닌 str이어야 합니다."

        # 명대사: 양쪽 큰따옴표 포함 + 내용 비어있지 않음
        assert is_quoted_nonempty(d["명대사"]), f"{dname}['명대사']는 양쪽 큰따옴표로 감싸고 내용이 있어야 합니다. 현재: {d['명대사']!r}"



def test_print_blocks_and_nonempty_values():
    _g, out = exec_module_with_inputs(["새제목", "수정제목", "장르", "주제"])

    sections = ["[드라마 1]", "[드라마 2]", "[드라마 3]"]
    for header in sections:
        assert header in out, f'출력에 "{header}" 섹션 헤더가 없습니다.'

    required_lines = ["제목:", "장르:", "주제:", "방영기간:", "배우:", "명대사:"]
    for header in sections:
        seg_start = out.index(header)
        seg = out[seg_start: seg_start + 400]  
        for key in required_lines:
            assert key in seg, f"{header} 섹션에 '{key}' 라인이 없습니다."
            line_match = re.search(rf"{re.escape(key)}\s*(.+)", seg)
            assert line_match, f"{header} 섹션의 '{key}' 라인을 파싱할 수 없습니다."
            content = line_match.group(1).strip()
            assert content not in ("", "[]", '""'), f"{header} 섹션의 '{key}' 값이 비어 있습니다."


def test_screenshot_exists():
    assert os.path.exists(SCREENSHOT_FILE), f"{SCREENSHOT_FILE} 파일(실행결과 캡처본)이 제출되지 않았습니다."
    assert os.path.getsize(SCREENSHOT_FILE) > 0, f"{SCREENSHOT_FILE} 파일이 비어있습니다."