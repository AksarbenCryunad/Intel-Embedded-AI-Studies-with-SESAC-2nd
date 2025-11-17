import sys
import nltk
from collections import Counter
from nltk.tokenize import word_tokenize


def get_word_list(raw_text: str) -> list[str]:
    """문장부호를 제외한 문자열들을 골라서 리스트로 반환"""
    word_list = [word for word in word_tokenize(raw_text) if word.isalnum()]
    return word_list


def download_nltk_data():
    """ Download tokenizers if not exist."""
    try:
        nltk.data.find('tokenizers/punkt')
    except LookupError:
        nltk.download('punkt')

    try:
        nltk.data.find('tokenizers/punkt_tab')
    except LookupError:
        nltk.download('punkt_tab')


download_nltk_data()

text_file = sys.argv[1]
with open(text_file) as f:
    # 모든 단어들을 소문자로 변경
    text = f.read().lower()
    wl = get_word_list(text)

    # Counter를 이용한 단어 수 세기
    cnt = Counter(wl)

    # 전체 단어 카운트 수.
    total_word_cnt = sum(cnt.values())
    top_10 = cnt.most_common(10)

    # 화면 출력.
    print(f"Top 10 frequent words:")
    
    for i in range(10):

        k = top_10[i][0]
        v = top_10[i][1]
        avg = v / total_word_cnt
        print(f"\t{i+1}. {k} ({v} - Avg. {avg:.2f})")
