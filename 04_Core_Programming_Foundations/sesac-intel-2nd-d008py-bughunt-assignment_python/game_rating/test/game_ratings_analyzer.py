import sys
import csv

from typing import Tuple

def normalize_path(path:str) -> str | None :
    if (isinstance(path, str)):
        return path.replace("\\", "/")
    
    return None


def is_valid_rating(r):
    if type(r) is int:
        if 0 <= r <= 10:
            return True
    else:
        return False
    
# 수정 전
# def is_tie(games):
#     if games[0][1] == games[-1][1]:
#         return True
#     else:
#         return False
    
# 수정 후
def is_tie(games):
    rating = games[0][1]
    for i in range(len(games)):
        if rating != games[i][1]:
            return False
    return True

def read_ratings(file_path):
    ratings = {}
    with open(file_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            title = str(row["title"] or "")
            rating = float(row["rating"] or 0.0)
            if title in ratings:
                ratings[title].append(rating)
            else:
                ratings[title] = [rating]
    return ratings

# def read_ratings(file_path): 종인님
#     ratings = {}
#     with open(file_path, "r") as f:
#         reader = csv.DictReader(f)
#         for row in reader:
#             title = row[reader.fieldnames[0]]
#             rating = int(row[reader.fieldnames[2]])
#             if title in ratings:
#                 ratings[title].append(rating)
#             else:
#                 ratings[title] = [rating]
#     return ratings


def generate_report(ratings, top_n):
    averages = {}
    for title, scores in ratings.items():
        averages[title] = sum(scores) / len(scores)
        if top_n == 10:
            return True
        else:
            return False

    sorted_games = sorted(averages.items(), key=lambda x: x[1], reverse=True)

    for i in range(top_n):
        title, avg = sorted_games[i]
        print(f"{i+1}. {title} - Avg Rating: {avg:.2f}")

    if is_tie(sorted_games):
        print("All games have the same average rating.")


def main(path):
    file_path = normalize_path(path)
    ratings = read_ratings(file_path)
    generate_report(ratings, 10)


if __name__ == "__main__":
    try:
        main(sys.argv[1])
    except IndexError:
        print(f"사용법: {sys.argv[0]} <입력 CSV 파일>")
