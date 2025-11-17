import torch
import cv2
from threading import Thread, Event


# Background Thread 동작을 위한 함수
def inference(model, input_frames, inference_results, stop_event):
    while not stop_event.is_set():
        if input_frames:
            # TODO: 가장 마지막 프레임을 하나 꺼내서 추론을 수행하고 결과를
            # inference_results queue에 넣는다.
            # 1. input_frames 리스트에서 가장 마지막 프레임((last_frame)가장 최신 프레임)을 가져옵니다.
            # pop() 메서드는 리스트의 마지막 요소를 제거하고 그 값을 반환합니다.
            last_frame = input_frames.pop()
            # 2. 가져온 프레임을 YOLOv5 모델에 넣어 추론을 수행합니다.
            inferenced_result = model(last_frame)
            # 3. 추론 결과를 inference_results 리스트에 추가합니다.
            inference_results.append(inferenced_result)


# Model​
model = torch.hub.load("ultralytics/yolov5", "yolov5m")

# Video capture
cap = cv2.VideoCapture(0)

# 추론 thread와 main thread간의 통신 queue
input_frames = []
inference_results = []

# Thread stop을 위한 Event 객체
stop_event = Event()

# TODO: Thread 객체를 생성하고 시작
inference_thread = Thread(target=inference, args=(model, input_frames, inference_results, stop_event))
inference_thread.daemon = True
inference_thread.start()
# Trick, 한번 설정되면 다음 업데이트 값을 유지한다.
results = None

# Loop for camera frames
while True:
    # Read frame (BGR to RGB)
    ret, frame = cap.read()
    if not ret:
        break

    # TODO: 추론을 수행할 데이터를 input_frames queue에 넣는다
    input_frames.append(frame)
    # TODO: inference_results queue를 검사해서 결과물을 출력
    if inference_results:
        results = inference_results.pop(0)
    # TODO: 추론 결과가 있으면 Boudning box 그리기
    if results is not None:
        for i, obj in enumerate(results.xyxy[0]):
            # 인식결과를 표시하기 위한 좌표를 얻음
            x1, y1, x2, y2, _, cls = map(int, obj)

            # 인식된 정확도(confidence)와 클래스를 label로 구성
            conf = obj[4]
            label = f"{model.names[cls]} {conf:.2f}"

            # OpenCV를 이용해서 해당 좌표에 사각형과 text를 출력
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"Object {i}: {label} at [{x1}, {y1}, {x2}, {y2}]")

    # 화면 표시
    cv2.imshow("YOLOv5", frame)

    # 종료를 위한 key 처리
    key = cv2.waitKey(20) & 0xFF
    if key == 27:
        # Background Thread 종료
        stop_event.set()
        break

# TODO: Thread 종료를 대기함
inference_thread.join()
cap.release()
cv2.destroyAllWindows()
