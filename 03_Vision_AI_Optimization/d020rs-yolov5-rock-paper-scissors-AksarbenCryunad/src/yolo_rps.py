import torch
import cv2
import os
import json

# # Load trained model
# model_path = os.path.abspath("./best.pt")
# model = torch.hub.load("ultralytics/yolov5", "yolov5m")

# TODO: 커스텀 모델 로드
model_path = os.path.abspath("./rps_model.onnx")
model = torch.hub.load("ultralytics/yolov5", "custom", path = model_path)
# TODO: Label 로드
label_path = os.path.abspath("./rps_model.names.json")
with open(label_path, "r") as f:
    label_names = json.load(f)
    
# Permit r,p,s class
valid_classes = {"0","1","2"} # jason 안에서 r:0, p:1, s:2 라고 가정

# Video capture
cap = cv2.VideoCapture(0)

# Loop for camera frames
while True:
    # Read frame (BGR to RGB)
    ret, frame = cap.read()
    # break the loop on error
    if not ret:
        break

    # 추론 실행 (BGR -> RGB)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # TODO: 추론 전 입력 크기 보정 (640x640)
    rgb_frame = cv2.resize(rgb_frame, (640,640))
    
    results = model(rgb_frame)

    # TODO: 카메라 입력의 크기(frame_h, frame_w)와 모델의 입력 크기(input_h, input_w) 구하기
    frame_h, frame_w = frame.shape[:2]
    input_h, input_w = rgb_frame.shape[:2]
    # Boudning box 그리기
    for i, obj in enumerate(results.xyxy[0]):
        # 인식결과를 표시하기 위한 좌표를 얻음
        x1, y1, x2, y2, _, cls = map(int, obj)
        conf = obj[4]

        # TODO: 인식된 정확도(confidence)와 클래스를 label로 구성
        label = f"{label_names[f"{str(cls)}"]} {conf:.2f}"
        # TODO: 출력 바운딩박스 크기 조절
        # 카메라 프레임 크기 가져오기
        frame_h, frame_w = frame.shape[:2]
        # 모델 입력 크기 (resize한 640 x 640)
        input_h, input_w = 640, 640
        
        # 모델 좌표를 실제 프레임 좌표로 스케일링 
        x1 = int(x1 * frame_w / input_w)
        x2 = int(x2 * frame_w / input_w)
        y1 = int(y1 * frame_h / input_h)
        y2 = int(y2 * frame_h / input_h)
        
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
        break

cap.release()
cv2.destroyAllWindows()
