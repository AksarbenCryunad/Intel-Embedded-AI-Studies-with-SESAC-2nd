import cv2
import torch

# Model load
model = torch.hub.load("ultralytics/yolov5", "yolov5s")


# camera
cap = cv2.VideoCapture(0)

# Video frame
ret, bgr_frame = cap.read()
#bgr -> rgb 로 변환해줘야함 ( model 은 rgb 를 받음 )
rgb_image = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)



# Do inference
result = model(rgb_image)



# Show result
result.show()

cap.release()
cv2.destroyAllWindows()