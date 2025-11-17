import torch

# Model load
model = torch.hub.load("ultralytics/yolov5", "yolov5m")
model2 = torch.hub.load("ultralytics/yolov5", "yolov5s")


# Image to inference
img = "https://www.ultralytics.com/images/zidane.jpg"


# Do inference
result = model(img)
result2 = model2(img)



# Show result
result.show()
result2.show()