# YOLOv5 Finetuning

YOLOv5를 finetune하고 결과물을 slide로 작성 하세요


## Dataset
* Dataset 선택​
  * 미리 작성된 YOLOv5용 dataset​
    e.g.) Roboflow, Kaggle 등​

 * Inference 방법​ 선택
   * Camera live stream
   * Pre-recorded video
   * 사진 등.
```
# Load video​
video_file = os.path.expanduser("~/Downloads/video.mp4")​
cap = cv2.VideoCapture(video_file)
```

 * Slide 포함내용​
   * Dataset 소개​
   * 사용된 hyper parameters​
   * Training 결과​
   * Inference 예제