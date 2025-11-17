# YOLOv5 ROS2 Finetune (CUDA)

## Training
### CUDA Training
```
python3 -m venv .venv_cuda
(.venv_cuda) source .venv/bin/activate
(.venv_cuda) pip install -r requirements_cuda.txt
(.venv_cuda) python -m torch.distributed.run --nproc_per_node=2  \​
  ./train.py --img 640 \
  --batch ? \
  --epochs ? \
  --patience ? \
  --data <dataset_path>/data.yaml \
  --weights yolov5m.pt
```
