import torch
import sys
import json
from pathlib import Path
from models.experimental import attempt_load
from models.yolo import Model, DetectionModel


# IPEX pytorch model을 ONNX format으로 export
def export_onnx(state_dict_path, onnx_path, imgsz=640):
    checkpoint = torch.load(
        state_dict_path, map_location="cpu", weights_only=False)
    label_path = Path(onnx_path).with_suffix('.names.json')
    print("Checkpoint keys:", checkpoint.keys())

    if "model" in checkpoint and isinstance(checkpoint["model"], Model):
        print("Using checkpoint['model'] directly for export.")
        model = checkpoint["model"].float().eval()

        # Patch state_dict to avoid keep_vars issue
        def patched_state_dict(self, *args, **kwargs):
            return torch.nn.Module.state_dict(self, *args, **kwargs)
        model.state_dict = patched_state_dict.__get__(model, type(model))

        # Class name을 파일로 저장
        with open(label_path, "w") as f:
            json.dump(model.names, f)
    else:
        print("PyTorch checkpoint 파일이 아닙니다.")
        sys.exit(1)
    dummy = torch.randn(1, 3, imgsz, imgsz)
    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        opset_version=12,
        input_names=["images"],
        output_names=["output"],
        dynamic_axes={"images": {0: "batch"}, "output": {0: "batch"}},
    )
    print(f"모델 출력 경로: {onnx_path}")
    print(f"클래스 출력 경로: {label_path}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(
            f"사용법: python {sys.argv[0]} <IPEX_model> <출력파일이름>")
        sys.exit(1)
    input = sys.argv[1]
    output = sys.argv[2]
    export_onnx(input, output)
