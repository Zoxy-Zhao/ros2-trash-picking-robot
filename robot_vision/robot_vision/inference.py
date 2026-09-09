"""Explicit model-family adapters for original YOLOv5 and Ultralytics models."""
from pathlib import Path


class Detector:
    def __init__(self, model_path, family='yolov5', repository='', device='0', imgsz=640):
        path = Path(model_path).expanduser().resolve()
        if not path.is_file() or path.suffix not in ('.pt', '.engine'):
            raise ValueError('Supply an existing .pt or .engine model')
        if imgsz <= 0 or imgsz % 32:
            raise ValueError('imgsz must be a positive multiple of 32')
        self.family, self.device, self.imgsz = family, device, imgsz
        self.path = path
        if family == 'yolov5':
            import torch
            repo = Path(repository).expanduser().resolve()
            if not (repo / 'hubconf.py').is_file():
                raise ValueError('yolov5_repo must point to a trusted local YOLOv5 checkout')
            self.model = torch.hub.load(str(repo), 'custom', path=str(path),
                                        source='local', device=device)
        elif family == 'ultralytics':
            from ultralytics import YOLO
            self.model = YOLO(str(path), task='detect')
        else:
            raise ValueError('model_family must be yolov5 or ultralytics')

    def predict(self, bgr):
        """Return (class name, confidence, xmin, ymin, xmax, ymax) records."""
        if self.family == 'yolov5':
            # AutoShape expects RGB; the ROS/OpenCV camera supplies BGR.
            result = self.model(bgr[:, :, ::-1].copy(), size=self.imgsz)
            return [(str(result.names[int(row[5])]), float(row[4]),
                     *map(int, row[:4])) for row in result.xyxy[0].cpu().numpy()]
        result = self.model.predict(bgr, imgsz=self.imgsz, device=self.device, verbose=False)[0]
        if result.boxes is None:
            return []
        return [(str(result.names[int(row[5])]), float(row[4]),
                 *map(int, row[:4])) for row in result.boxes.data.cpu().numpy()]
