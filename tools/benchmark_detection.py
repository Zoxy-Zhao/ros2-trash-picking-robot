"""Measure real video frames, including preprocessing/inference/postprocessing.

This is offline detector throughput, not live camera-to-ROS throughput.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_vision'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--video', required=True)
    parser.add_argument('--family', choices=['yolov5', 'ultralytics'], default='yolov5')
    parser.add_argument('--yolov5-repo', default='~/yolov5')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='0')
    parser.add_argument('--warmup', type=int, default=30)
    parser.add_argument('--frames', type=int, default=300)
    parser.add_argument('--output', default='output/benchmark.json')
    args = parser.parse_args()
    if args.frames < 1 or args.warmup < 0:
        parser.error('frames must be positive and warmup non-negative')
    import cv2
    import numpy as np
    import torch
    from robot_vision.inference import Detector
    detector = Detector(args.model, args.family, args.yolov5_repo, args.device, args.imgsz)
    capture = cv2.VideoCapture(str(Path(args.video).expanduser().resolve()))
    if not capture.isOpened():
        raise RuntimeError('Cannot open video')
    durations = []
    counts = []
    def sync():
        if args.device != 'cpu' and torch.cuda.is_available():
            torch.cuda.synchronize(torch.device('cuda:' + args.device.replace('cuda:', '')))
    try:
        for index in range(args.warmup + args.frames):
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError('Video shorter than requested warmup + measured frames')
            sync()
            begin = time.perf_counter()
            records = detector.predict(frame)
            sync()
            elapsed = time.perf_counter() - begin
            if index >= args.warmup:
                durations.append(elapsed)
                counts.append(len(records))
    finally:
        capture.release()
    samples = np.array(durations)
    report = {
        'measured_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'offline video; preprocess + inference + postprocess + CPU results; excludes decode, ROS and display',
        'model': str(detector.path),
        'model_sha256': hashlib.sha256(detector.path.read_bytes()).hexdigest(),
        'model_family': args.family, 'video': str(Path(args.video).resolve()),
        'imgsz': args.imgsz, 'warmup_frames': args.warmup, 'measured_frames': len(samples),
        'platform': platform.platform(), 'python': platform.python_version(),
        'torch': torch.__version__, 'cuda': torch.version.cuda,
        'device': args.device, 'gpu': torch.cuda.get_device_name(int(args.device.replace('cuda:', ''))) if args.device != 'cpu' and torch.cuda.is_available() else None,
        'fps': float(1/samples.mean()), 'latency_mean_ms': float(samples.mean()*1000),
        'latency_p50_ms': float(np.percentile(samples, 50)*1000),
        'latency_p95_ms': float(np.percentile(samples, 95)*1000),
        'latency_p99_ms': float(np.percentile(samples, 99)*1000),
        'latency_max_ms': float(samples.max()*1000),
        'fraction_within_33_33ms': float(np.mean(samples <= 1/30)),
    }
    try:
        import tensorrt
        report['tensorrt'] = tensorrt.__version__
    except ImportError:
        report['tensorrt'] = None
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    with output.with_suffix('.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['frame', 'latency_ms', 'detections'])
        writer.writerows((i, t*1000, n) for i, (t, n) in enumerate(zip(durations, counts)))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
