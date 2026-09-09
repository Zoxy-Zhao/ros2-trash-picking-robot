"""Export on the target Jetson/GPU; never substitute a dummy engine."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--family', choices=['yolov5', 'ultralytics'], default='yolov5')
    parser.add_argument('--yolov5-repo', default='~/yolov5')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='0')
    parser.add_argument('--fp32', action='store_true', help='Default export precision is FP16')
    args = parser.parse_args()
    weights = Path(args.weights).expanduser().resolve()
    if not weights.is_file() or weights.suffix != '.pt':
        parser.error('Supply an existing .pt checkpoint')
    if args.imgsz <= 0 or args.imgsz % 32:
        parser.error('imgsz must be a positive multiple of 32')
    engine = weights.with_suffix('.engine')
    if engine.exists():
        parser.error(f'Output already exists; move it before exporting: {engine}')
    if args.family == 'yolov5':
        repo = Path(args.yolov5_repo).expanduser().resolve()
        if not (repo / 'export.py').is_file():
            parser.error('yolov5-repo must be a trusted local YOLOv5 checkout')
        cmd = [sys.executable, str(repo/'export.py'), '--weights', str(weights),
               '--include', 'engine', '--imgsz', str(args.imgsz), '--device', args.device]
        if not args.fp32:
            cmd.append('--half')
        subprocess.run(cmd, cwd=repo, check=True)
    else:
        from ultralytics import YOLO
        YOLO(str(weights), task='detect').export(format='engine', imgsz=args.imgsz,
                                               device=args.device, half=not args.fp32,
                                               batch=1, dynamic=False)
    if not engine.is_file() or engine.stat().st_size == 0:
        raise RuntimeError('Exporter did not produce an engine; inspect its error output')
    print(f'Exported {engine}. Run benchmark_detection.py on target hardware.')


if __name__ == '__main__':
    main()
