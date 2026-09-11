"""python -m showcase.demo [--firmware-check] [--fault estop]"""
import argparse
import json
from pathlib import Path
from .pipeline import ROOT, load_config, run


def main():
    p = argparse.ArgumentParser(description='Offline trash-picking pipeline and interactive report')
    p.add_argument('--example', type=Path, default=ROOT/'showcase/example.json')
    p.add_argument('--config', type=Path, default=ROOT/'robot_launch/config/robot.yaml')
    p.add_argument('--out', type=Path, default=ROOT/'output/showcase')
    p.add_argument('--firmware-check', action='store_true', help='Compile actual C UART parser using host GCC')
    p.add_argument('--fault', choices=['unreachable', 'stale', 'transport', 'estop'])
    args = p.parse_args()
    example = json.loads(args.example.read_text(encoding='utf-8'))
    result = run(example, load_config(args.config), args.fault, args.firmware_check)
    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.fault or 'demo'
    (args.out/f'{stem}.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    template = (ROOT/'showcase/report.html').read_text(encoding='utf-8')
    data = json.dumps(result, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    (args.out/f'{stem}.html').write_text(template.replace('__DATA__', data), encoding='utf-8')
    print(json.dumps({'status': result['status'], 'fault': result['fault'],
                      'motion_samples': len(result['trace']), 'uart_frames': len(result['uart_commands']),
                      'virtual_duration_sec': result['virtual_duration_sec'],
                      'firmware': result['firmware_check']['status']}))
    print(args.out/f'{stem}.html')
    return 0 if result['status'] == 'DONE' else 2


if __name__ == '__main__':
    raise SystemExit(main())
