#!/usr/bin/env python3
"""Restore the Mustika to its initial pose in a running Gazebo world."""

import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='robocon_2027')
    parser.add_argument('--timeout-ms', type=int, default=3000)
    args = parser.parse_args()

    request = (
        'name: "mustika", '
        'position: {x: 0.0, y: 4.225, z: 0.545}, '
        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
    )
    command = [
        'gz', 'service',
        '-s', f'/world/{args.world}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', str(args.timeout_ms),
        '--req', request,
    ]
    try:
        result = subprocess.run(
            command, check=False, capture_output=True, text=True,
            timeout=args.timeout_ms / 1000.0 + 2.0,
        )
    except FileNotFoundError:
        print('找不到 gz 命令，请先安装并加载 Gazebo Harmonic 环境。', file=sys.stderr)
        return 2
    except subprocess.TimeoutExpired:
        print('Mustika 复位请求超时，请确认仿真世界正在运行。', file=sys.stderr)
        return 3

    output = '\n'.join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
    if result.returncode == 0 and 'data: true' in output.lower():
        print('Mustika 已复位到 Ground Mustika Pillar。')
        return 0
    print(output or 'Mustika 复位失败，请确认 robocon_2027 世界正在运行。', file=sys.stderr)
    return result.returncode or 1


if __name__ == '__main__':
    raise SystemExit(main())
