#!/usr/bin/env python3
"""Deterministic offline adapter for kr_param_map read_img, followed by C++ solve.
No Python corridor optimizer is used. Image-to-grid semantics follow upstream
read_grid_map.cpp:65-135, including integer ratios and alpha-channel averaging.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image


def export_cloud(image_path, config):
    im = np.asarray(Image.open(image_path), dtype=float)
    if im.ndim == 2:
        im = im[:, :, None]
    dims = (np.asarray(config['size']) / config['resolution']).astype(int)
    # Upstream uses int/int division before assigning ratio to double.
    ix = np.arange(dims[0]) * (im.shape[1] // dims[0])
    iy = np.arange(dims[1]) * (im.shape[0] // dims[1])
    pixels = im[iy[:, None], ix[None, :]].mean(axis=2)
    if config['negate']:
        pixels = 255 - pixels
    rows, cols = np.where((255 - pixels) / 255 > config['threshold'])
    xy = np.column_stack((cols, dims[1] - rows - 1))
    xyz = np.vstack([np.column_stack((xy, np.full(len(xy), z))) for z in range(dims[2])])
    return np.asarray(config['origin']) + (xyz + .5) * config['resolution']


def main():
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--map-repo', type=Path, default=root.parent / 'kr_param_map')
    parser.add_argument('--binary', type=Path, default=root / 'build/corrgen_benchmark')
    parser.add_argument('--output', type=Path, default=root / 'results')
    args = parser.parse_args()
    revision = subprocess.check_output(['git', '-C', str(args.map_repo), 'rev-parse', 'HEAD'], text=True).strip()
    metrics = []
    for name in ['balls', 'multi_gap', 'maze']:
        config = json.loads((root / 'config' / (name + '.json')).read_text())
        scene = args.output / name
        scene.mkdir(parents=True, exist_ok=True)
        image_path = args.map_repo / config['map']
        cloud = export_cloud(image_path, config)
        np.savetxt(scene / 'cloud.xyz', cloud, fmt='%.12g')
        np.savetxt(scene / 'path.xyz', config['controls'], fmt='%.12g')
        metadata = dict(config, map_revision=revision, image_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(), adapter='read_img-compatible offline, no ROS node', point_count=len(cloud))
        (scene / 'input.json').write_text(json.dumps(metadata, indent=2) + '\n')
        for degree in config['degrees']:
            output = scene / ('degree_' + str(degree))
            command = [str(args.binary.resolve()), str(scene / 'cloud.xyz'), str(scene / 'path.xyz'), str(output), str(degree)]
            run = subprocess.run(command, text=True, capture_output=True)
            output.mkdir(exist_ok=True)
            (output / 'run.log').write_text(run.stdout + run.stderr)
            if run.returncode:
                raise RuntimeError(f'{name}, degree {degree}: {run.stderr}')
            report = json.loads((output / 'metrics.json').read_text())
            metrics.append(dict(scene=name, **report))
            print(run.stdout.strip(), flush=True)
    (args.output / 'summary.json').write_text(json.dumps(metrics, indent=2) + '\n')


if __name__ == '__main__':
    main()
