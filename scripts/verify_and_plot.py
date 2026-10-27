#!/usr/bin/env python3
"""Independent NumPy polynomial-root and constraint checks; Matplotlib artifacts."""
import argparse
import json
from pathlib import Path
import numpy as np
from numpy.polynomial.chebyshev import chebval
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def verify(folder):
    cloud = np.loadtxt(folder.parent / 'cloud.xyz')
    controls = np.loadtxt(folder.parent / 'path.xyz')
    coefficients = np.loadtxt(folder / 'coefficients.csv', delimiter=',')
    power = np.array([controls[0], 3*(controls[1]-controls[0]), 3*(controls[2]-2*controls[1]+controls[0]), controls[3]-3*controls[2]+3*controls[1]-controls[0]])
    minimum, crossings, outside = float('inf'), 0, 0
    for point in cloud:
        relative = power.copy()
        relative[0] -= point
        polynomial = np.zeros(6)
        for axis in range(3):
            polynomial += np.convolve(relative[:, axis], np.arange(1, 4)*power[1:, axis])
        while len(polynomial) > 1 and abs(polynomial[-1]) <= 1e-13*np.max(np.abs(polynomial)):
            polynomial = polynomial[:-1]
        roots = np.roots(polynomial[::-1])
        count = 0
        for root in roots:
            if abs(root.imag) > 1e-7 or not -1e-10 <= root.real <= 1+1e-10:
                continue
            u = np.clip(root.real, 0, 1)
            position = np.array([1, u, u*u, u**3]) @ power
            tangent = np.array([1, 2*u, 3*u*u]) @ power[1:]
            tangent /= np.linalg.norm(tangent)
            normal = np.cross([0, 0, 1], tangent)
            normal /= np.linalg.norm(normal)
            binormal = np.cross(tangent, normal)
            w = np.array([normal @ (point-position), binormal @ (point-position)])
            a, b, c, d, e = chebval(2*u-1, coefficients.T)
            q = a*w[0]**2 + b*w[1]**2 + 2*c*w[0]*w[1] + d*w[0]+e*w[1]-1
            minimum = min(minimum, q)
            count += 1
        crossings += count
        outside += count == 0
    u = np.linspace(0, 1, 2001)
    a, b, c, _, _ = chebval(2*u-1, coefficients.T)
    min_eigen = np.min((a+b-np.sqrt((a-b)**2+4*c*c))/2)
    if minimum < 9e-6 or min_eigen <= 0:
        raise RuntimeError(f'{folder}: failed independent checks q={minimum}, eigen={min_eigen}')
    report = dict(min_q=float(minimum), dense_min_eigen=float(min_eigen), transverse_intersections=crossings, outside_finite_sweep=outside, checked_world_points=len(cloud), status='PASS')
    (folder / 'independent_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--results', type=Path, default=root / 'results')
    parser.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    fig = plt.figure(figsize=(15, 11))
    reports = {}
    for row, name in enumerate(['balls', 'multi_gap', 'maze']):
        scene = args.results / name
        run = scene / ('verified' if args.ros else 'degree_6')
        reports[name] = verify(run)
        cloud = np.loadtxt(scene / 'cloud.xyz')
        controls = np.loadtxt(scene / 'path.xyz')
        surf = np.genfromtxt(run / 'surface.csv', delimiter=',', names=True)
        u = np.linspace(0, 1, 401)[:, None]
        path = (1-u)**3*controls[0]+3*(1-u)**2*u*controls[1]+3*(1-u)*u*u*controls[2]+u**3*controls[3]
        ax = fig.add_subplot(3, 2, 2*row+1)
        xy = np.unique(cloud[:, :2], axis=0)
        ax.scatter(xy[:, 0], xy[:, 1], s=4, c='#54606d', label='kr_param_map')
        if args.ros:
            astar = np.loadtxt(scene / 'astar.xyz')
            ax.plot(astar[:, 0], astar[:, 1], color='#e6a329', lw=1.5, label='A*')
        ax.scatter(surf['x'][::3], surf['y'][::3], s=.5, c='#c2429b', alpha=.3, label='Corridor boundary')
        ax.plot(path[:, 0], path[:, 1], c='#008cb3', label='Cubic reference')
        ax.set(title=name.replace('_', ' ').title()+' — top view', xlabel='x [m]', ylabel='y [m]', xlim=(0, 20), ylim=(0, 10))
        ax.set_aspect('equal');ax.legend(loc='upper right', fontsize=7)
        ax = fig.add_subplot(3, 2, 2*row+2, projection='3d')
        ax.scatter(cloud[::5, 0], cloud[::5, 1], cloud[::5, 2], s=.3, c='#54606d', alpha=.2)
        xx=surf['x'].reshape(201, 48);yy=surf['y'].reshape(201, 48);zz=surf['z'].reshape(201, 48)
        ax.plot_wireframe(xx, yy, zz, rstride=10, cstride=6, color='#c2429b', lw=.4, alpha=.7)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='#008cb3', lw=2)
        ax.set(xlabel='x [m]', ylabel='y [m]', zlabel='z [m]', title='3D differentiable corridor')
        ax.set_box_aspect((2, 1, .5));ax.view_init(28, -65)
    fig.suptitle('CorrGen C++ | '+('ROS1 + A*' if args.ros else 'offline')+' | actual kr_param_map images', fontsize=16)
    fig.tight_layout()
    fig.savefig(args.results / 'overview.png', dpi=160)
    (args.results / 'independent_validation.json').write_text(json.dumps(reports, indent=2)+'\n')
    print(json.dumps(reports, indent=2))


if __name__ == '__main__':
    main()
