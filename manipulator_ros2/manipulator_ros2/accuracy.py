import itertools
import numpy as np
try:
    from manipulator_ros2.kinematics import fk, ik_position
except ImportError:
    from kinematics import fk, ik_position
import csv

Q_START = np.array([0.5, -1.2, 1.2, -1.6, -1.57, 0.0])
MAX_ERROR = 1.0

def make_grid(step=0.1, r_min=0.25, r_max=0.75, z_min=0.1, z_max=0.7):
    xs = np.arange(-r_max, r_max + 0.0001, step)
    ys = np.arange(-r_max, r_max + 0.0001, step)
    zs = np.arange(z_min, z_max + 0.0001, step)
    points = []
    for x, y, z in itertools.product(xs, ys, zs):
        r = np.hypot(x, y)
        if r_min <= r <= r_max:
            points.append(np.array([round(x, 3) + 0.0, round(y, 3) + 0.0, round(z, 3) + 0.0]))
    return points

def one_solver(target):
    q, converged, iters = ik_position(target, Q_START)
    err_mm = 1000 * np.linalg.norm(fk(q)[:3, 3] - target) 
    return err_mm, iters, err_mm < MAX_ERROR

def run_all(points):
    rows = []
    for p in points:
        err_mm, iters, ok = one_solver(p)
        rows.append((p[0], p[1], p[2], err_mm, iters, ok))
    return rows


def summarize(rows):
    n = len(rows)
    ok_rows = [r for r in rows if r[5]]
    print(f'targets: {n}')
    print(f'success (< {MAX_ERROR} mm): {len(ok_rows)} = {100 * len(ok_rows) / n:.1f} %')
    if ok_rows:
        errs = [r[3] for r in ok_rows]
        iters = [r[4] for r in ok_rows]
        print(f'error of successes: mean {np.mean(errs):.3f} mm, max {np.max(errs):.3f} mm')
        print(f'iterations of successes: mean {np.mean(iters):.1f}, max {max(iters)}')

def save_csv(rows, path):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'z', 'error_mm', 'iterations', 'success'])
        w.writerows(rows)
    print('saved', path)


def print_failures(rows, limit=15):
    failed = [r for r in rows if not r[5]]
    print(f'failed targets: {len(failed)}')
    for r in failed[:limit]:
        print(f'  ({r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f})  error {r[3]:.1f} mm')


def main():
    rows = run_all(make_grid())
    summarize(rows)
    save_csv(rows, 'ik_accuracy.csv')

    print('--- wide area ---')
    wide = run_all(make_grid(step=0.2, r_min=0.0, r_max=1.0, z_min=-0.3, z_max=1.0))
    summarize(wide)
    print_failures(wide)
    save_csv(wide, 'ik_accuracy_wide.csv')


if __name__ == '__main__':
    main()