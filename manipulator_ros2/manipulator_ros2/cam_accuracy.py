import math
import subprocess
import time
import rclpy
from detect import Detector, red_mask, find_cube, pixel_to_xy
import csv
import os
import random

def place_cube(x, y, yaw):
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    req = f'name: "cube_red", position: {{x: {x}, y: {y}, z: 0.72}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
    result = subprocess.run(
        ['gz', 'service', '-s', '/world/table_world/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', req],
        capture_output=True, text=True)
    return 'true' in result.stdout

def measure(nod):
    time.sleep(1.0)
    nod.frame = None
    while nod.frame is None:
        rclpy.spin_once(nod, timeout_sec=0.5)
    res = find_cube(red_mask(nod.frame))
    if res is None: return None
    return pixel_to_xy(res[0], res[1], nod.k)

def make_points():
    points=[]
    for i in range (5):
        x = round(0.35+0.1*i,2)
        for j in range (7):
            y = round(-0.3 + 0.1*j, 2)
            if math.hypot(x,y)<=0.75:
                k = [0.0, 0.5, 1.0]
                for z in k:
                    points.append((x, y, z))

         
    return points

def random_points(n=50, seed=1):
    position = []
    rng = random.Random(seed)
    while len(position) < n:
        
        x = rng.uniform(0.35, 0.75)
        y = rng.uniform(-0.3,0.3)
        if math.hypot(x, y) <= 0.75:
            position.append((round(x, 3), round(y, 3), round(rng.uniform(0, 1.5), 2)))

    return position

if __name__ == '__main__':
    rclpy.init()
    nod = Detector()
    try:
        while nod.frame is None or nod.k is None:
            rclpy.spin_once(nod, timeout_sec=0.5)
        pts = random_points()
        rows = []
        for x, y, yaw in pts:
            place_cube(x, y, yaw)
            m = measure(nod)
            if m is None:
                print('no detection at', x, y, yaw)
                continue
            mx, my = m
            err_mm = math.hypot(mx - x, my - y) * 1000
            rows.append((x, y, yaw, mx, my, err_mm))
            print(x, y, yaw, '->', round(err_mm, 1), 'mm')
        errs = [r[5] for r in rows]
        print('detected:', len(rows), 'of', len(pts))
        print('mean error (mm):', round(sum(errs) / len(errs), 2))
        print('max error (mm):', round(max(errs), 2))
        path = os.path.expanduser('~/ros2_ws/src/manipulator_ros2/results/camera_accuracy_random.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'yaw', 'measured_x', 'measured_y', 'dx_mm', 'dy_mm', 'error_mm'])
            for x, y, yaw, mx, my, err in rows:
                w.writerow([x, y, yaw,
                    round(mx, 4), round(my, 4),
                    round((mx - x) * 1000, 2),
                    round((my - y) * 1000, 2),
                    round(err, 2)])
        print('saved', path)
    finally:
        nod.destroy_node()
        rclpy.shutdown()