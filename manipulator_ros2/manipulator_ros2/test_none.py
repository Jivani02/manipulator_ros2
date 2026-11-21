import rclpy
from detect import Detector
from cam_accuracy import place_cube, measure

rclpy.init()
nod = Detector()
try:
    while nod.frame is None or nod.k is None:
        rclpy.spin_once(nod, timeout_sec=0.5)
    place_cube(3.0, 3.0, 0.0)
    print(measure(nod))
finally:
    nod.destroy_node()
    rclpy.shutdown()