from distance_detector import DistanceDetector
from time import sleep

# uses your built-in camera, looks for tags that are 125mm,
# and uses camera parameters from camera_params.json
dd = DistanceDetector(0, .125, 'camera_params.json')
dd.detect_tags(visualize=True)
sleep(5)
dd.detect_tags(visualize=True)
sleep(5)
