from distance_detector import DistanceDetector
from time import sleep

if __name__ == '__main__':
    # uses your built-in camera, looks for tags that are 125mm,
    # and uses camera parameters from camera_params.json
    dd = DistanceDetector(0, .125, 'camera_params.json')
    while True:
        frame = dd.take_picture()
        tags = dd.detect_tags(frame)
        print(dd.find_distance_to_tags(tags))
        dd.visualize_tags(tags, frame)

        sleep(1)
