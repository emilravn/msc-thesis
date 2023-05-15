#!/usr/bin/python

import cv2
import os
from time import sleep


def capture_plain_image(image_name="test"):
    camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)
    width_resolution = 2591
    height_resolution = 1944
    image_path = "/home/sfr/sfr_ros2_ws/src/images/"
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width_resolution)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height_resolution)

    fname = f"image_{str(image_name)}.jpg"
    ret, frame = camera.read()
    color_corrected_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    print("ret:", ret)
    if ret:
        print("h,w: ", frame.shape[:2])

        # write frame to file
        cv2.imwrite(os.path.join(image_path, fname), color_corrected_image) 
        print(f"Captured image {fname}")
    else:
        print("Frame not captured")

    # release camera
    camera.release()


# class PiCameraVideo(CameraSetup):
#     def __init__(self, width_resolution=1920, height_resolution=1080) -> None:
#         super().__init__()
#         self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width_resolution)
#         self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height_resolution)


if __name__ == "__main__":
    image_no = 0
    while True:
        capture_plain_image(image_no)
        image_no = image_no + 1
        sleep(2)
