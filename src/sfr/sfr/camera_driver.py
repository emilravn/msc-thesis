#!/usr/bin/python

import cv2
import os
from time import sleep


class CameraSetup:
    def __init__(self) -> None:
        self.camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)


class PiCameraImage(CameraSetup):
    def __init__(self, width_resolution=2591, height_resolution=1944) -> None:
        super().__init__()
        self.image_path = "../../images/"
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width_resolution)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height_resolution)

    def capture_plain_image(self, image_name="test"):
        fname = f"image_{image_name}.jpg"
        ret, frame = self.camera.read()
        color_corrected_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        print("ret:", ret)
        if ret:
            print("h,w: ", frame.shape[:2])

            # write frame to file
            cv2.imwrite(os.path.join(self.image_path, fname), color_corrected_image)
        else:
            print("Frame not captured")

        # release camera
        self.camera.release()


class PiCameraVideo(CameraSetup):
    def __init__(self, width_resolution=1920, height_resolution=1080) -> None:
        super().__init__()
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width_resolution)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height_resolution)


if __name__ == "__main__":
    while True:
        image_camera = PiCameraImage()
        image_camera.capture_plain_image()
        sleep(2)
