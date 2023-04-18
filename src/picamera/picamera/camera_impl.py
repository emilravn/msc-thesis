#!/usr/bin/python

import cv2
from datetime import datetime
import os


class CameraSetup():
    def __init__(self) -> None:
        self.camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)


class PiCameraImage(CameraSetup):
    def __init__(self, width_resolution=2591, height_resolution=1944) -> None:
        self.image_path = ".img/"
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width_resolution)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height_resolution)

    def capture_plain_image(self):
        # filename format
        now = datetime.now()
        date_string = now.strftime("%d-%m-%Y")
        time_string = now.strftime("%H.%M.%S")

        fname = f"image_{date_string}-{time_string}.jpg"
        save_path = self.image_path

        ret, frame = self.camera.read()
        color_corrected_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        print("ret:", ret)
        if ret:
            print("h,w: ", frame.shape[:2])

            # write frame to file
            cv2.imwrite(os.path.join(save_path, fname), color_corrected_image)
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
    image_camera = PiCameraImage()
    image_camera.capture_plain_image()
