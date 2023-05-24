import inferencing_one_image
import time


def simulate_capture():
    print("Simulating camera capture...")
    time.sleep(2)  # Simulate the delay for taking a photo
    print("Photo captured!")


simulate_capture()
inferencing_one_image.predict_new_disease()
