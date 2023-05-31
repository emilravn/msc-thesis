import inferencing_one_image as ml
import time

labels = "./model/labels.txt"


def simulate_capture():
    print("Simulating camera capture...")
    time.sleep(2)  # Simulate the delay for taking a photo
    print("Photo captured!")


if __name__ == "__main__":
    simulate_capture()
    class_folder, random_image, full_path = ml.get_a_random_image()
    raw_prediction, label_prediction = ml.predict_new_disease(full_path, random_image)
    print(raw_prediction)
    ml.compare_output(label_prediction, labels, class_folder, random_image)
    ml.normalize_values(raw_prediction)
