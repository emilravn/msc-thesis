from tflite_runtime.interpreter import Interpreter
from PIL import Image
import numpy as np
import time
import os
import random

# Get the images, load the labels, and load the model
images = "./images/plantvillage/"
labels = "./model/labels.txt"
model_path = "./model/efficient_test1_data10.tflite"

interpreter = Interpreter(model_path)
print("Model Loaded Successfully.")

interpreter.allocate_tensors()
# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

_, height, width, _ = interpreter.get_input_details()[0]["shape"]
print("Image Shape (", width, ",", height, ")")

classes = [
    "Bacterial spot",
    "Early blight",
    "Late blight",
    "Leaf mold",
    "Septoria leaf spot",
    "Spider mites two-spotted spider mite",
    "Target spot",
    "Tomato yellow leaf curl virus",
    "Tomato mosaic virus",
    "Healthy",
]


def get_a_random_image():
    # List a random file for the model to predict
    tomato_class_folder = str(np.random.choice(os.listdir(images)) + "/")
    full_path = os.path.join(images, tomato_class_folder)
    file_list = os.listdir(full_path)
    random_image = random.choice(file_list)
    return tomato_class_folder, random_image, full_path


def predict_new_disease(full_path, random_image):
    # Load the image
    image = Image.open(full_path + random_image).convert("RGB").resize((width, height))
    image = np.array(image)
    processed_image = np.expand_dims(image, axis=0).astype(np.float32)

    interpreter.set_tensor(input_details[0]["index"], processed_image)

    t1 = time.time()
    interpreter.invoke()
    t2 = time.time()
    time_taken = (t2 - t1) * 1000  # milliseconds
    print("time taken for Inference: ", str(time_taken), "ms")

    # Obtain results
    raw_prediction = interpreter.get_tensor(output_details[0]["index"])[0]
    label_prediction = np.argmax(raw_prediction)

    return raw_prediction, label_prediction


def normalize_values(prediction):
    percentage_list = []
    for i in range(len(prediction)):
        percentage_list.append("{:.9f}".format(prediction[i]))

    result = list(zip(classes, percentage_list))
    for i in range(len(result)):
        print(result[i])


def compare_output(prediction, label_path, tomato_class_folder, random_image):
    # Read the contents of the text file
    with open(label_path, "r") as file:
        lines = file.readlines()

    # Create a dictionary to store the text file contents
    labels = {}
    for line in lines:
        parts = line.strip().split(" ")
        label_key = int(parts[0])
        disease = " ".join(parts[1:])
        labels[label_key] = disease

    # Compare the output value with the text file entry
    if prediction in labels:
        print(f"Random tomato image from the dataset: {tomato_class_folder}{random_image}")
        print(f"Model predicted image as: {labels[prediction]}")
    else:
        print("Output value not found in the text file.")


if __name__ == "__main__":
    class_folder, random_image, full_path = get_a_random_image()
    raw_prediction, label_prediction = predict_new_disease(full_path, random_image)
    print(raw_prediction)
    compare_output(label_prediction, labels, class_folder, random_image)
    normalize_values(raw_prediction)
