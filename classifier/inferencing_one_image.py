from tflite_runtime.interpreter import Interpreter
from PIL import Image
import numpy as np
import time
import os
import random

# Find a random image in the dataset
images_folder = "./images/plantvillage/"
tomato_class_folder = str(np.random.choice(os.listdir(images_folder)) + "/")
full_path = os.path.join(images_folder, tomato_class_folder)

# Load the labels
label_path = "./model/labels.txt"

# Load the model
model_path = "./model/efficient_test1_data10.tflite"
interpreter = Interpreter(model_path)
print("Model Loaded Successfully.")

interpreter.allocate_tensors()
# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

_, height, width, _ = interpreter.get_input_details()[0]["shape"]
print("Image Shape (", width, ",", height, ")")

# List a random file for the model to predict
file_list = os.listdir(full_path)
random_image = random.choice(file_list)

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
prediction = interpreter.get_tensor(output_details[0]["index"])[0]
print(prediction)
prediction = np.argmax(prediction)
print(prediction)


def compare_output(output_value, label_path):
    # Read the contents of the text file
    with open(label_path, "r") as file:
        lines = file.readlines()

    # Create a dictionary to store the text file contents
    labels = {}
    for line in lines:
        parts = line.strip().split(" ")
        label_key = int(parts[0])
        disease = parts[1]
        labels[label_key] = disease

    # Compare the output value with the text file entry
    if output_value in labels:
        print(f"Output: {output_value}")
        print(f"Text: {labels[output_value]}")
    else:
        print("Output value not found in the text file.")


compare_output(prediction, label_path)
