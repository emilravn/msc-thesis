from tflite_runtime.interpreter import Interpreter 
from PIL import Image
import numpy as np
import time

data_folder = "./images/"

model_path = "./efficient_test1_data10.tflite"
label_path = "./labels.txt"

interpreter = Interpreter(model_path)
print("Model Loaded Successfully.")

interpreter.allocate_tensors()
# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

_, height, width, _ = interpreter.get_input_details()[0]['shape']
print("Image Shape (", width, ",", height, ")")
# Test the model on random input data.
image = Image.open(data_folder + "tomato_early_blight.jpg").convert('RGB').resize((width, height))
image = np.array(image)
processed_image = np.expand_dims(image, axis=0).astype(np.float32)

interpreter.set_tensor(input_details[0]['index'], processed_image)

t1=time.time()
interpreter.invoke()
t2=time.time()
time_taken=(t2-t1)*1000 #milliseconds
print("time taken for Inference: ",str(time_taken), "ms")

# Obtain results
predictions = interpreter.get_tensor(output_details[0]['index'])[0]
print(predictions)
predictions = np.argmax(predictions)
print(predictions)


def compare_output(output_value, label_path):
    # Step 1: Read the contents of the text file
    with open(label_path, "r") as file:
        lines = file.readlines()
    
    # Create a dictionary to store the text file contents
    text_dict = {}
    for line in lines:
        parts = line.strip().split(" ")
        key = int(parts[0])
        value = " ".join(parts[1:])
        text_dict[key] = value

    # Step 2: Retrieve the output value
    # output_value = <your variable that returns a number from 0-9>

    # Step 3: Compare the output value with the text file entry
    if output_value in text_dict:
        print(f"Output: {output_value}")
        print(f"Text: {text_dict[output_value]}")
    else:
        print("Output value not found in the text file.")

compare_output(predictions, label_path)

# if predictions == 0:
#     print("Organic")
# else:
#     print("Inorganic")