from tflite_runtime.interpreter import Interpreter
from PIL import Image
import numpy as np
import time

import time
from picamera2 import Picamera2, Preview

picam2 = Picamera2()
controls = {"AnalogueGain": 1.0}
preview_config = picam2.create_preview_configuration(main={"size": (224, 224)})
capture_config = picam2.create_still_configuration()
picam2.configure(preview_config)

data_folder = "/home/sfr/util/"

model_path = data_folder + "efficient_test2_data100.tflite"
# label_path = data_folder + "labels_mobilenet_quant_v1_224.txt"

interpreter = Interpreter(model_path)
print("Model Loaded Successfully.")

interpreter.allocate_tensors()
# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

_, height, width, _ = interpreter.get_input_details()[0]["shape"]
print("Image Shape (", width, ",", height, ")")
# Test the model on random input data.
# image = Image.open(data_folder + "R_10792.jpg").convert('RGB').resize((width, height))
picam2.start_preview(Preview.QTGL)

picam2.start()
time.sleep(5)

picam2.capture_file(data_folder + "test.jpg")
picam2.close()
image = image = Image.open(data_folder + "test.jpg").convert("RGB").resize((width, height))
image = np.array(image)
processed_image = np.expand_dims(image, axis=0).astype(np.float32)

interpreter.set_tensor(input_details[0]["index"], processed_image)

t1 = time.time()
interpreter.invoke()
t2 = time.time()
time_taken = (t2 - t1) * 1000  # milliseconds
print("time taken for Inference: ", str(time_taken), "ms")

# Obtain results
predictions = interpreter.get_tensor(output_details[0]["index"])[0]
predictions = np.argmax(predictions)
if predictions == 0:
    print("Organic")
else:
    print("Inorganic")
