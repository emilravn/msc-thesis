import inferencing_one_image
import time
import numpy as np
import psutil
import os


def simulate_capture():
    print("Simulating camera capture...")
    time.sleep(2)  # Simulate the delay for taking a photo
    print("Photo captured!")


execution_times = []
cpu_percentages = []
memory_usages = []

process = psutil.Process(os.getpid())

for _ in range(50):
    # Record the start time, CPU and RAM usage
    start_time = time.time()
    start_cpu = process.cpu_percent(interval=None)
    start_mem = process.memory_info().rss  # rss is the Resident Set Size and is used to show how much memory is allocated to that process and is in RAM

    inferencing_one_image.predict_new_disease()
    simulate_capture()

    # Record the end time, CPU and RAM usage
    end_time = time.time()
    end_cpu = process.cpu_percent(interval=None)
    end_mem = process.memory_info().rss

    # Compute the execution time and add it to the list
    execution_times.append(end_time - start_time)
    cpu_percentages.append(end_cpu - start_cpu)

    # Convert memory usage from bytes to megabytes and add it to the list
    memory_usages.append((end_mem - start_mem) / (1024 * 1024))

# Convert to a numpy array for easy computation
execution_times = np.array(execution_times)
cpu_percentages = np.array(cpu_percentages)
memory_usages = np.array(memory_usages)

# Compute some statistics
mean_time = np.mean(execution_times)
std_dev_time = np.std(execution_times)

mean_cpu = np.mean(cpu_percentages)
std_dev_cpu = np.std(cpu_percentages)

mean_memory = np.mean(memory_usages)
std_dev_memory = np.std(memory_usages)

print(f"The model took an average of {mean_time} seconds with a standard deviation of {std_dev_time} seconds to predict over 50 runs.")
print(f"The average CPU usage was {mean_cpu}% with a standard deviation of {std_dev_cpu}%.")
print(f"The average memory usage was {mean_memory} MB with a standard deviation of {std_dev_memory} MB.")