from scd30_i2c import SCD30
from time import sleep

scd30 = SCD30()

scd30.set_measurement_interval(2)
scd30.start_periodic_measurement()

sleep(2)

while True:
    if scd30.get_data_ready():
        m = scd30.read_measurement()
        if m is not None:
            print(f"CO2: {m[0]:.2f}ppm, temp: {m[1]:.2f}'C, humidity: {m[2]:.2f}%")
        sleep(2)
    else:
        sleep(0.2)
