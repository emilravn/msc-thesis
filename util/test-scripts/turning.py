import encoder
from time import sleep

if __name__ == "__main__":
    try:
        encoder.robot.forward()
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        encoder.cleanup_pins()