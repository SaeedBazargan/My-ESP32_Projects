import serial
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

# <---- ------ Settings ------ ---->
serial_port = 'COM3'
baud_rate = 115200
save_directory = r"C:\Users\Bazar\Desktop\My-ESP32_Projects\ESP32_CAM\ESP32-CAM_ObjectRecognition_TfLite\CameraCapture_Dataset_Python\captured_images"

# <---- ------ Create folder if not exists ------ ---->
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# <---- ------ Open Serial Port ------ ---->
ser = serial.Serial(serial_port, baud_rate, timeout=10)
print(f"<---- Connected to {serial_port} at {baud_rate} baud ---->")

image_counter = 0

def read_line():
    line = ser.readline().strip()
    return line.decode(errors='ignore')

while True:
    # <---- ------ Step 1: Wait for start marker ------ ---->
    marker = read_line()
    if marker != "<image>":
        print("<---- Sync lost, resyncing ---->")
        continue

    # <---- ------ Step 2: Read Frame Size ------ ---->
    width = int(read_line())
    height = int(read_line())
    print(f"<---- Receiving image: {width} x {height} ---->")

    # <---- ------ Step 3: Read the image data ------ ---->
    expected_bytes = width * height * 3
    data = ser.read(expected_bytes)

    if len(data) < expected_bytes:
        print("<---- Timeout or incomplete image received ---->")
        continue

    img = np.frombuffer(data, dtype=np.uint8)
    img = img.reshape((height, width, 3))

    # <---- ------ Step 4: Display with matplotlib ------ ---->
    plt.imshow(img)
    plt.axis('off')
    plt.title("ESP32-CAM Frame")
    plt.show(block=False)
    plt.pause(2)
    plt.close()

    # <---- ------ Step 5: Save image ------ ---->
    img_rgb = Image.fromarray(img, 'RGB')
    png_filename = os.path.join(save_directory, f"image_{image_counter:04d}.png")
    img_rgb.save(png_filename)
    print(f"<---- Saved {png_filename} ---->")

    image_counter += 1
    time.sleep(0.1)
