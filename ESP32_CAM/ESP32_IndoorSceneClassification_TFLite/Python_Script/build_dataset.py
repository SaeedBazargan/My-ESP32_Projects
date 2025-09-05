import serial
import numpy as np
from PIL import Image
import struct
import os
import time
import io
import sys
import uuid

# <---- ------ Settings ------ ---->
serial_port = 'COM3'
baud_rate = 115200
save_directory = r"C:\Users\Bazar\Desktop\My-ESP32_Projects\ESP32_CAM\ESP32_IndoorSceneClassification_TFLite\captured_images"

# <---- ------ Create folder if not exists ------ ---->
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# <---- ------ Open Serial Port ------ ---->
ser = serial.Serial(serial_port, baud_rate, timeout=10)
print(f"<---- ------ Connected to {serial_port} at {baud_rate} baud ------ ---->")
ser.reset_input_buffer()

label  = "test"
width  = 1
height = 1
num_ch = 3

image = np.empty((height, width, num_ch), dtype=np.uint8)

def serial_readline():
    data = ser.readline() # read a '\n' terminated line
    return data.decode("utf-8").strip()

while True:
    data_str = serial_readline()

    if str(data_str) == "<image>":
        width_str = serial_readline()
        height_str = serial_readline()
        w = int(width_str)
        h = int(height_str)
        if w != width or h != height:
            print("Resizing numpy array")

            if w * h != width * height:
                image.resize((h, w, num_ch))
            else:
                image.reshape((h, w, num_ch))

            width  = w
            height = h

        print("Reading frame:", width, height)
        for i in range(0, width * height * num_ch):
            c = int(i % num_ch)
            y = int((i / num_ch) / width)
            x = int((i / num_ch) % width)

            data_str = serial_readline()
            if str(data_str) == "</image>":
                break
            elif data_str.isdigit():
                image[y][x][c] = int(data_str)

        data_str = serial_readline()
        if str(data_str) == "</image>":
            print("Captured frame")
            crop_area = (0, 0, height, height)

            image_pil     = Image.fromarray(image)
            image_cropped = image_pil.crop(crop_area)
            image_cropped.show()

            unique_id = str(uuid.uuid4())
            filename = label + "_"+ unique_id + ".png"
            file_path = os.path.join(save_directory, filename)
            image_cropped.save(file_path)
            print("File", file_path, "saved")
        else:
            print("Error capture image")