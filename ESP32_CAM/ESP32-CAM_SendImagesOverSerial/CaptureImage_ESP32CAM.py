import serial
import struct
import os
import time

# <---- ------ Settings ------ ---->
serial_port = 'COM3'
baud_rate = 115200
save_directory = "captured_images"  # <---- ------ Folder to save images ------ ---->

# <---- ------ Create folder if not exists ------ ---->
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# <---- ------ Open Serial Port ------ ---->
ser = serial.Serial(serial_port, baud_rate, timeout=10)
print(f"<---- ------ Connected to {serial_port} at {baud_rate} baud ------ ---->")

image_counter = 0

while True:
    # <---- ------ Step 1: Read 4 bytes (image length) ------ ---->
    img_len_bytes = ser.read(4)
    if len(img_len_bytes) != 4:
        print("<---- ------ Failed to read image size ------ ---->")
        continue

    img_len = struct.unpack('<I', img_len_bytes)[0]  # Little endian uint32
    print(f"<---- ------ Receiving image of size: {img_len} bytes ------ ---->")

    # <---- ------ Step 2: Read the image data ------ ---->
    img_data = b''
    while len(img_data) < img_len:
        packet = ser.read(img_len - len(img_data))
        if not packet:
            print("<---- ------ Timeout or incomplete image received ------ ---->")
            break
        img_data += packet

    # <---- ------ Step 3: Save image ------ ---->
    if len(img_data) == img_len:
        filename = os.path.join(save_directory, f"image_{image_counter:04d}.jpg")
        with open(filename, 'wb') as f:
            f.write(img_data)
        print(f"<---- ------ Saved {filename} ------ ---->")
        image_counter += 1
    else:
        print("<---- ------ Image not fully received, skipped ------ ---->")

    # <---- ------ Optional: wait a bit ------ ---->
    time.sleep(0.1)
