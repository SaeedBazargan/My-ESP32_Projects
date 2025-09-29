import serial
import struct
import os
import time
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# <---- ------ Settings ------ ---->
serial_port = 'COM3'
baud_rate = 115200
save_directory = r"C:\Users\Bazar\Desktop\My-ESP32_Projects\ESP32_CAM\ESP32-CAM_SendImagesOverSerial\captured_images"

# <---- ------ Create folder if not exists ------ ---->
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# <---- ------ Open Serial Port ------ ---->
ser = serial.Serial(serial_port, baud_rate, timeout=10)
print(f"<---- Connected to {serial_port} at {baud_rate} baud ---->")

image_counter = 0

def read_exact(size):
    # <---- ------ Read exactly size bytes from serial ------ ---->
    data = b''
    while len(data) < size:
        packet = ser.read(size - len(data))
        if not packet:
            return None
        data += packet
    return data

while True:
    # <---- ------ Step 1: Wait for start marker ------ ---->
    marker = read_exact(4)
    if marker != b'IMG0':
        print("<---- Sync lost, resyncing ---->")
        continue

    # <---- ------ Step 2: Read 4 bytes (image length) ------ ---->
    img_len_bytes = read_exact(4)
    if img_len_bytes is None:
        print("<---- Failed to read image size ---->")
        continue

    img_len = struct.unpack('<I', img_len_bytes)[0]  # Little endian uint32
    print(f"<---- Receiving image of size: {img_len} bytes ---->")

    # <---- ------ Step 3: Read the image data ------ ---->
    img_data = read_exact(img_len)
    if img_data is None:
        print("<---- Timeout or incomplete image received ---->")
        continue

    # <---- ------ Step 4: Save image ------ ---->
    jpg_filename = os.path.join(save_directory, f"image_{image_counter:04d}.jpg")

    # <---- ------ Save as .jpg ------ ---->
    with open(jpg_filename, 'wb') as f:
        f.write(img_data)

    try:
        # <---- ------ Display with matplotlib ------ ---->
        img = mpimg.imread(jpg_filename)
        plt.imshow(img)
        plt.axis("off")
        plt.title(f"Image {image_counter}")
        plt.show(block=False)
        plt.pause(2)
        plt.close()

        print(f"<---- Saved and displayed {jpg_filename} ---->")
    except Exception as e:
        print(f"<---- Warning: Saved {jpg_filename} but could not display: {e} ---->")

    image_counter += 1
    time.sleep(0.1)
