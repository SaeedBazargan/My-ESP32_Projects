import serial
import time
import pandas as pd

# Open the serial port
ser = serial.Serial('COM12', 9600, timeout=1)

try:
    roll_var = []
    pitch_var = []
    direction_var = []

    while True:
        data = ser.readline().decode('utf-8').strip()
        
        if data:            
            if "Roll" in data:
                roll_value = int(data.split("--->")[1].strip())
                roll_var.append(roll_value)
                # print(f"Roll Value: {roll_value}")

                if roll_value > 112:
                    print(f"Left ----> {roll_value}")
                    direction_var.append("Left")
                elif roll_value < 94:
                    print(f"Right ----> {roll_value}")
                    direction_var.append("Right")
            elif "Pitch" in data:
                pitch_value = int(data.split("--->")[1].strip())
                pitch_var.append(pitch_value)
                # print(f"Pitch Value: {pitch_value}")

                if pitch_value > 107:
                    print(f"Up ----> {pitch_value}")
                    direction_var.append("Up")
                elif pitch_value < 96:
                    print(f"Down ----> {pitch_value}")
                    direction_var.append("Down")
                # else:
                #     direction_var.append("Motionless")  # Add empty string for neutral positions

except KeyboardInterrupt:
    print("Program terminated by user.")
    print(f"Roll Values: {roll_var}")
    print(f"Pitch Values: {pitch_var}")
    print(f"Directions: {direction_var}")

    # Create a DataFrame and save it to CSV file
    max_len = max(len(roll_var), len(pitch_var), len(direction_var))
    roll_var += [None] * (max_len - len(roll_var))
    pitch_var += [None] * (max_len - len(pitch_var))
    direction_var += [None] * (max_len - len(direction_var))

    df = pd.DataFrame({
        "Roll": roll_var,
        "Pitch": pitch_var,
        "Direction": direction_var})

    df.to_csv('imu_data.csv', index=False)  # Save to CSV

finally:
    ser.close()
