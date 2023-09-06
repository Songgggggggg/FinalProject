import serial.tools.list_ports
import time
import csv
from datetime import datetime

def timestamp_to_date(timestamp):
    # Convert timestamp to datetime object
    dt_object = datetime.utcfromtimestamp(timestamp)
    
    # Format datetime object as string
    date_str = dt_object.strftime('%Y-%m-%d %H:%M:%S')
    return date_str

def extract_values(data_str):
    try:
        return float(data_str.split('=')[1].strip())
    except (IndexError, ValueError) as e:
        print(f"Error parsing data string: {data_str}. Error: {e}")
        return 0.0  # or return some other default value
    

plist = list(serial.tools.list_ports.comports())

if len(plist) <= 0:
    print("The Serial port can't find!")
else:
    print("find!")
    for i, device in enumerate(plist):
        print(i+1, ':', device)
    Num = int(input("please select which port you connect:"))
    serialName = plist[Num-1][0]  # Select device
    print("connect to " + serialName)
    ser = serial.Serial(serialName, 9600, timeout=1)  # Initialize serial port

    # Name the file with the current time
    current_time = time.strftime("%Y%m%d_%H%M%S")
    file_name = f'data_{current_time}.csv'
    f = open(file_name, 'w', encoding='utf-8', newline='')  # The last parameter eliminates blank lines
    csv_writer = csv.writer(f)
    
    current_epoch_time = int(time.time())
    time_diff = current_epoch_time-1691596819
    
    # Write column names
    headers = ['Epoch', 'UTC Time', 'UK Time', 'Elapsed Time', 'x_accel', 'y_accel', 'z_accel', 'x_gyro', 'y_gyro', 'z_gyro', 'x_mag', 'y_mag', 'z_mag']
    csv_writer.writerow(headers)
    
    i = 0
    while True:
        ser.read_until(b'#')
        timestamp_str = ser.read_until(b'&').strip(b'&').decode()
        timestamp = int(timestamp_str.replace("#", ""))
        timestamp = timestamp + time_diff
        utc_1 = ser.read_until(b'&').strip(b'&').decode()
        utc_2 = ser.read_until(b'&').strip(b'&').decode()
        time_values = ser.read_until(b'&').strip(b'&').decode()
        accel_data = ser.read_until(b'&').strip(b'&').decode().split('/')
        gyro_data = ser.read_until(b'&').strip(b'&').decode().split('/')
        mag_data = ser.read_until(b'&').strip(b'&').decode().split('/')

        # Parse acceleration, gyroscope, and magnetic data
        x_accel, y_accel, z_accel = [extract_values(val) for val in accel_data]
        x_gyro, y_gyro, z_gyro = [extract_values(val) for val in gyro_data]
        x_mag, y_mag, z_mag = [extract_values(val) for val in mag_data]
        
        utc_1 = timestamp_to_date(timestamp)
        utc_2 = timestamp_to_date(timestamp)
        

        csv_writer.writerow([timestamp, utc_1, utc_2, time_values, x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro, x_mag, y_mag, z_mag])
        f.flush()  # Ensure the data is written to the file
        print([timestamp, utc_1, utc_2, time_values, x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro, x_mag, y_mag, z_mag])

        # Clear the buffer to prepare for next data reception
        ser.flush()
