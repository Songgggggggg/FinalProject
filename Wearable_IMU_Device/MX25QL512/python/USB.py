import serial

ser = serial.Serial('COM3', 9600)

while True:
    value = ser.readline()
    valueInString = str(value, 'UTF-8')
    print(valueInString)

