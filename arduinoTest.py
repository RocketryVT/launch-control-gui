import serial, time


arduino = serial.Serial('COM9', 115200, timeout=.1)
while True:
    arduino.write(str.encode("Hello from Python!"))
    time.sleep(1)
    data = arduino.readline()[:-2]  # the last bit gets rid of the new-line chars
    if data:
        print(data)


