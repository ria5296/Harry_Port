import serial

def read_serial(port='COM3', baudrate=9600):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            while True:
                line = ser.readline().decode('utf-8').rstrip('\r\n')
                if line:
                    yield line
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
