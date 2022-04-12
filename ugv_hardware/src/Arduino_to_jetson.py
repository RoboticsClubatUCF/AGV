import serial
import serial.tools.list_ports

# variables
port = "COM21"

ser = serial.Serial(port = "COM21")
# default initialization variales are
""" 
    baudrate = 9600,
    bytesize = EIGHTBITS,
    parity = PARITY_NONE,
    stopbits = STOPBITS_ONE,
    timeout = NONE
"""
# def connected(port):
#     # get list of connected ports
#     myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]

def read_serial():  
    # read the serial port
    while(1):      
        input = ser.read(size = 4)
        print(input.decode()) # decode from byte to string
    
    # close the serial port
    ser.close()
         

if __name__ == "__main__":
    read_serial()