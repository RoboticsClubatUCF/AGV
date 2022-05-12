import serial
import serial.tools.list_ports

# variables
PORT = "/dev/ttyUSB1"
inputs = {
    "RJ_x": 0,
    "RJ_y": 0,
    "LJ_x": 0,
    "LJ_y": 0,
    "ESTOP": 0,
    "DIAL": 0
}

ser = serial.Serial(port = PORT)
# default initialization variales are
""" 
    baudrate = 9600,
    bytesize = EIGHTBITS,
    parity = PARITY_NONE,
    stopbits = STOPBITS_ONE,
    timeout = NONE
"""

def read_serial():
    # used to determine how much of the input we should read
    is_short = False
    bytes_read = 0
    cut_at = 0
    
    while(1):
        bytes_read = 0

        # wait until the "E" byte is received to start reading the serial
        while(ser.read(size = 1).decode() != "G"):
            pass

        # read each of the 6 controller inputs 
        while(bytes_read != 24):
            # read serial values
            input = ser.read(size = 4)
            bytes_read += 4
            
            # decode the input
            input = input.decode()
            
            # find the marker that's added if the value is short
            if input[2] == "_": # has filler marker at the second index
                cut_at = 2
            elif input[3] == "_":
                cut_at = 3
            else:
                cut_at = 4
            
            # determine what input the value belongs to
            if input[0] == "A":
                inputs["RJ_x"] = input[1:cut_at]
            elif input[0] == "B":
                inputs["RJ_y"] = input[1:cut_at]
            elif input[0] == "C":
                inputs["LJ_y"] = input[1:cut_at]
            elif input[0] == "D":
                inputs["LJ_x"] = input[1:cut_at]
            elif input[0] == "E":
                inputs["ESTOP"] = input[1:cut_at]
            elif input[0] == "F":
                inputs["DIAL"] = input[1:cut_at]
                
            print(inputs["RJ_x"], inputs["RJ_y"], inputs["LJ_x"], inputs["LJ_y"], inputs["ESTOP"], inputs["DIAL"], sep = " ")
    
    # close the serial port
    ser.close()
         

if __name__ == "__main__":
    read_serial()