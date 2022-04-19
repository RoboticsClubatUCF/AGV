import serial
import serial.tools.list_ports

# variables
port = "COM21"
inputs = {
    "RJ_x": 0,
    "RJ_y": 0,
    "LJ_x": 0,
    "LJ_y": 0,
    "ESTOP": 0,
    "DIAL": 0
}

ser = serial.Serial(port = "COM21")
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
    cut_with = {True: 4, False: 5}
    
    # wait until the "A" byte is received to start reading the serial
    while(ser.read(size = 5).decode()[0] != "A"):
        pass
    
    while(1):
        # read serial values
        input = ser.read(size = 5)
        
        # decode the input
        input = input.decode()
        
        # find the marker that's added if the value < 1000
        if input[4] == "_": # has filler marker
            is_short = True
        else:
            is_short = False
            
        # determine what input the value belongs to
        if input[0] == "A":
            inputs["RJ_x"] = input[1:cut_with[is_short]]
        elif input[0] == "B":
            inputs["RJ_y"] = input[1:cut_with[is_short]]
        elif input[0] == "C":
            inputs["LJ_y"] = input[1:cut_with[is_short]]
        elif input[0] == "D":
            inputs["LJ_x"] = input[1:cut_with[is_short]]
        elif input[0] == "E":
            inputs["ESTOP"] = input[1:cut_with[is_short]]
        elif input[0] == "F":
            inputs["DIAL"] = input[1:cut_with[is_short]]
            
        print(inputs["RJ_x"], inputs["RJ_y"], inputs["LJ_x"], inputs["LJ_y"], inputs["ESTOP"], inputs["DIAL"], sep = " ")
    
    # close the serial port
    ser.close()
         

if __name__ == "__main__":
    read_serial()