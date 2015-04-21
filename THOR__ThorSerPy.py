# (c) daniel ford, daniel.jb.ford@gmail.com

# test and remote run script for Thor mobile robot

import serial
import sys
from struct import unpack

# note, can run cProfile from cmd line as 'python -m cProfile ThorSerPy.py [args]'

def ThorSer():

    # check for serial port argument
    if len(sys.argv) < 6:
        print "Usage is python ThorSerPy.py $num_com-port $drive $steer $arm $gripper"
        exit()

    # open serial port and print info
    ser = serial.Serial((int(sys.argv[1])-1), 115200, timeout=1)

    idx = 0
    error = 0
    count = 10
 
    # drive
    if sys.argv[2] == "fwd":
        mtrs = "+128" + '\0'    # full power fwd
    if sys.argv[2] == "rev":
        mtrs = "-128" + '\0'    # full power rev
    if sys.argv[2] == "stop":
        mtrs = "+000" + '\0'   
    
    # steer
    if sys.argv[3] == "left":
        steer = '0' + '\0'
    if sys.argv[3] == "right":
        steer = '9' + '\0'
    if sys.argv[3] == "center":
        steer = '5' + '\0'

    # arm position
    if sys.argv[4] == "up":
        arm = '1' + '\0'
    if sys.argv[4] == "down":
        arm = '0' + '\0'
        
    # grip sensor, grip position
    if sys.argv[5] == "open":
        grip = '1' + '\0'
    if sys.argv[5] == "close":
        grip = '0' + '\0'        

    # cam tilt
    cam = "000" + '\0'

    # red, yellow, green
    # 1 is off, 0 is on
    red = '0' + '\0'
    yel = '1' + '\0'
    grn = '0' + '\0'
    lights = red + yel + grn
    
    #print "motors", mtrs
    
    data = mtrs + steer + arm + grip + cam + lights
 
    # write message to chassis
    ser.write('W')
    ser.write(data)
 
    #print "here "
 
    # read message for a while
    while idx < count:

        print "index ",idx	
            
        # read and print incoming serial message
        ser.write('R')

        #print "here 2"
        
        # discard serial bytes until framing byte is detected
        while ser.read(1) != 'F':
            pass
        
        inMsg = ser.read(len(data))
        print len(inMsg)
        print inMsg
        inMsg = 0
        
        idx += 1
    
if __name__ == "__main__":
    ThorSer()