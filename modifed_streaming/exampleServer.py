
import time
import sys

from StreamServer import StreamServer
from Comms.Output import Output

from libcamera import controls

from signal import signal, SIGINT

# config
MULTICAST=False
IP = '192.168.124.110' # UDP multicast IP
PORT=5008
MODEL="ov5647" # camera model name (find using libcamera-vid --list-cameras)
WIDTH=1296
HEIGHT=972
NAME="camera" # stream name for display in console

out=Output("None") # console output, with optional TCP forwarding

# Initialise stream
stream=StreamServer(out,MODEL,NAME) # system finds the camera based upon the model number (assumes no duplicates)
stream.configure(WIDTH,HEIGHT)
stream.start(IP,PORT,MULTICAST) # using a multicast address 224.1.1.1:5008
stream.set_bitrate(5000000)

#stream.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # set a libcamera control

# exit handler
def handler(signal_received,frame):
    stream.stop()
    sys.exit()

signal(SIGINT, handler)

# Example of script to change settings of camera
i=10
while(True): # stream ends once this thread ends
    time.sleep(0.01)
    i+=10
    stream.set_exposure(i)

