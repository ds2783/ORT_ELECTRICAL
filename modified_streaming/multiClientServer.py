import sys
import time

from StreamServer import StreamServer
from Comms.Output import Output

from signal import signal, SIGINT

# config
MULTICAST=False
IP1 = '192.168.0.103' # UDP multicast IP
PORT1=5008
IP2 = '192.168.0.101'
PORT2 = 5009

MODEL="imx708_noir" # camera model name (find using libcamera-vid --list-cameras)
WIDTH=1296
HEIGHT=972
NAME="camera" # stream name for display in console

ip1 = '192.168.0.103'
port1 = 5007
ip2 = '192.168.0.101'
port2 = 5006

out=Output("None") # console output, with optional TCP forwarding

# Initialise stream
stream=StreamServer(out,MODEL,NAME) # system finds the camera based upon the model number (assumes no duplicates)
stream.configure(WIDTH,HEIGHT)
stream.start_two(IP1, PORT1, IP2, PORT2) # using a multicast address 224.1.1.1:5008
stream.set_bitrate(5000000)

#stream.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # set a libcamera control

# exit handler
def handler(signal_received,frame):
    stream.stop()
    sys.exit()

signal(SIGINT, handler)

while True:
    time.sleep(1)
