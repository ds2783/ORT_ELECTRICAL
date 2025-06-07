import sys
import time

from StreamServer import StreamServer
from Comms.Output import Output

from signal import signal, SIGINT

BASE_IP = '192.168.0.103'
# config
MULTICAST=False
PORT1=5008
PORT2 = 5020

MODEL="imx708_noir" # camera model name (find using libcamera-vid --list-cameras)
WIDTH=1296
HEIGHT=972
NAME="camera" # stream name for display in console

MODEL2="imx219"
IP1 = IP2 = IP_SECONDARY = BASE_IP
PORT_SECONDARY = 5030
NAME2="secondary"

out=Output("None") # console output, with optional TCP forwarding

# Initialise stream
stream=StreamServer(out,MODEL,NAME) # system finds the camera based upon the model number (assumes no duplicates)
stream.configure(WIDTH,HEIGHT)
stream.start_two(IP1, PORT1, IP2, PORT2) # using a multicast address 224.1.1.1:5008
stream.set_bitrate(5000000)

stream2=StreamServer(out,MODEL2,NAME2)
stream2.configure(WIDTH, HEIGHT)
stream2.start(IP_SECONDARY, PORT_SECONDARY)
stream2.set_bitrate(5000000)

#stream.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # set a libcamera control

# exit handler
def handler(signal_received,frame):
    stream.stop()
    sys.exit()

signal(SIGINT, handler)

while True:
    time.sleep(1)
