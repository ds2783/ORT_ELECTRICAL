import sys

from StreamServer import StreamServer
from Comms.Output import Output

from signal import signal, SIGINT
from threading import Thread

# Laptop IP
BASE_IP = '192.168.0.205'
PI_IP = '192.168.0.101'
# config
MULTICAST=False
PORT_MAIN_BASE=5008
PORT_MAIN=5020

WIDTH=1296
HEIGHT=972
NAME="camera" # stream name for display in console

MODEL="imx219"
IP_MAIN = IP_SECONDARY = BASE_IP

out=Output("None") # console output, with optional TCP forwarding

# Initialise stream
stream=StreamServer(out,MODEL,NAME) # system finds the camera based upon the model number (assumes no duplicates)
stream.configure(WIDTH,HEIGHT)
stream.start_stream(IP_MAIN, PORT_MAIN) # using a multicast address 224.1.1.1:5008
stream.start_server(PI_IP, PORT_MAIN_BASE)
stream.set_bitrate(5000000)

#stream.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # set a libcamera control

# exit handler
def handler(signal_received,frame):
    global run
    run = False

    stream.stop()
    sys.exit()

signal(SIGINT, handler)

run = True
while run:
    try:
        stream.run()
    except:
        pass
    # time.sleep(1)
