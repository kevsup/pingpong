import atexit
import threading
import time
import pingpong
from pingpong_constants import *

# work in progress. right now just testing multithreading

def startIt():
    time.sleep(3)
    print('starting!')
    pingpong.setRunning(True)
    time.sleep(5)
    print('bh only')
    pingpong.setShootingSequence([STATES['BACKHAND']])
    time.sleep(15)
    pingpong.shutdown()
    time.sleep(1)

def doPong():
    pingpong.setup()
    pingpong.finiteStateMachine()

pongThread = threading.Thread(target = doPong)
saberThread = threading.Thread(target = startIt)
saberThread.daemon = True
pongThread.daemon = True
pongThread.start()
saberThread.start()

time.sleep(30)
print('finishing')


