import atexit
import threading
import time
from flask import request
from flask_api import FlaskAPI
from subprocess import call
import pingpong
from pingpong_constants import *

app = FlaskAPI(__name__)

@app.route('/pingpong/', methods = ['GET'])
def api_root():
    return {}

@app.route('/pingpong/toggle/', methods = ['GET', 'POST'])
def api_toggle():
    if request.method == 'POST':
        action = request.data.get('action')
        if action == 'start':
            pingpong.setRunning(True)
        else:
            pingpong.setRunning(False)
        return {'running': action == 'start'}
    return {'running': pingpong.getRunning()}

@app.route('/pingpong/shot/', methods = ['GET', 'POST'])
def api_shot():
    if request.method == 'POST':
        location = request.data.get('location')
        if location.lower() == 'forehand' or location.lower() == 'fh':
            pingpong.setShootingSequence([STATES['FOREHAND']])
        elif location.lower() == 'backhand' or location.lower() == 'bh': 
            pingpong.setShootingSequence([STATES['BACKHAND']])
        else:
            pingpong.setShootingSequence(DEFAULT_SEQUENCE)
    seq = pingpong.getShootingSequence()
    seqStr = ''
    for i in range(len(seq)):
        if i == 0:
            seqStr = pingpong.getState(seq[i])
        else:
            seqStr = seqStr + ' -> ' + pingpong.getState(seq[i])
    return {'shot': seqStr}

@app.route('/pingpong/spin/', methods = ['GET', 'POST'])
def api_spin():
    if request.method == 'POST':
        spin = request.data.get('spin')
        if spin.lower() == 'topspin' or spin.lower() == 't':
            pingpong.setSpin(SPINS['TOPSPIN'])
        else:
            pingpong.setSpin(SPINS['BACKSPIN'])
    return {'spin': pingpong.getSpin()}     

@app.route('/pingpong/time/', methods = ['GET', 'POST'])
def api_time():
    if request.method == 'POST':
        delay = int(request.data.get('delay')) # milliseconds
        pingpong.setShootingTime(delay)
    return {'delay': pingpong.getShootingTime()}

@app.route('/pingpong/shutdown/', methods = ['GET'])
def api_shutdown():
    pingpong.shutdown()
    return {'shutdown': True}

def doPong():
    pingpong.setup()
    pingpong.finiteStateMachine()
    pingpong.shutdownRPi()

def startPongThread():
    pongThread = threading.Thread(target = doPong)
    pongThread.daemon = True
    pongThread.start()
    atexit.register(pingpong.GPIO.cleanup)

if __name__ == '__main__':
    try:
        startPongThread()
        app.run(host='0.0.0.0', port=80)
    except Exception as e:
        print(e)
else:
    startPongThread()
