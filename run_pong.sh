#export FLASK_APP=flask-pong.py
#sudo -E flask run --host=0.0.0.0 --port=80

# this way works despite Flask errors - needed for headless RPi
sudo python3 flask-pong.py
