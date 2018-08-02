#!/bin/bash
cd /home/pi/NightCam
source /home/pi/.profile
workon cv
python -u  night_cam4.py -c conf2.json >> nightcam.txt &

