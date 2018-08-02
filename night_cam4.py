# USAGE
# python night_cam2.py  --conf conf.json

# import the necessary packages
from pyimagesearch.tempimage import TempImage
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Timer
import argparse
import warnings
import datetime
import imutils
import json
import time
import cv2
import shutil
import os
import sys

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=True,
                help="path to the JSON configuration file")
args = vars(ap.parse_args())

# filter warnings, load the configuration and initialize the Dropbox
# client
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"]))
frame = None
camera = None


def initCamera():
    global camera
    # Initialise the camera
    camera = PiCamera()
    camera.resolution = tuple(conf["resolution"])
    camera.framerate = conf["fps"]
    camera.rotation = conf["rotation"]
    # allow the camera to warmup
    print("[INFO] warming up...")
    time.sleep(conf["camera_warmup_time"])
    return


def lookForMotion():
    global frame
    frame_count = 0
    state_motion = "Motion Detected"
    state_still = "No motion"

    # Grab a reference to the raw camera capture
    rawCapture = PiRGBArray(camera, size=tuple(conf["resolution"]))

    # Initialize the average frame, last uploaded timestamp, and frame motion counter
    avg = None
    lastUploaded = datetime.datetime.now()
    motionCounter = 0

    # capture frames from the camera
    for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image and initialize
        # the timestamp and occupied/unoccupied text
        frame = f.array
        timestamp = datetime.datetime.now()
        text = state_still

        # resize the frame, convert it to grayscale, and blur it
        frame = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # if the average frame is None, initialize it
        if avg is None:
            print("[INFO] starting background model...")
            avg = gray.copy().astype("float")
            rawCapture.truncate(0)
            continue

        # accumulate the weighted average between the current frame and
        # previous frames, then compute the difference between the current
        # frame and running average
        cv2.accumulateWeighted(gray, avg, 0.5)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))

        # threshold the delta image, dilate the thresholded image to fill
        # in holes, then find contours on thresholded image
        thresh = cv2.threshold(frameDelta, conf["delta_thresh"], 255,
                               cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        (_, cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        # loop over the contours
        for c in cnts:
            # if the contour is too small, ignore it
            if cv2.contourArea(c) < conf["min_area"]:
                continue

            # compute the bounding box for the contour, draw it on the frame,
            # and update the text
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = state_motion

        # draw the text and timestamp on the frame
        ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
        cv2.putText(frame, "Room Status: {}".format(text), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (0, 0, 255), 1)

        # check to see if the room is occupied
        if text == state_motion:
            # check to see if enough time has passed between uploads
            if (timestamp - lastUploaded).seconds >= conf["min_upload_seconds"]:
                # increment the motion counter
                motionCounter += 1

                # check to see if the number of frames with consistent motion is
                # high enough
                if motionCounter >= conf["min_motion_frames"]:
                    # Motion detected
                    saveToFile()
                    return True

            # otherwise, the room is not occupied
            else:
                motionCounter = 0

        # check to see if the frames should be displayed to screen
        if conf["show_video"]:
            # display the security feed
            cv2.imshow("Night Camera", frame)
            key = cv2.waitKey(1) & 0xFF

            # if the `q` key is pressed, break from the loop
            if key == ord("q"):
                return False

        if frame_count > 10:
            frame_count = 0
            # Dump each frame to file
            saveToFile()
        frame_count = frame_count + 1

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

    return False


def saveToFile():
    if conf["dump_picture"]:
        print("Saving image to file")
        # write picture to file
        cv2.imwrite(conf["dump_file"], frame)
        print("Done!")

    return


def saveToVideo():
    global frame

    print("Triggered, recording {sec} seconds of video".format(sec=conf["video_length"]))
    if conf["show_video"]:
        cv2.putText(frame, "Triggered, recording {sec} seconds of video".format(sec=conf["video_length"]), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("Night Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key is pressed, break from the loop
        if key == ord("q"):
            return

    # Save video to file
    #t = TempImage(ext=".h264")
    timestamp = datetime.datetime.now()
    rawFile = "{base_path}/{timestamp}{ext}".format(base_path=sys.path[0],
			timestamp=timestamp.strftime("%y-%m-%d %H%M%S"), ext=".h264")

    print("Saving to RAW file" + rawFile.path)
    tmr = Timer(conf["video_length"] * 2, timeout);

    camera.start_recording(rawFile.path, quality=10, bitrate=17000000)

    camera.wait_recording(conf["video_length"])
    camera.stop_recording()
    tmr.cancel()

    # Now use MP4Box to convert to MP4
    print("Converting to MP4")
    mp4File = rawFile.path.replace(".h264", ".mp4")
    os.system("MP4Box -add {source} {dest} -fps {fps}".format(source=rawFile.path, dest=mp4File, fps=conf["fps"]))


    destFile = "{dest}/{timestamp}.mp4".format(dest=conf["folder_path"], timestamp=ts)

    try:
        print("Copying to " + destFile)
        shutil.copyfile(mp4File, destFile)
        os.remove(mp4File)
    except:
        print("Error copying file to {dest}".format(dest=conf["folder_path"]))

    # Clean up temporary file
    os.remove(rawFile.path)
    return


def timeout():
    # Restart
    print("Timeout")


# START MAIN PROGRAM LOOP
print("STARTING MAIN LOOP")
initCamera()
# capture on startup
saveToVideo()

while lookForMotion():
    try:
        current_time = datetime.datetime.now().time()
        print("Current hour: ", current_time.hour)
        if current_time.hour < 8 or current_time.hour >= 22:
            if conf["use_folder"]:
                if conf["save_as_video"]:
                    saveToVideo()
                else:
                    saveToFile()
        else:
            print("Not within capture hours 9pm to 8am")
    except:
        print("Unexpected error:", sys.exc_info()[0])
