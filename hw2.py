#import packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from datetime import datetime

#initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))

#warmup
time.sleep(0.1)



#video writier initialization
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
recording = False

#keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	
	#grab current frame
	image = frame.array
	
	#show the frame to our screen
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	
	if key == ord("r") and not recording: #start recording
		recording = True
		rec_start = datetime.now()
		rec_start_string = rec_start.strftime("%d-%m-%Y-%H_%M_%S")
		out = cv2.VideoWriter(rec_start_string+'.avi', fourcc, 10, (640,480))
		print("recording started at "+rec_start_string)
	
	if key == ord("t") and recording: #stop recording
		recording = False
		out.release()
		out = None
		end_time_string = now.strftime("%d-%m-%Y-%H_%M_%S")
		print("recording stopped at "+end_time_string)
		
	if recording:
		out.write(image)
		now = datetime.now()
		rec_time = (now-rec_start).total_seconds()
		print(f"Elapsed time: {rec_time} seconds")
		
	if key == ord("s"):
		scr_time = datetime.now()
		scr_name = scr_time.strftime("%d-%m-%Y-%H_%M_%S")
		cv2.imwrite(scr_name+".jpg", image)
		print("screenshot captured")
		
	
	
	#clear the stream for next frame
	rawCapture.truncate(0)
	
	#press the q key to stop the stream
	if key == ord("q"):
		break
	
if recording:
	out.release()
cv2.destroyAllWindows()
