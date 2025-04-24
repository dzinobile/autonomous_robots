from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
import serial
import threading
import heapq
import pigpio
import signal
import sys
from matplotlib import pyplot as plt

robot_position = [0,0,0]

#IMU STARTUP -----------------------------------------------------------
ser = serial.Serial('/dev/ttyUSB0', 9600)
IMU_ready = False
def read_serial_data():
	ser_count = 0 # Initial serial count
	global robot_position
	global IMU_ready # Boolean to prevent any movement until first 10 lines are read
	
	# Continuously read data
	while True:
		ser.reset_input_buffer() # Reset buffer to ignore old readings
		line = ser.readline() # Wait for and read the next new line 
		ser_count +=1
		
		if ser_count > 10:
			
			# Strip unneccessary info
			line = line.rstrip().lstrip()
			line = str(line)
			line = line.strip("'")
			line = line.strip("b'")
			line = line.strip("X: ")
			
			robot_position[2] = round(float(line)) # Update current angle global variable
			IMU_ready = True # First 10 lines have been read
			
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Pause until first 10 serial readings are taken
print("Starting IMU...")
while True:
	if IMU_ready:
		print("Complete")
		break
#-----------------------------------------------------------------------
#GPIO STARTUP-----------------------------------------------------------
pi = pigpio.pi()

trig = 23 #16
echo = 24 #18
lf = 6
lb = 13
rb = 19
rf = 26
gr = 16
re = 18
le = 4

pi.set_mode(lf, pigpio.OUTPUT) #31 lf
pi.set_mode(lb, pigpio.OUTPUT)#33 lb
pi.set_mode(rb, pigpio.OUTPUT)#35 rb
pi.set_mode(rf, pigpio.OUTPUT)#37 rf
pi.set_mode(gr, pigpio.OUTPUT)#36 gripper
pi.set_mode(trig, pigpio.OUTPUT)
pi.set_mode(echo, pigpio.INPUT)
pi.set_mode(re, pigpio.INPUT)#12 right encoder
pi.set_pull_up_down(re, pigpio.PUD_UP)
pi.set_mode(le, pigpio.INPUT)  #7 left encoder
pi.set_pull_up_down(le, pigpio.PUD_UP)

pi.set_PWM_frequency(lf,50)
pi.set_PWM_frequency(lb,50)
pi.set_PWM_frequency(rb,50)
pi.set_PWM_frequency(rf,50)
pi.set_PWM_frequency(gr,50)

pi.set_PWM_dutycycle(lf,0)
pi.set_PWM_dutycycle(lb,0)
pi.set_PWM_dutycycle(rb,0)
pi.set_PWM_dutycycle(rf,0)
pi.set_PWM_dutycycle(gr,int(round(0.08*255)))

def gameover():
	pi.write(lf,0)
	pi.write(lb,0)
	pi.write(rb,0)
	pi.write(rf,0)
	pi.write(gr,0)
	pi.write(trig,0)
	pi.stop()

def emergency_stop(signal_received, frame):
	print("Emergency stop")
	gameover()
	sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)

#-----------------------------------------------------------------------





#BLOCK POSITIONS TRACKER STARTUP----------------------------------------
blue_positions = [(0,0),(0,0),(0,0)]
green_positions = [(0,0),(0,0),(0,0)]
red_positions = [(0,0),(0,0),(0,0)]
rotate_target_closest_blue = 0
pixel_y_closest_blue = 0
rotate_target_closest_green = 0
pixel_y_closest_green = 0
rotate_target_closest_red = 0
pixel_y_closest_red = 0
update_blue = True
update_green = True
update_red = True


def euc_distance(p1,p2):
	p1x,p1y = p1[:2]
	p2x,p2y = p2[:2]
	dist = np.sqrt((p1x-p2x)**2 + (p1y-p2y)**2)
	dist = int(round(dist))
	return dist
	
def average(p1,p2):
	p1x,p1y = p1
	p2x,p2y = p2
	x = int(round((p1x+p2x)/2))
	y = int(round((p1y+p2y)/2))
	return (x,y)

def update_positions(color,coordinates):
	if color == 0:
		if blue_positions[0] == (0,0):
			blue_positions[0] = coordinates
			return
		if euc_distance(blue_positions[0],coordinates) < 20:
			blue_positions[0] = average(blue_positions[0],coordinates)
		else:
			if blue_positions[1] == (0,0):
				blue_positions[1] = coordinates
				return
			if euc_distance(blue_positions[1],coordinates) < 20:
				blue_positions[1] = average(blue_positions[1],coordinates)
			else:
				if blue_positions[2] == (0,0):
					blue_positions[2] = coordinates
					return
				if euc_distance(blue_positions[2],coordinates) < 20:
					blue_positions[2] = average(blue_positions[2],coordinates)
		
		
		
	elif color == 1:
		if green_positions[0] == (0,0):
			green_positions[0] = coordinates
			return
		if euc_distance(green_positions[0],coordinates) < 20:
			green_positions[0] = average(green_positions[0],coordinates)
		else:
			if green_positions[1] == (0,0):
				green_positions[1] = coordinates
				return
			if euc_distance(green_positions[1],coordinates)<20:
				green_positions[1] = average(green_positions[1],coordinates)
			else:
				if green_positions[2] == (0,0):
					green_positions[2] = coordinates
					return
				if euc_distance(green_positions[2],coordinates)<20:
					green_positions[2] = average(green_positions[2],coordinates)
		
		
	else:
		if red_positions[0] == (0,0):
			red_positions[0] = coordinates
			return
		if euc_distance(red_positions[0],coordinates) < 20:
			red_positions[0] = average(red_positions[0],coordinates)
		else:
			if red_positions[1] == (0,0):
				red_positions[1] = coordinates
				return
			if euc_distance(red_positions[1],coordinates)<20:
				red_positions[1] = average(red_positions[1],coordinates)
			else:
				if red_positions[2] == (0,0):
					red_positions[2] = coordinates
					return
				if euc_distance(red_positions[2],coordinates)<20:
					red_positions[2] = average(red_positions[2],coordinates)
		

def object_tracker():

	global blue_positions
	global green_positions
	global red_positions
	global rotate_target_closest_blue
	global rotate_target_closest_green
	global rotate_target_closest_red
	global pixel_y_closest_blue
	global pixel_y_closest_green
	global pixel_y_closest_red
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 12
	rawCapture = PiRGBArray(camera, size=(640,480))
	out = None
	recording = False
	
	#warmup
	time.sleep(0.1)
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = None
	
	blue_lower = np.array([70,47,0]) 
	blue_upper = np.array([115,255,255])
	green_lower = np.array([21,81,135])
	green_upper = np.array([53,255,255])
	red_lower = np.array([161,123,99])
	red_upper = np.array([192,255,255])
	
	
	
	
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		image = cv2.flip(image, -1)
		image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		image_hsv[0:220,:] = (0,0,0)
		#image_hsv[455:480,:] = (0,0,0)
		blue_mask = image_hsv.copy()
		green_mask = image_hsv.copy()
		red_mask = image_hsv.copy()

		blue_mask = cv2.inRange(blue_mask, blue_lower, blue_upper)
		green_mask = cv2.inRange(green_mask, green_lower, green_upper)
		red_mask = cv2.inRange(red_mask, red_lower, red_upper)
		 
		blue_cont, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		green_cont,_ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		red_cont, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		min_area = 150
		blue_contours = [cnt for cnt in blue_cont if cv2.contourArea(cnt) > min_area]
		green_contours = [cnt for cnt in green_cont if cv2.contourArea(cnt) > min_area]
		red_contours = [cnt for cnt in red_cont if cv2.contourArea(cnt) > min_area]

		if len(blue_contours) > 0:
			blue_contours = sorted(blue_contours, key=cv2.contourArea, reverse=True)[:(min(3,len(blue_contours)))]
			b_i = 0
			for contour in blue_contours:
				rect = cv2.minAreaRect(contour)
				(rect_x,rect_y),(rect_w,rect_h),rect_a = rect
				rotate_target = round((320-rect_x)*0.061)
				distance = 5360.8*(rect_h**-1.219)+23
				block_angle = (robot_position[2]-rotate_target)%360
				block_x = int(round((distance*np.cos(np.deg2rad(block_angle)))+robot_position[0]))
				block_y = int(round((distance*np.sin(np.deg2rad(block_angle)))+robot_position[1]))
				block_coords = (block_x,block_y)
				if update_block_positions:
					update_positions(0,block_coords)
				box = cv2.boxPoints(rect)
				box = np.int0(box)
				cv2.drawContours(image, [box], 0, (255,0,0), 1)
				cv2.putText(image,str(block_coords),(int(rect_x+rect_w/2),int(rect_y+rect_h/2)),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1)
				if b_i == 0:
					rotate_target_closest_blue = rotate_target
					pixel_y_closest_blue = rect_y
				i += 1
				
		if len(green_contours) > 0:
			green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)[:(min(3,len(green_contours)))]
			g_i = 0
			for contour in green_contours:
				rect = cv2.minAreaRect(contour)
				(rect_x,rect_y),(rect_w,rect_h),rect_a = rect
				rotate_target = round((320-rect_x)*0.061)
				distance = 5360.8*(rect_h**-1.219)+23
				block_angle = (robot_position[2]-rotate_target)%360
				block_x = int(round((distance*np.cos(np.deg2rad(block_angle)))+robot_position[0]))
				block_y = int(round((distance*np.sin(np.deg2rad(block_angle)))+robot_position[1]))
				block_coords = (block_x,block_y)
				if update_block_positions:
					update_positions(1,block_coords)
				box = cv2.boxPoints(rect)
				box = np.int0(box)
				cv2.drawContours(image, [box], 0, (0,255,0), 1)
				if g_i == 0:
					rotate_target_closest_green = rotate_target
					pixel_y_closest_green = rect_y
				i += 1
				
		if len(red_contours) > 0:
			red_contours = sorted(red_contours, key=cv2.contourArea, reverse=True)[:(min(3,len(red_contours)))]
			r_i = 0
			for contour in red_contours:
				rect = cv2.minAreaRect(contour)
				(rect_x,rect_y),(rect_w,rect_h),rect_a = rect
				rotate_target = round((320-rect_x)*0.061)
				distance = 5360.8*(rect_h**-1.219)+23
				block_angle = (robot_position[2]-rotate_target)%360
				block_x = int(round((distance*np.cos(np.deg2rad(block_angle)))+robot_position[0]))
				block_y = int(round((distance*np.sin(np.deg2rad(block_angle)))+robot_position[1]))
				block_coords = (block_x,block_y)
				if update_block_positions:
					update_positions(2,block_coords)
				box = cv2.boxPoints(rect)
				box = np.int0(box)
				cv2.drawContours(image, [box], 0, (0,0,255), 1)	
				if r_i == 0:
					rotate_target_closest_red = rotate_target
					pixel_y_closest_red = rect_y
				r_i += 1
		cv2.putText(image,"Robot position: "+str(robot_position),(10,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),1))	
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF
		rawCapture.truncate(0)
		print("blue: "+str(blue_positions))
		print("green: "+str(green_positions))
		print("red: "+str(red_positions))
		
		if key == ord("q"):
			break
	cv2.destroyAllWindows()
		
tracking_thread = threading.Thread(target=object_tracker)
tracking_thread.start()

#MOVEMENT COMMANDS------------------------------------------------------



def turn_by_angle(angle):
	
	start_angle = robot_position[2]
	if angle < 0:
		rw = rb
		lw = lf
	else:
		rw = rf
		lw = lb
	duty = int(round(0.7*255))
	i = 0
	while True:

		angle_turned = ((current_angle_imu-start_angle+540)%360-180)*-1 # Negative output = moved clockwise 
		delay = 0.5 - (0.1*(int((abs(angle_turned)/abs(angle))*5)))
		if delay > 0.5:
			delay = 0.5
			
		if angle-angle_turned > 2 or angle-angle_turned < -2:
			pi.set_PWM_dutycycle(lw,duty)
			pi.set_PWM_dutycycle(rw,duty)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lw,0)
			pi.set_PWM_dutycycle(rw,0)
			i = 0
		else:
			pi.set_PWM_dutycycle(lw,0)
			pi.set_PWM_dutycycle(rw,0)
			i += 1
		if i > 5:
			break
		time.sleep(0.3)
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(rb,0)
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(lb,0)
	
def move_forward_by_distance(inpt_dist):
	global robot_position
	start_angle = robot_position[2]
	travel = 0
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)

	Kp = 0.7
	integral = 0
	previous_error = 0
	left_duty = int(round(0.7*255))
	right_duty = int(round(0.7*255))
	pi.set_PWM_dutycycle(lf,left_duty)
	pi.set_PWM_dutycycle(rf,right_duty)

	while travel < inpt_dist:
		error = left_counter - right_counter
		# Update tick count if status changes
		if int(pi.read(le)) != int(left_button):
			left_button = int(pi.read(le))
			left_counter += 1
		if int(pi.read(re)) != int(right_button):
			right_button = int(pi.read(re))
			right_counter += 1

	
		correction = int(round(Kp*error))
		
		left_duty -= correction
		right_duty += correction
		
		
		left_duty = max(min(left_duty, 255),30)
		right_duty = max(min(right_duty, 255),30)
		
		pi.set_PWM_dutycycle(lf,left_duty)
		pi.set_PWM_dutycycle(rf,right_duty)
		
		left_travel =  (left_counter/20)*(3.1415*6.5)
		right_travel = (right_counter/20)*(3.1415*6.5)
		travel = (left_travel+right_travel)/2
	robot_position[0] = int(round(robot_position[0]+(travel*np.cos(np.deg2rad(start_angle)))))
	robot_position[1] = int(round(robot_position[1]+(travel*np.sin(np.deg2rad(start_angle)))))
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(lb,0)
	pi.set_PWM_dutycycle(rb,0)	

def move_backward_by_distance(inpt_dist):
	global robot_position
	start_angle = robot_position[2]
	travel = 0
	left_counter = np.uint64(0)
	left_button = int(0)
	right_counter = np.uint64(0)
	right_button = int(0)

	Kp = 0.7
	integral = 0
	previous_error = 0
	left_duty = int(round(0.7*255))
	right_duty = int(round(0.7*255))
	pi.set_PWM_dutycycle(lb,left_duty)
	pi.set_PWM_dutycycle(rb,right_duty)
	while travel < inpt_dist:
		error = left_counter - right_counter
		# Update tick count if status changes
		if int(pi.read(le)) != int(left_button):
			left_button = int(pi.read(le))
			left_counter += 1
		if int(pi.read(re)) != int(right_button):
			right_button = int(pi.read(re))
			right_counter += 1

	
		correction = int(round(Kp*error))
		
		left_duty += correction
		right_duty -= correction
		left_duty = max(min(left_duty, 255),30)
		right_duty = max(min(right_duty, 255),30)
		
		pi.set_PWM_dutycycle(lb,left_duty)
		pi.set_PWM_dutycycle(rb,right_duty)
		left_travel =  (left_counter/20)*(3.1415*6.5)
		right_travel = (right_counter/20)*(3.1415*6.5)
		travel = (left_travel+right_travel)/2
	robot_position[0] = int(round(robot_position[0]-(travel*np.cos(np.deg2rad(start_angle)))))
	robot_position[1] = int(round(robot_position[1]-(travel*np.sin(np.deg2rad(start_angle)))))
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(lb,0)
	pi.set_PWM_dutycycle(rb,0)

	
def move_forward_to_point(point_xy,isblock):
	(x1,y1) = current_position[0],current_position[1]
	(x2,y2) = point_xy
	current_angle = current_position[2]
	x3 = x2-x1
	y3 = y2-y1
	angle_to_reach = np.degrees(np.arctan2(y3,x3))
	if angle_to_reach < 0:
		angle_to_reach +=360
	angle_amount_to_turn = int(round(angle_to_reach - current_angle))
	distance_to_move = int(round(euc_distance((x1,y1),point_xy)))
	if isblock:
		distance_to_move -= 28
	turn_by_angle(angle_amount_to_turn)
	move_forward_by_distance(distance_to_move)
	

	
def pick_up_block(cycle):
	duty = int(round(0.5*255))
	i = 0
	color = cycle%3
	while True:
		if color == 0:
			rot_targ = rotate_target_closest_blue
		elif color == 1:
			rot_targ = rotate_target_closest_green
		elif color == 2:
			rot_targ = rotate_target_closest_red
		delay = abs(rot_targ)*0.05
		if rot_targ <= -2:
			pi.set_PWM_dutycycle(lf,duty)
			pi.set_PWM_dutycycle(rb,duty)
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			i = 0
		elif rot_targ >= 2:
			pi.set_PWM_dutycycle(lb,duty)
			pi.set_PWM_dutycycle(rf,duty)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			time.sleep(delay)
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			i = 0
		else:
			pi.set_PWM_dutycycle(lb,0)
			pi.set_PWM_dutycycle(rf,0)
			pi.set_PWM_dutycycle(lf,0)
			pi.set_PWM_dutycycle(rb,0)
			i += 1
		if i > 5:
			break
		time.sleep(0.3)
		
	while True:
		if color == 0:
			pix_y = pixel_y_closest_blue
		elif color == 1:
			pix_y = pixel_y_closest_green
		elif color == 2:
			pix_y = pixel_y_closest_red
			
		if pix_y >= 420:
			break
			
		pi.set_PWM_dutycycle(rf,duty)
		pi.set_PWM_dutycycle(lf,duty)
		time.sleep(0.2)
		pi.set_PWM_dutycycle(rf,0)
		pi.set_PWM_dutycycle(lf,0)
		time.sleep(0.3)
		
	pi.set_PWM_dutycycle(rf,0)
	pi.set_PWM_dutycycle(rb,0)
	pi.set_PWM_dutycycle(lf,0)
	pi.set_PWM_dutycycle(lb,0)
	pi.set_PWM_dutycycle(gr,int(round(0.04*255)))
	time.sleep(2)
	pi.set_PWM_dutycycle(gr,int(round(0.08*255)))
	time.sleep(2)
	pi.set_PWM_dutycycle(gr,int(round(0.04*255)))
	time.sleep(2)
#-----------------------------------------------------------------------

# ULTRASONIC SENSOR SETUP-----------------------------------------------
def ping_distance():
	distance_readings = []
	for i in range(0,21):
		pi.write(trig,0)
		time.sleep(0.01)
		
		pi.write(trig,1)
		time.sleep(0.00001)
		pi.write(trig,0)
		
		pulse_start = time.time()
		while pi.read(echo) == 0:
			pulse_start = time.time()
		while pi.read(echo) == 1:
			pulse_end = time.time()
		
		pulse_duration = pulse_end - pulse_start
		sonar_distance = pulse_duration*17150
		sonar_distance = round(sonar_distance, 2)
		distance_readings.append(sonar_distance)
	sonar_distance = np.median(distance_readings)+10
	return int(round(sonar_distance))



#-----------------------------------------------------------------------

# PATH FINDING ALGORITHM------------------------------------------------

const_home = (61,305)
land_home = (30,30)

def get_goal_block(cycle_num,blue_blocks,green_blocks,red_blocks):
    color = cycle_num%3
    temp_heap = []
    remaining_block_coords = []
    if color == 0:
        print("blue")
        block_list = blue_blocks
    elif color == 1:
        print("green")
        block_list = green_blocks   
    elif color == 2:
        print("red")
        block_list = red_blocks
        
    for block in block_list:
        if block != (0,0):
            d = euc_distance(block,const_home)
            heapq.heappush(temp_heap,(d,block))
    goal = heapq.heappop(temp_heap)[1]
    for block in block_list:
        if block == goal:
            block_list.remove(block)
            block_list.append((0,0))
    if color == 0:
        blue_blocks = block_list
    elif color == 1:
        green_blocks = block_list
    elif color == 2:
        red_blocks = block_list
    for block in blue_blocks:
        if block != (0,0):
            remaining_block_coords.append((block))
    for block in green_blocks:
        if block != (0,0):
            remaining_block_coords.append((block))
    for block in red_blocks:
        if block != (0,0):
            remaining_block_coords.append((block))
    
    return goal,blue_blocks,green_blocks,red_blocks,remaining_block_coords
    


def find_buffer_set(block_coords):
    buffer_set = set()
    b_map = np.zeros((366,366,1),dtype=np.uint8)
    b_map  = cv2.rectangle(b_map,(0,0),(365,365),(255,255,255),20)
    
    for item in block_coords:
        b_map = cv2.circle(b_map,item,23,255,-1)
    
    buffer_coords = np.where(b_map==255)
    for i in range(0,len(buffer_coords[0])):
        buffer_set.add((buffer_coords[1][i],buffer_coords[0][i]))
    return b_map, buffer_set
	
def line_cross(p1,p2,inpt_bufferset):
    lc_map = np.zeros((h,w,1))
    lc_map = cv2.line(lc_map,p1[:2],p2[:2],255,1)
    line_coords = np.where(lc_map==255)
    for i in range(0,len(line_coords[0])):
        if (line_coords[1][i],line_coords[0][i]) in inpt_bufferset:
            return True
    return False

def get_child_coords(inpt_point,inpt_angle,inpt_step):
    px,py = inpt_point[:2]
    cx = int(round(inpt_step*np.cos(np.deg2rad(inpt_angle))+px))
    cy = int(round(inpt_step*np.sin(np.deg2rad(inpt_angle))+py))
    return((cx,cy,inpt_angle))


def m_coords(point):
    x,y,t = point
    t = int(t/45)
    return y,x,t
    
def cv2_to_xy(point):
	x,y = point
	y = 366 - y
	return (x,y)
	
def xy_to_cv2(point):
	x,y = point
	y = abs(y-366)
	return (x,y)

def a_star(goal,step_dist,block_coords_xy):
    start_xy = (robot_position[0],robot_position[1])
    start_xy = xy_to_cv2(start_xy)
    start_t = (360-robot_position[2])%360
    start_xyt = (start_xy[0],start_xy[1],start_t)
    goal = xy_to_cv2(goal)
    
    block_coords = []
    for item in block_coords_xy:
        block_coords.append((xy_to_cv2(item)))
    
    
    h = 366
    w = 366
    r_radius = 20
    b_radius = 3
    a_map,buffer_set = find_buffer_set(block_coords)
	
    m_c2c = np.zeros((h,w,8))
    m_ctot = np.zeros((h,w,8))
    m_px = np.zeros((h,w,8))
    m_py = np.zeros((h,w,8))
    m_pt = np.zeros((h,w,8))
    
    open_list = []
    closed_list = []
    heapq.heapify(open_list)
    heapq.heapify(closed_list)
    first_node = (euc_distance(start_xyt,goal),start_xyt)
    heapq.heappush(open_list,first_node)
    m_c2c[m_coords(start_xyt)] = 0
    m_ctot[m_coords(start_xyt)] = first_node[0]
    m_px[m_coords(start_xyt)] = start_xyt[0]
    m_py[m_coords(start_xyt)] = start_xyt[1]
    m_pt[m_coords(start_xyt)] = start_xyt[2]
    
    while True:

        parent_node = heapq.heappop(open_list)
        heapq.heappush(closed_list,parent_node)
        
        p_xyt = parent_node[1]
        if euc_distance(p_xyt,goal) < 20:
            break
        for item in [-90, -45, 0, 45, 90]:
            ang = p_xyt[2] + item
            ang = int(ang%360)
            c_xyt = get_child_coords(p_xyt,ang,step_dist)
            a_map = cv2.line(a_map,p_xyt[:2],c_xyt[:2],255,1)
            cv2.imshow('amap',a_map)
            cv2.waitKey(1)
            #print(c_xyt)
            if c_xyt[:2] in buffer_set:
                #print("in buffer set")
                continue
            if line_cross(p_xyt,c_xyt,buffer_set):
                #print("line cross")
                continue
            c_c2c = int(round(m_c2c[m_coords(p_xyt)] + step_dist))
            c_ctg = euc_distance(c_xyt,goal)
            c_ctot = c_c2c+(c_ctg)
            c_node = (c_ctot,c_xyt)
            previous_ctot = m_ctot[m_coords(c_xyt)]
            if previous_ctot == 0:
                heapq.heappush(open_list,c_node)
                #print("push1")
                m_c2c[m_coords(c_xyt)] = c_c2c
                m_ctot[m_coords(c_xyt)] = c_ctot
                m_px[m_coords(c_xyt)] = int(p_xyt[0])
                m_py[m_coords(c_xyt)] = int(p_xyt[1])
                m_pt[m_coords(c_xyt)] = int(p_xyt[2])
            else:
                if previous_ctot > c_ctot:
                    previous_node = (previous_ctot,c_xyt)
                    open_list.remove(previous_node)
                    heapq.heappush(open_list,c_node)
                    #print("push2")
                    m_c2c[m_coords(c_xyt)] = c_c2c
                    m_ctot[m_coords(c_xyt)] = c_ctot
                    m_px[m_coords(c_xyt)] = p_xyt[0]
                    m_py[m_coords(c_xyt)] = p_xyt[1]
                    m_pt[m_coords(c_xyt)] = p_xyt[2]
    final_path = []
    path_xyt = p_xyt
    while path_xyt != start_xyt:
        final_path.append(path_xyt[:2])
        px = int(m_px[m_coords(path_xyt)])
        py = int(m_py[m_coords(path_xyt)])
        pt = int(m_pt[m_coords(path_xyt)])
        path_xyt = (px,py,pt)
    final_path.reverse()
        
    final_path_xy = []
    for item in final_path:
        final_path_xy.append((cv2_to_xy(item)))
	
    return final_path_xy

def find_next_block_path(cycle,blue_blocks,green_blocks,red_blocks):
	goal,blue_blocks,green_blocks,red_blocks,remain_blocks = get_goal_block(cycle,blue_blocks,green_blocks,red_blocks)
	step_size = 30
	while True:
		try:
			robot_path = a_star(goal,step_size,remain_blocks)
			break
		except:
			step_size -= 5
	return robot_path,remain_blocks 
#-----------------------------------------------------------------------

#HOME POSITION ---------------------------------------------------------
def home_position():
	angle_to_wall = 180 - robot_position[2]
	turn_by_angle(angle_to_wall)
	x_pos = ping_distance()
	turn_by_angle(90)
	y_pos = ping_distance()
	return x_pos,y_pos

#-----------------------------------------------------------------------
def switch_updater(update_block_pos):
	
	if update_block_pos:
		return False
	else:
		return True

def grab_block(block_path,cycle):
	isblock = False
	for item in robot_path:
		if item == robot_path[-1]:
			isblock = True
		move_forward_to_point(item,isblock)
	pick_up_block(cycle)

def re_home(remain_block_pos):
	step_size = 30
	while True:
		try:
			robot_path = a_star(land_home,step_size,remain_block_pos)
			break
		except:
			step_size -= 5
	for item in robot_path:
		move_forward_to_point(item,False)
	x_pos,y_pos = home_position()
	return x_pos, y_pos

def drop_off(cycle,remain_block_pos):
	pre_goal_x = 30*(1+(cycle%3))
	pre_goal_y = 260
	pre_goal_coords = (pre_goal_x,pre_goal_y)
	step_size = 30
	while True:
		try:
			robot_path = a_star(pre_goal_coords,step_size,remain_block_pos)
			break
		except:
			step_size -= 5
	for item in robot_path:
		move_forward_to_point(item,False)
	turn_by_angle((90-robot_position[2]))
	move_dist = 60-(10*(cycle%3))
	move_forward_by_distance(move_dist)
	pi.set_PWM_dutycycle(gr,int(round(0.08*255)))
	time.sleep(2)
	move_backward_by_distance(move_dist)
	turn_by_angle(90)
	move_backward_by_distance(122-robot_position[0])
	
	
	

# SEQUENTIAL ACTIONS
# start facing construction zone
# turn 90, ping and update x position
# turn 90, ping and update y position
# turn 180

# turn off block position updating
# path plan to nearest block
# move along path
# grab block
# path plan to first dropoff
# move along path
# release block and reverse to 122,122
# turn to face left 
# turn on block position updating 
# sweep 270


# repeat above until all complete

blocks_acquired = 0
update_block_positions = True

robot_position[0],robot_position[1] = home_position()
turn_by_angle(180)
update_block_positions = switch_updater(update_block_positions)
robot_path,remaining_block_coords = find_next_block_path(blocks_acquired,blue_positions,green_positions,red_positions)
grab_block(robot_path,blocks_acquired)
drop_off(blocks_acquired,remaining_block_coords)
blocks_acquired += 1
update_block_positions = switch_updater(update_block_positions)
turn_by_angle(270)

while blocks_acquired < 9:
	update_block_positions = switch_updater(update_block_positions)
	robot_path,remaining_block_coords = find_next_block_path(blocks_acquired,blue_positions,green_positions,red_positions)
	grab_block(robot_path,blocks_acquired)
	robot_position[0],robot_position[1] = re_home(remaining_block_coords)
	drop_off(blocks_acquired,remaining_block_coords)
	blocks_acquired += 1
	update_block_positions = switch_updater(update_block_positions)
	turn_by_angle(270)

