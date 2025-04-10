# Import libraries
import RPi.GPIO as gpio
import numpy as np
import time
from matplotlib import pyplot as plt
import serial
import threading
from datetime import datetime
import signal
import sys

# Erase all data from text file
with open("hw8.txt","w") as file:
	file.close()

# Initialize variables for IMU
ser = serial.Serial('/dev/ttyUSB0', 9600)
current_angle_imu = 0
start_angle_imu = 0
start_moving = False

# Function to update current angle from IMU
def read_serial_data():
	ser_count = 0 # Initial serial count
	global current_angle_imu # Current angle reading
	global start_angle_imu # Start angle when movement begins
	global start_moving # Boolean to prevent any movement until first 10 lines are read
	
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
			
			current_angle_imu = float(line) # Update current angle global variable
			start_moving = True # First 10 lines have been read

# GPIO pin initialization function
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT)
	gpio.setup(33, gpio.OUT)
	gpio.setup(35, gpio.OUT)
	gpio.setup(37, gpio.OUT)
	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)

# Function for turning off all pin outputs
def gameover():
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)
	#gpio.cleanup()

def emergency_stop(signal_received, frame):
	print("Emergency stop")
	gameover()
	gpio.cleanup()
	sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)

# Initialize GPIO pins and pwms for both wheel sides and directions
init()
lf_pwm = gpio.PWM(31,50)
rf_pwm = gpio.PWM(37,50)
lb_pwm = gpio.PWM(33,50)
rb_pwm = gpio.PWM(35,50)
time.sleep(0.1)

start_time = datetime.now() # Run start time
elapsed_time = 0 # Initial elapsed time
position = (0,0,0) # Initial x, y, theta position

# Function to current x, y, theta
def update_position(position,action,trav_incrmnt):
	x,y,t = position
	if action == 1:
		x = x+(trav_incrmnt*np.cos(np.radians(t)))
		y = y+(trav_incrmnt*np.sin(np.radians(t)))
	elif action == 2:
		x = x-(trav_incrmnt*np.cos(np.radians(t)))
		y = y-(trav_incrmnt*np.sin(np.radians(t)))
	elif action == 3:
		t = t+trav_incrmnt
		if t > 360:
			t = t-360
	elif action == 4:
		t = t-trav_incrmnt
		if t < 0:
			t = 360+t
	return (x,y,t)
		

# Function to return difference between current and start angle
# Positive result = moved clockwise
# Negative result = moved counterclockwise
def diff(a1,a2):
	difference = (a1-a2+540)%360-180
	return difference


# Function to get goal distance or angle
def get_goal(action):
	
	# Get target distance for forward or backward move
	if action == "dist":
		while True: # Loop until valid input recieved
			goal = float(input("Enter goal distance between 0 and 2 m: "))
			if 0 < goal <= 2:
				break
			else:
				print("Enter goal between range")
	
	# Get target angle for left or right turn
	elif action =="ang":
		while True:
			goal = int(input("Enter goal angle between 0 and 180: "))
			if 0 < goal <= 180: 
				break
			else:
				print("Enter goal between range")
	return goal


# Function to get movement direction
def get_action():

	while True: # Loop until valid inputs recieved
		
		# Get user input for direction
		direction = input((
		"Enter action: \n"+
		" f - forward\n"+
		" b - backward\n"+
		" l - left\n"+
		" r - right\n"+
		" q - no more actions\n"))
		
		# Convert movement direction to interger 1-4
		# Pass user input to get_goal function
		if direction == "f":
			int_dir = 1
			goal = get_goal("dist")
			break
		elif direction == "b":
			int_dir = 2
			goal = get_goal("dist")
			break
		elif direction == "l":
			int_dir = 3
			goal = get_goal("ang")
			break
		elif direction == "r":
			int_dir = 4
			goal = get_goal("ang")
			break
		elif direction == "q":
			int_dir = 0
			goal = None
			break
		else:
			print("Enter f, b, l, r, or q")
	return int_dir,goal # Return direction (1-4) and goal 

#recursions = 0	
# Movement function
# Inputs are a tuple with direction and goal, 
# and a string e or i for encoder-controlled or IMU-controlled
def move(action_tuple,control_type):
	
	# Global variables for recording position and time info
	global start_time
	global position
	
	
	
	with open("hw8.txt","a") as file: # Open text file to append position info
		init() # Initialize gpio pins
		
		# Initialize travel distance based on encoder data
		left_travel = 0
		right_travel = 0
		
		travel = 0 # Current travel distance or angle based on encoder or IMU data
		
		# default duty cycle value and left/right adjustments
		duty = 50 
		left_adjustment = 0
		right_adjustment = 0
		
		
		tolerance = 1 # Tolerance for difference between encoder ticks
		
		# Initial encoder states and counts
		left_counter = np.uint64(0)
		left_button = int(0)
		right_counter = np.uint64(0)
		right_button = int(0)
		
		
		action,goal = action_tuple # split tuple into direction interger and goal value
		
		# Record start angle if IMU being used
		if control_type == "i":
			start_angle_imu = current_angle_imu
		
		if action == 1: # Forward
			lpwm = lf_pwm # Left wheels forward
			rpwm = rf_pwm # Right wheels forward
			travel_type = "distance" # Distance travel type
			correction = 1 # No wheel slip compensation needed
			 
		elif action == 2: # Backward
			lpwm = lb_pwm # Left wheels backward
			rpwm = rb_pwm # Right wheels backward
			travel_type = "distance" # Distance travel type
			correction = 1 # No wheel slip compensation needed
			
		elif action == 3: # Left
			lpwm = lb_pwm # Left wheels backward
			rpwm = rf_pwm # Right wheels forward
			travel_type = "angle" # Angle travel type
			
			# If encoder-controlled, compensate for wheel slip
			if control_type == "e": 
				correction = 1.6
		
		elif action == 4: # Right
			lpwm = lf_pwm # Left wheels forward
			rpwm = rb_pwm # Right wheels backward
			travel_type = "angle" # Angle travel type
			
			# if encoder-controlled, compensate for wheel slip
			if control_type == "e":
				correction = 2
		
		# Start pwms at 0	
		lpwm.start(0)
		rpwm.start(0)
				
		# Control loop
		#while goal - travel > (goal/100): # Stop if very close to or past goal value 
		while travel < goal:
			# Update tick count if status changes
			if int(gpio.input(7)) != int(left_button):
				left_button = int(gpio.input(7))
				left_counter += 1
			if int(gpio.input(12)) != int(right_button):
				right_button = int(gpio.input(12))
				right_counter += 1
			
			# If IMU data being used during forward or backward movement,
			# Use IMU data to keep straight path movement
			if control_type == "i" and travel_type == "distance":
				angle_deviation = diff(current_angle_imu,start_angle_imu) # Error value
				if angle_deviation < -0.5: # If turning counterclockwise and out of tolerance
					if action == 1: # If moving forward
						if duty+left_adjustment < duty+20:
							left_adjustment += 0.5
						if duty+right_adjustment > duty-20:
							right_adjustment -= 0.5
							
					else: # If moving backward
						if duty+left_adjustment > duty-20:
							left_adjustment -= 0.5
						if duty+right_adjustment < duty+20:
							right_adjustment += 0.5
							
					lpwm.ChangeDutyCycle(duty+left_adjustment)
					rpwm.ChangeDutyCycle(duty+right_adjustment)
						
				elif angle_deviation > 0.5: # If turning clockwise and out of tolerance
					if action == 1: # If moving forward
						if duty+left_adjustment > duty-20:
							left_adjustment -= 0.5
						if duty+right_adjustment < duty+20:
							right_adjustment += 0.5
					else: # If moving backward
						if duty+left_adjustment < duty+20:
							left_adjustment += 0.5
						if duty+right_adjustment > duty-20:
							right_adjustment -= 0.5
							
					lpwm.ChangeDutyCycle(duty+left_adjustment)
					rpwm.ChangeDutyCycle(duty+right_adjustment)
					
				else: # Within turn error tolerance
					left_adjustment = 0
					right_adjustment = 0
					lpwm.ChangeDutyCycle(duty+left_adjustment)
					rpwm.ChangeDutyCycle(duty+right_adjustment)				

			# If left or right movement or IMU data not being used,
			# use encoder ticks to maintain straight line movement
			# or even turn
			else:
				# If left ticks < right ticks by more than tolerance,
				# increase left speed and decrease right speed
				# by adjusting duty cycles
				if left_counter+tolerance < right_counter:# turning left
					if duty+left_adjustment < duty+20:
						left_adjustment += 0.5
						lpwm.ChangeDutyCycle(duty+left_adjustment)
					if duty+right_adjustment > duty-20:
						#right_duty -= 0.5
						right_adjustment -= 0.5
						rpwm.ChangeDutyCycle(duty+right_adjustment)
						
				# If right ticks < left ticks by more than tolerance,
				# increase left speed and decrease right speed
				# by adjusting duty cycles
				elif right_counter+tolerance < left_counter:
					if duty+left_adjustment > duty-20:
						left_adjustment -= 0.5
						lpwm.ChangeDutyCycle(duty+left_adjustment)
					if duty+right_adjustment < duty+20:
						right_adjustment += 0.5
						rpwm.ChangeDutyCycle(duty+right_adjustment)
				else:
					left_adjustment = 0
					right_adjustment = 0
					lpwm.ChangeDutyCycle(duty+left_adjustment)
					rpwm.ChangeDutyCycle(duty+right_adjustment)
			
				
			old_travel = travel # Previous travel amount
			
			# Distance wheels have traveled based on encoder ticks
			left_travel =  (left_counter/20)*(3.1415*0.065)
			right_travel = (right_counter/20)*(3.1415*0.065)
			
			# Calculate travel distance or angle based on encoder or IMU data
			if travel_type == "distance":
				travel = (right_travel+left_travel)/2
			elif travel_type == "angle":
				if control_type == "e":
					travel = (np.rad2deg((left_travel + right_travel)/0.15))/correction
				elif control_type == "i":
					travel = abs(diff(current_angle_imu,start_angle_imu))
			
			# Update and write position to file if travel amount has changed
			if travel != old_travel:
				current_time = datetime.now()
				elapsed_time = str((current_time - start_time).total_seconds())
				elapsed_travel = travel - old_travel
				position = update_position(position,action,elapsed_travel)
				sx,sy,st = position
				file.write(elapsed_time+","+str(sx)+","+str(sy)+","+str(st)+"\n")

		
		# Stop motors and close file
		lpwm.stop()
		rpwm.stop()
		gameover()
		file.close()
		return action,travel, goal

		
# EXECUTE 
#----------------------------------------------------------------------

# Collect encoder vs IMU control type
while True:
	feedback = input((
	"Enter feedback type: \n"+
	"e - encoder only\n"+
	"i - IMU\n"))
	if feedback == "e":
		break
		
	elif feedback == "i":
		# Start IMU serial thread to continuously update current angle
		serial_thread = threading.Thread(target=read_serial_data, daemon=True)
		serial_thread.start()
		
		# Pause until first 10 serial readings are taken
		print("Starting IMU...")
		while True:
			if start_moving:
				print("Complete")
				break
		break
		
	else:
		print("Enter 'e' or 'i'\n")

actions_list = [] # list of tuples with direction and goal value

# Get desired actions from user
while True:
	action,goal = get_action()

	if action == 0:
		break
	actions_list.append((action,goal))

# Execute actions in sequence
recursions = 0
for item in actions_list:
	
	r_action, r_travel, r_goal = move(item,feedback)
	
	# For IMU-controlled turning, correct overshoot
	# by finding overshoot amount and setting that 
	# as new goal 
	# repeat until overshoot less than 1 or 10 recursions
	if r_action == 3 or r_action == 4:
		error = r_travel - r_goal
		while error > 1 and recursions < 10:
			recursions += 1
			r_goal = error
			if r_action == 3:
				r_action = 4
			else:
				r_action = 3
			r_action, r_travel, r_goal = move((r_action, r_goal),feedback)
			error = r_travel - r_goal
			
	time.sleep(0.5) # Pause between moves
		
gpio.cleanup()
#----------------------------------------------------------------------

	
# PLOT DATA
#----------------------------------------------------------------------

# Initialize x and y data
plot_x = []
plot_y = []
if feedback == "e":
	title_str = "Encoder-controlled path"
else:
	title_str = "IMU-controlled path"

# Read file
with open("hw8.txt","r") as file:
	for line in file:
		item = line.split(",") # Strip items in line
		
		# Append items to lists
		plot_x.append(float(item[1]))
		plot_y.append(float(item[2]))
	
file.close()

# Plot data and save figure
plt.plot(plot_x,plot_y)
plt.title(title_str)
plt.xlabel("Distance [m]")
plt.ylabel("Distance [m]")
plt.savefig("hw8.png")
# ---------------------------------------------------------------------

