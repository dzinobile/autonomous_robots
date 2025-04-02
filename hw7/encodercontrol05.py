# Import dependencies
import RPi.GPIO as gpio
import numpy as np
import time
from matplotlib import pyplot as plt

# Clear all info from encondercontrol05.txt file 
with open("encodercontrol05.txt","w") as file:
	file.close()

# GPIO pin initialization function
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT) # Left wheels forward
	gpio.setup(33, gpio.OUT) # Left wheels backward
	gpio.setup(35, gpio.OUT) # Right wheels backward
	gpio.setup(37, gpio.OUT) # Right wheels forward
	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # Right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # Left encoder

# Function for turning off all pin outputs
def gameover():
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)
	gpio.cleanup()

# Initialize GPIO pins and pwms for both wheel sides and directions
init()
lf_pwm = gpio.PWM(31,50)
rf_pwm = gpio.PWM(37,50)
lb_pwm = gpio.PWM(33,50)
rb_pwm = gpio.PWM(35,50)

reading = 0 # Variable for number of readings taken, for plot
time.sleep(0.1) # Give extra time for setup

# Function for moving car forward
def forward(goal,reading):
	with open("encodercontrol05.txt","a") as file:
		init() # Run pin initialization
		
		# Initial travel distances [m]
		left_travel = 0
		right_travel = 0
		travel = 0
		
		# Initial duty cycle setting and start
		left_duty = 30
		right_duty = 30
		lf_pwm.start(left_duty)
		rf_pwm.start(right_duty)
		
		tolerance = 1 # Tolerance for control loop [encoder ticks]
		
		# Initial tick counts and status
		left_counter = np.uint64(0)
		left_button = int(0)
		right_counter = np.uint64(0)
		right_button = int(0)
		
		# Control loop
		while travel < goal: 
			
			# Update tick count if status changes
			if int(gpio.input(7)) != int(left_button):
				left_button = int(gpio.input(7))
				left_counter += 1
			if int(gpio.input(12)) != int(right_button):
				right_button = int(gpio.input(12))
				right_counter += 1
			
			# If left ticks < right ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			if left_counter+tolerance < right_counter:
				if left_duty < 60: # Limit max speed
					left_duty  += 0.5
					lf_pwm.ChangeDutyCycle(left_duty)
				if right_duty > 0: # Limit min speed
					right_duty -= 0.5
					rf_pwm.ChangeDutyCycle(right_duty)
					
			# If right ticks < left ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			elif right_counter+tolerance < left_counter:
				if left_duty > 0:
					left_duty -= 0.5
					lf_pwm.ChangeDutyCycle(left_duty)
				if right_duty < 60:
					right_duty += 0.5
					rf_pwm.ChangeDutyCycle(right_duty)
					
			# If matching number of ticks, continue at equal speed
			else:
				left_duty = 30
				right_duty = 30
				lf_pwm.ChangeDutyCycle(left_duty)
				rf_pwm.ChangeDutyCycle(right_duty)
			
			# Calculate left, right, and average travel distances
			left_travel =  (left_counter/20)*(3.1415*0.065)
			right_travel = (right_counter/20)*(3.1415*0.065)
			travel = (right_travel+left_travel)/2
			
			file.write(str(reading)+","+str(left_button)+","+str(right_button)+"\n") # Write encoder status to file
			reading += 1 # Increment number of encoder readings
		
		# Stop all outputs and close file
		rf_pwm.stop()
		lf_pwm.stop()
		gameover()
		file.close()
		
		return reading # Pass updated number of readings

# Function for moving car backward
def backward(goal,reading):
	with open("encodercontrol05.txt","a") as file:
		init() # Run pin initialization
		
		# Initial travel distances [m]
		left_travel = 0
		right_travel = 0
		travel = 0
		
		# Initial duty cycle setting and start
		left_duty = 30
		right_duty = 30
		lb_pwm.start(left_duty)
		rb_pwm.start(right_duty)
		
		tolerance = 1 # Tolerance for control loop [encoder ticks]
		
		# Initial tick counts and status
		left_counter = np.uint64(0)
		left_button = int(0)
		right_counter = np.uint64(0)
		right_button = int(0)
		
		# Control loop
		while travel < goal:
			
			# Update tick count if status changes
			if int(gpio.input(7)) != int(left_button):
				left_button = int(gpio.input(7))
				left_counter += 1
			if int(gpio.input(12)) != int(right_button):
				right_button = int(gpio.input(12))
				right_counter += 1
			
			# If left ticks < right ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			if left_counter+tolerance < right_counter:
				if left_duty < 60:
					left_duty  += 0.5
					lb_pwm.ChangeDutyCycle(left_duty)
				if right_duty > 0:
					right_duty -= 0.5
					rb_pwm.ChangeDutyCycle(right_duty)
					
			# If right ticks < left ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			elif right_counter+tolerance < left_counter:
				if left_duty > 0:
					left_duty -= 0.5
					lb_pwm.ChangeDutyCycle(left_duty)
				if right_duty < 60:
					right_duty += 0.5
					rb_pwm.ChangeDutyCycle(right_duty)
			
			# If matching number of ticks, continue at equal speed
			else:
				left_duty = 30
				right_duty = 30
				lb_pwm.ChangeDutyCycle(left_duty)
				rb_pwm.ChangeDutyCycle(right_duty)
			
			# Calculate left, right, and average travel distances
			left_travel =  (left_counter/20)*(3.1415*0.065)
			right_travel = (right_counter/20)*(3.1415*0.065)
			travel = (right_travel+left_travel)/2
			
			file.write(str(reading)+","+str(left_button)+","+str(right_button)+"\n") # Write encoder statuses to file
			reading += 1 # Increment number of encoder readings for plot
			
		# Stop all outputs and close file
		rb_pwm.stop()
		lb_pwm.stop()
		gameover()
		file.close()
		
		return reading # Pass updated number of readings

# Function for turning car left in place		
def turn_left(goal,reading):
	with open("encodercontrol05.txt","a") as file:
		init() # Run pin initialization
		
		# Initial travel distances [m] and angle [deg]
		left_travel = 0
		right_travel = 0
		angle = 0
		
		# Initial duty cycle setting and start
		left_duty = 80
		right_duty = 80
		lb_pwm.start(left_duty)
		rf_pwm.start(right_duty)
		
		tolerance = 1 # Tolerance for control loop [encoder ticks]
		correction = 1.6 # Correction factor to account for wheel slip (determined experimentally)
		
		# Initial tick counts and status
		left_counter = np.uint64(0)
		left_button = int(0)
		right_counter = np.uint64(0)
		right_button = int(0)
		
		# Control loop
		while angle < goal*correction: # Correction factor applied to correct for wheel slip
			
			# Update tick count if status changes
			if int(gpio.input(7)) != int(left_button):
				left_button = int(gpio.input(7))
				left_counter += 1
			if int(gpio.input(12)) != int(right_button):
				right_button = int(gpio.input(12))
				right_counter += 1
			
			# If left ticks < right ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			if left_counter+tolerance < right_counter:
				if left_duty < 100:
					left_duty  += 0.5
					lb_pwm.ChangeDutyCycle(left_duty)
				if right_duty > 60:
					right_duty -= 0.5
					rf_pwm.ChangeDutyCycle(right_duty)
			
			# If right ticks < left ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			elif right_counter+tolerance < left_counter:
				if left_duty > 60:
					left_duty -= 0.5
					lb_pwm.ChangeDutyCycle(left_duty)
				if right_duty < 100:
					right_duty += 0.5
					rf_pwm.ChangeDutyCycle(right_duty)
			
			# If matching number of ticks, continue at equal speed
			else:
				left_duty = 80
				right_duty = 80
				lb_pwm.ChangeDutyCycle(left_duty)
				rf_pwm.ChangeDutyCycle(right_duty)
			
			# Calculate left and right travel distances
			left_travel =  (left_counter/20)*(3.1415*0.065)
			right_travel = (right_counter/20)*(3.1415*0.065)
			
			# Calculate angle based on average of travel distance and 0.15 diameter
			angle = np.rad2deg((left_travel + right_travel)/0.15) 
			
			file.write(str(reading)+","+str(left_button)+","+str(right_button)+"\n") # Write encoder statuses to file
			reading += 1 # Increment number of encoder readings for plot
			
		# Stop all outputs and close file
		lb_pwm.stop()
		rf_pwm.stop()
		gameover()
		file.close()
		
		return reading # Pass updated number of readings

# Function for turning car right in place
def turn_right(goal,reading):
	with open("encodercontrol05.txt","a") as file:
		init() # Run pin initialization
		
		# Initial travel distances [m] and angle [deg]
		left_travel = 0
		right_travel = 0
		angle = 0
		
		# Initial duty cycle settings and start
		left_duty = 80
		right_duty = 80
		lf_pwm.start(left_duty)
		rb_pwm.start(right_duty)
		
		tolerance = 1 # Tolerance for control loop [encoder ticks]
		correction = 2 # Correction factor to account for wheel slip (determined experimentally)
		
		# Initial tick counts and status
		left_counter = np.uint64(0)
		left_button = int(0)
		right_counter = np.uint64(0)
		right_button = int(0)
		
		# Control loop
		while angle < goal*correction: # Correction factor applied to correct for wheel slip
			
			# Update tick count if status changes
			if int(gpio.input(7)) != int(left_button):
				left_button = int(gpio.input(7))
				left_counter += 1
			if int(gpio.input(12)) != int(right_button):
				right_button = int(gpio.input(12))
				right_counter += 1
			
			# If left ticks < right ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles
			if left_counter+tolerance < right_counter:
				if left_duty < 100:
					left_duty  += 0.5
					lf_pwm.ChangeDutyCycle(left_duty)
				if right_duty > 60:
					right_duty -= 0.5
					rb_pwm.ChangeDutyCycle(right_duty)

			# If right ticks < left ticks by more than tolerance,
			# increase left speed and decrease right speed
			# by adjusting duty cycles	
			elif right_counter+tolerance < left_counter:
				if left_duty > 60:
					left_duty -= 0.5
					lf_pwm.ChangeDutyCycle(left_duty)
				if right_duty < 100:
					right_duty += 0.5
					rb_pwm.ChangeDutyCycle(right_duty)
					
			# If matching number of ticks, continue at equal speed
			else:
				left_duty = 80
				right_duty = 80
				lf_pwm.ChangeDutyCycle(left_duty)
				rb_pwm.ChangeDutyCycle(right_duty)
			
			# Calculate left and right travel distances
			left_travel =  (left_counter/20)*(3.1415*0.065)
			right_travel = (right_counter/20)*(3.1415*0.065)
			
			# Calculate angle based on average of travel distance and 0.15 diameter
			angle = np.rad2deg((left_travel + right_travel)/0.15)
			
			file.write(str(reading)+","+str(left_button)+","+str(right_button)+"\n") # Write encoder statuses to file
			reading += 1 # Increment number of encoder readings for plot
		
		# Stop all outputs and close file
		lf_pwm.stop()
		rb_pwm.stop()
		gameover()
		file.close()
		
		return reading # Pass updated number of readings

# Function for getting user input for action
def get_action():
	while True: # Loop until valid inputs recieved
		
		# Get user input for direction
		direction = input("Enter action: \n 'f' for forward\n 'b' for backward \n 'l' for left\n 'r' for right\n 'q' for quit ")
		
		# Pass user input to get_goal function
		if direction == "f" or direction == "b":
			goal = get_goal("dist")
			break
		elif direction == "l" or direction == "r":
			goal = get_goal("ang")
			break
		elif direction == "q":
			goal = None
			break
		else:
			print("Enter f, b, l, r, or q")
	return direction,goal

# Function for getting user input for target distance or target angle
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

# Start program
while True: # Loop until recieves user input to end program
	
	action,goal = get_action() # Get user inputs
	
	# Run appropriate move function
	if action == "f":
		reading = forward(goal,reading)
	elif action == "b":
		reading = backward(goal,reading)
	elif action == "l":
		reading = turn_left(goal,reading)
	elif action == "r":
		reading = turn_right(goal,reading)
	
	elif action == "q":
		print("End program")
		break
	
# Load encoder data
x = [] # Reading number
e_left = [] # Left encoder status
e_right = [] # Right encoder status

# Read file
with open("encodercontrol05.txt","r") as file:
	for line in file:
		item = line.split(",") # Strip items in line
		
		# Append items to lists
		x.append(int(item[0]))
		e_left.append(int(item[1]))
		e_right.append(int(item[2]))	
file.close()

# Plot encoder data
fig, axs = plt.subplots(2)
plt.suptitle("Motor Encoder Analysis")

axs[0].plot(x, e_right)
axs[0].set(ylabel="Back right encoder")

axs[1].plot(x,e_left)
axs[1].set(ylabel="Front left encoder")

plt.savefig("encodercontrol05.png")

