import RPi.GPIO as gpio
import time

def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT) #IN 1
	gpio.setup(33, gpio.OUT) #IN 2
	gpio.setup(35, gpio.OUT) #IN 3
	gpio.setup(37, gpio.OUT) #IN 4
	
def gameover():
	#set all pins low
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)
	
def forward(tf):
	init()
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)
	
	#Right wheels
	gpio.output(35, False)
	gpio.output(37, True)
	
	#Wait
	time.sleep(tf)
	
	#Send all pins to low and clean up
	gameover()
	gpio.cleanup()
	
def backward(tf):
	init()
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)
	
	#Right wheels
	gpio.output(35, True)
	gpio.output(37, False)
	
	#Wait
	time.sleep(tf)
	
	#Send all pins to low and clean up
	gameover()
	gpio.cleanup()
	
def pivot_right(tf):
	init()
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)
	
	#Right wheels
	gpio.output(35, True)
	gpio.output(37, False)
	
	#Wait
	time.sleep(tf)
	
	#Send all pins to low and clean up
	gameover()
	gpio.cleanup()
	
def pivot_left(tf):
	init()
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)
	
	#Right wheels
	gpio.output(35, False)
	gpio.output(37, True)
	
	#Wait
	time.sleep(tf)
	
	#Send all pins to low and clean up
	gameover()
	gpio.cleanup()

def key_input(event):
	init()
	print("Key: ",event)
	key_press = event
	tf = 1
	if key_press.lower() == 'w':
		forward(tf)
	elif key_press.lower() == 'd':
		pivot_right(tf)
	elif key_press.lower() == 'a':
		pivot_left(tf)
	elif key_press.lower() == 's':
		backward(tf)
	else:
		print("Invalid key pressed")

while True:
	key_press = input("Select driving mode: ")
	if key_press == 'p':
		break
	key_input(key_press)
	

