import RPi.GPIO as gpio

# define trig and echo pins
trig = 16
echo = 18

# initialize input and output pins
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT) #IN 1
	gpio.setup(33, gpio.OUT) #IN 2
	gpio.setup(35, gpio.OUT) #IN 3
	gpio.setup(37, gpio.OUT) #IN 4
	gpio.setup(36, gpio.OUT) #servo
	gpio.setup(trig, gpio.OUT) #sonar
	gpio.setup(echo, gpio.IN) #sonar

def gameover():
	# set all output pins low
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)
	gpio.output(36, False)
	gpio.output(trig, False)
	gpio.cleanup()
	

init()
gameover()
