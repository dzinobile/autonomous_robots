from matplotlib import pyplot as plt
import numpy as np
import ast
import time
with open('final_challenge/robot_positions.txt','r') as file:
    positions = []
    for line in file:
        positions.append(ast.literal_eval(line.strip()))


first_trip = []
for position in positions:
    first_trip.append(position)
    if abs(position[2]-45) < 3:
        break

red_trip = []
for i in range(len(first_trip),len(positions)):
    red_trip.append(positions[i])
    if abs(positions[i][2]-45)<3 and len(red_trip) > 150:
        break

green_trip = []
for i in range(len(first_trip)+len(red_trip),len(positions)):
    green_trip.append(positions[i])
    if abs(positions[i][2]-45)<3 and len(green_trip) > 150:
        break

blue_trip = []
for i in range(len(first_trip)+len(red_trip)+len(green_trip),len(positions)):
    blue_trip.append(positions[i])


xi = [row[0] for row in first_trip]
yi = [row[1] for row in first_trip]
xr = [row[0] for row in red_trip]
yr = [row[1] for row in red_trip]
xg = [row[0] for row in green_trip]
yg = [row[1] for row in green_trip]
xb = [row[0] for row in blue_trip]
yb = [row[1] for row in blue_trip]


plt.plot(xi,yi,color='black')
plt.plot(xr,yr,color='red')
plt.plot(xg,yg,color='green')
plt.plot(xb,yb,color='blue')
plt.title('Robot Positions')
plt.legend(["Initial path","Retrieving Red Block","Retrieving Green Block","Retrieving Blue Block"])
plt.show()


        
