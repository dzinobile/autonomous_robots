import cv2
import numpy as np
import heapq

map = np.zeros((366,366,3),dtype=np.uint8)
map = cv2.rectangle(map,(0,366-61),(61,366),(255,255,255),-1)
map = cv2.rectangle(map, (0,0),(122,122),(255,255,255),-1)
blue_blocks = []
green_blocks = []
red_blocks = []

color_list = [(255,0,0),(0,255,0),(0,0,255)]
while len(blue_blocks)<3:
    block_x = np.random.randint(1,365)
    block_y = np.random.randint(1,365)
    block_d = np.sqrt((block_x-122)**2 + (block_y-122)**2)
    if (map[block_y,block_x] == [0,0,0]).all():
        heapq.heappush(blue_blocks,(block_d,block_x,block_y,0))

while len(green_blocks)<3:
    block_x = np.random.randint(1,365)
    block_y = np.random.randint(1,365)
    block_d = np.sqrt((block_x-122)**2 + (block_y-122)**2)
    if (map[block_y,block_x] == [0,0,0]).all():
        heapq.heappush(green_blocks,(block_d,block_x,block_y,1))

while len(red_blocks)<3:
    block_x = np.random.randint(1,365)
    block_y = np.random.randint(1,365)
    block_d = np.sqrt((block_x-122)**2 + (block_y-122)**2)
    if (map[block_y,block_x] == [0,0,0]).all():
        heapq.heappush(red_blocks,(block_d,block_x,block_y,2))

block_coords = []
while len(block_coords) < 9:
    block_coords.append(heapq.heappop(blue_blocks))
    block_coords.append(heapq.heappop(green_blocks))
    block_coords.append(heapq.heappop(red_blocks))

def draw_circles(block_coords):
    map = np.zeros((366,366,3),dtype=np.uint8)
    map = cv2.rectangle(map,(0,366-61),(61,366),(255,255,255),-1)
    map = cv2.rectangle(map, (0,0),(122,122),(255,255,255),-1)
    for item in block_coords:
        map = cv2.circle(map,item[1:3],40,(255,255,255),-1)

    for item in block_coords:
        map = cv2.circle(map,item[1:3],5,color_list[item[3]],-1)

path_coords =[]
path_coords.append((30,366-30))
path_coords.append((61,61))

for item in block_coords:




cv2.imshow('map',map)
cv2.waitKey(0)
cv2.destroyAllWindows()