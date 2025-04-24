import cv2
import numpy as np
import heapq
h = 366
w = 366
map = np.zeros((h,w,3),dtype=np.uint8)

def get_child_coords(inpt_point,inpt_angle,inpt_step):
    px,py = inpt_point[:2]
    cx = int(round(inpt_step*np.cos(np.deg2rad(inpt_angle))+px))
    cy = int(round(inpt_step*np.sin(np.deg2rad(inpt_angle))+py))
    return((cx,cy,inpt_angle))

p1 = (100,100)
p2 = (get_child_coords(p1,45,30))[:2]

map = cv2.line(map,p1,p2,(255,255,255),2)
cv2.imshow('map',map)
cv2.waitKey(0)
cv2.destroyAllWindows()