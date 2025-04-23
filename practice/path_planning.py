import cv2
import numpy as np
import heapq
from matplotlib import pyplot as plt

h = 366
w = 366
r_radius = 20
b_radius = 3
goal_coords = [(10,10),(20,10),(30,10),(10,20),(20,20),(30,20),(10,30),(20,30),(30,30)]



map = np.zeros((366,366,3),dtype=np.uint8)
map = cv2.rectangle(map,(0,305),(61,365),(0,255,255),-1)
map = cv2.rectangle(map, (0,0),(122,122),(0,255,255),-1)
map = cv2.rectangle(map,(0,0),(w-1,h-1),(255,255,255),r_radius)
# CREATE RANDOM LIST OF BLOCK LOCATIONS
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




def draw_circles(b_coords):

    map = np.zeros((366,366,1),dtype=np.uint8)
    map = cv2.rectangle(map,(0,0),(365,365),(255,255,255),20)
    for item in b_coords:
        map = cv2.circle(map,item[1:3],23,255,-1)
    return map


def m_coords(point):
    x,y,t = point
    t = int(t/45)
    return y,x,t

def heuristic_c2g(point,goal):
    p1x,p1y = point[:2]
    p2x,p2y = goal[:2]

    dist = round(np.sqrt((p1x-p2x)**2 + (p1y-p2y)**2))
    return int(dist)

def get_child_coords(inpt_point,inpt_angle,inpt_step):
    px,py = inpt_point[:2]
    cx = int(round(inpt_step*np.cos(np.deg2rad(inpt_angle))+px))
    cy = int(round(inpt_step*np.sin(np.deg2rad(inpt_angle))+py))
    return((cx,cy,inpt_angle))

def line_cross(p1,p2,inpt_bufferset):
    lc_map = np.zeros((h,w,1))
    lc_map = cv2.line(lc_map,p1[:2],p2[:2],255,1)
    line_coords = np.where(lc_map==255)
    for i in range(0,len(line_coords[0])):
        if (line_coords[1][i],line_coords[0][i]) in inpt_bufferset:
            return True
    return False


def astar(inpt_map,step_dist,start_xyt,goal_xy):
    start_xyt = (int(start_xyt[0]),int(start_xyt[1]),int(start_xyt[2]))
    goal_xy = (int(goal_xy[0]),int(goal_xy[1]))
    buffer_set = set()
    buffer_coord = np.where(inpt_map==255)
    for i in range(0,len(buffer_coord[0])):
        buffer_set.add((buffer_coord[1][i],buffer_coord[0][i]))
    del buffer_coord
    
    m_c2c = np.zeros((h,w,8))
    m_ctot = np.zeros((h,w,8))
    m_px = np.zeros((h,w,8))
    m_py = np.zeros((h,w,8))
    m_pt = np.zeros((h,w,8))

    open_list = []
    closed_list = []
    heapq.heapify(open_list)
    heapq.heapify(closed_list)
    first_node = (heuristic_c2g(start_xyt,goal_xy),start_xyt)
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
        if heuristic_c2g(p_xyt,goal_xy) < 20:
            #print(p_xyt)
            break
        
        
        for item in [-90, -45, 0, 45, 90]:
            ang = p_xyt[2] + item
            ang = int(ang%360)
            c_xyt = get_child_coords(p_xyt,ang,step_dist)
            #print(c_xyt)
            if c_xyt[:2] in buffer_set:
                #print("in buffer set")
                continue
            if line_cross(p_xyt,c_xyt,buffer_set):
                #print("line cross")
                continue
            c_c2c = int(round(m_c2c[m_coords(p_xyt)] + step_dist))
            c_ctg = heuristic_c2g(c_xyt,goal_xy)
            c_ctot = c_c2c+(c_ctg*1.5) # WEIGHTED
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
    #print(final_path[0])
    #print(final_path[-1])
    display_map = inpt_map.copy()
    display_map = cv2.cvtColor(display_map, cv2.COLOR_GRAY2BGR)
    display_map = cv2.rectangle(display_map,(0,305),(61,365),(0,255,255),-1)
    display_map = cv2.rectangle(display_map, (0,0),(122,122),(0,255,255),-1)
    for node in closed_list:
        cxyt = node[1]
        cx,cy,ct = cxyt
        px = int(m_px[m_coords(cxyt)])
        py = int(m_py[m_coords(cxyt)])
        display_map = cv2.line(display_map,(px,py),(cx,cy),(0,255,255),1)
        #cv2.imshow('animation',display_map)
        #cv2.waitKey(1)
    
    for i in range(1,len(final_path)):

        display_map = cv2.line(display_map,final_path[i-1],final_path[i],(0,0,255),2)
        #cv2.imshow('animation',display_map)
        #cv2.waitKey(1)

    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return final_path


initial_block_coords = block_coords
all_paths = []
home = (30,336,0)
retrieve_start = (61,61,0)

step_size = 30
while step_size > 5:
    try:
        map = draw_circles(block_coords)
        all_paths.append(astar(map,step_size,home,retrieve_start[:2]))
        if len(block_coords) < 1:
            break
        next_goal = heapq.heappop(block_coords)[1:3]
        map = draw_circles(block_coords)
        all_paths.append(astar(map,step_size,retrieve_start,next_goal))
        current_start = (next_goal[0],next_goal[1],0)
        all_paths.append(astar(map,step_size,current_start,home[:2]))
    except:
        print("except")
        step_size -= 5


final_map = np.zeros((h,w,3),dtype=np.uint8)
final_map = cv2.rectangle(final_map,(0,305),(61,365),(0,255,255),-1)
final_map = cv2.rectangle(final_map, (0,0),(122,122),(0,255,255),-1)
for item in block_coords:
    final_map = cv2.circle(final_map,item[1:3],3,color_list[item[3]],-1)

for path in all_paths:
    for i in range(1,len(path)):
        final_map = cv2.line(final_map,path[i-1],path[i],(0,0,255),2)
        cv2.imshow('final_animation',final_map)
        cv2.waitKey(1)

cv2.waitKey(0)
cv2.destroyAllWindows()

all_paths_converted = []
x = []
y = []
for path in all_paths:
    path_converted = []
    for item in path:
        xy = (item[0],h-item[1])
        x.append(item[0])
        y.append(h-item[1])
        path_converted.append(xy)
    all_paths_converted.append(path_converted)

# plt.plot(x,y)
# plt.show()







